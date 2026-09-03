/*
 * connectivity/ble_scan.c - see ble_scan.h.
 */
#include "ble_scan.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "store/config/ble_store_config.h"

#include <string.h>

static const char *TAG = "ble_scan";

#define BLE_SCAN_NVS_NS "ble_scan"
#define BLE_SCAN_DURATION_MS 30000

/* NimBLE's store-config init is not in a public header - ble_debug.c
 * declares the same forward reference for the same reason. Harmless to
 * call once from here too (idempotent, and ble_debug stays disabled). */
void ble_store_config_init(void);

/* Internal DMA-capable RAM is chronically tight on this board and swings
 * widely with whatever else is transiently happening (SD mount cycles,
 * mesh_log, sensor polling) - confirmed live: as low as ~5-9 KB during an
 * otherwise idle moment, vs a ~40 KB peak. A BT controller init attempt
 * during one of those low dips fails outright (BLE_INIT: Malloc failed ->
 * watchdog panic - reproduced live). Rather than chase an exact buffer
 * configuration that might dodge that ceiling most of the time, this is a
 * pre-flight check: refuse to even attempt bringing up the host stack
 * when there's clearly not enough headroom, and tell the UI why, instead
 * of gambling on a crash.
 *
 * 45 KB, down from an initial 60 KB guess: that first crash happened at
 * ~44 KB free with a bigger controller buffer footprint (BLE_MAX_ACT=6)
 * and ~24 KB more RAM permanently claimed by oversized task stacks, both
 * since fixed (BLE_MAX_ACT=1, various task stack-size trims - see
 * docs/application.md section 12). This value is a live experiment, not
 * a proven-safe number yet - lower it further only with real testing,
 * never by assumption. */
#define BLE_SCAN_MIN_FREE_DMA_BYTES (45 * 1024)

static bool s_host_started;
static bool s_enabled;     /* target state, persisted, default off */
static volatile bool s_scanning;
static bool s_low_mem;     /* last enable attempt was refused for low memory */

static SemaphoreHandle_t s_mux;
static ble_scan_result_t s_results[BLE_SCAN_MAX_RESULTS];
static size_t s_result_count;

static void ble_scan_start_disc(void);

static void ble_scan_clear_results(void)
{
    if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_result_count = 0;
        xSemaphoreGive(s_mux);
    }
}

/* Dedup by address; add a new row if there's room. First-seen RSSI is kept
 * (matches WiFi's own "one snapshot per scan" behavior) rather than
 * continuously overwritten by every repeat advertisement. */
static void ble_scan_record(const ble_addr_t *addr, int8_t rssi, const char *name)
{
    if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    for (size_t i = 0; i < s_result_count; i++) {
        if (memcmp(s_results[i].addr, addr->val, 6) == 0) {
            if (name[0] != '\0' && s_results[i].name[0] == '\0') {
                snprintf(s_results[i].name, sizeof(s_results[i].name), "%s", name);
            }
            xSemaphoreGive(s_mux);
            return;
        }
    }
    if (s_result_count < BLE_SCAN_MAX_RESULTS) {
        ble_scan_result_t *r = &s_results[s_result_count++];
        memcpy(r->addr, addr->val, 6);
        r->rssi_dbm = rssi;
        snprintf(r->name, sizeof(r->name), "%s", name);
    }
    xSemaphoreGive(s_mux);
}

static int ble_scan_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        char name[BLE_SCAN_NAME_MAX + 1] = "";
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0 &&
            fields.name != NULL && fields.name_len > 0) {
            size_t len = fields.name_len < BLE_SCAN_NAME_MAX ? fields.name_len : BLE_SCAN_NAME_MAX;
            memcpy(name, fields.name, len);
            name[len] = '\0';
        }
        ble_scan_record(&event->disc.addr, event->disc.rssi, name);
        break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_scanning = false;
        s_enabled = false;   /* one-shot: reflect "done" back to the switch */
        ESP_LOGI(TAG, "scan complete: %u device(s)", (unsigned)s_result_count);
        break;
    default:
        break;
    }
    return 0;
}

static void ble_scan_start_disc(void)
{
    uint8_t own_addr_type;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        ESP_LOGW(TAG, "ble_hs_id_infer_auto failed");
        s_enabled = false;
        return;
    }
    struct ble_gap_disc_params params;
    memset(&params, 0, sizeof(params));
    params.passive = 1;
    params.filter_duplicates = 0;   /* dedup ourselves, so repeat adverts can fill in a late name */

    ble_scan_clear_results();
    s_scanning = true;
    int rc = ble_gap_disc(own_addr_type, BLE_SCAN_DURATION_MS, &params, ble_scan_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_disc failed: %d", rc);
        s_scanning = false;
        s_enabled = false;
    }
}

static void ble_scan_on_sync(void)
{
    if (s_enabled && !s_scanning) {
        ble_scan_start_disc();
    }
}

static void ble_scan_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset: %d", reason);
    s_scanning = false;
}

static void ble_scan_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* Brings up the NimBLE host stack on first use - same sequence
 * ble_debug.c's (disabled) init used, standalone here since that module
 * never runs. Only called once, from the control task below, the first
 * time the switch is turned on - never at boot, so an otherwise BLE-idle
 * session never pays this cost. */
static void ble_scan_ensure_host(void)
{
    if (s_host_started) {
        return;
    }
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_init: %d", rc);
        return;
    }
    ble_hs_cfg.reset_cb = ble_scan_on_reset;
    ble_hs_cfg.sync_cb = ble_scan_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    nimble_port_freertos_init(ble_scan_host_task);
    s_host_started = true;
}

static void ble_scan_task(void *arg)
{
    (void)arg;
    bool was_enabled = false;
    for (;;) {
        if (s_enabled && !was_enabled) {
            /* Only actually attempt bringing up the BT stack (first use)
             * when there's real headroom for it - see
             * BLE_SCAN_MIN_FREE_DMA_BYTES's comment above. A later
             * toggle-on, once the host is already up, doesn't need to
             * repeat this check - the controller's one-time buffer
             * allocation already succeeded and isn't retaken per scan. */
            size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
            if (!s_host_started && free_dma < BLE_SCAN_MIN_FREE_DMA_BYTES) {
                ESP_LOGW(TAG, "refusing to start: only %u bytes free DMA RAM (need %u)",
                         (unsigned)free_dma, (unsigned)BLE_SCAN_MIN_FREE_DMA_BYTES);
                s_low_mem = true;
                s_enabled = false;
                was_enabled = false;
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            s_low_mem = false;
            ble_scan_ensure_host();
            /* Host may already be synced (a later toggle-on) or still
             * coming up (the very first one) - either way ble_scan_on_sync()
             * starts the actual disc once it's ready. Kick it directly too
             * in case sync already happened before this flag flipped. */
            if (ble_hs_synced()) {
                ble_scan_start_disc();
            }
        } else if (!s_enabled && was_enabled && s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }
        was_enabled = s_enabled;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

bool ble_scan_get_enabled(void)
{
    return s_enabled;
}

void ble_scan_set_enabled(bool on)
{
    if (on == s_enabled) {
        return;
    }
    s_enabled = on;
    nvs_handle_t h;
    if (nvs_open(BLE_SCAN_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "en", on ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool ble_scan_is_scanning(void)
{
    return s_scanning;
}

bool ble_scan_low_mem(void)
{
    return s_low_mem;
}

size_t ble_scan_get_results(ble_scan_result_t *out, size_t max)
{
    size_t n = 0;
    if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        n = (s_result_count < max) ? s_result_count : max;
        memcpy(out, s_results, n * sizeof(ble_scan_result_t));
        xSemaphoreGive(s_mux);
    }
    return n;
}

void ble_scan_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
    /* s_enabled deliberately stays false at boot regardless of what was
     * persisted last session - unlike WiFi/LoRa, this is a one-shot 30s
     * scan, not a state that makes sense to silently resume on its own;
     * the NVS value exists for a future "remember last choice" UI nicety,
     * not to auto-restart scanning. */
    xTaskCreate(ble_scan_task, "ble_scan", 4096, NULL, 3, NULL);
}
