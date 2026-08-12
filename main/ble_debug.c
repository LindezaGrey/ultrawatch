/*
 * ble_debug.c - BLE debug bridge (NimBLE peripheral).
 *
 * A small GATT service ("UWatch") that lets a phone/PC drive the watch
 * console wirelessly:
 *   - CMD characteristic  (write)  -> uwatch_debug_process_cmd()
 *   - RESP characteristic (notify) <- console output mirrored via a vprintf
 *                                     hook so command results stream back.
 *   - TELEM characteristic (read/notify) <- steps + GNSS snapshot.
 *
 * Telemetry is pushed once per second while connected; the console mirror is
 * best-effort (drops are fine for debug).
 */
#include "ble_debug.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "host/util/util.h"
#include "store/config/ble_store_config.h"

#include "bhi260ap.h"
#include "m10q.h"
#include "tracking.h"
#include "uwatch_main.h"

/* NimBLE's store-config init is not in a public header; the example declares
 * it explicitly. */
void ble_store_config_init(void);

static const char *TAG = "ble_debug";

/* Custom service: 0xDEAD  (16-bit, easy to scan for)
 *   CMD    0xDE01  write
 *   RESP   0xDE02  read + notify
 *   TELEM  0xDE03  read + notify
 */
#define BLE_DBG_SVC_UUID   0xDEAD
#define BLE_DBG_CMD_UUID   0xDE01
#define BLE_DBG_RESP_UUID  0xDE02
#define BLE_DBG_TELEM_UUID 0xDE03

#define TELEM_PERIOD_MS    1000

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_resp_val_handle;
static uint16_t s_telem_val_handle;
static bool s_adv_active;
static volatile bool s_telem_subscribed;   /* client subscribed to TELEM */
static volatile bool s_resp_subscribed;    /* client subscribed to RESP */

static int ble_debug_gap_event(struct ble_gap_event *event, void *arg);
static void ble_debug_advertise(void);
static void ble_debug_send_capture(void);
static void ble_debug_process_command_queue(void);

/* ---- GATT access ---- */

static int ble_debug_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (attr_handle == s_telem_val_handle) {
            char buf[128];
            uint32_t steps = 0;
            bhi260ap_get_step_count(&steps);
            m10q_fix_t fix;
            m10q_get_fix(&fix);
            int n = snprintf(buf, sizeof(buf),
                             "steps=%lu gnss_state=%d fix_valid=%d",
                             (unsigned long)steps, (int)m10q_get_state(),
                             (int)fix.valid);
            os_mbuf_append(ctxt->om, buf, (uint16_t)n);
            return 0;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ESP_LOGI(TAG, "WRITE op, attr_handle=%u len=%u",
                 (unsigned)attr_handle, (unsigned)OS_MBUF_PKTLEN(ctxt->om));
        /* Commands are NUL-terminated by the client; truncate safely. */
        static char cmd[64];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= sizeof(cmd)) {
            len = sizeof(cmd) - 1;
        }
        os_mbuf_copydata(ctxt->om, 0, len, cmd);
        cmd[len] = '\0';
        ESP_LOGI(TAG, "BLE cmd: %s", cmd);
        ble_debug_run_command(cmd);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def ble_debug_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_DBG_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(BLE_DBG_CMD_UUID),
                .access_cb = ble_debug_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BLE_DBG_RESP_UUID),
                .access_cb = ble_debug_access,
                .val_handle = &s_resp_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BLE_DBG_TELEM_UUID),
                .access_cb = ble_debug_access,
                .val_handle = &s_telem_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static void ble_debug_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    (void)arg;
    if (ctxt && ctxt->op == BLE_GATT_REGISTER_OP_CHR) {
        const ble_uuid_t *u = ctxt->chr.chr_def->uuid;
        ESP_LOGI(TAG, "registered CHR uuid=0x%04x def=0x%04x val=0x%04x",
                 (unsigned)((u && u->type == BLE_UUID_TYPE_16) ? BLE_UUID16(u)->value : 0),
                 (unsigned)ctxt->chr.def_handle,
                 (unsigned)ctxt->chr.val_handle);
    }
}

static void ble_debug_on_sync(void)
{
    /* Figure out address type and start advertising. */
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto: %d", rc);
        return;
    }
    (void)own_addr_type;
    ble_debug_advertise();
}

static void ble_debug_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset: %d", reason);
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
}

/* ---- GAP ---- */

static int ble_debug_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_adv_active = false;   /* advertising stops while connected */
            s_telem_subscribed = false;
            s_resp_subscribed = false;
            ESP_LOGI(TAG, "connected, handle=%d", (int)s_conn_handle);
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(TAG, "connect failed, status=%d", event->connect.status);
            ble_debug_advertise();   /* reconnect */
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_telem_subscribed = false;
        s_resp_subscribed = false;
        ble_debug_advertise();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGW(TAG, "advertising complete (reason=%d), restarting",
                 event->adv_complete.reason);
        ble_debug_advertise();
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %u", (unsigned)event->mtu.value);
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_telem_val_handle) {
            s_telem_subscribed = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_resp_val_handle) {
            s_resp_subscribed = event->subscribe.cur_notify;
        }
        ESP_LOGI(TAG, "subscribe: attr=%u notify=%d",
                 (unsigned)event->subscribe.attr_handle,
                 (int)event->subscribe.cur_notify);
        return 0;
    default:
        return 0;
    }
}

static void ble_debug_advertise(void)
{
    if (s_adv_active) {
        return;
    }
    s_adv_active = true;   /* assume success; cleared on connect/complete-fail */
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    uint8_t own_addr_type;

    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "addr type: %d", rc);
        s_adv_active = false;
        return;
    }

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"UWatch";
    fields.name_len = strlen("UWatch");
    fields.name_is_complete = 1;
    fields.uuids16 = (ble_uuid16_t[]){ BLE_UUID16_INIT(BLE_DBG_SVC_UUID) };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields: %d", rc);
        s_adv_active = false;
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_debug_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv start: %d", rc);
        s_adv_active = false;
        return;
    }
    ESP_LOGI(TAG, "advertising as 'UWatch'");
}

/* ---- Telemetry push ---- */

static void telemetry_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEM_PERIOD_MS));
        /* Execute any pending BLE commands (outside the NimBLE host task). */
        ble_debug_process_command_queue();
        ble_debug_send_capture();   /* deliver any pending command response */
        ble_debug_notify_telemetry();
    }
}

void ble_debug_notify_telemetry(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_telem_subscribed) {
        return;
    }
    char buf[128];
    uint32_t steps = 0;
    bhi260ap_get_step_count(&steps);
    m10q_fix_t fix;
    m10q_get_fix(&fix);
    tracking_totals_t t;
    tracking_get_totals(&t);
    int n = snprintf(buf, sizeof(buf),
                     "steps=%lu track_dist_m=%lu track_steps=%lu "
                     "gnss_state=%d fix_valid=%d sats=%u lat=%.5f lon=%.5f",
                     (unsigned long)steps, (unsigned long)(t.dist_cm / 100),
                     (unsigned long)t.steps, (int)m10q_get_state(),
                     (int)fix.valid, (unsigned)fix.sat_count, fix.lat, fix.lon);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, (uint16_t)n);
    if (!om) {
        return;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_telem_val_handle, om);
    if (rc != 0 && rc != BLE_HS_ENOTCONN) {
        ESP_LOGW(TAG, "telem notify: %d", rc);
    }
}

/* ---- Command + console mirror ---- */

/* Only capture the output of a BLE-invoked command (NOT all system logs -
 * mirroring every printf floods the link and drops the connection). Output is
 * buffered while a command runs, then sent as RESP notifications from the
 * telemetry task (NOT from the GATT callback, where notify is re-entrant). */
#define CMD_CAPTURE_MAX 2048
static char s_capture[CMD_CAPTURE_MAX];
static size_t s_capture_len;
static volatile bool s_capturing;
static volatile bool s_resp_pending;

static void ble_debug_console_write(const char *text, size_t len)
{
    if (!s_capturing || len == 0) {
        return;
    }
    while (len > 0 && s_capture_len < sizeof(s_capture) - 1) {
        s_capture[s_capture_len++] = *text++;
        len--;
    }
}

/* Send the buffered command output as RESP notifications. Runs on the
 * telemetry task, outside the NimBLE host task context. */
static void ble_debug_send_capture(void)
{
    if (!s_resp_pending) {
        return;
    }
    s_resp_pending = false;
    if (s_capture_len == 0 || s_conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        !s_resp_subscribed) {
        s_capture_len = 0;
        return;
    }
    const size_t chunk = 120;
    size_t off = 0;
    while (off < s_capture_len) {
        size_t c = (s_capture_len - off) < chunk ? (s_capture_len - off) : chunk;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_capture + off, (uint16_t)c);
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_resp_val_handle, om);
        }
        off += c;
    }
    s_capture_len = 0;
}

/* vprintf hook: when capturing a BLE command, tee output into the buffer. */
static vprintf_like_t s_prev_vprintf;

static int ble_debug_vprintf(const char *fmt, va_list args)
{
    static char line[160];
    int n = vsnprintf(line, sizeof(line), fmt, args);
    if (n > 0 && s_capturing) {
        ESP_LOGI(TAG, "vprintf capture: '%s'", line);
        ble_debug_console_write(line, (size_t)n);
    }
    if (s_prev_vprintf) {
        return s_prev_vprintf(fmt, args);
    }
    return vprintf(fmt, args);
}

/* Execute a console command over BLE: capture its output, then flag it for
 * delivery (sent by the telemetry task). Runs on the telemetry task, NOT the
 * GATT write callback (NimBLE host task): printf from inside a GATT access
 * callback is suppressed/risky, so we defer the command execution. */
#define CMD_QUEUE_LEN 4
static QueueHandle_t s_cmd_queue;

void ble_debug_run_command(const char *cmd)
{
    char *copy = strdup(cmd);
    if (copy && s_cmd_queue && xQueueSend(s_cmd_queue, &copy, 0) != pdTRUE) {
        free(copy);
    }
}

/* Deferred command runner: executes a queued command, capturing its output. */
static void ble_debug_exec_command(const char *cmd)
{
    s_capture_len = 0;
    s_capturing = true;
    uwatch_debug_process_cmd(cmd);
    s_capturing = false;
    s_resp_pending = true;
    ESP_LOGI(TAG, "cmd '%s' captured %u bytes", cmd, (unsigned)s_capture_len);
}

static void ble_debug_process_command_queue(void)
{
    char *cmd;
    while (s_cmd_queue && xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
        ble_debug_exec_command(cmd);
        free(cmd);
    }
}

/* Debug: report BLE state (called from the console). */
void ble_debug_print_status(void)
{
    printf("ble: advertising=%d conn_handle=%d\n",
           (int)s_adv_active, (int)s_conn_handle);
}

/* Enable/disable BLE advertising at runtime. */
void ble_debug_set_advertising(bool on)
{
    if (on) {
        ble_debug_advertise();
    } else {
        if (s_adv_active) {
            ble_gap_adv_stop();
            s_adv_active = false;
            ESP_LOGI(TAG, "advertising stopped");
        }
    }
}

/* ---- Init ---- */

static void ble_debug_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_debug_init(void)
{
    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %d", rc);
        return ESP_FAIL;
    }

    ble_hs_cfg.reset_cb = ble_debug_on_reset;
    ble_hs_cfg.sync_cb = ble_debug_on_sync;
    ble_hs_cfg.gatts_register_cb = ble_debug_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = ble_gatts_count_cfg(ble_debug_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg: %d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(ble_debug_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_svc_gap_device_name_set("UWatch");
    if (rc != 0) {
        ESP_LOGE(TAG, "device name: %d", rc);
    }

    ble_store_config_init();

    /* Mirror console output to the BLE RESP characteristic (chains onto the
     * sd_log vprintf hook, which keeps console + SD logging intact). */
    s_cmd_queue = xQueueCreate(CMD_QUEUE_LEN, sizeof(char *));
    s_prev_vprintf = esp_log_set_vprintf(ble_debug_vprintf);

    nimble_port_freertos_init(ble_debug_host_task);

    xTaskCreate(telemetry_task, "ble_telem", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "BLE debug bridge started (service 0xDEAD)");
    return ESP_OK;
}
