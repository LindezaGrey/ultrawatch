/*
 * wifi_scan.c - see wifi_scan.h.
 */
#include "wifi_scan.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "wifi_scan";

#define WIFI_NVS_NS "wifi"
#define WIFI_RESCAN_INTERVAL_MS 8000

static bool s_wifi_enabled;   /* target state, persisted, default off */
static bool s_wifi_active;    /* actual state - only wifi_scan_task writes this */
static volatile bool s_scanning;
static bool s_netif_inited;   /* netif/event-loop scaffolding, set up once, never torn down */

static SemaphoreHandle_t s_mux;
static wifi_scan_result_t s_results[WIFI_SCAN_MAX_RESULTS];
static size_t s_result_count;

/* esp_netif_init()/esp_event_loop_create_default()/esp_netif_create_default_wifi_sta()
 * create process-wide singletons - safe to call once, not meant to be torn
 * down and recreated on every on/off toggle the way esp_wifi_init()/deinit()
 * are. None of this touches the radio or costs meaningful DMA/heap. */
static void wifi_ensure_netif(void)
{
    if (s_netif_inited) {
        return;
    }
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_netif_init: %s", esp_err_to_name(err));
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_event_loop_create_default: %s", esp_err_to_name(err));
    }
    esp_netif_create_default_wifi_sta();
    s_netif_inited = true;
}

/* The actual radio power transition - esp_wifi_init()/deinit() is what
 * reserves/releases the driver's internal DMA-capable RAM (see wifi_scan.h's
 * header comment on why this only ever happens on an explicit user toggle,
 * never at boot). Only ever called from wifi_scan_task's own context. */
static void wifi_power(bool on)
{
    if (on) {
        wifi_ensure_netif();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_wifi_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
            return;
        }
        esp_wifi_set_mode(WIFI_MODE_STA);
        err = esp_wifi_start();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_start: %s", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "WiFi radio on");
    } else {
        esp_wifi_stop();
        esp_wifi_deinit();
        if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_result_count = 0;
            xSemaphoreGive(s_mux);
        }
        ESP_LOGI(TAG, "WiFi radio off");
    }
}

/* Blocking scan (runs on this task's own stack, never the UI thread) +
 * copy results into the shared cache. esp_wifi_scan_get_ap_records() also
 * frees the driver's internal scan-result list, so this must run exactly
 * once per esp_wifi_scan_start() - never skipped, or that list leaks. */
static void wifi_do_scan(void)
{
    s_scanning = true;
    esp_err_t err = esp_wifi_scan_start(NULL, true);
    s_scanning = false;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan_start: %s", esp_err_to_name(err));
        return;
    }

    uint16_t num = WIFI_SCAN_MAX_RESULTS;
    wifi_ap_record_t recs[WIFI_SCAN_MAX_RESULTS];
    if (esp_wifi_scan_get_ap_records(&num, recs) != ESP_OK) {
        return;
    }
    if (num > WIFI_SCAN_MAX_RESULTS) {
        num = WIFI_SCAN_MAX_RESULTS;
    }
    if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_result_count = num;
        for (size_t i = 0; i < num; i++) {
            /* recs[i].ssid is uint8_t[33], not guaranteed NUL-terminated
             * for a full-length SSID - snprintf via %s still reads past
             * the end in that edge case, so bound the copy explicitly. */
            size_t len = strnlen((const char *)recs[i].ssid, WIFI_SCAN_SSID_MAX);
            memcpy(s_results[i].ssid, recs[i].ssid, len);
            s_results[i].ssid[len] = '\0';
            s_results[i].rssi_dbm = recs[i].rssi;
            s_results[i].open = (recs[i].authmode == WIFI_AUTH_OPEN);
        }
        xSemaphoreGive(s_mux);
    }
}

static void wifi_scan_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_wifi_enabled != s_wifi_active) {
            wifi_power(s_wifi_enabled);
            s_wifi_active = s_wifi_enabled;
        }
        if (s_wifi_active) {
            wifi_do_scan();
            vTaskDelay(pdMS_TO_TICKS(WIFI_RESCAN_INTERVAL_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

bool wifi_scan_get_enabled(void)
{
    return s_wifi_enabled;
}

void wifi_scan_set_enabled(bool on)
{
    if (on == s_wifi_enabled) {
        return;
    }
    s_wifi_enabled = on;
    nvs_handle_t h;
    if (nvs_open(WIFI_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "en", on ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool wifi_scan_is_scanning(void)
{
    return s_scanning;
}

size_t wifi_scan_get_results(wifi_scan_result_t *out, size_t max)
{
    size_t n = 0;
    if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        n = (s_result_count < max) ? s_result_count : max;
        memcpy(out, s_results, n * sizeof(wifi_scan_result_t));
        xSemaphoreGive(s_mux);
    }
    return n;
}

void wifi_scan_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
    nvs_handle_t h;
    if (nvs_open(WIFI_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 0;   /* default off */
        if (nvs_get_u8(h, "en", &v) == ESP_OK) {
            s_wifi_enabled = (v != 0);
        }
        nvs_close(h);
    }
    xTaskCreate(wifi_scan_task, "wifi_scan", 4096, NULL, 3, NULL);
}
