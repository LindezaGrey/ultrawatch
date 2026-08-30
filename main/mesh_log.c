/*
 * mesh_log.c - see mesh_log.h.
 */
#include "mesh_log.h"
#include "meshtastic_radio.h"
#include "meshtastic_packet.h"
#include "meshtastic_data.h"
#include "meshtastic_user.h"
#include "lvgl_app.h"
#include "twatch_board.h"
#include "drv2605.h"
#include "sd_log.h"
#include "sensor_cache.h"
#include "pcf85063a.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "mesh_log";

#define RX_BUF_LEN 256
#define RX_POLL_MS 1000

static mesh_msg_t s_msgs[MESH_LOG_COUNT];
static size_t s_count;     /* valid entries, up to MESH_LOG_COUNT */
static size_t s_next;      /* next write index (ring) */
static SemaphoreHandle_t s_mux;

#define MESH_TEXT_LOG_PATH "/sdcard/log/mesh.txt"

/* Node table: in-RAM only, upserted from every decoded packet. Guarded by
 * the same s_mux as the ring buffer - both are only ever touched serially
 * from mesh_log_task() (writer) or a UI timer (reader), never nested. */
static mesh_node_t s_nodes[MESH_NODE_TABLE_MAX];
static size_t s_node_count;

#define MESH_NVS_NS "mesh"

static bool s_notify_enabled = true;
static char s_presets[MESH_PRESET_COUNT][MESH_PRESET_MAX_LEN + 1] = {
    "Bin ok", "Verzoegerung", "Notfall", "Standort senden",
};

/* Inserts/refreshes the entry for `from`, evicting the least-recently-seen
 * node if the table is full. `name` is NULL to leave an existing name
 * untouched (most packets), or a NodeInfo long_name to (over)write it. */
static void mesh_node_upsert(uint32_t from, const char *name, int16_t rssi, int8_t snr, int64_t now_us)
{
    if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    size_t idx = s_node_count;
    for (size_t i = 0; i < s_node_count; i++) {
        if (s_nodes[i].node_id == from) {
            idx = i;
            break;
        }
    }
    if (idx == s_node_count) {
        if (s_node_count < MESH_NODE_TABLE_MAX) {
            s_node_count++;
        } else {
            /* Table full: evict the least-recently-seen entry. */
            idx = 0;
            for (size_t i = 1; i < MESH_NODE_TABLE_MAX; i++) {
                if (s_nodes[i].last_seen_us < s_nodes[idx].last_seen_us) {
                    idx = i;
                }
            }
            memset(&s_nodes[idx], 0, sizeof(s_nodes[idx]));
        }
        s_nodes[idx].node_id = from;
    }
    s_nodes[idx].last_seen_us = now_us;
    s_nodes[idx].last_rssi_dbm = rssi;
    s_nodes[idx].last_snr_db = snr;
    if (name) {
        snprintf(s_nodes[idx].name, sizeof(s_nodes[idx].name), "%s", name);
    }
    xSemaphoreGive(s_mux);
}

/* Appends one "Zeitstempel | Absender | Nachricht" line for a real text
 * message - see docs/application.md section 7.2 point 6. Same
 * fopen/fprintf/fclose-per-write pattern as daily_log.c/gpx_log.c, bracketed
 * by sd_log_session_begin()/end() (on-demand mount, see sd_log.h) rather
 * than assuming the card is already mounted; the parent dir is already
 * created by sd_log_mount(). */
static void mesh_text_log_append(const char *sender, const char *text)
{
    pcf85063a_time_t rtc;
    if (!sensor_cache_get_rtc(&rtc)) {
        return;
    }
    time_t epoch = pcf85063a_time_to_epoch(&rtc);
    struct tm lt;
    localtime_r(&epoch, &lt);
    char ts[24];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &lt);

    if (sd_log_session_begin() != ESP_OK) {
        sd_log_session_end();
        return;
    }
    FILE *f = fopen(MESH_TEXT_LOG_PATH, "a");
    if (!f) {
        ESP_LOGW(TAG, "mesh_text_log_append: fopen failed");
        sd_log_session_end();
        return;
    }
    fprintf(f, "%s | %s | %s\n", ts, sender, text);
    fclose(f);
    sd_log_session_end();
}

static void mesh_log_task(void *arg)
{
    (void)arg;
    esp_err_t err = meshtastic_radio_start_rx();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "start_rx failed: %s", esp_err_to_name(err));
    }

    uint8_t buf[RX_BUF_LEN];
    for (;;) {
        size_t len = 0;
        int16_t rssi = 0;
        int8_t snr = 0;
        bool crc_ok = false;
        err = meshtastic_radio_recv(buf, sizeof(buf), &len, &rssi, &snr, &crc_ok, RX_POLL_MS);
        if (err != ESP_OK || !crc_ok) {
            continue;   /* timeout is the normal case; a bad-CRC packet is skipped, not logged */
        }

        meshtastic_packet_t pkt;
        if (!meshtastic_packet_decode(buf, len, &pkt)) {
            continue;   /* shorter than a PacketHeader - nothing to show */
        }

        /* Every packet with a valid header goes in the log, known channel or
         * not: an unrecognized channel_hash still tells the user something
         * is out there, which is the whole point of showing headers always
         * rather than only fully-decoded text. */
        mesh_msg_t m = { 0 };
        m.from = pkt.from;
        m.channel_hash = pkt.channel_hash;
        m.rssi_dbm = rssi;
        m.snr_db = snr;
        m.received_at_us = esp_timer_get_time();

        if (!pkt.channel_hash_matches || !pkt.data.payload) {
            m.kind = MESH_MSG_UNKNOWN;   /* no key for this channel, or not a valid Data message */
            /* `from` is in the cleartext header even when the payload can't
             * be decrypted - still a real node out there, same reasoning
             * already applied to showing it in the message ring buffer. */
            mesh_node_upsert(m.from, NULL, m.rssi_dbm, m.snr_db, m.received_at_us);
        } else if (pkt.data.portnum == 1 /* TEXT_MESSAGE_APP */) {
            m.kind = MESH_MSG_TEXT;
            size_t text_len = pkt.data.payload_len;
            if (text_len > MESH_LOG_TEXT_MAX) {
                text_len = MESH_LOG_TEXT_MAX;
            }
            memcpy(m.text, pkt.data.payload, text_len);
            m.text[text_len] = '\0';
        } else {
            m.kind = MESH_MSG_OTHER;
            if (pkt.data.portnum == 4 /* NODEINFO_APP */) {
                meshtastic_user_t user;
                if (meshtastic_user_decode(pkt.data.payload, pkt.data.payload_len, &user) &&
                        user.long_name && user.long_name_len > 0) {
                    snprintf(m.text, sizeof(m.text), "NodeInfo: %.*s",
                             (int)user.long_name_len, user.long_name);
                    char name_buf[32];
                    snprintf(name_buf, sizeof(name_buf), "%.*s",
                             (int)user.long_name_len, user.long_name);
                    mesh_node_upsert(m.from, name_buf, m.rssi_dbm, m.snr_db, m.received_at_us);
                } else {
                    snprintf(m.text, sizeof(m.text), "NodeInfo");
                    mesh_node_upsert(m.from, NULL, m.rssi_dbm, m.snr_db, m.received_at_us);
                }
            } else {
                const char *name = meshtastic_portnum_name(pkt.data.portnum);
                if (name) {
                    snprintf(m.text, sizeof(m.text), "%s", name);
                } else {
                    snprintf(m.text, sizeof(m.text), "portnum=%lu", (unsigned long)pkt.data.portnum);
                }
                mesh_node_upsert(m.from, NULL, m.rssi_dbm, m.snr_db, m.received_at_us);
            }
        }

        if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_msgs[s_next] = m;
            s_next = (s_next + 1) % MESH_LOG_COUNT;
            if (s_count < MESH_LOG_COUNT) {
                s_count++;
            }
            xSemaphoreGive(s_mux);
        }

        if (m.kind == MESH_MSG_TEXT) {
            mesh_node_upsert(m.from, NULL, m.rssi_dbm, m.snr_db, m.received_at_us);

            char sender_buf[40];
            mesh_log_node_name(m.from, sender_buf, sizeof(sender_buf));
            if (sender_buf[0] == '\0') {
                snprintf(sender_buf, sizeof(sender_buf), "!%08lx", (unsigned long)m.from);
            }
            const char *sender = sender_buf;
            mesh_text_log_append(sender, m.text);

            ESP_LOGI(TAG, "msg from !%08lx ch=0x%02x rssi=%ddBm snr=%ddB: %s",
                    (unsigned long)m.from, (unsigned)m.channel_hash,
                    (int)m.rssi_dbm, (int)m.snr_db, m.text);

            /* Surface a real text message immediately: jump to the Mesh
             * screen (waking the display if asleep) and give a short
             * haptic tap. The radio listens continuously regardless of
             * which screen is open, so without this a message could
             * arrive and sit unseen indefinitely. Non-text entries
             * (NodeInfo, telemetry, unknown channel, ...) are logged for
             * visibility but don't interrupt - only a real message does. */
            lvgl_mesh_screen_show();
            if (s_notify_enabled) {
                drv2605_play(twatch_haptic_dev, 47);   /* strong click, same as alarm.c */
            }
        } else if (m.kind == MESH_MSG_OTHER) {
            ESP_LOGI(TAG, "packet from !%08lx ch=0x%02x rssi=%ddBm snr=%ddB: %s",
                    (unsigned long)m.from, (unsigned)m.channel_hash,
                    (int)m.rssi_dbm, (int)m.snr_db, m.text);
        } else {
            ESP_LOGI(TAG, "packet from !%08lx ch=0x%02x rssi=%ddBm snr=%ddB (unknown channel)",
                    (unsigned long)m.from, (unsigned)m.channel_hash,
                    (int)m.rssi_dbm, (int)m.snr_db);
        }
    }
}

size_t mesh_log_get_recent(mesh_msg_t *out, size_t max)
{
    size_t n = 0;
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        size_t avail = s_count;
        if (max < avail) {
            avail = max;
        }
        for (size_t i = 0; i < avail; i++) {
            size_t idx = (s_next + MESH_LOG_COUNT - 1 - i) % MESH_LOG_COUNT;
            out[i] = s_msgs[idx];
        }
        n = avail;
        xSemaphoreGive(s_mux);
    }
    return n;
}

size_t mesh_log_get_nodes(mesh_node_t *out, size_t max)
{
    size_t n = 0;
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        n = (max < s_node_count) ? max : s_node_count;
        memcpy(out, s_nodes, n * sizeof(mesh_node_t));
        xSemaphoreGive(s_mux);
    }
    /* Insertion sort by last_seen_us descending - N is capped at
     * MESH_NODE_TABLE_MAX (32), trivial cost either way. */
    for (size_t i = 1; i < n; i++) {
        mesh_node_t key = out[i];
        size_t j = i;
        while (j > 0 && out[j - 1].last_seen_us < key.last_seen_us) {
            out[j] = out[j - 1];
            j--;
        }
        out[j] = key;
    }
    return n;
}

size_t mesh_log_node_count(void)
{
    size_t n = 0;
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        n = s_node_count;
        xSemaphoreGive(s_mux);
    }
    return n;
}

void mesh_log_node_name(uint32_t node_id, char *out, size_t outlen)
{
    if (outlen > 0) {
        out[0] = '\0';
    }
    if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (size_t i = 0; i < s_node_count; i++) {
            if (s_nodes[i].node_id == node_id) {
                snprintf(out, outlen, "%s", s_nodes[i].name);
                break;
            }
        }
        xSemaphoreGive(s_mux);
    }
}

bool mesh_log_get_notify_enabled(void)
{
    return s_notify_enabled;
}

void mesh_log_set_notify_enabled(bool enabled)
{
    if (enabled == s_notify_enabled) {
        return;
    }
    s_notify_enabled = enabled;
    nvs_handle_t h;
    if (nvs_open(MESH_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "notify_en", enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

void mesh_preset_get(int idx, char *out, size_t outlen)
{
    if (outlen > 0) {
        out[0] = '\0';
    }
    if (idx < 0 || idx >= MESH_PRESET_COUNT) {
        return;
    }
    snprintf(out, outlen, "%s", s_presets[idx]);
}

void mesh_preset_set(int idx, const char *text)
{
    if (idx < 0 || idx >= MESH_PRESET_COUNT || !text) {
        return;
    }
    snprintf(s_presets[idx], sizeof(s_presets[idx]), "%s", text);
    nvs_handle_t h;
    if (nvs_open(MESH_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "presets", s_presets, sizeof(s_presets));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void mesh_config_load(void)
{
    nvs_handle_t h;
    if (nvs_open(MESH_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 1;
        if (nvs_get_u8(h, "notify_en", &v) == ESP_OK) {
            s_notify_enabled = (v != 0);
        }
        size_t len = sizeof(s_presets);
        char tmp[sizeof(s_presets)];
        if (nvs_get_blob(h, "presets", tmp, &len) == ESP_OK && len == sizeof(s_presets)) {
            memcpy(s_presets, tmp, sizeof(s_presets));
        }
        nvs_close(h);
    }
}

int64_t mesh_log_now_us(void)
{
    return esp_timer_get_time();
}

void mesh_log_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
    mesh_config_load();
    /* 8192, not 4096: on a text message this task calls
     * lvgl_mesh_screen_show() directly, which - the first time the Mesh
     * screen hasn't been built yet this boot - synchronously constructs
     * the full screen (scrollable list, 8 message rows, 4 preset
     * buttons, event callbacks) on this same stack. Confirmed via a
     * live crash report: "stack overflow in task mesh_log" on an
     * incoming LoRa message, PC inside LVGL widget-creation code. No
     * other background task in this app builds a full LVGL screen
     * directly on its own stack this way. */
    xTaskCreate(mesh_log_task, "mesh_log", 8192, NULL, 3, NULL);
}
