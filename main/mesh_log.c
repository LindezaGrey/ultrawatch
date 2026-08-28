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
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "mesh_log";

#define RX_BUF_LEN 256
#define RX_POLL_MS 1000

static mesh_msg_t s_msgs[MESH_LOG_COUNT];
static size_t s_count;     /* valid entries, up to MESH_LOG_COUNT */
static size_t s_next;      /* next write index (ring) */
static SemaphoreHandle_t s_mux;

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
                } else {
                    snprintf(m.text, sizeof(m.text), "NodeInfo");
                }
            } else {
                const char *name = meshtastic_portnum_name(pkt.data.portnum);
                if (name) {
                    snprintf(m.text, sizeof(m.text), "%s", name);
                } else {
                    snprintf(m.text, sizeof(m.text), "portnum=%lu", (unsigned long)pkt.data.portnum);
                }
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
            drv2605_play(twatch_haptic_dev, 47);   /* strong click, same as alarm.c */
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

void mesh_log_init(void)
{
    if (!s_mux) {
        s_mux = xSemaphoreCreateMutex();
    }
    xTaskCreate(mesh_log_task, "mesh_log", 4096, NULL, 3, NULL);
}
