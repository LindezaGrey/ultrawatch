/*
 * debug_lora.c - LoRa/Meshtastic debug console commands.
 *
 * Moved out of uwatch_main.c's debug_process_cmd() dispatch (mechanical
 * refactor, same convention as debug_gnss.c).
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "meshtastic_radio.h"
#include "sx1262.h"
#include "debug_lora.h"

#define MESHDUMP_DEFAULT_S 15
#define MESHDUMP_BUF_LEN   256
#define MESHDUMP_POLL_MS   200
#define MESHDUMP_RSSI_INTERVAL_MS 2000

void debug_lora_meshdump(const char *args)
{
    int seconds = (args[0] != '\0') ? atoi(args) : 0;
    if (seconds <= 0) {
        seconds = MESHDUMP_DEFAULT_S;
    }

    printf("meshdump: listening on 869.525 MHz (SF11/BW250/CR4:5) for %ds...\n", seconds);

    uint16_t dev_errors = 0;
    if (sx1262_get_device_errors(&dev_errors) == ESP_OK && dev_errors != 0) {
        printf("meshdump: WARNING device errors set: 0x%04x\n", dev_errors);
    }

    esp_err_t err = meshtastic_radio_start_rx();
    if (err != ESP_OK) {
        printf("meshdump: start_rx failed: %s\n", esp_err_to_name(err));
        return;
    }

    uint8_t buf[MESHDUMP_BUF_LEN];
    int pkt_count = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t last_rssi_log = start;
    while (pdTICKS_TO_MS(xTaskGetTickCount() - start) < (uint32_t)(seconds * 1000)) {
        if (pdTICKS_TO_MS(xTaskGetTickCount() - last_rssi_log) >= MESHDUMP_RSSI_INTERVAL_MS) {
            last_rssi_log = xTaskGetTickCount();
            int16_t floor_rssi = 0;
            /* Instantaneous RSSI - confirms the RX front-end is alive and
             * picking up real RF noise, independent of any valid packet.
             * A dead/unconnected antenna or a stuck receiver typically
             * reads a suspiciously flat or extreme value; a live antenna
             * in a normal RF environment reads roughly -90 to -120 dBm. */
            if (sx1262_get_rssi_inst(&floor_rssi) == ESP_OK) {
                printf("meshdump: rssi floor %ddBm\n", (int)floor_rssi);
            }
        }

        size_t len = 0;
        int16_t rssi = 0;
        int8_t snr = 0;
        bool crc_ok = false;
        err = meshtastic_radio_recv(buf, sizeof(buf), &len, &rssi, &snr, &crc_ok,
                                    MESHDUMP_POLL_MS);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            printf("meshdump: recv error: %s\n", esp_err_to_name(err));
            continue;
        }

        pkt_count++;
        printf("meshdump: pkt #%d len=%u rssi=%ddBm snr=%ddB crc=%s\n",
               pkt_count, (unsigned)len, (int)rssi, (int)snr, crc_ok ? "ok" : "FAIL");
        for (size_t i = 0; i < len; i += 16) {
            printf("  ");
            for (size_t j = i; j < i + 16 && j < len; j++) {
                printf("%02x ", buf[j]);
            }
            printf("\n");
        }
    }

    meshtastic_radio_stop_rx();
    printf("meshdump: done, %d packet(s) captured\n", pkt_count);
}
