#include "mesh_service.h"

#include <stdio.h>
#include <string.h>

#include "ble_rtc.h"
#include "board.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "meshtastic_crypto.h"
#include "meshtastic_radio.h"
#include "meshtastic_user.h"
#include "nvs.h"
#include "screen_control.h"
#include "shared_spi2.h"
#include "sx1262.h"

#define MESH_NVS_NAMESPACE "meshtastic"
#define MESH_NVS_CONFIG_KEY "config_v1"
#define MESH_RX_WAIT_MS 500

static const char *TAG = "meshtastic";
static SemaphoreHandle_t mesh_lock;
static mesh_config_t mesh_config;
static mesh_message_t messages[MESH_MESSAGE_COUNT];
static size_t message_count;
static size_t message_next;
static volatile bool radio_active;
static volatile uint32_t config_generation;

static void set_defaults(mesh_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->enabled = false;
    config->notify = true;
    config->frequency_hz = 869525000UL;
    config->spreading_factor = 8;
    config->bandwidth = 0x05;
    config->coding_rate = 0x01;
    config->preamble_length = 16;
    config->sync_word = 0x2b;
    config->boosted_rx_gain = true;
    snprintf(config->channels[0].name, sizeof(config->channels[0].name),
             "ShortSlow");
    memcpy(config->channels[0].key, meshtastic_default_psk, 16);
    config->channels[0].key_len = 16;
    snprintf(config->channels[1].name, sizeof(config->channels[1].name),
             "Mesh Hessen");
    memcpy(config->channels[1].key, meshtastic_hessen_psk, 32);
    config->channels[1].key_len = 32;
}

static bool config_valid(const mesh_config_t *config)
{
    if (config == NULL || config->frequency_hz < 400000000UL ||
        config->frequency_hz > 960000000UL ||
        config->spreading_factor < 5 || config->spreading_factor > 12 ||
        config->bandwidth < 0x04 || config->bandwidth > 0x06 ||
        config->coding_rate < 0x01 || config->coding_rate > 0x04 ||
        config->preamble_length == 0) {
        return false;
    }
    for (size_t index = 0; index < MESHTASTIC_CHANNEL_COUNT; index++) {
        const meshtastic_channel_t *channel = &config->channels[index];
        if (channel->name[0] == '\0' ||
            (channel->key_len != 16 && channel->key_len != 32) ||
            strnlen(channel->name, sizeof(channel->name)) >
                MESHTASTIC_CHANNEL_NAME_MAX) {
            return false;
        }
    }
    return true;
}

void mesh_config_encode(const mesh_config_t *config,
                        uint8_t output[MESH_CONFIG_WIRE_LENGTH])
{
    memset(output, 0, MESH_CONFIG_WIRE_LENGTH);
    output[0] = MESH_CONFIG_WIRE_VERSION;
    output[1] = config->enabled;
    output[2] = config->notify;
    output[3] = (uint8_t)config->frequency_hz;
    output[4] = (uint8_t)(config->frequency_hz >> 8);
    output[5] = (uint8_t)(config->frequency_hz >> 16);
    output[6] = (uint8_t)(config->frequency_hz >> 24);
    output[7] = config->spreading_factor;
    output[8] = config->bandwidth;
    output[9] = config->coding_rate;
    output[10] = config->low_data_rate_optimize;
    output[11] = (uint8_t)config->preamble_length;
    output[12] = (uint8_t)(config->preamble_length >> 8);
    output[13] = config->sync_word;
    output[14] = config->boosted_rx_gain;
    size_t offset = 15;
    for (size_t index = 0; index < MESHTASTIC_CHANNEL_COUNT; index++) {
        const meshtastic_channel_t *channel = &config->channels[index];
        output[offset++] = (uint8_t)strnlen(channel->name,
                                             MESHTASTIC_CHANNEL_NAME_MAX);
        memcpy(output + offset, channel->name, MESHTASTIC_CHANNEL_NAME_MAX);
        offset += MESHTASTIC_CHANNEL_NAME_MAX;
        output[offset++] = channel->key_len;
        memcpy(output + offset, channel->key, MESHTASTIC_CHANNEL_KEY_MAX);
        offset += MESHTASTIC_CHANNEL_KEY_MAX;
    }
}

bool mesh_config_decode(const uint8_t input[MESH_CONFIG_WIRE_LENGTH],
                        mesh_config_t *config)
{
    if (input == NULL || config == NULL ||
        input[0] != MESH_CONFIG_WIRE_VERSION) return false;
    memset(config, 0, sizeof(*config));
    config->enabled = input[1] != 0;
    config->notify = input[2] != 0;
    config->frequency_hz = (uint32_t)input[3] |
        ((uint32_t)input[4] << 8) | ((uint32_t)input[5] << 16) |
        ((uint32_t)input[6] << 24);
    config->spreading_factor = input[7];
    config->bandwidth = input[8];
    config->coding_rate = input[9];
    config->low_data_rate_optimize = input[10] != 0;
    config->preamble_length = (uint16_t)input[11] |
                              ((uint16_t)input[12] << 8);
    config->sync_word = input[13];
    config->boosted_rx_gain = input[14] != 0;
    size_t offset = 15;
    for (size_t index = 0; index < MESHTASTIC_CHANNEL_COUNT; index++) {
        uint8_t name_length = input[offset++];
        if (name_length == 0 || name_length > MESHTASTIC_CHANNEL_NAME_MAX) {
            return false;
        }
        memcpy(config->channels[index].name, input + offset, name_length);
        offset += MESHTASTIC_CHANNEL_NAME_MAX;
        config->channels[index].key_len = input[offset++];
        memcpy(config->channels[index].key, input + offset,
               MESHTASTIC_CHANNEL_KEY_MAX);
        offset += MESHTASTIC_CHANNEL_KEY_MAX;
    }
    return config_valid(config);
}

static esp_err_t route_internal_antenna(void)
{
    uint8_t value;
    uint8_t reg = BOARD_XL9555_OUTPUT1;
    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(
                            BOARD_I2C_PORT, BOARD_XL9555_ADDR, &reg, 1,
                            &value, 1, portMAX_DELAY),
                        TAG, "LoRa antenna output read failed");
    value |= 1U << BOARD_XL9555_LORA_SELECT_BIT;
    const uint8_t output[] = {BOARD_XL9555_OUTPUT1, value};
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(
                            BOARD_I2C_PORT, BOARD_XL9555_ADDR, output,
                            sizeof(output), portMAX_DELAY),
                        TAG, "LoRa antenna selection failed");
    reg = BOARD_XL9555_CONFIG1;
    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(
                            BOARD_I2C_PORT, BOARD_XL9555_ADDR, &reg, 1,
                            &value, 1, portMAX_DELAY),
                        TAG, "LoRa antenna direction read failed");
    value &= ~(1U << BOARD_XL9555_LORA_SELECT_BIT);
    const uint8_t direction[] = {BOARD_XL9555_CONFIG1, value};
    return i2c_master_write_to_device(BOARD_I2C_PORT, BOARD_XL9555_ADDR,
                                      direction, sizeof(direction),
                                      portMAX_DELAY);
}

static esp_err_t radio_start(spi_device_handle_t *device,
                             const mesh_config_t *config)
{
    ESP_RETURN_ON_ERROR(shared_spi2_acquire(), TAG, "SPI2 acquire failed");
    esp_err_t result = route_internal_antenna();
    if (result != ESP_OK) goto fail;
    const spi_device_interface_config_t device_config = {
        .mode = 0,
        .clock_speed_hz = 10 * 1000 * 1000,
        .queue_size = 7,
        .spics_io_num = BOARD_LORA_CS,
    };
    result = spi_bus_add_device(BOARD_SD_SPI_HOST, &device_config, device);
    if (result != ESP_OK) goto fail;
    result = sx1262_init(*device);
    if (result != ESP_OK) goto remove;
    if (!meshtastic_packet_set_channels(config->channels,
                                         MESHTASTIC_CHANNEL_COUNT)) {
        result = ESP_ERR_INVALID_ARG;
        goto remove;
    }
    const sx1262_lora_params_t parameters = {
        .freq_hz = config->frequency_hz,
        .sf = config->spreading_factor,
        .bw = config->bandwidth,
        .cr = config->coding_rate,
        .low_data_rate_optimize = config->low_data_rate_optimize,
        .preamble_len = config->preamble_length,
        .explicit_header = true,
        .payload_len_max = 0xff,
        .crc_on = true,
        .invert_iq = false,
        .sync_word = config->sync_word,
        .boosted_rx_gain = config->boosted_rx_gain,
    };
    result = meshtastic_radio_configure(&parameters);
    if (result == ESP_OK) result = meshtastic_radio_start_rx();
    if (result == ESP_OK) return ESP_OK;
remove:
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_remove(
        BOARD_LORA_INTERRUPT));
    spi_bus_remove_device(*device);
    *device = NULL;
fail:
    shared_spi2_release();
    return result;
}

static void radio_stop(spi_device_handle_t *device)
{
    if (*device == NULL) return;
    ESP_ERROR_CHECK_WITHOUT_ABORT(meshtastic_radio_stop_rx());
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_remove(
        BOARD_LORA_INTERRUPT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_remove_device(*device));
    *device = NULL;
    shared_spi2_release();
}

static void store_message(const meshtastic_packet_t *packet, int16_t rssi,
                          int8_t snr)
{
    mesh_message_t message = {
        .from = packet->from,
        .channel_hash = packet->channel_hash,
        .rssi_dbm = rssi,
        .snr_db = snr,
        .received_at_us = esp_timer_get_time(),
    };
    if (packet->channel_name != NULL) {
        snprintf(message.channel, sizeof(message.channel), "%s",
                 packet->channel_name);
    }
    snprintf(message.sender, sizeof(message.sender), "!%08lx",
             (unsigned long)packet->from);
    if (packet->data.payload != NULL && packet->data.portnum == 1) {
        message.kind = MESH_MESSAGE_TEXT;
        size_t length = packet->data.payload_len;
        if (length > MESH_MESSAGE_TEXT_MAX) length = MESH_MESSAGE_TEXT_MAX;
        memcpy(message.text, packet->data.payload, length);
        message.text[length] = '\0';
    } else if (packet->data.payload != NULL) {
        message.kind = MESH_MESSAGE_OTHER;
        if (packet->data.portnum == 4) {
            meshtastic_user_t user;
            if (meshtastic_user_decode(packet->data.payload,
                                       packet->data.payload_len, &user) &&
                user.long_name != NULL) {
                snprintf(message.sender, sizeof(message.sender), "%.*s",
                         (int)user.long_name_len, user.long_name);
            }
        }
        const char *name = meshtastic_portnum_name(packet->data.portnum);
        snprintf(message.text, sizeof(message.text), "%s",
                 name != NULL ? name : "Other packet");
    } else {
        message.kind = MESH_MESSAGE_UNKNOWN;
        snprintf(message.text, sizeof(message.text), "Unknown channel");
    }

    xSemaphoreTake(mesh_lock, portMAX_DELAY);
    messages[message_next] = message;
    message_next = (message_next + 1) % MESH_MESSAGE_COUNT;
    if (message_count < MESH_MESSAGE_COUNT) message_count++;
    bool notify = mesh_config.notify && message.kind == MESH_MESSAGE_TEXT;
    xSemaphoreGive(mesh_lock);
    if (notify) {
        screen_show_messages();
        ESP_ERROR_CHECK_WITHOUT_ABORT(ble_haptic_click());
    } else {
        screen_request_refresh();
    }
}

static void mesh_task(void *parameter)
{
    (void)parameter;
    spi_device_handle_t device = NULL;
    uint32_t applied_generation = UINT32_MAX;
    for (;;) {
        mesh_config_t config;
        uint32_t generation;
        xSemaphoreTake(mesh_lock, portMAX_DELAY);
        config = mesh_config;
        generation = config_generation;
        xSemaphoreGive(mesh_lock);

        if (!config.enabled || generation != applied_generation) {
            radio_active = false;
            radio_stop(&device);
            applied_generation = generation;
        }
        if (config.enabled && device == NULL) {
            esp_err_t result = radio_start(&device, &config);
            radio_active = result == ESP_OK;
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "radio start failed: %s", esp_err_to_name(result));
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            ESP_LOGI(TAG, "receiver active at %lu Hz",
                     (unsigned long)config.frequency_hz);
        }
        if (device == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint8_t buffer[256];
        size_t length = 0;
        int16_t rssi = 0;
        int8_t snr = 0;
        bool crc_ok = false;
        esp_err_t result = meshtastic_radio_recv(
            buffer, sizeof(buffer), &length, &rssi, &snr, &crc_ok,
            MESH_RX_WAIT_MS);
        if (result == ESP_OK && crc_ok) {
            meshtastic_packet_t packet;
            if (meshtastic_packet_decode(buffer, length, &packet)) {
                store_message(&packet, rssi, snr);
            }
        } else if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "receive failed: %s", esp_err_to_name(result));
        }
    }
}

void mesh_service_get_config(mesh_config_t *config)
{
    if (config == NULL) return;
    xSemaphoreTake(mesh_lock, portMAX_DELAY);
    *config = mesh_config;
    xSemaphoreGive(mesh_lock);
}

esp_err_t mesh_service_set_config(const mesh_config_t *config)
{
    if (!config_valid(config)) return ESP_ERR_INVALID_ARG;
    uint8_t wire[MESH_CONFIG_WIRE_LENGTH];
    mesh_config_encode(config, wire);
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(MESH_NVS_NAMESPACE, NVS_READWRITE, &handle),
                        TAG, "configuration store open failed");
    esp_err_t result = nvs_set_blob(handle, MESH_NVS_CONFIG_KEY, wire,
                                    sizeof(wire));
    if (result == ESP_OK) result = nvs_commit(handle);
    nvs_close(handle);
    if (result != ESP_OK) return result;
    xSemaphoreTake(mesh_lock, portMAX_DELAY);
    mesh_config = *config;
    config_generation++;
    xSemaphoreGive(mesh_lock);
    screen_request_refresh();
    return ESP_OK;
}

bool mesh_service_radio_active(void)
{
    return radio_active;
}

size_t mesh_service_get_recent(mesh_message_t *output, size_t maximum)
{
    if (output == NULL) return 0;
    xSemaphoreTake(mesh_lock, portMAX_DELAY);
    size_t count = message_count < maximum ? message_count : maximum;
    for (size_t index = 0; index < count; index++) {
        size_t source = (message_next + MESH_MESSAGE_COUNT - 1 - index) %
                        MESH_MESSAGE_COUNT;
        output[index] = messages[source];
    }
    xSemaphoreGive(mesh_lock);
    return count;
}

esp_err_t mesh_service_init(void)
{
    mesh_lock = xSemaphoreCreateMutex();
    if (mesh_lock == NULL) return ESP_ERR_NO_MEM;
    set_defaults(&mesh_config);
    nvs_handle_t handle;
    if (nvs_open(MESH_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        uint8_t wire[MESH_CONFIG_WIRE_LENGTH];
        size_t length = sizeof(wire);
        if (nvs_get_blob(handle, MESH_NVS_CONFIG_KEY, wire, &length) == ESP_OK &&
            length == sizeof(wire)) {
            mesh_config_t loaded;
            if (mesh_config_decode(wire, &loaded)) mesh_config = loaded;
        }
        nvs_close(handle);
    }
    return xTaskCreate(mesh_task, "meshtastic", 6144, NULL, 3, NULL) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}
