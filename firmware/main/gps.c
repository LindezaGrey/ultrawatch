#include "gps.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#define GPS_INITIAL_BAUD 38400
#define GPS_RUN_BAUD 115200
#define GPS_UART_BUFFER_SIZE 1024
#define GPS_UBX_MAX_PAYLOAD 256
#define GPS_PROBE_PER_BAUD_MS 15000
#define GPS_ACK_TIMEOUT_MS 500
#define GPS_RF_POLL_MS 10000

#define UBX_CLASS_ACK 0x05
#define UBX_ID_ACK_NAK 0x00
#define UBX_ID_ACK_ACK 0x01
#define UBX_CLASS_CFG 0x06
#define UBX_ID_CFG_VALSET 0x8a
#define UBX_CLASS_NAV 0x01
#define UBX_ID_NAV_PVT 0x07
#define UBX_CLASS_MON 0x0a
#define UBX_ID_MON_VER 0x04
#define UBX_ID_MON_RF 0x38

#define CFG_UART1_BAUDRATE 0x40520001UL
#define CFG_SIGNAL_GPS_ENA 0x1031001fUL
#define CFG_SIGNAL_GAL_ENA 0x10310021UL
#define CFG_SIGNAL_BDS_ENA 0x10310022UL
#define CFG_SIGNAL_BDS_B1_ENA 0x1031000dUL
#define CFG_SIGNAL_BDS_B1C_ENA 0x1031000fUL
#define CFG_SIGNAL_QZSS_ENA 0x10310024UL
#define CFG_SIGNAL_SBAS_ENA 0x10310020UL
#define CFG_SIGNAL_GLO_ENA 0x10310025UL
#define CFG_ANA_USE_ANA 0x10230001UL
#define CFG_NAVSPG_FIXMODE 0x20110011UL
#define CFG_NAVSPG_INFIL_MINSVS 0x201100a1UL
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007UL

static const char *TAG = "gps";

typedef struct {
    uint8_t state;
    uint8_t message_class;
    uint8_t message_id;
    uint16_t length;
    uint16_t index;
    uint8_t checksum_a;
    uint8_t checksum_b;
    uint8_t received_checksum_a;
    uint8_t payload[GPS_UBX_MAX_PAYLOAD];
} ubx_parser_t;

static portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;
static gps_status_t current_status;
static bool driver_started;
static bool uart_installed;
static volatile bool cancel_requested;
static TaskHandle_t gps_task_handle;
static esp_pm_lock_handle_t gps_sleep_lock;

typedef struct {
    uint8_t version;
    uint8_t reserved[3];
    int32_t latitude_e7;
    int32_t longitude_e7;
} saved_position_t;

static bool position_valid(int32_t latitude_e7, int32_t longitude_e7)
{
    return latitude_e7 >= -900000000 && latitude_e7 <= 900000000 &&
           longitude_e7 >= -1800000000 && longitude_e7 <= 1800000000;
}

static bool load_saved_position(int32_t *latitude_e7, int32_t *longitude_e7)
{
    nvs_handle_t handle;
    if (nvs_open("gps", NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    saved_position_t saved = {0};
    size_t size = sizeof(saved);
    esp_err_t result = nvs_get_blob(handle, "last_pos", &saved, &size);
    nvs_close(handle);
    if (result != ESP_OK || size != sizeof(saved) || saved.version != 1 ||
        !position_valid(saved.latitude_e7, saved.longitude_e7)) {
        return false;
    }
    *latitude_e7 = saved.latitude_e7;
    *longitude_e7 = saved.longitude_e7;
    return true;
}

static void write_uint32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
    output[2] = (uint8_t)(value >> 16);
    output[3] = (uint8_t)(value >> 24);
}

static uint32_t read_uint32_le(const uint8_t *input)
{
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8) |
           ((uint32_t)input[2] << 16) | ((uint32_t)input[3] << 24);
}

static int32_t read_int32_le(const uint8_t *input)
{
    return (int32_t)read_uint32_le(input);
}

static void parser_checksum(ubx_parser_t *parser, uint8_t byte)
{
    parser->checksum_a += byte;
    parser->checksum_b += parser->checksum_a;
}

static void parser_reset(ubx_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

static bool parser_feed(ubx_parser_t *parser, uint8_t byte)
{
    switch (parser->state) {
    case 0:
        if (byte == 0xb5) {
            parser->state = 1;
        }
        break;
    case 1:
        if (byte == 0x62) {
            parser->state = 2;
            parser->checksum_a = 0;
            parser->checksum_b = 0;
        } else {
            parser->state = byte == 0xb5 ? 1 : 0;
        }
        break;
    case 2:
        parser->message_class = byte;
        parser_checksum(parser, byte);
        parser->state = 3;
        break;
    case 3:
        parser->message_id = byte;
        parser_checksum(parser, byte);
        parser->state = 4;
        break;
    case 4:
        parser->length = byte;
        parser_checksum(parser, byte);
        parser->state = 5;
        break;
    case 5:
        parser->length |= (uint16_t)byte << 8;
        parser_checksum(parser, byte);
        parser->index = 0;
        if (parser->length > sizeof(parser->payload)) {
            parser_reset(parser);
        } else {
            parser->state = parser->length == 0 ? 7 : 6;
        }
        break;
    case 6:
        parser->payload[parser->index++] = byte;
        parser_checksum(parser, byte);
        if (parser->index == parser->length) {
            parser->state = 7;
        }
        break;
    case 7:
        parser->received_checksum_a = byte;
        parser->state = 8;
        break;
    case 8: {
        bool valid = parser->received_checksum_a == parser->checksum_a &&
                     byte == parser->checksum_b;
        parser->state = 0;
        return valid;
    }
    default:
        parser_reset(parser);
        break;
    }
    return false;
}

static esp_err_t send_ubx(uint8_t message_class, uint8_t message_id,
                          const uint8_t *payload, uint16_t length)
{
    if (length > 32) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t frame[40] = {0xb5, 0x62, message_class, message_id,
                         (uint8_t)length, (uint8_t)(length >> 8)};
    if (length != 0) {
        memcpy(frame + 6, payload, length);
    }
    uint8_t checksum_a = 0;
    uint8_t checksum_b = 0;
    for (size_t index = 2; index < (size_t)length + 6; index++) {
        checksum_a += frame[index];
        checksum_b += checksum_a;
    }
    frame[6 + length] = checksum_a;
    frame[7 + length] = checksum_b;
    int written = uart_write_bytes(BOARD_GPS_UART, frame, length + 8);
    return written == length + 8 ? ESP_OK : ESP_FAIL;
}

static esp_err_t wait_for_frame(uint8_t message_class, uint8_t message_id,
                                uint32_t timeout_ms, uint8_t *payload,
                                uint16_t *payload_length)
{
    ubx_parser_t parser = {0};
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    uint8_t buffer[128];
    while (esp_timer_get_time() < deadline) {
        if (cancel_requested) {
            return ESP_ERR_INVALID_STATE;
        }
        int count = uart_read_bytes(BOARD_GPS_UART, buffer, sizeof(buffer),
                                    pdMS_TO_TICKS(20));
        for (int index = 0; index < count; index++) {
            if (parser_feed(&parser, buffer[index]) &&
                parser.message_class == message_class &&
                parser.message_id == message_id) {
                if (payload != NULL && payload_length != NULL) {
                    uint16_t copy_length = parser.length < *payload_length
                                               ? parser.length
                                               : *payload_length;
                    memcpy(payload, parser.payload, copy_length);
                    *payload_length = copy_length;
                }
                return ESP_OK;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t poll_mon_ver(uint32_t timeout_ms)
{
    uart_flush_input(BOARD_GPS_UART);
    ESP_RETURN_ON_ERROR(send_ubx(UBX_CLASS_MON, UBX_ID_MON_VER, NULL, 0),
                        TAG, "MON-VER poll failed");
    uint8_t payload[GPS_UBX_MAX_PAYLOAD];
    uint16_t length = sizeof(payload);
    ESP_RETURN_ON_ERROR(wait_for_frame(UBX_CLASS_MON, UBX_ID_MON_VER,
                                       timeout_ms, payload, &length),
                        TAG, "MON-VER response timeout");
    static const uint8_t identity[] = {'M', 'I', 'A', '-'};
    for (uint16_t index = 0; index + sizeof(identity) <= length; index++) {
        if (memcmp(payload + index, identity, sizeof(identity)) == 0) {
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t probe_receiver(int *found_baud)
{
    static const int baud_rates[] = {38400, 115200, 9600};
    for (size_t baud_index = 0;
         baud_index < sizeof(baud_rates) / sizeof(baud_rates[0]); baud_index++) {
        if (cancel_requested) {
            return ESP_ERR_INVALID_STATE;
        }
        ESP_RETURN_ON_ERROR(uart_set_baudrate(BOARD_GPS_UART,
                                              baud_rates[baud_index]),
                            TAG, "GPS baud setup failed");
        vTaskDelay(pdMS_TO_TICKS(200));
        int64_t deadline = esp_timer_get_time() +
                           (int64_t)GPS_PROBE_PER_BAUD_MS * 1000;
        while (esp_timer_get_time() < deadline) {
            if (poll_mon_ver(2000) == ESP_OK) {
                *found_baud = baud_rates[baud_index];
                ESP_LOGI(TAG, "MIA-M10Q found at %d baud", *found_baud);
                return ESP_OK;
            }
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t wait_for_ack(uint8_t message_class, uint8_t message_id)
{
    ubx_parser_t parser = {0};
    int64_t deadline = esp_timer_get_time() +
                       (int64_t)GPS_ACK_TIMEOUT_MS * 1000;
    uint8_t buffer[128];
    while (esp_timer_get_time() < deadline) {
        if (cancel_requested) {
            return ESP_ERR_INVALID_STATE;
        }
        int count = uart_read_bytes(BOARD_GPS_UART, buffer, sizeof(buffer),
                                    pdMS_TO_TICKS(20));
        for (int index = 0; index < count; index++) {
            if (!parser_feed(&parser, buffer[index]) ||
                parser.message_class != UBX_CLASS_ACK || parser.length < 2 ||
                parser.payload[0] != message_class ||
                parser.payload[1] != message_id) {
                continue;
            }
            return parser.message_id == UBX_ID_ACK_ACK ? ESP_OK : ESP_FAIL;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t valset(uint32_t key, uint32_t value, size_t value_size)
{
    uint8_t payload[12] = {0x00, 0x01, 0x00, 0x00};
    write_uint32_le(payload + 4, key);
    for (size_t index = 0; index < value_size; index++) {
        payload[8 + index] = (uint8_t)(value >> (8 * index));
    }
    uart_flush_input(BOARD_GPS_UART);
    ESP_RETURN_ON_ERROR(send_ubx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload,
                                 8 + value_size),
                        TAG, "VALSET write failed");
    return wait_for_ack(UBX_CLASS_CFG, UBX_ID_CFG_VALSET);
}

static esp_err_t configure_receiver(int probe_baud)
{
    esp_err_t baud_result = valset(CFG_UART1_BAUDRATE, GPS_RUN_BAUD, 4);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(uart_set_baudrate(BOARD_GPS_UART, GPS_RUN_BAUD), TAG,
                        "GPS run baud setup failed");
    if (baud_result != ESP_OK && poll_mon_ver(2000) != ESP_OK) {
        ESP_RETURN_ON_ERROR(uart_set_baudrate(BOARD_GPS_UART, probe_baud), TAG,
                            "GPS baud fallback failed");
        return baud_result;
    }

    static const struct {
        uint32_t key;
        uint8_t value;
    } settings[] = {
        {CFG_SIGNAL_BDS_B1C_ENA, 0},
        {CFG_SIGNAL_GLO_ENA, 0},
        {CFG_SIGNAL_BDS_ENA, 1},
        {CFG_SIGNAL_BDS_B1_ENA, 1},
        {CFG_SIGNAL_GPS_ENA, 1},
        {CFG_SIGNAL_GAL_ENA, 1},
        {CFG_SIGNAL_QZSS_ENA, 1},
        {CFG_SIGNAL_SBAS_ENA, 1},
        {CFG_ANA_USE_ANA, 1},
        {CFG_NAVSPG_FIXMODE, 2},
        {CFG_NAVSPG_INFIL_MINSVS, 3},
        {CFG_MSGOUT_UBX_NAV_PVT_UART1, 1},
    };
    for (size_t index = 0; index < sizeof(settings) / sizeof(settings[0]);
         index++) {
        if (cancel_requested) {
            return ESP_ERR_INVALID_STATE;
        }
        ESP_RETURN_ON_ERROR(valset(settings[index].key, settings[index].value,
                                   1),
                            TAG, "GPS setting 0x%08lx failed",
                            (unsigned long)settings[index].key);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return ESP_OK;
}

static void process_nav_pvt(const uint8_t *payload, uint16_t length)
{
    if (length < 92) {
        return;
    }
    uint8_t fix_type = payload[20];
    bool fix_valid = (payload[21] & 0x01) != 0 &&
                     fix_type >= 2 && fix_type <= 4;
    portENTER_CRITICAL(&status_lock);
    current_status.fix_valid = fix_valid;
    current_status.fix_type = fix_type;
    current_status.satellites = payload[23];
    current_status.longitude_e7 = read_int32_le(payload + 24);
    current_status.latitude_e7 = read_int32_le(payload + 28);
    current_status.altitude_mm = read_int32_le(payload + 36);
    current_status.horizontal_accuracy_mm = read_uint32_le(payload + 40);
    current_status.ground_speed_mm_s = read_uint32_le(payload + 60);
    current_status.heading_e5 = read_int32_le(payload + 64);
    portEXIT_CRITICAL(&status_lock);
}

static void process_mon_rf(const uint8_t *payload, uint16_t length)
{
    if (length < 20) {
        return;
    }
    uint16_t agc = (uint16_t)payload[18] | (uint16_t)payload[19] << 8;
    portENTER_CRITICAL(&status_lock);
    current_status.agc = agc;
    portEXIT_CRITICAL(&status_lock);
}

static void gps_task(void *parameter)
{
    (void)parameter;
    ubx_parser_t parser = {0};
    uint8_t buffer[256];
    int64_t last_rf_poll = 0;
    for (;;) {
        int count = uart_read_bytes(BOARD_GPS_UART, buffer, sizeof(buffer),
                                    pdMS_TO_TICKS(100));
        for (int index = 0; index < count; index++) {
            if (!parser_feed(&parser, buffer[index])) {
                continue;
            }
            if (parser.message_class == UBX_CLASS_NAV &&
                parser.message_id == UBX_ID_NAV_PVT) {
                process_nav_pvt(parser.payload, parser.length);
            } else if (parser.message_class == UBX_CLASS_MON &&
                       parser.message_id == UBX_ID_MON_RF) {
                process_mon_rf(parser.payload, parser.length);
            }
        }
        int64_t now = esp_timer_get_time();
        if (now - last_rf_poll >= (int64_t)GPS_RF_POLL_MS * 1000) {
            send_ubx(UBX_CLASS_MON, UBX_ID_MON_RF, NULL, 0);
            last_rf_poll = now;
        }
    }
}

esp_err_t gps_initialize(void)
{
    if (driver_started || uart_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    cancel_requested = false;
    esp_err_t result = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0,
                                          "gps_uart", &gps_sleep_lock);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_pm_lock_acquire(gps_sleep_lock);
    if (result != ESP_OK) {
        esp_pm_lock_delete(gps_sleep_lock);
        gps_sleep_lock = NULL;
        return result;
    }
    const uart_config_t uart_config = {
        .baud_rate = GPS_INITIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    result = uart_param_config(BOARD_GPS_UART, &uart_config);
    if (result != ESP_OK) {
        gps_deinitialize();
        return result;
    }
    result = uart_set_pin(BOARD_GPS_UART, BOARD_GPS_TX, BOARD_GPS_RX,
                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (result != ESP_OK) {
        gps_deinitialize();
        return result;
    }
    result = uart_driver_install(BOARD_GPS_UART, GPS_UART_BUFFER_SIZE,
                                 GPS_UART_BUFFER_SIZE, 0, NULL, 0);
    if (result != ESP_OK) {
        gps_deinitialize();
        return result;
    }
    uart_installed = true;

    int probe_baud = 0;
    result = probe_receiver(&probe_baud);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "MIA-M10Q probe stopped: %s", esp_err_to_name(result));
        gps_deinitialize();
        return result;
    }
    result = configure_receiver(probe_baud);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "MIA-M10Q configuration stopped: %s",
                 esp_err_to_name(result));
        gps_deinitialize();
        return result;
    }

    portENTER_CRITICAL(&status_lock);
    current_status.ready = true;
    portEXIT_CRITICAL(&status_lock);
    if (xTaskCreate(gps_task, "gps", 4096, NULL, 5,
                    &gps_task_handle) != pdPASS) {
        gps_deinitialize();
        return ESP_ERR_NO_MEM;
    }
    driver_started = true;
    ESP_LOGI(TAG, "MIA-M10Q ready: 115200 baud, NAV-PVT 1 Hz");
    return ESP_OK;
}

void gps_cancel_initialize(void)
{
    cancel_requested = true;
}

esp_err_t gps_deinitialize(void)
{
    cancel_requested = true;
    if (driver_started) {
        esp_err_t save_result = gps_save_current_position();
        if (save_result != ESP_OK && save_result != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "cannot save final position: %s",
                     esp_err_to_name(save_result));
        }
    }
    driver_started = false;
    if (gps_task_handle != NULL) {
        vTaskDelete(gps_task_handle);
        gps_task_handle = NULL;
    }
    esp_err_t result = ESP_OK;
    if (uart_installed) {
        result = uart_driver_delete(BOARD_GPS_UART);
        uart_installed = false;
    }
    if (gps_sleep_lock != NULL) {
        esp_err_t release_result = esp_pm_lock_release(gps_sleep_lock);
        if (result == ESP_OK) {
            result = release_result;
        }
        esp_pm_lock_delete(gps_sleep_lock);
        gps_sleep_lock = NULL;
    }
    portENTER_CRITICAL(&status_lock);
    memset(&current_status, 0, sizeof(current_status));
    portEXIT_CRITICAL(&status_lock);
    return result;
}

bool gps_get_status(gps_status_t *status)
{
    if (!driver_started || status == NULL) {
        return false;
    }
    portENTER_CRITICAL(&status_lock);
    *status = current_status;
    portEXIT_CRITICAL(&status_lock);
    return true;
}

esp_err_t gps_save_current_position(void)
{
    gps_status_t status;
    if (!gps_get_status(&status) || !status.fix_valid ||
        !position_valid(status.latitude_e7, status.longitude_e7)) {
        return ESP_ERR_NOT_FOUND;
    }
    saved_position_t saved = {
        .version = 1,
        .latitude_e7 = status.latitude_e7,
        .longitude_e7 = status.longitude_e7,
    };
    int32_t saved_latitude;
    int32_t saved_longitude;
    if (load_saved_position(&saved_latitude, &saved_longitude) &&
        saved_latitude == saved.latitude_e7 &&
        saved_longitude == saved.longitude_e7) {
        return ESP_OK;
    }
    nvs_handle_t handle;
    esp_err_t result = nvs_open("gps", NVS_READWRITE, &handle);
    if (result != ESP_OK) {
        return result;
    }
    result = nvs_set_blob(handle, "last_pos", &saved, sizeof(saved));
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

bool gps_get_best_position(int32_t *latitude_e7, int32_t *longitude_e7,
                           bool *is_live)
{
    if (latitude_e7 == NULL || longitude_e7 == NULL || is_live == NULL) {
        return false;
    }
    gps_status_t status;
    if (gps_get_status(&status) && status.fix_valid &&
        position_valid(status.latitude_e7, status.longitude_e7)) {
        *latitude_e7 = status.latitude_e7;
        *longitude_e7 = status.longitude_e7;
        *is_live = true;
        esp_err_t result = gps_save_current_position();
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "cannot save current position: %s",
                     esp_err_to_name(result));
        }
        return true;
    }
    *is_live = false;
    return load_saved_position(latitude_e7, longitude_e7);
}
