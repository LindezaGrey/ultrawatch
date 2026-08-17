#include "ble_rtc.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "board.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_pm.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gps.h"
#include "hal/gpio_ll.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "screen_control.h"

#ifndef CONFIG_PM_ENABLE
#error "The BLE CPU-frequency and automatic light-sleep controls require CONFIG_PM_ENABLE"
#endif

#define RTC_CONTROL1_REGISTER 0x00
#define RTC_CONTROL2_REGISTER 0x01
#define RTC_SECONDS_REGISTER  0x04
#define RTC_ALARM_SECONDS_REGISTER 0x0b
#define RTC_24_HOUR_BIT       5
#define RTC_OSCILLATOR_STOP   7
#define RTC_ALARM_FLAG_BIT    6
#define RTC_ALARM_ENABLE_BIT  7
#define RTC_MINUTE_INTERRUPT_BIT 5
#define RTC_HALF_MINUTE_INTERRUPT_BIT 4
#define RTC_TIMER_FLAG_BIT 3
#define RTC_PAYLOAD_LENGTH    19
#define POWER_PAYLOAD_MAX_LENGTH 20
#define POWER_CONFIG_PAYLOAD_MAX_LENGTH 20
#define POWER_CONFIG_WRITE_MAX_LENGTH 12
#define IMU_PAYLOAD_LENGTH 10
#define IMU_FIFO_BUFFER_SIZE 1024
#define IMU_SAMPLE_RATE_HZ 25.0f
#define TOUCH_PAYLOAD_LENGTH 6
#define TOUCH_POLL_INTERVAL_MS 20
#define GPS_PAYLOAD_LENGTH 30
#define HAPTIC_STATUS_PAYLOAD_LENGTH 6
#define HAPTIC_COMMAND_PAYLOAD_LENGTH 2
#define SENSOR_CONTROL_PAYLOAD_LENGTH 2
#define SYSTEM_POWER_PAYLOAD_LENGTH 3
#define ALARM_PAYLOAD_LENGTH 3

#define SENSOR_IMU_BIT   (1U << 0)
#define SENSOR_GPS_BIT   (1U << 1)
#define SENSOR_TOUCH_BIT (1U << 2)
#define SENSOR_ALL_MASK  (SENSOR_IMU_BIT | SENSOR_GPS_BIT | SENSOR_TOUCH_BIT)
#define SENSOR_DEFAULT_MASK SENSOR_TOUCH_BIT
#define SENSOR_NVS_NAMESPACE "sensor_control"
#define SENSOR_NVS_KEY "enabled"
#define SYSTEM_POWER_NVS_NAMESPACE "system_power"
#define SYSTEM_POWER_NVS_FREQUENCY_KEY "max_mhz"
#define SYSTEM_POWER_NVS_SLEEP_KEY "light_sleep"
#define SYSTEM_POWER_DEFAULT_MAX_MHZ 160
#define SYSTEM_POWER_MIN_MHZ 40

#define ALARM_NVS_NAMESPACE "alarm"
#define ALARM_NVS_KEY "cfg_minimal"
#define ALARM_AUDIO_SAMPLE_RATE 16000
#define ALARM_TONE_HZ 880
#define ALARM_TONE_MS 170
#define ALARM_GAP_MS 130
#define ALARM_BEEPS_PER_CYCLE 3
#define ALARM_CYCLE_PAUSE_MS 1100
#define ALARM_HAPTIC_EFFECT 47

#define DRV2605_STATUS_REGISTER       0x00
#define DRV2605_MODE_REGISTER         0x01
#define DRV2605_LIBRARY_REGISTER      0x03
#define DRV2605_SEQUENCE_REGISTER     0x04
#define DRV2605_GO_REGISTER           0x0c
#define DRV2605_LIBRARY_ERM_A         0x01
#define DRV2605_DEVICE_ID             3
#define DRV2605L_DEVICE_ID            7
#define DRV2605_MAX_EFFECT            123
#define DRV2605_MAX_REPEATS           4

#define CST9217_REGISTER_TOUCH_DATA 0xd000
#define CST9217_ACK 0xab
#define CST9217_MAX_POINTS 2
#define CST9217_EVENT_CONTACT 0x06
#define CST9217_COMMAND_PREFIX 0xd1
#define CST9217_COMMAND_ENABLE_REPORTING 0x01
#define CST9217_COMMAND_NORMAL_MODE 0x09

#define AXP_STATUS1_REGISTER       0x00
#define AXP_STATUS2_REGISTER       0x01
#define AXP_CHIP_ID_REGISTER       0x03
#define AXP_INPUT_CURRENT_REGISTER 0x16
#define AXP_MODULE_ENABLE_REGISTER 0x18
#define AXP_ADC_CHANNEL_REGISTER   0x30
#define AXP_BATTERY_VOLTAGE_HIGH  0x34
#define AXP_CHARGE_CURRENT_REGISTER 0x62
#define AXP_CHARGE_VOLTAGE_REGISTER 0x64
#define AXP_BATTERY_DETECT_REGISTER 0x68
#define AXP_IRQ_ENABLE1_REGISTER   0x41
#define AXP_IRQ_STATUS0_REGISTER   0x48
#define AXP_CHIP_ID                0x4a
#define AXP_INPUT_CURRENT_MASK     0x07
#define AXP_CELL_CHARGE_ENABLE_BIT 1
#define AXP_CHARGE_CURRENT_MASK    0x1f
#define AXP_CHARGE_VOLTAGE_MASK    0x07
#define AXP_BATTERY_ADC_ENABLE_BIT 0
#define AXP_BATTERY_DETECT_BIT     0
#define AXP_POWERON_SHORT_PRESS_BIT 3
#define AXP_POWERON_LONG_PRESS_BIT  2
#define AXP_POWERON_POSITIVE_EDGE_BIT 0
#define AXP_BATTERY_EMPTY_MV       3200
#define AXP_BATTERY_FULL_MV        4200

static const char *TAG = "ble_rtc";
static const char *DEVICE_NAME = "UltraWatch";

/* 7a1e0001-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t rtc_service_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x01, 0x00, 0x1e, 0x7a);

/* 7a1e0002-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t rtc_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x02, 0x00, 0x1e, 0x7a);

/* 7a1e0003-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t power_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x03, 0x00, 0x1e, 0x7a);

/* 7a1e0004-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t power_config_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x04, 0x00, 0x1e, 0x7a);

/* 7a1e0005-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t imu_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x05, 0x00, 0x1e, 0x7a);

/* 7a1e0006-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t screen_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x06, 0x00, 0x1e, 0x7a);

/* 7a1e0007-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t touch_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x07, 0x00, 0x1e, 0x7a);

/* 7a1e0008-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t gps_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x08, 0x00, 0x1e, 0x7a);

/* 7a1e0009-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t haptic_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x09, 0x00, 0x1e, 0x7a);

/* 7a1e000a-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t sensor_control_characteristic_uuid =
    BLE_UUID128_INIT(
        0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
        0x6c, 0x4b, 0x1e, 0x7a, 0x0a, 0x00, 0x1e, 0x7a);

/* 7a1e000b-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t system_power_characteristic_uuid =
    BLE_UUID128_INIT(
        0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
        0x6c, 0x4b, 0x1e, 0x7a, 0x0b, 0x00, 0x1e, 0x7a);

/* 7a1e000c-7a1e-4b6c-8d9e-001122334455 */
static const ble_uuid128_t alarm_characteristic_uuid = BLE_UUID128_INIT(
    0x55, 0x44, 0x33, 0x22, 0x11, 0x00, 0x9e, 0x8d,
    0x6c, 0x4b, 0x1e, 0x7a, 0x0c, 0x00, 0x1e, 0x7a);

typedef struct {
    unsigned charge_current_ma;
    unsigned input_current_ma;
    unsigned charge_voltage_mv;
    bool charger_enabled;
} power_config_t;

static SemaphoreHandle_t i2c_mutex;
static uint8_t own_address_type;
static uint16_t rtc_value_handle;
static uint16_t power_value_handle;
static uint16_t imu_value_handle;
static uint16_t touch_value_handle;
static uint16_t gps_value_handle;
static uint16_t sensor_control_value_handle;
static volatile uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool rtc_notifications_enabled;
static volatile bool power_notifications_enabled;
static volatile bool imu_notifications_enabled;
static volatile bool touch_notifications_enabled;
static volatile bool gps_notifications_enabled;
static volatile bool sensor_control_notifications_enabled;
static volatile bool imu_ready;
static volatile bool imu_sample_available;
static bool imu_first_sample_logged;
static portMUX_TYPE imu_lock = portMUX_INITIALIZER_UNLOCKED;
static struct bhy2_dev imu_device;
static struct bhy2_data_quaternion imu_quaternion = { .w = 16384 };
static volatile bool touch_ready;
static portMUX_TYPE touch_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t touch_address = BOARD_CST9217_ADDR_PRIMARY;
static bool touch_pressed;
static uint8_t touch_event;
static uint16_t touch_x;
static uint16_t touch_y;
static bool haptic_ready;
static uint8_t haptic_device_id;
static uint8_t haptic_last_effect;
static uint8_t haptic_last_repeats;
static volatile uint8_t requested_sensor_mask = SENSOR_DEFAULT_MASK;
static TaskHandle_t imu_control_task_handle;
static TaskHandle_t gps_control_task_handle;
static TaskHandle_t touch_control_task_handle;
static TaskHandle_t touch_task_handle;
static TaskHandle_t power_button_task_handle;
static TaskHandle_t alarm_task_handle;
static uint16_t system_power_max_mhz = SYSTEM_POWER_DEFAULT_MAX_MHZ;
static bool system_power_light_sleep = true;
static volatile bool advertising_enabled = true;
static alarm_config_t alarm_config = {.hour = 7, .minute = 0, .enabled = false};
static volatile bool alarm_ringing;
static i2s_chan_handle_t alarm_audio_tx;
static bool alarm_audio_ready;

extern const uint8_t
    bhi260_firmware_start[] asm("_binary_BHI260AP_fw_start");
extern const uint8_t
    bhi260_firmware_end[] asm("_binary_BHI260AP_fw_end");

static esp_err_t rtc_read_registers(uint8_t reg, uint8_t *data,
                                    size_t length);
static esp_err_t rtc_write_registers(uint8_t reg, const uint8_t *data,
                                     size_t length);

static uint8_t decimal_to_bcd(int value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static int bcd_to_decimal(uint8_t value)
{
    return ((value >> 4) * 10) + (value & 0x0f);
}

static esp_err_t i2c_read_registers(uint8_t address, uint8_t reg,
                                    uint8_t *data, size_t length)
{
    esp_err_t result;
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    result = i2c_master_write_read_device(BOARD_I2C_PORT,
                                          address,
                                          &reg, 1, data, length,
                                          portMAX_DELAY);
    xSemaphoreGive(i2c_mutex);
    return result;
}

static esp_err_t i2c_write_registers(uint8_t address, uint8_t reg,
                                     const uint8_t *data, size_t length)
{
    uint8_t buffer[9];
    if (length > sizeof(buffer) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }

    buffer[0] = reg;
    memcpy(buffer + 1, data, length);

    esp_err_t result;
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    result = i2c_master_write_to_device(BOARD_I2C_PORT,
                                        address,
                                        buffer, length + 1,
                                        portMAX_DELAY);
    xSemaphoreGive(i2c_mutex);
    return result;
}

static esp_err_t drv2605_initialize(void)
{
    uint8_t status;
    ESP_RETURN_ON_ERROR(i2c_read_registers(BOARD_DRV2605_ADDR,
                                           DRV2605_STATUS_REGISTER,
                                           &status, 1),
                        TAG, "DRV2605 status read failed");
    uint8_t device_id = status >> 5;
    if (device_id != DRV2605_DEVICE_ID && device_id != DRV2605L_DEVICE_ID) {
        ESP_LOGE(TAG, "unexpected haptic driver ID: %u", device_id);
        return ESP_ERR_NOT_FOUND;
    }

    const uint8_t mode = 0x00;
    const uint8_t library = DRV2605_LIBRARY_ERM_A;
    const uint8_t sequence[8] = {0};
    const uint8_t stop = 0;
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_MODE_REGISTER,
                                            &mode, 1),
                        TAG, "DRV2605 wake failed");
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_LIBRARY_REGISTER,
                                            &library, 1),
                        TAG, "DRV2605 library setup failed");
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_SEQUENCE_REGISTER,
                                            sequence, sizeof(sequence)),
                        TAG, "DRV2605 sequencer reset failed");
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_GO_REGISTER,
                                            &stop, 1),
                        TAG, "DRV2605 stop failed");
    haptic_device_id = device_id;
    haptic_ready = true;
    ESP_LOGI(TAG, "haptic ready: DRV2605%s, ERM library A",
             device_id == DRV2605L_DEVICE_ID ? "L" : "");
    return ESP_OK;
}

static esp_err_t drv2605_stop(void)
{
    if (!haptic_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t stop = 0;
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_GO_REGISTER,
                                            &stop, 1),
                        TAG, "DRV2605 stop failed");
    haptic_last_effect = 0;
    haptic_last_repeats = 0;
    return ESP_OK;
}

static esp_err_t drv2605_play(uint8_t effect, uint8_t repeats)
{
    if (!haptic_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (effect == 0) {
        return repeats == 0 ? drv2605_stop() : ESP_ERR_INVALID_ARG;
    }
    if (effect > DRV2605_MAX_EFFECT || repeats == 0 ||
        repeats > DRV2605_MAX_REPEATS) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t mode = 0x00;
    uint8_t sequence[8] = {0};
    for (uint8_t index = 0; index < repeats; index++) {
        sequence[index] = effect;
    }
    const uint8_t go = 1;
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_GO_REGISTER,
                                            &(uint8_t){0}, 1),
                        TAG, "DRV2605 cancel failed");
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_MODE_REGISTER,
                                            &mode, 1),
                        TAG, "DRV2605 mode setup failed");
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_SEQUENCE_REGISTER,
                                            sequence, sizeof(sequence)),
                        TAG, "DRV2605 sequence setup failed");
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_DRV2605_ADDR,
                                            DRV2605_GO_REGISTER,
                                            &go, 1),
                        TAG, "DRV2605 trigger failed");
    haptic_last_effect = effect;
    haptic_last_repeats = repeats;
    return ESP_OK;
}

esp_err_t ble_haptic_click(void)
{
    /* ERM library A effect 1 is the driver's short strong-click waveform. */
    return drv2605_play(1, 1);
}

static bool haptic_get_payload(uint8_t output[HAPTIC_STATUS_PAYLOAD_LENGTH])
{
    memset(output, 0, HAPTIC_STATUS_PAYLOAD_LENGTH);
    if (!haptic_ready) {
        return true;
    }
    uint8_t status;
    uint8_t go;
    if (i2c_read_registers(BOARD_DRV2605_ADDR, DRV2605_STATUS_REGISTER,
                           &status, 1) != ESP_OK ||
        i2c_read_registers(BOARD_DRV2605_ADDR, DRV2605_GO_REGISTER,
                           &go, 1) != ESP_OK) {
        return false;
    }
    output[0] = 1;
    output[1] = haptic_device_id;
    output[2] = go & 0x01;
    output[3] = haptic_last_effect;
    output[4] = haptic_last_repeats;
    output[5] = status & 0x0f;
    return true;
}

static esp_err_t sensor_rail_set(uint8_t voltage_register,
                                 uint8_t voltage_code, uint8_t enable_bit,
                                 bool enabled)
{
    uint8_t voltage;
    uint8_t ldo_enable;
    ESP_RETURN_ON_ERROR(i2c_read_registers(BOARD_AXP2101_ADDR,
                                           voltage_register, &voltage, 1),
                        TAG, "sensor voltage read failed");
    voltage = (voltage & 0xe0) | voltage_code;
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_AXP2101_ADDR,
                                            voltage_register, &voltage, 1),
                        TAG, "sensor voltage write failed");
    ESP_RETURN_ON_ERROR(i2c_read_registers(BOARD_AXP2101_ADDR,
                                           BOARD_AXP2101_LDO_ENABLE,
                                           &ldo_enable, 1),
                        TAG, "sensor rail state read failed");
    if (enabled) {
        ldo_enable |= 1U << enable_bit;
    } else {
        ldo_enable &= ~(1U << enable_bit);
    }
    return i2c_write_registers(BOARD_AXP2101_ADDR,
                               BOARD_AXP2101_LDO_ENABLE, &ldo_enable, 1);
}

static esp_err_t alarm_config_store(void)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(ALARM_NVS_NAMESPACE, NVS_READWRITE, &handle),
                        TAG, "alarm preference open failed");
    esp_err_t result = nvs_set_blob(handle, ALARM_NVS_KEY, &alarm_config,
                                    sizeof(alarm_config));
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

static esp_err_t alarm_config_load(void)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(ALARM_NVS_NAMESPACE, NVS_READWRITE, &handle),
                        TAG, "alarm preference open failed");
    alarm_config_t stored;
    size_t length = sizeof(stored);
    esp_err_t result = nvs_get_blob(handle, ALARM_NVS_KEY, &stored, &length);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (result != ESP_OK) {
        return result;
    }
    if (length != sizeof(stored) || stored.hour > 23 || stored.minute > 59) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    alarm_config = stored;
    return ESP_OK;
}

static esp_err_t alarm_rtc_apply(void)
{
    uint8_t control2;
    ESP_RETURN_ON_ERROR(rtc_read_registers(RTC_CONTROL2_REGISTER, &control2, 1),
                        TAG, "RTC alarm control read failed");
    control2 &= ~((1U << RTC_ALARM_FLAG_BIT) |
                  (1U << RTC_MINUTE_INTERRUPT_BIT) |
                  (1U << RTC_HALF_MINUTE_INTERRUPT_BIT) |
                  (1U << RTC_TIMER_FLAG_BIT));
    if (alarm_config.enabled) {
        const uint8_t alarm_registers[] = {
            decimal_to_bcd(0), decimal_to_bcd(alarm_config.minute),
            decimal_to_bcd(alarm_config.hour), 0x80, 0x80,
        };
        ESP_RETURN_ON_ERROR(rtc_write_registers(RTC_ALARM_SECONDS_REGISTER,
                                                alarm_registers,
                                                sizeof(alarm_registers)),
                            TAG, "RTC alarm time write failed");
        control2 |= 1U << RTC_ALARM_ENABLE_BIT;
    } else {
        control2 &= ~(1U << RTC_ALARM_ENABLE_BIT);
    }
    return rtc_write_registers(RTC_CONTROL2_REGISTER, &control2, 1);
}

static esp_err_t alarm_audio_initialize(void)
{
    i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_1, I2S_ROLE_MASTER);
    channel.dma_desc_num = 6;
    channel.dma_frame_num = 240;
    channel.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&channel, &alarm_audio_tx, NULL), TAG,
                        "alarm I2S channel allocation failed");
    i2s_std_config_t standard = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(ALARM_AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BOARD_AUDIO_BCLK,
            .ws = BOARD_AUDIO_WCLK,
            .dout = BOARD_AUDIO_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(alarm_audio_tx, &standard),
                        TAG, "alarm I2S mode setup failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(alarm_audio_tx), TAG,
                        "alarm I2S enable failed");
    alarm_audio_ready = true;
    ESP_LOGI(TAG, "alarm audio ready on I2S1 (BCLK %d, WCLK %d, DOUT %d)",
             BOARD_AUDIO_BCLK, BOARD_AUDIO_WCLK, BOARD_AUDIO_DOUT);
    return ESP_OK;
}

static void alarm_fill_beep(int16_t *samples, size_t sample_count)
{
    const size_t tone_samples =
        ALARM_AUDIO_SAMPLE_RATE * ALARM_TONE_MS / 1000;
    const size_t edge_samples = ALARM_AUDIO_SAMPLE_RATE * 12 / 1000;
    for (size_t index = 0; index < sample_count; index++) {
        if (index >= tone_samples) {
            samples[index] = 0;
            continue;
        }
        float envelope = 1.0f;
        if (index < edge_samples) {
            envelope = (float)index / (float)edge_samples;
        } else if (index + edge_samples > tone_samples) {
            envelope = (float)(tone_samples - index) / (float)edge_samples;
        }
        samples[index] = (int16_t)(sinf(2.0f * 3.14159265f *
                                           ALARM_TONE_HZ * index /
                                           ALARM_AUDIO_SAMPLE_RATE) *
                                   3276.0f * envelope);
    }
}

static esp_err_t alarm_audio_write(const int16_t *samples, size_t count)
{
    if (!alarm_audio_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t written = 0;
    ESP_RETURN_ON_ERROR(i2s_channel_write(alarm_audio_tx, samples,
                                          count * sizeof(*samples), &written,
                                          portMAX_DELAY),
                        TAG, "alarm audio write failed");
    return written == count * sizeof(*samples) ? ESP_OK : ESP_FAIL;
}

static void IRAM_ATTR alarm_interrupt_handler(void *argument)
{
    (void)argument;
    /* The PCF85063A keeps INT low until AF is cleared. Mask its GPIO source
     * before waking the task, or the level wake condition can starve CPU0. */
    gpio_ll_intr_disable(GPIO_LL_GET_HW(0), BOARD_RTC_INTERRUPT);
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (alarm_task_handle != NULL) {
        vTaskNotifyGiveFromISR(alarm_task_handle, &higher_priority_task_woken);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static bool alarm_take_rtc_flag(void)
{
    uint8_t control2;
    if (rtc_read_registers(RTC_CONTROL2_REGISTER, &control2, 1) != ESP_OK) {
        return false;
    }
    const bool alarm_flag =
        (control2 & (1U << RTC_ALARM_FLAG_BIT)) != 0;
    control2 &= ~((1U << RTC_ALARM_FLAG_BIT) |
                  (1U << RTC_MINUTE_INTERRUPT_BIT) |
                  (1U << RTC_HALF_MINUTE_INTERRUPT_BIT) |
                  (1U << RTC_TIMER_FLAG_BIT));
    if (rtc_write_registers(RTC_CONTROL2_REGISTER, &control2, 1) != ESP_OK) {
        return false;
    }
    return alarm_flag && alarm_config.enabled;
}

static void alarm_task(void *parameter)
{
    (void)parameter;
    const size_t unit_samples =
        ALARM_AUDIO_SAMPLE_RATE * (ALARM_TONE_MS + ALARM_GAP_MS) / 1000;
    int16_t *unit = heap_caps_malloc(unit_samples * sizeof(*unit),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (unit != NULL) {
        alarm_fill_beep(unit, unit_samples);
    }
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const bool alarm_triggered = alarm_take_rtc_flag();
        esp_rom_delay_us(100);
        if (gpio_get_level(BOARD_RTC_INTERRUPT) == 1) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                gpio_intr_enable(BOARD_RTC_INTERRUPT));
        } else {
            ESP_LOGE(TAG, "RTC interrupt remained low after flag clear");
        }
        if (!alarm_triggered || alarm_ringing) {
            continue;
        }
        alarm_ringing = true;
        screen_alarm_ring_started();
        ESP_LOGI(TAG, "alarm ringing at %02u:%02u with sound and vibration",
                 alarm_config.hour, alarm_config.minute);
        esp_err_t power_result = sensor_rail_set(
            BOARD_AXP2101_BLDO2_VOLTAGE, 28, BOARD_AXP2101_BLDO2_BIT, true);
        if (power_result != ESP_OK) {
            ESP_LOGE(TAG, "alarm speaker power failed: %s",
                     esp_err_to_name(power_result));
        }
        while (alarm_ringing) {
            for (int beep = 0;
                 beep < ALARM_BEEPS_PER_CYCLE && alarm_ringing; beep++) {
                esp_err_t haptic_result = drv2605_play(ALARM_HAPTIC_EFFECT, 1);
                if (haptic_result != ESP_OK) {
                    ESP_LOGW(TAG, "alarm vibration failed: %s",
                             esp_err_to_name(haptic_result));
                }
                if (unit != NULL && power_result == ESP_OK) {
                    esp_err_t audio_result = alarm_audio_write(unit, unit_samples);
                    if (audio_result != ESP_OK) {
                        ESP_LOGW(TAG, "alarm sound failed: %s",
                                 esp_err_to_name(audio_result));
                    }
                } else {
                    vTaskDelay(pdMS_TO_TICKS(ALARM_TONE_MS + ALARM_GAP_MS));
                }
            }
            for (int elapsed = 0;
                 elapsed < ALARM_CYCLE_PAUSE_MS && alarm_ringing;
                 elapsed += 100) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        drv2605_stop();
        ESP_ERROR_CHECK_WITHOUT_ABORT(sensor_rail_set(
            BOARD_AXP2101_BLDO2_VOLTAGE, 28, BOARD_AXP2101_BLDO2_BIT, false));
        screen_request_refresh();
        ESP_LOGI(TAG, "alarm dismissed");
    }
}

static esp_err_t alarm_initialize(void)
{
    esp_err_t load_result = alarm_config_load();
    if (load_result != ESP_OK) {
        ESP_LOGW(TAG, "stored alarm unavailable; using 07:00 off: %s",
                 esp_err_to_name(load_result));
        alarm_config = (alarm_config_t){.hour = 7, .minute = 0,
                                        .enabled = false};
    }
    ESP_RETURN_ON_ERROR(alarm_rtc_apply(), TAG, "RTC alarm setup failed");
    esp_err_t audio_result = alarm_audio_initialize();
    if (audio_result != ESP_OK) {
        ESP_LOGE(TAG, "alarm sound initialization failed: %s",
                 esp_err_to_name(audio_result));
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(sensor_rail_set(
        BOARD_AXP2101_BLDO2_VOLTAGE, 28, BOARD_AXP2101_BLDO2_BIT, false));
    if (xTaskCreate(alarm_task, "alarm", 4096, NULL, 6,
                    &alarm_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    gpio_config_t interrupt = {
        .pin_bit_mask = 1ULL << BOARD_RTC_INTERRUPT,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&interrupt), TAG,
                        "RTC alarm interrupt GPIO setup failed");
    /* GPIO interrupt configuration survives an ESP32-S3 software reset.
     * Mask all shared wake sources before the first ISR-service install. A
     * retained PMIC low-level source otherwise interrupts the allocator and
     * causes an interrupt-watchdog reboot loop. */
    gpio_dev_t *gpio_hw = GPIO_LL_GET_HW(0);
    gpio_ll_intr_disable(gpio_hw, BOARD_RTC_INTERRUPT);
    gpio_ll_intr_disable(gpio_hw, BOARD_TOUCH_INTERRUPT);
    gpio_ll_intr_disable(gpio_hw, BOARD_PMU_INTERRUPT);
    gpio_ll_clear_intr_status(gpio_hw,
                              (1U << BOARD_RTC_INTERRUPT) |
                                  (1U << BOARD_TOUCH_INTERRUPT) |
                                  (1U << BOARD_PMU_INTERRUPT));
    esp_err_t isr_result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
        return isr_result;
    }
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(BOARD_RTC_INTERRUPT,
                                           GPIO_INTR_NEGEDGE),
                        TAG, "RTC alarm interrupt type failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BOARD_RTC_INTERRUPT,
                                             alarm_interrupt_handler, NULL),
                        TAG, "RTC alarm interrupt handler failed");
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext0_wakeup(BOARD_RTC_INTERRUPT, 0),
                        TAG, "RTC alarm EXT0 wake source failed");
    if (gpio_get_level(BOARD_RTC_INTERRUPT) == 0) {
        xTaskNotifyGive(alarm_task_handle);
    }
    ESP_LOGI(TAG, "alarm %s at %02u:%02u",
             alarm_config.enabled ? "enabled" : "disabled",
             alarm_config.hour, alarm_config.minute);
    screen_request_refresh();
    return ESP_OK;
}

void ble_alarm_get_config(alarm_config_t *config)
{
    if (config != NULL) {
        *config = alarm_config;
    }
}

esp_err_t ble_alarm_set(uint8_t hour, uint8_t minute, bool enabled)
{
    if (hour > 23 || minute > 59) {
        return ESP_ERR_INVALID_ARG;
    }
    const alarm_config_t previous = alarm_config;
    alarm_config.hour = hour;
    alarm_config.minute = minute;
    alarm_config.enabled = enabled;
    esp_err_t result = alarm_config_store();
    if (result == ESP_OK) {
        result = alarm_rtc_apply();
    }
    if (result != ESP_OK) {
        alarm_config = previous;
        ESP_ERROR_CHECK_WITHOUT_ABORT(alarm_config_store());
        ESP_ERROR_CHECK_WITHOUT_ABORT(alarm_rtc_apply());
        ESP_LOGE(TAG, "alarm update failed; restored prior setting: %s",
                 esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(TAG, "alarm set: %s %02u:%02u, sound and vibration",
             enabled ? "on" : "off", hour, minute);
    screen_request_refresh();
    return ESP_OK;
}

bool ble_alarm_is_ringing(void)
{
    return alarm_ringing;
}

esp_err_t ble_alarm_dismiss(void)
{
    alarm_ringing = false;
    return ESP_OK;
}

static uint8_t sensor_ready_mask(void)
{
    uint8_t mask = 0;
    if (imu_ready) {
        mask |= SENSOR_IMU_BIT;
    }
    gps_status_t gps_status;
    if (gps_get_status(&gps_status) && gps_status.ready) {
        mask |= SENSOR_GPS_BIT;
    }
    if (touch_ready) {
        mask |= SENSOR_TOUCH_BIT;
    }
    return mask;
}

static void sensor_control_get_payload(
    uint8_t output[SENSOR_CONTROL_PAYLOAD_LENGTH])
{
    output[0] = requested_sensor_mask;
    output[1] = sensor_ready_mask();
}

static esp_err_t sensor_control_load(void)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(SENSOR_NVS_NAMESPACE, NVS_READWRITE, &handle),
                        TAG, "sensor preference open failed");
    uint8_t stored_mask = SENSOR_DEFAULT_MASK;
    esp_err_t result = nvs_get_u8(handle, SENSOR_NVS_KEY, &stored_mask);
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        requested_sensor_mask = SENSOR_DEFAULT_MASK;
        return ESP_OK;
    }
    if (result != ESP_OK || (stored_mask & ~SENSOR_ALL_MASK) != 0) {
        return result == ESP_OK ? ESP_ERR_INVALID_RESPONSE : result;
    }
    requested_sensor_mask = stored_mask;
    return ESP_OK;
}

static esp_err_t sensor_control_store(uint8_t mask)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(SENSOR_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "sensor preference open failed");
    esp_err_t result = nvs_set_u8(handle, SENSOR_NVS_KEY, mask);
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

static bool system_power_frequency_valid(uint16_t frequency_mhz)
{
    return frequency_mhz == 80 || frequency_mhz == 160 ||
           frequency_mhz == 240;
}

static esp_err_t system_power_apply(uint16_t frequency_mhz,
                                    bool light_sleep)
{
    if (!system_power_frequency_valid(frequency_mhz)) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_pm_config_t config = {
        .max_freq_mhz = frequency_mhz,
        .min_freq_mhz = SYSTEM_POWER_MIN_MHZ,
        .light_sleep_enable = light_sleep,
    };
    ESP_RETURN_ON_ERROR(esp_pm_configure(&config), TAG,
                        "ESP32 power configuration failed");
    system_power_max_mhz = frequency_mhz;
    system_power_light_sleep = light_sleep;
    ESP_LOGI(TAG, "ESP32 power: %u MHz max, automatic light sleep %s",
             frequency_mhz, light_sleep ? "enabled" : "disabled");
    return ESP_OK;
}

static esp_err_t system_power_load(void)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(SYSTEM_POWER_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "system power preference open failed");
    uint16_t frequency_mhz = SYSTEM_POWER_DEFAULT_MAX_MHZ;
    uint8_t light_sleep = 1;
    esp_err_t frequency_result = nvs_get_u16(
        handle, SYSTEM_POWER_NVS_FREQUENCY_KEY, &frequency_mhz);
    esp_err_t sleep_result = nvs_get_u8(
        handle, SYSTEM_POWER_NVS_SLEEP_KEY, &light_sleep);
    nvs_close(handle);
    if (frequency_result != ESP_OK && frequency_result != ESP_ERR_NVS_NOT_FOUND) {
        return frequency_result;
    }
    if (sleep_result != ESP_OK && sleep_result != ESP_ERR_NVS_NOT_FOUND) {
        return sleep_result;
    }
    if (!system_power_frequency_valid(frequency_mhz) || light_sleep > 1) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* Apply this only after all GPIO wake handlers are installed. Enabling
     * automatic light sleep while the shared GPIO ISR is being allocated can
     * leave the ESP32-S3 in a GPIO interrupt loop. */
    system_power_max_mhz = frequency_mhz;
    system_power_light_sleep = light_sleep != 0;
    return ESP_OK;
}

static esp_err_t system_power_store(uint16_t frequency_mhz,
                                    bool light_sleep)
{
    ESP_RETURN_ON_ERROR(system_power_apply(frequency_mhz, light_sleep), TAG,
                        "system power apply failed");
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(SYSTEM_POWER_NVS_NAMESPACE, NVS_READWRITE,
                                 &handle),
                        TAG, "system power preference open failed");
    esp_err_t result = nvs_set_u16(handle,
                                   SYSTEM_POWER_NVS_FREQUENCY_KEY,
                                   frequency_mhz);
    if (result == ESP_OK) {
        result = nvs_set_u8(handle, SYSTEM_POWER_NVS_SLEEP_KEY,
                            light_sleep ? 1 : 0);
    }
    if (result == ESP_OK) {
        result = nvs_commit(handle);
    }
    nvs_close(handle);
    return result;
}

static void system_power_get_payload(
    uint8_t output[SYSTEM_POWER_PAYLOAD_LENGTH])
{
    output[0] = (uint8_t)system_power_max_mhz;
    output[1] = (uint8_t)(system_power_max_mhz >> 8);
    output[2] = system_power_light_sleep ? 1 : 0;
}

static void sensor_control_notify(void)
{
    uint16_t handle = connection_handle;
    if (handle == BLE_HS_CONN_HANDLE_NONE ||
        !sensor_control_notifications_enabled) {
        return;
    }
    int result = ble_gatts_notify(handle, sensor_control_value_handle);
    if (result != 0 && result != BLE_HS_ENOTCONN) {
        ESP_LOGD(TAG, "sensor control notification skipped: %d", result);
    }
}

static void sensor_control_wake_tasks(void)
{
    if (imu_control_task_handle != NULL) {
        xTaskNotifyGive(imu_control_task_handle);
    }
    if (gps_control_task_handle != NULL) {
        xTaskNotifyGive(gps_control_task_handle);
    }
    if (touch_control_task_handle != NULL) {
        xTaskNotifyGive(touch_control_task_handle);
    }
}

static esp_err_t i2c_write_raw(uint8_t address, const uint8_t *data,
                               size_t length)
{
    esp_err_t result;
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    result = i2c_master_write_to_device(BOARD_I2C_PORT, address, data, length,
                                        portMAX_DELAY);
    xSemaphoreGive(i2c_mutex);
    return result;
}

static esp_err_t i2c_transmit_receive(uint8_t address,
                                      const uint8_t *write_data,
                                      size_t write_length, uint8_t *read_data,
                                      size_t read_length)
{
    esp_err_t result;
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    result = i2c_master_write_read_device(BOARD_I2C_PORT, address, write_data,
                                          write_length, read_data, read_length,
                                          portMAX_DELAY);
    xSemaphoreGive(i2c_mutex);
    return result;
}

static int8_t bhi260_i2c_read(uint8_t reg, uint8_t *data, uint32_t length,
                              void *interface)
{
    (void)interface;
    return i2c_read_registers(BOARD_BHI260_ADDR, reg, data, length) == ESP_OK
               ? BHY2_INTF_RET_SUCCESS
               : -1;
}

static int8_t bhi260_i2c_write(uint8_t reg, const uint8_t *data,
                               uint32_t length, void *interface)
{
    (void)interface;
    if (length > 256) {
        return -1;
    }

    uint8_t buffer[257];
    buffer[0] = reg;
    memcpy(buffer + 1, data, length);

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    esp_err_t result = i2c_master_write_to_device(
        BOARD_I2C_PORT, BOARD_BHI260_ADDR, buffer, length + 1,
        portMAX_DELAY);
    xSemaphoreGive(i2c_mutex);
    return result == ESP_OK ? BHY2_INTF_RET_SUCCESS : -1;
}

static void bhi260_delay_us(uint32_t period_us, void *interface)
{
    (void)interface;
    esp_rom_delay_us(period_us);
}

static void imu_fifo_callback(
    const struct bhy2_fifo_parse_data_info *callback_info, void *reference)
{
    (void)reference;
    if (callback_info->data_size != 11) {
        return;
    }

    struct bhy2_data_quaternion sample;
    bhy2_parse_quaternion(callback_info->data_ptr, &sample);
    portENTER_CRITICAL(&imu_lock);
    imu_quaternion = sample;
    imu_sample_available = true;
    portEXIT_CRITICAL(&imu_lock);
    if (!imu_first_sample_logged) {
        imu_first_sample_logged = true;
        ESP_LOGI(TAG, "BHI260 first quaternion: x=%d y=%d z=%d w=%d",
                 sample.x, sample.y, sample.z, sample.w);
    }
}

static esp_err_t bhi260_initialize(void)
{
    gpio_config_t interrupt_config = {
        .pin_bit_mask = 1ULL << BOARD_BHI260_INTERRUPT,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&interrupt_config), TAG,
                        "IMU interrupt GPIO setup failed");

    int8_t result = bhy2_init(BHY2_I2C_INTERFACE, bhi260_i2c_read,
                              bhi260_i2c_write, bhi260_delay_us, 256, NULL,
                              &imu_device);
    if (result != BHY2_OK) {
        return ESP_FAIL;
    }
    if (bhy2_soft_reset(&imu_device) != BHY2_OK) {
        return ESP_FAIL;
    }

    uint8_t product_id = 0;
    uint8_t boot_status = 0;
    if (bhy2_get_product_id(&product_id, &imu_device) != BHY2_OK ||
        product_id != BHY2_PRODUCT_ID ||
        bhy2_get_boot_status(&boot_status, &imu_device) != BHY2_OK ||
        !(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
        ESP_LOGE(TAG, "BHI260 not ready (product 0x%02x, boot 0x%02x)",
                 product_id, boot_status);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t interrupt_control =
        BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
    if (bhy2_set_host_interrupt_ctrl(interrupt_control, &imu_device) !=
            BHY2_OK ||
        bhy2_set_host_intf_ctrl(0, &imu_device) != BHY2_OK) {
        return ESP_FAIL;
    }

    size_t firmware_length = bhi260_firmware_end - bhi260_firmware_start;
    ESP_LOGI(TAG, "uploading %u-byte BHI260 firmware",
             (unsigned)firmware_length);
    if (bhy2_upload_firmware_to_ram(bhi260_firmware_start, firmware_length,
                                    &imu_device) != BHY2_OK ||
        bhy2_boot_from_ram(&imu_device) != BHY2_OK) {
        return ESP_FAIL;
    }

    uint16_t kernel_version = 0;
    if (bhy2_get_kernel_version(&kernel_version, &imu_device) != BHY2_OK ||
        kernel_version == 0) {
        return ESP_FAIL;
    }
    const struct bhy2_orient_matrix watch_mount = {
        .c = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
    };
    if (bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAMERV,
                                           imu_fifo_callback, NULL,
                                           &imu_device) != BHY2_OK ||
        bhy2_update_virtual_sensor_list(&imu_device) != BHY2_OK ||
        bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_ACCELEROMETER,
                                    watch_mount, &imu_device) != BHY2_OK ||
        bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_GYROSCOPE,
                                    watch_mount, &imu_device) != BHY2_OK ||
        bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GAMERV, IMU_SAMPLE_RATE_HZ,
                                 0, &imu_device) != BHY2_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BHI260 ready: kernel %u, game rotation vector %.0f Hz",
             kernel_version, IMU_SAMPLE_RATE_HZ);
    imu_ready = true;
    return ESP_OK;
}

static void encode_int16_le(uint8_t *output, int16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)((uint16_t)value >> 8);
}

static void encode_uint16_le(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
}

static void encode_uint32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
    output[2] = (uint8_t)(value >> 16);
    output[3] = (uint8_t)(value >> 24);
}

static void encode_int32_le(uint8_t *output, int32_t value)
{
    encode_uint32_le(output, (uint32_t)value);
}

static bool imu_get_payload(uint8_t output[IMU_PAYLOAD_LENGTH])
{
    struct bhy2_data_quaternion sample;
    if (!imu_ready || !imu_sample_available) {
        return false;
    }
    portENTER_CRITICAL(&imu_lock);
    sample = imu_quaternion;
    portEXIT_CRITICAL(&imu_lock);
    encode_int16_le(output, sample.x);
    encode_int16_le(output + 2, sample.y);
    encode_int16_le(output + 4, sample.z);
    encode_int16_le(output + 6, sample.w);
    encode_uint16_le(output + 8, sample.accuracy);
    return true;
}

enum {
    TOUCH_EVENT_IDLE = 0,
    TOUCH_EVENT_DOWN = 1,
    TOUCH_EVENT_MOVE = 2,
    TOUCH_EVENT_UP = 3,
};

static void IRAM_ATTR touch_interrupt_handler(void *argument)
{
    (void)argument;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (touch_task_handle != NULL) {
        vTaskNotifyGiveFromISR(touch_task_handle,
                               &higher_priority_task_woken);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static esp_err_t cst9217_initialize(void)
{
    gpio_config_t interrupt_config = {
        .pin_bit_mask = 1ULL << BOARD_TOUCH_INTERRUPT,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&interrupt_config), TAG,
                        "touch interrupt GPIO setup failed");

    uint8_t output;
    uint8_t config;
    ESP_RETURN_ON_ERROR(i2c_read_registers(BOARD_XL9555_ADDR,
                                           BOARD_XL9555_OUTPUT1, &output, 1),
                        TAG, "touch reset output read failed");
    output &= ~(1U << BOARD_XL9555_TOUCH_RESET_BIT);
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_XL9555_ADDR,
                                            BOARD_XL9555_OUTPUT1, &output, 1),
                        TAG, "touch reset low failed");
    ESP_RETURN_ON_ERROR(i2c_read_registers(BOARD_XL9555_ADDR,
                                           BOARD_XL9555_CONFIG1, &config, 1),
                        TAG, "touch reset config read failed");
    config &= ~(1U << BOARD_XL9555_TOUCH_RESET_BIT);
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_XL9555_ADDR,
                                            BOARD_XL9555_CONFIG1, &config, 1),
                        TAG, "touch reset config failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    output |= 1U << BOARD_XL9555_TOUCH_RESET_BIT;
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_XL9555_ADDR,
                                            BOARD_XL9555_OUTPUT1, &output, 1),
                        TAG, "touch reset high failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t probe;
    if (i2c_read_registers(BOARD_CST9217_ADDR_PRIMARY, 0x00, &probe, 1) !=
        ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    touch_address = BOARD_CST9217_ADDR_PRIMARY;

    const uint8_t enable_reporting[] = {
        CST9217_COMMAND_PREFIX, CST9217_COMMAND_ENABLE_REPORTING, 0x01,
    };
    ESP_RETURN_ON_ERROR(i2c_write_raw(touch_address, enable_reporting,
                                      sizeof(enable_reporting)),
                        TAG, "touch reporting enable failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    const uint8_t normal_mode[] = {
        CST9217_COMMAND_PREFIX, CST9217_COMMAND_NORMAL_MODE,
    };
    ESP_RETURN_ON_ERROR(i2c_write_raw(touch_address, normal_mode,
                                      sizeof(normal_mode)),
                        TAG, "touch normal mode failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    esp_err_t isr_result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
        return isr_result;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BOARD_TOUCH_INTERRUPT,
                                             touch_interrupt_handler, NULL),
                        TAG, "touch interrupt handler failed");
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext1_wakeup_io(
                            1ULL << BOARD_TOUCH_INTERRUPT,
                            ESP_EXT1_WAKEUP_ANY_LOW),
                        TAG, "touch EXT1 wake source failed");
    ESP_RETURN_ON_ERROR(gpio_intr_enable(BOARD_TOUCH_INTERRUPT), TAG,
                        "touch interrupt enable failed");
    touch_ready = true;
    if (gpio_get_level(BOARD_TOUCH_INTERRUPT) == 0 &&
        touch_task_handle != NULL) {
        xTaskNotifyGive(touch_task_handle);
    }
    ESP_LOGI(TAG, "CST9217 touch ready at 0x%02x", touch_address);
    return ESP_OK;
}

static esp_err_t cst9217_disable(void)
{
    touch_ready = false;
    gpio_intr_disable(BOARD_TOUCH_INTERRUPT);
    gpio_isr_handler_remove(BOARD_TOUCH_INTERRUPT);
    esp_sleep_disable_ext1_wakeup_io(1ULL << BOARD_TOUCH_INTERRUPT);
    portENTER_CRITICAL(&touch_lock);
    touch_pressed = false;
    touch_event = TOUCH_EVENT_IDLE;
    portEXIT_CRITICAL(&touch_lock);

    uint8_t output;
    uint8_t config;
    ESP_RETURN_ON_ERROR(i2c_read_registers(BOARD_XL9555_ADDR,
                                           BOARD_XL9555_OUTPUT1, &output, 1),
                        TAG, "touch reset output read failed");
    output &= ~(1U << BOARD_XL9555_TOUCH_RESET_BIT);
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_XL9555_ADDR,
                                            BOARD_XL9555_OUTPUT1, &output, 1),
                        TAG, "touch reset low failed");
    ESP_RETURN_ON_ERROR(i2c_read_registers(BOARD_XL9555_ADDR,
                                           BOARD_XL9555_CONFIG1, &config, 1),
                        TAG, "touch reset config read failed");
    config &= ~(1U << BOARD_XL9555_TOUCH_RESET_BIT);
    return i2c_write_registers(BOARD_XL9555_ADDR,
                               BOARD_XL9555_CONFIG1, &config, 1);
}

static esp_err_t cst9217_read_point(bool *pressed, uint16_t *x, uint16_t *y)
{
    uint8_t data[CST9217_MAX_POINTS * 5 + 5] = {0};
    const uint8_t command[] = {
        (uint8_t)(CST9217_REGISTER_TOUCH_DATA >> 8),
        (uint8_t)CST9217_REGISTER_TOUCH_DATA,
    };
    ESP_RETURN_ON_ERROR(i2c_transmit_receive(touch_address, command,
                                             sizeof(command), data,
                                             sizeof(data)),
                        TAG, "touch data read failed");

    *pressed = false;
    uint8_t points = data[5] & 0x7f;
    if (data[0] == CST9217_ACK || data[6] != CST9217_ACK || data[0] == 0x00 ||
        points == 0 || points > CST9217_MAX_POINTS) {
        return ESP_OK;
    }

    uint8_t event = data[0] & 0x0f;
    uint8_t id = data[0] >> 4;
    if (event == CST9217_EVENT_CONTACT && id < CST9217_MAX_POINTS) {
        *x = ((uint16_t)data[1] << 4) | (data[3] >> 4);
        *y = ((uint16_t)data[2] << 4) | (data[3] & 0x0f);
        *pressed = true;
    }
    return ESP_OK;
}

static bool touch_get_payload(uint8_t output[TOUCH_PAYLOAD_LENGTH])
{
    if (!touch_ready) {
        return false;
    }
    portENTER_CRITICAL(&touch_lock);
    output[0] = touch_pressed ? 1 : 0;
    output[1] = touch_event;
    encode_uint16_le(output + 2, touch_x);
    encode_uint16_le(output + 4, touch_y);
    portEXIT_CRITICAL(&touch_lock);
    return true;
}

static bool gps_get_payload(uint8_t output[GPS_PAYLOAD_LENGTH])
{
    gps_status_t status;
    if (!gps_get_status(&status)) {
        return false;
    }
    output[0] = status.ready ? 1 : 0;
    output[1] = status.fix_valid ? 1 : 0;
    output[2] = status.fix_type;
    output[3] = status.satellites;
    encode_int32_le(output + 4, status.latitude_e7);
    encode_int32_le(output + 8, status.longitude_e7);
    encode_int32_le(output + 12, status.altitude_mm);
    encode_uint32_le(output + 16, status.horizontal_accuracy_mm);
    encode_uint32_le(output + 20, status.ground_speed_mm_s);
    encode_int32_le(output + 24, status.heading_e5);
    encode_uint16_le(output + 28, status.agc);
    return true;
}

static esp_err_t rtc_read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_read_registers(BOARD_PCF85063_ADDR, reg, data, length);
}

static esp_err_t rtc_write_registers(uint8_t reg, const uint8_t *data,
                                     size_t length)
{
    return i2c_write_registers(BOARD_PCF85063_ADDR, reg, data, length);
}

static esp_err_t axp_read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_read_registers(BOARD_AXP2101_ADDR, reg, data, length);
}

static esp_err_t axp_write_register(uint8_t reg, uint8_t value)
{
    return i2c_write_registers(BOARD_AXP2101_ADDR, reg, &value, 1);
}

static int charge_current_from_code(uint8_t code)
{
    if (code <= 8) {
        return code * 25;
    }
    if (code <= 16) {
        return 200 + (code - 8) * 100;
    }
    return -1;
}

static int charge_current_to_code(unsigned milliamps)
{
    if (milliamps >= 25 && milliamps <= 200 && milliamps % 25 == 0) {
        return milliamps / 25;
    }
    if (milliamps >= 300 && milliamps <= 500 && milliamps % 100 == 0) {
        return 8 + (milliamps - 200) / 100;
    }
    return -1;
}

static int input_current_from_code(uint8_t code)
{
    static const int values[] = {100, 500, 900, 1000, 1500, 2000};
    return code < 6 ? values[code] : -1;
}

static int input_current_to_code(unsigned milliamps)
{
    static const unsigned values[] = {100, 500, 900, 1000, 1500, 2000};
    for (int code = 0; code < 6; code++) {
        if (values[code] == milliamps) {
            return code;
        }
    }
    return -1;
}

static int charge_voltage_from_code(uint8_t code)
{
    static const int values[] = {5000, 4000, 4100, 4200, 4350, 4400};
    return code < 6 ? values[code] : -1;
}

static esp_err_t axp_read_power_config(power_config_t *config)
{
    uint8_t input_current;
    uint8_t module_enable;
    uint8_t charge_current;
    uint8_t charge_voltage;
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_INPUT_CURRENT_REGISTER,
                                           &input_current, 1),
                        TAG, "input current read failed");
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_MODULE_ENABLE_REGISTER,
                                           &module_enable, 1),
                        TAG, "charger enable read failed");
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_CHARGE_CURRENT_REGISTER,
                                           &charge_current, 1),
                        TAG, "charge current read failed");
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_CHARGE_VOLTAGE_REGISTER,
                                           &charge_voltage, 1),
                        TAG, "charge voltage read failed");

    int decoded_charge =
        charge_current_from_code(charge_current & AXP_CHARGE_CURRENT_MASK);
    int decoded_input =
        input_current_from_code(input_current & AXP_INPUT_CURRENT_MASK);
    int decoded_voltage =
        charge_voltage_from_code(charge_voltage & AXP_CHARGE_VOLTAGE_MASK);
    if (decoded_charge < 0 || decoded_input < 0 || decoded_voltage < 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    config->charge_current_ma = decoded_charge;
    config->input_current_ma = decoded_input;
    config->charge_voltage_mv = decoded_voltage;
    config->charger_enabled =
        (module_enable & (1U << AXP_CELL_CHARGE_ENABLE_BIT)) != 0;
    return ESP_OK;
}

static esp_err_t axp_write_masked_register(uint8_t reg, uint8_t mask,
                                           uint8_t bits)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(axp_read_registers(reg, &value, 1),
                        TAG, "PMIC setting read failed");
    value = (value & ~mask) | (bits & mask);
    return axp_write_register(reg, value);
}

static esp_err_t axp_set_power_config(unsigned charge_current_ma,
                                      unsigned input_current_ma,
                                      bool charger_enabled)
{
    int charge_code = charge_current_to_code(charge_current_ma);
    int input_code = input_current_to_code(input_current_ma);
    if (charge_code < 0 || input_code < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t enable_mask = 1U << AXP_CELL_CHARGE_ENABLE_BIT;
    if (!charger_enabled) {
        ESP_RETURN_ON_ERROR(
            axp_write_masked_register(AXP_MODULE_ENABLE_REGISTER, enable_mask,
                                      0),
            TAG, "charger disable failed");
    }
    ESP_RETURN_ON_ERROR(
        axp_write_masked_register(AXP_CHARGE_CURRENT_REGISTER,
                                  AXP_CHARGE_CURRENT_MASK, charge_code),
        TAG, "charge current write failed");
    ESP_RETURN_ON_ERROR(
        axp_write_masked_register(AXP_INPUT_CURRENT_REGISTER,
                                  AXP_INPUT_CURRENT_MASK, input_code),
        TAG, "input current write failed");
    if (charger_enabled) {
        ESP_RETURN_ON_ERROR(
            axp_write_masked_register(AXP_MODULE_ENABLE_REGISTER, enable_mask,
                                      enable_mask),
            TAG, "charger enable failed");
    }

    power_config_t actual;
    ESP_RETURN_ON_ERROR(axp_read_power_config(&actual),
                        TAG, "power setting verification failed");
    if (actual.charge_current_ma != charge_current_ma ||
        actual.input_current_ma != input_current_ma ||
        actual.charger_enabled != charger_enabled) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t axp_initialize_measurement(void)
{
    uint8_t value;
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_CHIP_ID_REGISTER, &value, 1),
                        TAG, "PMIC chip ID read failed");
    if (value != AXP_CHIP_ID) {
        ESP_LOGE(TAG, "unexpected PMIC chip ID 0x%02x", value);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_ADC_CHANNEL_REGISTER,
                                           &value, 1),
                        TAG, "PMIC ADC control read failed");
    value |= 1U << AXP_BATTERY_ADC_ENABLE_BIT;
    ESP_RETURN_ON_ERROR(axp_write_register(AXP_ADC_CHANNEL_REGISTER, value),
                        TAG, "PMIC ADC enable failed");

    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_BATTERY_DETECT_REGISTER,
                                           &value, 1),
                        TAG, "PMIC battery detection read failed");
    value |= 1U << AXP_BATTERY_DETECT_BIT;
    return axp_write_register(AXP_BATTERY_DETECT_REGISTER, value);
}

static bool is_leap_year(int year)
{
    return year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
}

static bool rtc_time_is_valid(const rtc_datetime_t *time)
{
    static const uint8_t days_in_month[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    };

    if (time->year < 2000 || time->year > 2099 ||
        time->month < 1 || time->month > 12 ||
        time->hour < 0 || time->hour > 23 ||
        time->minute < 0 || time->minute > 59 ||
        time->second < 0 || time->second > 59) {
        return false;
    }

    int maximum_day = days_in_month[time->month - 1];
    if (time->month == 2 && is_leap_year(time->year)) {
        maximum_day++;
    }
    return time->day >= 1 && time->day <= maximum_day;
}

static bool parse_two_digits(const char *text, int *value)
{
    if (text[0] < '0' || text[0] > '9' ||
        text[1] < '0' || text[1] > '9') {
        return false;
    }
    *value = (text[0] - '0') * 10 + (text[1] - '0');
    return true;
}

static bool parse_rtc_payload(const char *text, size_t length,
                              rtc_datetime_t *time)
{
    if (length != RTC_PAYLOAD_LENGTH ||
        text[4] != '-' || text[7] != '-' ||
        (text[10] != 'T' && text[10] != ' ') ||
        text[13] != ':' || text[16] != ':') {
        return false;
    }

    int century;
    int year;
    if (!parse_two_digits(text, &century) ||
        !parse_two_digits(text + 2, &year) ||
        !parse_two_digits(text + 5, &time->month) ||
        !parse_two_digits(text + 8, &time->day) ||
        !parse_two_digits(text + 11, &time->hour) ||
        !parse_two_digits(text + 14, &time->minute) ||
        !parse_two_digits(text + 17, &time->second)) {
        return false;
    }
    time->year = century * 100 + year;
    return rtc_time_is_valid(time);
}

/* PCF85063A weekday encoding is Sunday=0 through Saturday=6. */
static int rtc_weekday(const rtc_datetime_t *time)
{
    static const int month_offsets[] = {
        0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4,
    };
    int year = time->year;
    if (time->month < 3) {
        year--;
    }
    return (year + year / 4 - year / 100 + year / 400 +
            month_offsets[time->month - 1] + time->day) % 7;
}

static esp_err_t rtc_read_time(rtc_datetime_t *time)
{
    uint8_t registers[7];
    ESP_RETURN_ON_ERROR(rtc_read_registers(RTC_SECONDS_REGISTER, registers,
                                           sizeof(registers)),
                        TAG, "RTC read failed");

    if (registers[0] & (1U << RTC_OSCILLATOR_STOP)) {
        return ESP_ERR_INVALID_STATE;
    }

    time->second = bcd_to_decimal(registers[0] & 0x7f);
    time->minute = bcd_to_decimal(registers[1] & 0x7f);
    time->hour = bcd_to_decimal(registers[2] & 0x3f);
    time->day = bcd_to_decimal(registers[3] & 0x3f);
    time->weekday = registers[4] & 0x07;
    time->month = bcd_to_decimal(registers[5] & 0x1f);
    time->year = 2000 + bcd_to_decimal(registers[6]);

    return rtc_time_is_valid(time) && time->weekday <= 6
               ? ESP_OK
               : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t rtc_write_time(const rtc_datetime_t *time)
{
    const uint8_t registers[] = {
        decimal_to_bcd(time->second),
        decimal_to_bcd(time->minute),
        decimal_to_bcd(time->hour),
        decimal_to_bcd(time->day),
        (uint8_t)rtc_weekday(time),
        decimal_to_bcd(time->month),
        decimal_to_bcd(time->year - 2000),
    };
    return rtc_write_registers(RTC_SECONDS_REGISTER, registers,
                               sizeof(registers));
}

esp_err_t ble_rtc_get_time_payload(char *output, size_t output_size)
{
    if (output == NULL || output_size < RTC_PAYLOAD_LENGTH + 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    rtc_datetime_t time;
    ESP_RETURN_ON_ERROR(rtc_read_time(&time), TAG, "RTC unavailable");
    int length = snprintf(output, output_size,
                          "%04d-%02d-%02dT%02d:%02d:%02d",
                          time.year, time.month, time.day,
                          time.hour, time.minute, time.second);
    return length == RTC_PAYLOAD_LENGTH ? ESP_OK : ESP_FAIL;
}

esp_err_t ble_rtc_get_datetime(rtc_datetime_t *datetime)
{
    if (datetime == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return rtc_read_time(datetime);
}

esp_err_t ble_power_get_payload(char *output, size_t output_size)
{
    if (output == NULL || output_size < POWER_PAYLOAD_MAX_LENGTH) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t status[2];
    uint8_t voltage[2];
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_STATUS1_REGISTER, status,
                                           sizeof(status)),
                        TAG, "PMIC status read failed");
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_BATTERY_VOLTAGE_HIGH, voltage,
                                           sizeof(voltage)),
                        TAG, "battery voltage read failed");

    const unsigned millivolts = ((voltage[0] & 0x3f) << 8) | voltage[1];
    const unsigned direction = (status[1] >> 5) & 0x03;
    const unsigned vbus_good = (status[0] >> 5) & 0x01;
    const unsigned battery_present = (status[0] >> 3) & 0x01;
    unsigned percentage = 0;
    if (battery_present && millivolts >= AXP_BATTERY_FULL_MV) {
        percentage = 100;
    } else if (battery_present && millivolts > AXP_BATTERY_EMPTY_MV) {
        percentage = ((millivolts - AXP_BATTERY_EMPTY_MV) * 100U) /
                     (AXP_BATTERY_FULL_MV - AXP_BATTERY_EMPTY_MV);
    }
    int length = snprintf(output, output_size, "%u,%u,%u,%u,%u",
                          percentage, millivolts, direction,
                          vbus_good, battery_present);
    return length > 0 && (size_t)length < output_size ? ESP_OK : ESP_FAIL;
}

static esp_err_t power_config_get_payload(char *output, size_t output_size)
{
    if (output == NULL || output_size < POWER_CONFIG_PAYLOAD_MAX_LENGTH) {
        return ESP_ERR_INVALID_SIZE;
    }
    power_config_t config;
    ESP_RETURN_ON_ERROR(axp_read_power_config(&config),
                        TAG, "power configuration unavailable");
    int length = snprintf(output, output_size, "%u,%u,%u,%u",
                          config.charge_current_ma, config.input_current_ma,
                          config.charge_voltage_mv,
                          config.charger_enabled ? 1 : 0);
    return length > 0 && (size_t)length < output_size ? ESP_OK : ESP_FAIL;
}

static bool parse_power_config_write(const char *payload,
                                     unsigned *charge_current_ma,
                                     unsigned *input_current_ma,
                                     bool *charger_enabled)
{
    unsigned enabled;
    char trailing;
    if (sscanf(payload, "%u,%u,%u%c", charge_current_ma, input_current_ma,
               &enabled, &trailing) != 3 || enabled > 1) {
        return false;
    }
    *charger_enabled = enabled != 0;
    return charge_current_to_code(*charge_current_ma) >= 0 &&
           input_current_to_code(*input_current_ma) >= 0;
}

static int rtc_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        char payload[RTC_PAYLOAD_LENGTH + 1];
        if (ble_rtc_get_time_payload(payload, sizeof(payload)) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        return os_mbuf_append(context->om, payload, RTC_PAYLOAD_LENGTH) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char payload[RTC_PAYLOAD_LENGTH + 1];
        uint16_t length = 0;
        int result = ble_hs_mbuf_to_flat(context->om, payload,
                                         RTC_PAYLOAD_LENGTH, &length);
        if (result != 0 || length != RTC_PAYLOAD_LENGTH) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        payload[length] = '\0';

        rtc_datetime_t time;
        if (!parse_rtc_payload(payload, length, &time)) {
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        if (rtc_write_time(&time) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "RTC set to %s", payload);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int power_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    char payload[POWER_PAYLOAD_MAX_LENGTH];
    if (ble_power_get_payload(payload, sizeof(payload)) != ESP_OK) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return os_mbuf_append(context->om, payload, strlen(payload)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int power_config_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *context,
                                    void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        char payload[POWER_CONFIG_PAYLOAD_MAX_LENGTH];
        if (power_config_get_payload(payload, sizeof(payload)) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        return os_mbuf_append(context->om, payload, strlen(payload)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char payload[POWER_CONFIG_WRITE_MAX_LENGTH + 1];
        uint16_t length = 0;
        int result = ble_hs_mbuf_to_flat(context->om, payload,
                                         POWER_CONFIG_WRITE_MAX_LENGTH,
                                         &length);
        if (result != 0 || length == 0 ||
            length > POWER_CONFIG_WRITE_MAX_LENGTH) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        payload[length] = '\0';

        unsigned charge_current_ma;
        unsigned input_current_ma;
        bool charger_enabled;
        if (!parse_power_config_write(payload, &charge_current_ma,
                                      &input_current_ma, &charger_enabled)) {
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        if (axp_set_power_config(charge_current_ma, input_current_ma,
                                 charger_enabled) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "power config set: %umA charge, %umA input, %s",
                 charge_current_ma, input_current_ma,
                 charger_enabled ? "enabled" : "disabled");
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int imu_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    uint8_t payload[IMU_PAYLOAD_LENGTH];
    if (!imu_get_payload(payload)) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return os_mbuf_append(context->om, payload, sizeof(payload)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int screen_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        const uint8_t percentage = screen_get_brightness();
        return os_mbuf_append(context->om, &percentage, 1) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t percentage;
        uint16_t length = 0;
        int result = ble_hs_mbuf_to_flat(context->om, &percentage, 1,
                                         &length);
        if (result != 0 || length != 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (percentage > 100) {
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        if (screen_set_brightness(percentage) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "screen brightness set to %u%%", percentage);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int touch_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    uint8_t payload[TOUCH_PAYLOAD_LENGTH];
    if (!touch_get_payload(payload)) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return os_mbuf_append(context->om, payload, sizeof(payload)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int gps_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    uint8_t payload[GPS_PAYLOAD_LENGTH];
    if (!gps_get_payload(payload)) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return os_mbuf_append(context->om, payload, sizeof(payload)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int haptic_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t payload[HAPTIC_STATUS_PAYLOAD_LENGTH];
        if (!haptic_get_payload(payload)) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        return os_mbuf_append(context->om, payload, sizeof(payload)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t payload[HAPTIC_COMMAND_PAYLOAD_LENGTH];
        uint16_t length = 0;
        int result = ble_hs_mbuf_to_flat(context->om, payload,
                                         sizeof(payload), &length);
        if (result != 0 || length != sizeof(payload)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if ((payload[0] == 0 && payload[1] != 0) ||
            payload[0] > DRV2605_MAX_EFFECT ||
            (payload[0] != 0 &&
             (payload[1] == 0 || payload[1] > DRV2605_MAX_REPEATS))) {
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        if (drv2605_play(payload[0], payload[1]) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        if (payload[0] == 0) {
            ESP_LOGI(TAG, "haptic stopped via BLE");
        } else {
            ESP_LOGI(TAG, "haptic effect %u x%u via BLE",
                     payload[0], payload[1]);
        }
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int sensor_control_gatt_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t payload[SENSOR_CONTROL_PAYLOAD_LENGTH];
        sensor_control_get_payload(payload);
        return os_mbuf_append(context->om, payload, sizeof(payload)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t mask;
        uint16_t length = 0;
        int result = ble_hs_mbuf_to_flat(context->om, &mask, 1, &length);
        if (result != 0 || length != 1) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if ((mask & ~SENSOR_ALL_MASK) != 0) {
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        if (sensor_control_store(mask) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        requested_sensor_mask = mask;
        if ((mask & SENSOR_GPS_BIT) == 0) {
            gps_cancel_initialize();
        }
        sensor_control_wake_tasks();
        sensor_control_notify();
        ESP_LOGI(TAG, "sensor state stored: IMU %s, GPS %s, touch %s",
                 mask & SENSOR_IMU_BIT ? "on" : "off",
                 mask & SENSOR_GPS_BIT ? "on" : "off",
                 mask & SENSOR_TOUCH_BIT ? "on" : "off");
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int system_power_gatt_access(
    uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t payload[SYSTEM_POWER_PAYLOAD_LENGTH];
        system_power_get_payload(payload);
        return os_mbuf_append(context->om, payload, sizeof(payload)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t payload[SYSTEM_POWER_PAYLOAD_LENGTH];
        uint16_t length = 0;
        int result = ble_hs_mbuf_to_flat(context->om, payload,
                                         sizeof(payload), &length);
        if (result != 0 || length != sizeof(payload)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint16_t frequency_mhz = (uint16_t)payload[0] |
                                 (uint16_t)payload[1] << 8;
        if (!system_power_frequency_valid(frequency_mhz) || payload[2] > 1) {
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        if (system_power_store(frequency_mhz, payload[2] != 0) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int alarm_gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        alarm_config_t config;
        ble_alarm_get_config(&config);
        const uint8_t payload[ALARM_PAYLOAD_LENGTH] = {
            config.hour, config.minute, config.enabled ? 1 : 0,
        };
        return os_mbuf_append(context->om, payload, sizeof(payload)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t payload[ALARM_PAYLOAD_LENGTH];
        uint16_t length = 0;
        int result = ble_hs_mbuf_to_flat(context->om, payload,
                                         sizeof(payload), &length);
        if (result != 0 || length != sizeof(payload)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (payload[0] > 23 || payload[1] > 59 || payload[2] > 1) {
            return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
        }
        return ble_alarm_set(payload[0], payload[1], payload[2] != 0) == ESP_OK
                   ? 0
                   : BLE_ATT_ERR_UNLIKELY;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def rtc_gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &rtc_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &rtc_characteristic_uuid.u,
                .access_cb = rtc_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &rtc_value_handle,
            },
            {
                .uuid = &power_characteristic_uuid.u,
                .access_cb = power_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &power_value_handle,
            },
            {
                .uuid = &power_config_characteristic_uuid.u,
                .access_cb = power_config_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &imu_characteristic_uuid.u,
                .access_cb = imu_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &imu_value_handle,
            },
            {
                .uuid = &screen_characteristic_uuid.u,
                .access_cb = screen_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &touch_characteristic_uuid.u,
                .access_cb = touch_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &touch_value_handle,
            },
            {
                .uuid = &gps_characteristic_uuid.u,
                .access_cb = gps_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gps_value_handle,
            },
            {
                .uuid = &haptic_characteristic_uuid.u,
                .access_cb = haptic_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &sensor_control_characteristic_uuid.u,
                .access_cb = sensor_control_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &sensor_control_value_handle,
            },
            {
                .uuid = &system_power_characteristic_uuid.u,
                .access_cb = system_power_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &alarm_characteristic_uuid.u,
                .access_cb = alarm_gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {0},
        },
    },
    {0},
};

static void start_advertising(void);

bool ble_rtc_advertising_enabled(void)
{
    return advertising_enabled;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            connection_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE client connected");
            if (!advertising_enabled) {
                int result = ble_gap_terminate(event->connect.conn_handle,
                                               BLE_ERR_REM_USER_CONN_TERM);
                if (result != 0) {
                    ESP_LOGW(TAG,
                             "late BLE connection termination failed: %d",
                             result);
                }
            }
        } else {
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        rtc_notifications_enabled = false;
        power_notifications_enabled = false;
        imu_notifications_enabled = false;
        touch_notifications_enabled = false;
        gps_notifications_enabled = false;
        sensor_control_notifications_enabled = false;
        ESP_LOGI(TAG, "BLE client disconnected");
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == rtc_value_handle) {
            rtc_notifications_enabled = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == power_value_handle) {
            power_notifications_enabled = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == imu_value_handle) {
            imu_notifications_enabled = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == touch_value_handle) {
            touch_notifications_enabled = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == gps_value_handle) {
            gps_notifications_enabled = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle ==
                   sensor_control_value_handle) {
            sensor_control_notifications_enabled =
                event->subscribe.cur_notify;
        }
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;
    default:
        return 0;
    }
}

static void start_advertising(void)
{
    if (!advertising_enabled || connection_handle != BLE_HS_CONN_HANDLE_NONE ||
        ble_gap_adv_active()) {
        return;
    }
    const struct ble_hs_adv_fields advertising_fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .uuids128 = (ble_uuid128_t *)&rtc_service_uuid,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    int result = ble_gap_adv_set_fields(&advertising_fields);
    if (result != 0) {
        ESP_LOGE(TAG, "advertising fields failed: %d", result);
        return;
    }

    const struct ble_hs_adv_fields scan_response_fields = {
        .name = (uint8_t *)DEVICE_NAME,
        .name_len = strlen(DEVICE_NAME),
        .name_is_complete = 1,
    };
    result = ble_gap_adv_rsp_set_fields(&scan_response_fields);
    if (result != 0) {
        ESP_LOGE(TAG, "scan response failed: %d", result);
        return;
    }

    const struct ble_gap_adv_params parameters = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    result = ble_gap_adv_start(own_address_type, NULL, BLE_HS_FOREVER,
                               &parameters, gap_event, NULL);
    if (result != 0) {
        ESP_LOGE(TAG, "advertising start failed: %d", result);
    } else {
        ESP_LOGI(TAG, "advertising as %s", DEVICE_NAME);
    }
}

esp_err_t ble_rtc_set_advertising_enabled(bool enabled)
{
    advertising_enabled = enabled;
    if (enabled) {
        start_advertising();
        if (connection_handle == BLE_HS_CONN_HANDLE_NONE &&
            !ble_gap_adv_active()) {
            return ESP_FAIL;
        }
    } else {
        esp_err_t status = ESP_OK;
        if (ble_gap_adv_active()) {
            int result = ble_gap_adv_stop();
            if (result != 0) {
                ESP_LOGW(TAG, "advertising stop failed: %d", result);
                status = ESP_FAIL;
            }
        }
        uint16_t handle = connection_handle;
        if (handle != BLE_HS_CONN_HANDLE_NONE) {
            int result = ble_gap_terminate(handle,
                                           BLE_ERR_REM_USER_CONN_TERM);
            if (result != 0 && result != BLE_HS_ENOTCONN) {
                ESP_LOGW(TAG, "BLE disconnect request failed: %d", result);
                status = ESP_FAIL;
            } else {
                ESP_LOGI(TAG, "BLE disconnect requested");
            }
        }
        if (status != ESP_OK) {
            return status;
        }
    }
    ESP_LOGI(TAG, "BLE advertising %s", enabled ? "enabled" : "disabled");
    screen_request_refresh();
    return ESP_OK;
}

static void IRAM_ATTR power_button_interrupt_handler(void *argument)
{
    (void)argument;
    BaseType_t higher_priority_task_woken = pdFALSE;
    gpio_ll_intr_disable(GPIO_LL_GET_HW(0), BOARD_PMU_INTERRUPT);
    if (power_button_task_handle != NULL) {
        vTaskNotifyGiveFromISR(power_button_task_handle,
                               &higher_priority_task_woken);
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void power_button_task(void *parameter)
{
    (void)parameter;
    bool long_press_seen = false;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t status[3];
        esp_err_t result = axp_read_registers(AXP_IRQ_STATUS0_REGISTER,
                                              status, sizeof(status));
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "PMIC interrupt status read failed: %s",
                     esp_err_to_name(result));
            xTaskNotifyGive(power_button_task_handle);
            continue;
        }
        result = i2c_write_registers(BOARD_AXP2101_ADDR,
                                     AXP_IRQ_STATUS0_REGISTER,
                                     status, sizeof(status));
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "PMIC interrupt status clear failed: %s",
                     esp_err_to_name(result));
        }
        ESP_LOGI(TAG, "PMIC IRQ status %02x %02x %02x",
                 status[0], status[1], status[2]);
        if ((status[1] & (1U << AXP_POWERON_LONG_PRESS_BIT)) != 0) {
            long_press_seen = true;
        }
        if ((status[1] & (1U << AXP_POWERON_POSITIVE_EDGE_BIT)) != 0) {
            if (!long_press_seen) {
                esp_err_t toggle_result =
                    ble_rtc_set_advertising_enabled(!advertising_enabled);
                if (toggle_result != ESP_OK) {
                    ESP_LOGW(TAG, "BLE advertising toggle failed: %s",
                             esp_err_to_name(toggle_result));
                }
            }
            long_press_seen = false;
        }
        if (gpio_get_level(BOARD_PMU_INTERRUPT) == 1) {
            gpio_intr_enable(BOARD_PMU_INTERRUPT);
        } else {
            xTaskNotifyGive(power_button_task_handle);
        }
    }
}

static esp_err_t power_button_initialize(void)
{
    gpio_config_t interrupt_config = {
        .pin_bit_mask = 1ULL << BOARD_PMU_INTERRUPT,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&interrupt_config), TAG,
                        "PMIC interrupt GPIO setup failed");

    uint8_t irq_enable;
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_IRQ_ENABLE1_REGISTER,
                                           &irq_enable, 1),
                        TAG, "PMIC interrupt enable read failed");
    irq_enable |= (1U << AXP_POWERON_SHORT_PRESS_BIT) |
                  (1U << AXP_POWERON_LONG_PRESS_BIT) |
                  (1U << AXP_POWERON_POSITIVE_EDGE_BIT);
    ESP_RETURN_ON_ERROR(axp_write_register(AXP_IRQ_ENABLE1_REGISTER,
                                           irq_enable),
                        TAG, "power-button interrupt enable failed");

    uint8_t pending[3];
    ESP_RETURN_ON_ERROR(axp_read_registers(AXP_IRQ_STATUS0_REGISTER,
                                           pending, sizeof(pending)),
                        TAG, "PMIC pending interrupt read failed");
    ESP_RETURN_ON_ERROR(i2c_write_registers(BOARD_AXP2101_ADDR,
                                            AXP_IRQ_STATUS0_REGISTER,
                                            pending, sizeof(pending)),
                        TAG, "PMIC pending interrupt clear failed");

    esp_err_t isr_result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
        return isr_result;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BOARD_PMU_INTERRUPT,
                                             power_button_interrupt_handler,
                                             NULL),
                        TAG, "power-button interrupt handler failed");
    return esp_sleep_enable_ext1_wakeup_io(1ULL << BOARD_PMU_INTERRUPT,
                                           ESP_EXT1_WAKEUP_ANY_LOW);
}

static void host_sync(void)
{
    int result = ble_hs_id_infer_auto(0, &own_address_type);
    if (result != 0) {
        ESP_LOGE(TAG, "address inference failed: %d", result);
        return;
    }
    start_advertising();
}

static void host_task(void *parameter)
{
    (void)parameter;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void notify_task(void *parameter)
{
    (void)parameter;
    unsigned seconds = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        seconds++;
        uint16_t handle = connection_handle;
        if (handle != BLE_HS_CONN_HANDLE_NONE && rtc_notifications_enabled &&
            seconds % 60 == 0) {
            int result = ble_gatts_notify(handle, rtc_value_handle);
            if (result != 0 && result != BLE_HS_ENOTCONN) {
                ESP_LOGD(TAG, "RTC notification skipped: %d", result);
            }
        }
        if (handle != BLE_HS_CONN_HANDLE_NONE && gps_notifications_enabled) {
            int result = ble_gatts_notify(handle, gps_value_handle);
            if (result != 0 && result != BLE_HS_ENOTCONN) {
                ESP_LOGD(TAG, "GPS notification skipped: %d", result);
            }
        }
        if (handle != BLE_HS_CONN_HANDLE_NONE &&
            power_notifications_enabled && seconds % 5 == 0) {
            int result = ble_gatts_notify(handle, power_value_handle);
            if (result != 0 && result != BLE_HS_ENOTCONN) {
                ESP_LOGD(TAG, "power notification skipped: %d", result);
            }
        }
    }
}

static void imu_task(void *parameter)
{
    (void)parameter;
    uint8_t work_buffer[IMU_FIFO_BUFFER_SIZE];
    for (;;) {
        if (imu_ready &&
            gpio_get_level(BOARD_BHI260_INTERRUPT) != 0) {
            int8_t result = bhy2_get_and_process_fifo(
                work_buffer, sizeof(work_buffer), &imu_device);
            if (result != BHY2_OK) {
                ESP_LOGW(TAG, "BHI260 FIFO read failed: %d", result);
            } else {
                uint16_t handle = connection_handle;
                if (handle != BLE_HS_CONN_HANDLE_NONE &&
                    imu_notifications_enabled && imu_sample_available) {
                    int notify_result = ble_gatts_notify(handle,
                                                         imu_value_handle);
                    if (notify_result != 0 &&
                        notify_result != BLE_HS_ENOTCONN) {
                        ESP_LOGD(TAG, "IMU notification skipped: %d",
                                 notify_result);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void touch_task(void *parameter)
{
    (void)parameter;
    unsigned read_failures = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!touch_ready) {
            continue;
        }
        screen_wake_from_touch();
        bool pressed;
        uint16_t x = 0;
        uint16_t y = 0;
        esp_err_t result = cst9217_read_point(&pressed, &x, &y);
        if (result != ESP_OK) {
            if (read_failures++ % 50 == 0) {
                ESP_LOGW(TAG, "CST9217 read failed: %s",
                         esp_err_to_name(result));
            }
            if (gpio_get_level(BOARD_TOUCH_INTERRUPT) == 0) {
                vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_INTERVAL_MS));
                xTaskNotifyGive(touch_task_handle);
            }
            continue;
        }
        read_failures = 0;

        bool notify = false;
        uint16_t ui_x = 0;
        uint16_t ui_y = 0;
        uint8_t ui_event = TOUCH_EVENT_IDLE;
        portENTER_CRITICAL(&touch_lock);
        if (pressed && !touch_pressed) {
            touch_event = TOUCH_EVENT_DOWN;
            touch_x = x;
            touch_y = y;
            notify = true;
        } else if (pressed && touch_pressed &&
                   (x != touch_x || y != touch_y)) {
            touch_event = TOUCH_EVENT_MOVE;
            touch_x = x;
            touch_y = y;
            notify = true;
        } else if (!pressed && touch_pressed) {
            touch_event = TOUCH_EVENT_UP;
            notify = true;
        }
        if (notify) {
            ui_x = touch_x;
            ui_y = touch_y;
            ui_event = touch_event;
        }
        touch_pressed = pressed;
        portEXIT_CRITICAL(&touch_lock);

        if (notify) {
            screen_handle_touch((screen_touch_event_t)ui_event, ui_x, ui_y);
        }

        uint16_t handle = connection_handle;
        if (notify && handle != BLE_HS_CONN_HANDLE_NONE &&
            touch_notifications_enabled) {
            int notify_result = ble_gatts_notify(handle, touch_value_handle);
            if (notify_result != 0 && notify_result != BLE_HS_ENOTCONN) {
                ESP_LOGD(TAG, "touch notification skipped: %d",
                         notify_result);
            }
        }
    }
}

static void imu_control_task(void *parameter)
{
    (void)parameter;
    for (;;) {
        bool enabled = (requested_sensor_mask & SENSOR_IMU_BIT) != 0;
        if (enabled && !imu_ready) {
            esp_err_t result = sensor_rail_set(
                BOARD_AXP2101_ALDO4_VOLTAGE, 13,
                BOARD_AXP2101_ALDO4_BIT, true);
            if (result == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(50));
                imu_sample_available = false;
                imu_first_sample_logged = false;
                result = bhi260_initialize();
            }
            if (result != ESP_OK) {
                imu_ready = false;
                sensor_rail_set(BOARD_AXP2101_ALDO4_VOLTAGE, 13,
                                BOARD_AXP2101_ALDO4_BIT, false);
                ESP_LOGW(TAG, "IMU enable failed: %s",
                         esp_err_to_name(result));
            }
        } else if (!enabled) {
            imu_ready = false;
            imu_sample_available = false;
            esp_err_t result = sensor_rail_set(
                BOARD_AXP2101_ALDO4_VOLTAGE, 13,
                BOARD_AXP2101_ALDO4_BIT, false);
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "IMU power-down failed: %s",
                         esp_err_to_name(result));
            } else {
                ESP_LOGI(TAG, "IMU powered down");
            }
        }
        sensor_control_notify();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

static void gps_control_task(void *parameter)
{
    (void)parameter;
    for (;;) {
        bool enabled = (requested_sensor_mask & SENSOR_GPS_BIT) != 0;
        gps_status_t status;
        bool ready = gps_get_status(&status) && status.ready;
        if (enabled && !ready) {
            esp_err_t result = sensor_rail_set(
                BOARD_AXP2101_BLDO1_VOLTAGE, 28,
                BOARD_AXP2101_BLDO1_BIT, true);
            if (result == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(100));
                result = gps_initialize();
            }
            if (result != ESP_OK) {
                gps_deinitialize();
                sensor_rail_set(BOARD_AXP2101_BLDO1_VOLTAGE, 28,
                                BOARD_AXP2101_BLDO1_BIT, false);
                if ((requested_sensor_mask & SENSOR_GPS_BIT) != 0) {
                    ESP_LOGW(TAG, "GPS enable failed: %s",
                             esp_err_to_name(result));
                }
            }
        } else if (!enabled) {
            gps_cancel_initialize();
            gps_deinitialize();
            esp_err_t result = sensor_rail_set(
                BOARD_AXP2101_BLDO1_VOLTAGE, 28,
                BOARD_AXP2101_BLDO1_BIT, false);
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "GPS power-down failed: %s",
                         esp_err_to_name(result));
            } else {
                ESP_LOGI(TAG, "GPS powered down");
            }
        }
        sensor_control_notify();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

static void touch_control_task(void *parameter)
{
    (void)parameter;
    for (;;) {
        bool enabled = (requested_sensor_mask & SENSOR_TOUCH_BIT) != 0;
        if (enabled && !touch_ready) {
            esp_err_t result = cst9217_initialize();
            if (result != ESP_OK) {
                cst9217_disable();
                ESP_LOGW(TAG, "touch enable failed: %s",
                         esp_err_to_name(result));
            }
        } else if (!enabled) {
            esp_err_t result = cst9217_disable();
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "touch disable failed: %s",
                         esp_err_to_name(result));
            } else {
                ESP_LOGI(TAG, "touch held in reset");
            }
        }
        sensor_control_notify();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

esp_err_t ble_rtc_initialize(void)
{
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uint8_t control1;
    ESP_RETURN_ON_ERROR(rtc_read_registers(RTC_CONTROL1_REGISTER,
                                           &control1, 1),
                        TAG, "RTC control read failed");
    control1 &= ~(1U << RTC_24_HOUR_BIT);
    ESP_RETURN_ON_ERROR(rtc_write_registers(RTC_CONTROL1_REGISTER,
                                            &control1, 1),
                        TAG, "RTC control write failed");
    return ESP_OK;
}

esp_err_t ble_rtc_start(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        result = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(result, TAG, "NVS init failed");
    ESP_RETURN_ON_ERROR(sensor_control_load(), TAG,
                        "sensor preference load failed");
    esp_err_t system_power_result = system_power_load();
    if (system_power_result != ESP_OK) {
        system_power_light_sleep = false;
        ESP_LOGW(TAG,
                 "stored ESP32 power state unavailable; continuing with "
                 "the compiled clock configuration: %s",
                 esp_err_to_name(system_power_result));
    }
    ESP_LOGI(TAG, "restoring sensor mask 0x%02x", requested_sensor_mask);

    ESP_RETURN_ON_ERROR(axp_initialize_measurement(),
                        TAG, "PMIC measurement initialization failed");
    power_config_t config;
    esp_err_t config_result = axp_read_power_config(&config);
    if (config_result == ESP_OK) {
        ESP_LOGI(TAG, "power config: %umA charge, %umA input, %umV, %s",
                 config.charge_current_ma, config.input_current_ma,
                 config.charge_voltage_mv,
                 config.charger_enabled ? "enabled" : "disabled");
    } else {
        ESP_LOGW(TAG, "power configuration unavailable: %s",
                 esp_err_to_name(config_result));
    }
    esp_err_t haptic_result = drv2605_initialize();
    if (haptic_result != ESP_OK) {
        ESP_LOGW(TAG, "haptic unavailable; other features remain active: %s",
                 esp_err_to_name(haptic_result));
    }
    ESP_RETURN_ON_ERROR(alarm_initialize(), TAG,
                        "alarm initialization failed");

    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "NimBLE init failed");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);

    int ble_result = ble_gatts_count_cfg(rtc_gatt_services);
    if (ble_result == 0) {
        ble_result = ble_gatts_add_svcs(rtc_gatt_services);
    }
    if (ble_result != 0) {
        return ESP_FAIL;
    }

    ble_hs_cfg.sync_cb = host_sync;
    nimble_port_freertos_init(host_task);
    if (xTaskCreate(notify_task, "rtc_notify", 3072, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(imu_task, "imu", 4096, NULL, 6, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(touch_task, "touch", 3072, NULL, 5,
                    &touch_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(power_button_task, "power_button", 3072, NULL, 5,
                    &power_button_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_RETURN_ON_ERROR(power_button_initialize(), TAG,
                        "power-button initialization failed");
    ESP_RETURN_ON_ERROR(system_power_apply(system_power_max_mhz,
                                            system_power_light_sleep),
                        TAG, "saved ESP32 power state apply failed");
    if (xTaskCreate(imu_control_task, "imu_control", 4096, NULL, 5,
                    &imu_control_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(gps_control_task, "gps_control", 4096, NULL, 5,
                    &gps_control_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(touch_control_task, "touch_control", 3072, NULL, 5,
                    &touch_control_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
