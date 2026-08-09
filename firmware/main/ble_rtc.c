#include "ble_rtc.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define RTC_CONTROL1_REGISTER 0x00
#define RTC_SECONDS_REGISTER  0x04
#define RTC_24_HOUR_BIT       5
#define RTC_OSCILLATOR_STOP   7
#define RTC_PAYLOAD_LENGTH    19
#define POWER_PAYLOAD_MAX_LENGTH 20
#define POWER_CONFIG_PAYLOAD_MAX_LENGTH 20
#define POWER_CONFIG_WRITE_MAX_LENGTH 12
#define IMU_PAYLOAD_LENGTH 10
#define IMU_FIFO_BUFFER_SIZE 1024
#define IMU_SAMPLE_RATE_HZ 25.0f

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
#define AXP_CHIP_ID                0x4a
#define AXP_INPUT_CURRENT_MASK     0x07
#define AXP_CELL_CHARGE_ENABLE_BIT 1
#define AXP_CHARGE_CURRENT_MASK    0x1f
#define AXP_CHARGE_VOLTAGE_MASK    0x07
#define AXP_BATTERY_ADC_ENABLE_BIT 0
#define AXP_BATTERY_DETECT_BIT     0
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

typedef struct {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} rtc_time_t;

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
static volatile uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool rtc_notifications_enabled;
static volatile bool power_notifications_enabled;
static volatile bool imu_notifications_enabled;
static volatile bool imu_ready;
static volatile bool imu_sample_available;
static bool imu_first_sample_logged;
static portMUX_TYPE imu_lock = portMUX_INITIALIZER_UNLOCKED;
static struct bhy2_dev imu_device;
static struct bhy2_data_quaternion imu_quaternion = { .w = 16384 };

extern const uint8_t
    bhi260_firmware_start[] asm("_binary_BHI260AP_fw_start");
extern const uint8_t
    bhi260_firmware_end[] asm("_binary_BHI260AP_fw_end");

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
    uint8_t buffer[8];
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

static bool rtc_time_is_valid(const rtc_time_t *time)
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
                              rtc_time_t *time)
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
static int rtc_weekday(const rtc_time_t *time)
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

static esp_err_t rtc_read_time(rtc_time_t *time)
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
    time->month = bcd_to_decimal(registers[5] & 0x1f);
    time->year = 2000 + bcd_to_decimal(registers[6]);

    return rtc_time_is_valid(time) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t rtc_write_time(const rtc_time_t *time)
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
    rtc_time_t time;
    ESP_RETURN_ON_ERROR(rtc_read_time(&time), TAG, "RTC unavailable");
    int length = snprintf(output, output_size,
                          "%04d-%02d-%02dT%02d:%02d:%02d",
                          time.year, time.month, time.day,
                          time.hour, time.minute, time.second);
    return length == RTC_PAYLOAD_LENGTH ? ESP_OK : ESP_FAIL;
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

        rtc_time_t time;
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
            {0},
        },
    },
    {0},
};

static void start_advertising(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            connection_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE client connected");
        } else {
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        rtc_notifications_enabled = false;
        power_notifications_enabled = false;
        imu_notifications_enabled = false;
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
        if (handle != BLE_HS_CONN_HANDLE_NONE && rtc_notifications_enabled) {
            int result = ble_gatts_notify(handle, rtc_value_handle);
            if (result != 0 && result != BLE_HS_ENOTCONN) {
                ESP_LOGD(TAG, "RTC notification skipped: %d", result);
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
    esp_err_t imu_result = bhi260_initialize();
    if (imu_result != ESP_OK) {
        ESP_LOGW(TAG, "IMU unavailable; RTC and power remain active: %s",
                 esp_err_to_name(imu_result));
    }
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
    return ESP_OK;
}
