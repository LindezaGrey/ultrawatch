#include <stddef.h>
#include <inttypes.h>
#include <stdatomic.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "twatch_board.h"

static const char *TAG = "twatch_board";
// V1.0 schematic and pinned variant; see docs/bringup/phase-2.md.
enum { BOARD_SDA = 3, BOARD_SCL = 2, PMU_ADDRESS = 0x34, EXPANDER_ADDRESS = 0x20,
       TOUCH_PRIMARY_ADDRESS = 0x1a, TOUCH_FALLBACK_ADDRESS = 0x5a,
       RTC_CANDIDATE_ADDRESS = 0x51, BHI_CANDIDATE_ADDRESS = 0x28,
       I2C_SPEED_HZ = 100000, I2C_TIMEOUT_MS = 100 };
// Pinned XPowersAXP2101/AXP2101Constants, bundle c8e6de3b.
enum { PMU_STATUS1 = 0x00, PMU_CHIP_ID = 0x03, PMU_EXPECTED_ID = 0x4a,
       PMU_POWEROFF_ENABLE = 0x22, PMU_ADC_CONTROL = 0x30,
       PMU_BATTERY_ADC_HIGH = 0x34, PMU_IRQ_STATUS1 = 0x48,
       STATUS_VBUS_GOOD = 1U << 5, STATUS_BATTERY_PRESENT = 1U << 3,
       STATUS_VBUS_ABSENT = 1U << 3, STATUS_CHARGE_SHIFT = 5,
       STATUS_CHARGING = 1, CHARGER_STATE_MASK = 7,
       ADC_BATTERY_ENABLE = 1U << 0, BATTERY_ADC_HIGH_MASK = 0x1f };
enum { PMU_DCDC_CONTROL = 0x80, PMU_DC1_VOLTAGE = 0x82,
       PMU_LDO_CONTROL = 0x90, PMU_ALDO2_VOLTAGE = 0x93, PMU_ALDO4_VOLTAGE = 0x95,
       DC1_ENABLE = 1U << 0, ALDO2_ENABLE = 1U << 1, ALDO4_ENABLE = 1U << 3,
       VOLTAGE_CODE_MASK = 0x1f,
       DC1_CODE_MAX = 19, ALDO2_CODE_MAX = 30, DC1_MIN_MV = 1500,
       ALDO2_MIN_MV = 500, VOLTAGE_STEP_MV = 100,
       XL_OUTPUT0 = 0x02, XL_POLARITY0 = 0x04, XL_CONFIG0 = 0x06,
       XL_DISPLAY_ENABLE_P07 = 1U << 7, XL_TOUCH_RESET_P10 = 1U << 0 };
enum { REQUIRED_DISPLAY_MV = 3300, REQUIRED_MCU_MV = 3300 };
enum { REQUIRED_SENSOR_MV = 1800, ALDO4_CODE = (REQUIRED_SENSOR_MV - ALDO2_MIN_MV) / VOLTAGE_STEP_MV };

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t pmu;
static i2c_master_dev_handle_t expander;
static i2c_master_dev_handle_t touch;
static i2c_master_dev_handle_t rtc;
static i2c_master_dev_handle_t bhi;
static bool ready;
static atomic_uint_fast32_t transfer_sequence;

static unsigned device_address(i2c_master_dev_handle_t device)
{
    if (device == pmu) return PMU_ADDRESS;
    if (device == expander) return EXPANDER_ADDRESS;
    if (device == touch) return TOUCH_PRIMARY_ADDRESS;
    if (device == rtc) return RTC_CANDIDATE_ADDRESS;
    if (device == bhi) return BHI_CANDIDATE_ADDRESS;
    return 0; // Unknown handle; never invent an address for diagnostic output.
}

static esp_err_t timed_transmit_receive(i2c_master_dev_handle_t device,
                                        const uint8_t *write_data, size_t write_size,
                                        uint8_t *read_data, size_t read_size)
{
    const uint32_t sequence = (uint32_t)atomic_fetch_add_explicit(&transfer_sequence, 1,
                                                                    memory_order_relaxed) + 1;
    ESP_LOGI(TAG, "I2C_BEGIN seq=%" PRIu32 " kind=read task=%s priority=%u addr=0x%02x "
             "write_bytes=%u read_bytes=%u", sequence, pcTaskGetName(NULL),
             (unsigned)uxTaskPriorityGet(NULL), device_address(device),
             (unsigned)write_size, (unsigned)read_size);
    const int64_t started_us = esp_timer_get_time();
    const esp_err_t result = i2c_master_transmit_receive(device, write_data, write_size,
                                                         read_data, read_size, I2C_TIMEOUT_MS);
    const int64_t elapsed_us = esp_timer_get_time() - started_us;
    if (result != ESP_OK || elapsed_us >= 50000) {
        ESP_LOGW(TAG, "I2C_XFER kind=read task=%s priority=%u addr=0x%02x "
                 "write_bytes=%u read_bytes=%u elapsed_us=%" PRId64
                 " result=%s lock_timeout=%u",
                 pcTaskGetName(NULL), (unsigned)uxTaskPriorityGet(NULL), device_address(device),
                 (unsigned)write_size, (unsigned)read_size, elapsed_us,
                 esp_err_to_name(result), result == ESP_ERR_TIMEOUT);
    }
    ESP_LOGI(TAG, "I2C_END seq=%" PRIu32 " result=%s elapsed_us=%" PRId64,
             sequence, esp_err_to_name(result), elapsed_us);
    return result;
}

static esp_err_t timed_transmit(i2c_master_dev_handle_t device,
                                const uint8_t *data, size_t size)
{
    const uint32_t sequence = (uint32_t)atomic_fetch_add_explicit(&transfer_sequence, 1,
                                                                    memory_order_relaxed) + 1;
    ESP_LOGI(TAG, "I2C_BEGIN seq=%" PRIu32 " kind=write task=%s priority=%u addr=0x%02x "
             "write_bytes=%u read_bytes=0", sequence, pcTaskGetName(NULL),
             (unsigned)uxTaskPriorityGet(NULL), device_address(device), (unsigned)size);
    const int64_t started_us = esp_timer_get_time();
    const esp_err_t result = i2c_master_transmit(device, data, size, I2C_TIMEOUT_MS);
    const int64_t elapsed_us = esp_timer_get_time() - started_us;
    if (result != ESP_OK || elapsed_us >= 50000) {
        ESP_LOGW(TAG, "I2C_XFER kind=write task=%s priority=%u addr=0x%02x "
                 "write_bytes=%u read_bytes=0 elapsed_us=%" PRId64
                 " result=%s lock_timeout=%u",
                 pcTaskGetName(NULL), (unsigned)uxTaskPriorityGet(NULL), device_address(device),
                 (unsigned)size, elapsed_us, esp_err_to_name(result),
                 result == ESP_ERR_TIMEOUT);
    }
    ESP_LOGI(TAG, "I2C_END seq=%" PRIu32 " result=%s elapsed_us=%" PRId64,
             sequence, esp_err_to_name(result), elapsed_us);
    return result;
}

static esp_err_t read_device_regs(i2c_master_dev_handle_t device, uint8_t reg, uint8_t *data, size_t count)
{
    // Match vendor single-register reads; do not assume burst auto-increment.
    // Transmit byte selects the register; there is no PMU value write.
    for (size_t index = 0; index < count; ++index) {
        uint8_t address = reg + index;
        esp_err_t err = timed_transmit_receive(device, &address, 1, &data[index], 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C address=0x%02x reg=0x%02x failed: %s",
                     device_address(device), address, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t count)
{
    return read_device_regs(pmu, reg, data, count);
}

esp_err_t twatch_board_init(void)
{
    if (ready) return ESP_OK;
    if (bus || pmu) return ESP_ERR_INVALID_STATE;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_SDA,
        .scl_io_num = BOARD_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Schematic R3/R4 provide 2.2 kohm pull-ups to VDD3V3.
        .flags.enable_internal_pullup = false,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus), TAG, "Create shared I2C0");
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PMU_ADDRESS,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &device_config, &pmu);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Add PMU device: %s", esp_err_to_name(err));
        return err; // Fatal to owner; retained handles prevent unsafe retry.
    }
    uint8_t id;
    ESP_RETURN_ON_ERROR(read_regs(PMU_CHIP_ID, &id, 1), TAG, "Read PMU identity");
    if (id != PMU_EXPECTED_ID) {
        ESP_LOGE(TAG, "PMU identity=0x%02x expected=0x%02x; refusing telemetry decode", id, PMU_EXPECTED_ID);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const i2c_device_config_t expander_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = EXPANDER_ADDRESS,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &expander_config, &expander),
                        TAG, "Add schematic XL9555 candidate; identity not probed");
    ready = true;
    ESP_LOGI(TAG, "PMU identity=0x%02x I2C0 SDA=%d SCL=%d hz=%d read_only=1",
             id, BOARD_SDA, BOARD_SCL, I2C_SPEED_HZ);
    return ESP_OK;
}

esp_err_t twatch_board_read_rails(twatch_rail_snapshot_t *snapshot)
{
    if (!snapshot) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    twatch_rail_snapshot_t value = {0};
    ESP_RETURN_ON_ERROR(read_regs(PMU_DCDC_CONTROL, &value.dcdc_control, 1), TAG, "Read DCDC control");
    ESP_RETURN_ON_ERROR(read_regs(PMU_DC1_VOLTAGE, &value.dc1_voltage, 1), TAG, "Read DC1 setting");
    ESP_RETURN_ON_ERROR(read_regs(PMU_LDO_CONTROL, value.ldo_control, 2), TAG, "Read LDO control");
    ESP_RETURN_ON_ERROR(read_regs(PMU_ALDO2_VOLTAGE, &value.aldo2_voltage, 1), TAG, "Read ALDO2 setting");
    uint8_t aldo4_voltage;
    ESP_RETURN_ON_ERROR(read_regs(PMU_ALDO4_VOLTAGE, &aldo4_voltage, 1), TAG, "Read ALDO4 setting");
    // Do not read input ports: this audit needs latches/direction, not IRQ acknowledgement.
    ESP_RETURN_ON_ERROR(read_device_regs(expander, XL_OUTPUT0, value.expander_output, 2), TAG, "Read XL output latches");
    ESP_RETURN_ON_ERROR(read_device_regs(expander, XL_POLARITY0, value.expander_polarity, 2), TAG, "Read XL polarity");
    ESP_RETURN_ON_ERROR(read_device_regs(expander, XL_CONFIG0, value.expander_config, 2), TAG, "Read XL direction");
    value.dc1_enabled = (value.dcdc_control & DC1_ENABLE) != 0;
    value.aldo2_enabled = (value.ldo_control[0] & ALDO2_ENABLE) != 0;
    value.aldo4_enabled = (value.ldo_control[0] & ALDO4_ENABLE) != 0;
    const unsigned dc1_code = value.dc1_voltage & VOLTAGE_CODE_MASK;
    const unsigned aldo2_code = value.aldo2_voltage & VOLTAGE_CODE_MASK;
    value.dc1_setting_valid = dc1_code <= DC1_CODE_MAX;
    value.aldo2_setting_valid = aldo2_code <= ALDO2_CODE_MAX;
    const unsigned aldo4_code = aldo4_voltage & VOLTAGE_CODE_MASK;
    value.aldo4_setting_valid = aldo4_code <= 30;
    if (value.dc1_setting_valid) value.dc1_configured_mv = DC1_MIN_MV + dc1_code * VOLTAGE_STEP_MV;
    if (value.aldo2_setting_valid) value.aldo2_configured_mv = ALDO2_MIN_MV + aldo2_code * VOLTAGE_STEP_MV;
    if (value.aldo4_setting_valid) value.aldo4_configured_mv = ALDO2_MIN_MV + aldo4_code * VOLTAGE_STEP_MV;
    value.display_enable_output = !(value.expander_config[0] & XL_DISPLAY_ENABLE_P07);
    value.display_enable_latch_high = (value.expander_output[0] & XL_DISPLAY_ENABLE_P07) != 0;
    value.touch_reset_output = !(value.expander_config[1] & XL_TOUCH_RESET_P10);
    value.touch_reset_latch_high = (value.expander_output[1] & XL_TOUCH_RESET_P10) != 0;
    *snapshot = value;
    return ESP_OK;
}

esp_err_t twatch_board_read_power(twatch_power_snapshot_t *snapshot)
{
    if (!snapshot) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    twatch_power_snapshot_t value = {0};
    uint8_t status[2], battery[2];
    ESP_RETURN_ON_ERROR(read_regs(PMU_STATUS1, status, sizeof(status)), TAG, "Read status");
    ESP_RETURN_ON_ERROR(read_regs(PMU_ADC_CONTROL, &value.adc_control, 1), TAG, "Read ADC config");
    ESP_RETURN_ON_ERROR(read_regs(PMU_BATTERY_ADC_HIGH, battery, sizeof(battery)), TAG, "Read battery ADC");
    ESP_RETURN_ON_ERROR(read_regs(PMU_IRQ_STATUS1, value.irq_status, sizeof(value.irq_status)), TAG, "Read IRQ status");
    ESP_RETURN_ON_ERROR(read_regs(PMU_POWEROFF_ENABLE, &value.poweroff_enable, 1), TAG, "Read power-key config");
    value.status1 = status[0];
    value.status2 = status[1];
    value.usb_present = (status[0] & STATUS_VBUS_GOOD) && !(status[1] & STATUS_VBUS_ABSENT);
    value.battery_present = (status[0] & STATUS_BATTERY_PRESENT) != 0;
    value.charging = (status[1] >> STATUS_CHARGE_SHIFT) == STATUS_CHARGING;
    value.charger_state = status[1] & CHARGER_STATE_MASK;
    value.battery_voltage_available = value.battery_present && (value.adc_control & ADC_BATTERY_ENABLE);
    value.battery_mv = ((battery[0] & BATTERY_ADC_HIGH_MASK) << 8) | battery[1];
    *snapshot = value;
    return ESP_OK;
}

static esp_err_t update_expander_bit(uint8_t reg, uint8_t mask, bool set)
{
    uint8_t before;
    ESP_RETURN_ON_ERROR(read_device_regs(expander, reg, &before, 1), TAG, "Read XL before update");
    const uint8_t expected = set ? (before | mask) : (before & (uint8_t)~mask);
    if (before != expected) {
        const uint8_t command[2] = {reg, expected};
        ESP_RETURN_ON_ERROR(timed_transmit(expander, command, sizeof(command)),
                            TAG, "XL configuration write failed; state may be partial");
    }
    uint8_t after;
    ESP_RETURN_ON_ERROR(read_device_regs(expander, reg, &after, 1), TAG, "Read XL after update");
    ESP_LOGI(TAG, "XL reg=%02x before=%02x expected=%02x after=%02x wrote=%u",
             reg, before, expected, after, before != expected);
    if (after != expected) {
        ESP_LOGE(TAG, "XL readback mismatch; stopping without rollback");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t twatch_board_prepare_bhi(void)
{
    if (!ready) return ESP_ERR_INVALID_STATE;
    uint8_t control, voltage;
    ESP_RETURN_ON_ERROR(read_regs(PMU_LDO_CONTROL, &control, 1), TAG, "Read ALDO control");
    ESP_RETURN_ON_ERROR(read_regs(PMU_ALDO4_VOLTAGE, &voltage, 1), TAG, "Read ALDO4 voltage");
    const uint8_t expected_control = control | ALDO4_ENABLE;
    const uint8_t expected_voltage = (uint8_t)((voltage & (uint8_t)~VOLTAGE_CODE_MASK) | ALDO4_CODE);
    if (voltage != expected_voltage) {
        const uint8_t command[2] = {PMU_ALDO4_VOLTAGE, expected_voltage};
        ESP_RETURN_ON_ERROR(timed_transmit(pmu, command, sizeof(command)),
                            TAG, "ALDO4 voltage write failed; state may be partial");
    }
    if (control != expected_control) {
        const uint8_t command[2] = {PMU_LDO_CONTROL, expected_control};
        ESP_RETURN_ON_ERROR(timed_transmit(pmu, command, sizeof(command)),
                            TAG, "ALDO4 enable write failed; state may be partial");
    }
    uint8_t after_control, after_voltage;
    ESP_RETURN_ON_ERROR(read_regs(PMU_LDO_CONTROL, &after_control, 1), TAG, "Read ALDO control after write");
    ESP_RETURN_ON_ERROR(read_regs(PMU_ALDO4_VOLTAGE, &after_voltage, 1), TAG, "Read ALDO4 voltage after write");
    ESP_LOGI(TAG, "ALDO4 prepare before_ctrl=%02x before_volt=%02x expected_ctrl=%02x expected_volt=%02x "
             "after_ctrl=%02x after_volt=%02x", control, voltage, expected_control, expected_voltage,
             after_control, after_voltage);
    if (after_control != expected_control || after_voltage != expected_voltage) {
        ESP_LOGE(TAG, "ALDO4 readback mismatch; refusing BHI probe");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t twatch_board_prepare_display_touch(void)
{
    twatch_rail_snapshot_t before, after;
    ESP_RETURN_ON_ERROR(twatch_board_read_rails(&before), TAG, "Read essential rail preconditions");
    if (!before.dc1_enabled || !before.aldo2_enabled ||
        !before.dc1_setting_valid || !before.aldo2_setting_valid ||
        before.dc1_configured_mv != REQUIRED_MCU_MV ||
        before.aldo2_configured_mv != REQUIRED_DISPLAY_MV) {
        ESP_LOGE(TAG, "Required DC1/ALDO2 3300mV enable/settings mismatch; no writes performed");
        return ESP_ERR_INVALID_STATE;
    }
    // Preload BOTH high latches before changing either direction. Preserve
    // every other bit, including haptic enable P06 and radio switch P13.
    ESP_RETURN_ON_ERROR(update_expander_bit(XL_OUTPUT0, XL_DISPLAY_ENABLE_P07, true), TAG, "Preload P07 high");
    ESP_RETURN_ON_ERROR(update_expander_bit(XL_OUTPUT0 + 1, XL_TOUCH_RESET_P10, true), TAG, "Preload P10 high");
    ESP_RETURN_ON_ERROR(update_expander_bit(XL_CONFIG0, XL_DISPLAY_ENABLE_P07, false), TAG, "Set P07 output");
    ESP_RETURN_ON_ERROR(update_expander_bit(XL_CONFIG0 + 1, XL_TOUCH_RESET_P10, false), TAG, "Set P10 output");
    ESP_RETURN_ON_ERROR(twatch_board_read_rails(&after), TAG, "Verify final rail/XL state");
    for (unsigned port = 0; port < 2; ++port) {
        const uint8_t mask = port == 0 ? XL_DISPLAY_ENABLE_P07 : XL_TOUCH_RESET_P10;
        if (after.expander_output[port] != (uint8_t)(before.expander_output[port] | mask) ||
            after.expander_config[port] != (uint8_t)(before.expander_config[port] & (uint8_t)~mask) ||
            after.expander_polarity[port] != before.expander_polarity[port]) {
            ESP_LOGE(TAG, "XL final state mismatch port=%u", port);
            return ESP_FAIL;
        }
    }
    if (after.dcdc_control != before.dcdc_control || after.dc1_voltage != before.dc1_voltage ||
        after.ldo_control[0] != before.ldo_control[0] || after.ldo_control[1] != before.ldo_control[1] ||
        after.aldo2_voltage != before.aldo2_voltage) {
        ESP_LOGE(TAG, "PMU settings changed during XL setup");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ESSENTIAL_IO ready P07=output-high P10=output-high PMU_unchanged=1");
    return ESP_OK;
}

esp_err_t twatch_board_probe_touch(uint8_t *address, bool *present)
{
    if (!address || !present) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    *address = 0;
    *present = false;
    const uint8_t candidates[] = {TOUCH_PRIMARY_ADDRESS, TOUCH_FALLBACK_ADDRESS};
    for (size_t index = 0; index < sizeof(candidates); ++index) {
        esp_err_t err = i2c_master_probe(bus, candidates[index], I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            *address = candidates[index];
            *present = true;
            ESP_LOGI(TAG, "TOUCH_PROBE candidate=0x%02x present=1 identity=unverified", *address);
            return ESP_OK;
        }
        if (err != ESP_ERR_NOT_FOUND && err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "TOUCH_PROBE candidate=0x%02x probe=%s", candidates[index], esp_err_to_name(err));
        }
    }
    ESP_LOGW(TAG, "TOUCH_PROBE present=0 candidates=0x%02x,0x%02x", TOUCH_PRIMARY_ADDRESS,
             TOUCH_FALLBACK_ADDRESS);
    return ESP_OK;
}

esp_err_t twatch_board_probe_rtc(bool *present)
{
    if (!present) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    const esp_err_t err = i2c_master_probe(bus, RTC_CANDIDATE_ADDRESS, I2C_TIMEOUT_MS);
    *present = (err == ESP_OK);
    ESP_LOGI(TAG, "RTC_PROBE candidate=0x%02x present=%u identity=unverified",
             RTC_CANDIDATE_ADDRESS, *present);
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND && err != ESP_ERR_TIMEOUT) return err;
    return ESP_OK;
}

esp_err_t twatch_board_probe_bhi(bool *present)
{
    if (!present) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    const esp_err_t err = i2c_master_probe(bus, BHI_CANDIDATE_ADDRESS, I2C_TIMEOUT_MS);
    *present = (err == ESP_OK);
    ESP_LOGI(TAG, "BHI_PROBE candidate=0x%02x present=%u identity=unverified",
             BHI_CANDIDATE_ADDRESS, *present);
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND && err != ESP_ERR_TIMEOUT) return err;
    return ESP_OK;
}

esp_err_t twatch_board_read_bhi_identity(twatch_bhi_identity_t *identity)
{
    if (!identity) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    if (!bhi) {
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BHI_CANDIDATE_ADDRESS,
            .scl_speed_hz = I2C_SPEED_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &bhi), TAG, "Add BHI260AP candidate");
    }
    ESP_RETURN_ON_ERROR(read_device_regs(bhi, 0x1c, &identity->product_id, 1), TAG, "Read BHI product ID");
    ESP_RETURN_ON_ERROR(read_device_regs(bhi, 0x2b, &identity->chip_id, 1), TAG, "Read BHI chip ID");
    ESP_LOGI(TAG, "BHI_ID product=0x%02x chip=0x%02x expected_product=0x89",
             identity->product_id, identity->chip_id);
    return ESP_OK;
}

esp_err_t twatch_board_bhi_read(uint8_t reg, uint8_t *data, size_t length)
{
    if (!data || length == 0 || length > 255) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    if (!bhi) {
        const i2c_device_config_t config = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BHI_CANDIDATE_ADDRESS, .scl_speed_hz = I2C_SPEED_HZ};
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &bhi), TAG, "Add BHI260AP candidate");
    }
    return timed_transmit_receive(bhi, &reg, 1, data, length);
}

esp_err_t twatch_board_bhi_write(uint8_t reg, const uint8_t *data, size_t length)
{
    if (!data || length == 0 || length > 254) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    if (!bhi) {
        const i2c_device_config_t config = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = BHI_CANDIDATE_ADDRESS, .scl_speed_hz = I2C_SPEED_HZ};
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &bhi), TAG, "Add BHI260AP candidate");
    }
    uint8_t buffer[255];
    buffer[0] = reg;
    memcpy(&buffer[1], data, length);
    return timed_transmit(bhi, buffer, length + 1);
}

static bool bcd_byte_plausible(uint8_t value, uint8_t max_value)
{
    return (value & 0x0f) <= 9 && ((value >> 4) & 0x0f) <= (max_value / 10);
}

esp_err_t twatch_board_read_rtc(twatch_rtc_snapshot_t *snapshot)
{
    if (!snapshot) return ESP_ERR_INVALID_ARG;
    if (!ready) return ESP_ERR_INVALID_STATE;
    if (!rtc) {
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = RTC_CANDIDATE_ADDRESS,
            .scl_speed_hz = I2C_SPEED_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &rtc), TAG, "Add RTC candidate");
    }
    twatch_rtc_snapshot_t value = {0};
    uint8_t time_regs[7];
    ESP_RETURN_ON_ERROR(read_device_regs(rtc, 0x00, &value.control1, 1), TAG, "Read RTC control");
    ESP_RETURN_ON_ERROR(read_device_regs(rtc, 0x04, time_regs, sizeof(time_regs)), TAG, "Read RTC time");
    value.seconds = time_regs[0]; value.minutes = time_regs[1]; value.hours = time_regs[2];
    value.day = time_regs[3]; value.weekday = time_regs[4]; value.month = time_regs[5]; value.year = time_regs[6];
    value.bcd_fields_plausible = bcd_byte_plausible(value.seconds & 0x7f, 59) &&
        bcd_byte_plausible(value.minutes & 0x7f, 59) && bcd_byte_plausible(value.hours & 0x3f, 23) &&
        bcd_byte_plausible(value.day & 0x3f, 31) && (value.weekday & 0xf8) == 0 &&
        bcd_byte_plausible(value.month & 0x1f, 12);
    *snapshot = value;
    ESP_LOGI(TAG, "RTC_READ ctrl1=%02x time=%02x/%02x/%02x date=%02x wd=%02x/%02x/%02x bcd_plausible=%u",
             value.control1, value.hours, value.minutes, value.seconds, value.day,
             value.weekday, value.month, value.year, value.bcd_fields_plausible);
    return ESP_OK;
}

esp_err_t twatch_board_read_touch_info(void)
{
    if (!ready) return ESP_ERR_INVALID_STATE;
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TOUCH_PRIMARY_ADDRESS,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    if (!touch) {
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &config, &touch), TAG,
                            "Add confirmed CST92xx touch address");
    }
    const uint8_t enter_command[] = {0xd1, 0x01};
    ESP_RETURN_ON_ERROR(timed_transmit(touch, enter_command, sizeof(enter_command)),
                        TAG, "Enter CST92xx command mode");
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t checkcode_bytes[4], resolution[4], chip_bytes[4], firmware[8];
    const uint8_t checkcode_reg[] = {0xd1, 0xfc};
    const uint8_t resolution_reg[] = {0xd1, 0xf8};
    const uint8_t chip_reg[] = {0xd2, 0x04};
    const uint8_t firmware_reg[] = {0xd2, 0x08};
    ESP_RETURN_ON_ERROR(timed_transmit_receive(touch, checkcode_reg, 2, checkcode_bytes, 4),
                        TAG, "Read CST92xx checkcode");
    ESP_RETURN_ON_ERROR(timed_transmit_receive(touch, resolution_reg, 2, resolution, 4),
                        TAG, "Read CST92xx resolution");
    ESP_RETURN_ON_ERROR(timed_transmit_receive(touch, chip_reg, 2, chip_bytes, 4),
                        TAG, "Read CST92xx chip type");
    ESP_RETURN_ON_ERROR(timed_transmit_receive(touch, firmware_reg, 2, firmware, 8),
                        TAG, "Read CST92xx firmware");
    const uint32_t checkcode = ((uint32_t)checkcode_bytes[3] << 24) |
                               ((uint32_t)checkcode_bytes[2] << 16) |
                               ((uint32_t)checkcode_bytes[1] << 8) | checkcode_bytes[0];
    const uint16_t resolution_x = ((uint16_t)resolution[1] << 8) | resolution[0];
    const uint16_t resolution_y = ((uint16_t)resolution[3] << 8) | resolution[2];
    const uint16_t chip_type = ((uint16_t)chip_bytes[3] << 8) | chip_bytes[2];
    const uint32_t firmware_version = ((uint32_t)firmware[3] << 24) |
                                      ((uint32_t)firmware[2] << 16) |
                                      ((uint32_t)firmware[1] << 8) | firmware[0];
    ESP_LOGI(TAG, "TOUCH_INFO addr=0x1a checkcode=0x%08" PRIx32 " chip=0x%04x "
             "resolution=%ux%u firmware=0x%08" PRIx32,
             checkcode, chip_type, resolution_x, resolution_y, firmware_version);
    if ((checkcode & 0xffff0000U) != 0xcaca0000U) {
        ESP_LOGW(TAG, "TOUCH_INFO checkcode prefix unexpected; identity remains unverified");
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (chip_type != 0x9217 && chip_type != 0x9220) {
        ESP_LOGW(TAG, "TOUCH_INFO unsupported chip type=0x%04x", chip_type);
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

esp_err_t twatch_board_read_touch(twatch_touch_sample_t *sample)
{
    static uint32_t read_count;
    if (!sample) return ESP_ERR_INVALID_ARG;
    if (!touch) return ESP_ERR_INVALID_STATE;
    *sample = (twatch_touch_sample_t){0};
    uint8_t command[] = {0xd0, 0x00};
    uint8_t packet[15] = {0};
    ESP_RETURN_ON_ERROR(timed_transmit_receive(touch, command, sizeof(command), packet,
                                               sizeof(packet)), TAG,
                        "Read CST92xx touch packet");
    const uint8_t ack = 0xab;
    uint8_t acknowledge[] = {0xd0, 0x00, ack};
    ESP_RETURN_ON_ERROR(timed_transmit(touch, acknowledge, sizeof(acknowledge)),
                        TAG, "Acknowledge CST92xx packet");
    if (packet[6] != ack) {
        if ((read_count++ % 100U) == 0U) {
            ESP_LOGW(TAG, "TOUCH_PACKET ack=0x%02x raw=%02x %02x %02x %02x %02x %02x %02x",
                     packet[6], packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6]);
        }
        return ESP_ERR_INVALID_RESPONSE;
    }
    const uint8_t points = packet[5] & 0x7f;
    if ((read_count++ % 100U) == 0U) {
        ESP_LOGI(TAG, "TOUCH_PACKET points=%u raw=%02x %02x %02x %02x %02x %02x %02x",
                 points, packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6]);
    }
    if (points == 0) return ESP_OK;
    const uint8_t *finger = packet;
    const uint8_t event = finger[0] & 0x0f;
    if (event != 0x06) return ESP_OK;
    sample->pressed = true;
    sample->x = ((uint16_t)finger[1] << 4) | (finger[3] >> 4);
    sample->y = ((uint16_t)finger[2] << 4) | (finger[3] & 0x0f);
    return ESP_OK;
}
