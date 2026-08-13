#include "twatch_board.h"
#include <time.h>
#include <sys/time.h>
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "axp2101.h"
#include "xl9555.h"
#include "pcf85063a.h"
#include "cst9217.h"
#include "drv2605.h"
#include "co5300.h"
#include "max98357a.h"
#include "t3902.h"
#include "m10q.h"
#include "sx1262.h"
#include "st25r3916.h"
#include "driver/i2s_common.h"
#include "driver/i2s_types.h"

static const char *TAG = "twatch_board";

i2c_master_bus_handle_t twatch_i2c_bus;
spi_host_device_t       twatch_spi_bus;
spi_device_handle_t     twatch_sd_spi_dev;
spi_device_handle_t     twatch_lora_spi_dev;
spi_device_handle_t     twatch_nfc_spi_dev;

i2c_master_dev_handle_t twatch_pmu_dev;
i2c_master_dev_handle_t twatch_rtc_dev;
i2c_master_dev_handle_t twatch_imu_dev;
i2c_master_dev_handle_t twatch_haptic_dev;
i2c_master_dev_handle_t twatch_touch_dev;
i2c_master_dev_handle_t twatch_xl9555_dev;

/* I2S audio: separate controllers so each keeps its own mode.
 * I2S1 = MAX98357A amp (STD TX), I2S0 = T3902 PDM mic (PDM RX). The two
 * modes cannot share one controller (the mode register is per-controller). */
i2s_chan_handle_t twatch_audio_tx;
i2s_chan_handle_t twatch_audio_rx;

static esp_err_t twatch_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_1,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear = true,
    };
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &twatch_audio_tx, NULL),
                        TAG, "i2s tx channel alloc failed");
    chan_cfg.id = I2S_NUM_0;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &twatch_audio_rx),
                        TAG, "i2s rx channel alloc failed");
    return ESP_OK;
}

static esp_err_t twatch_i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = TWATCH_PIN_I2C_SDA,
        .scl_io_num = TWATCH_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &twatch_i2c_bus), TAG, "i2c bus init failed");

    i2c_device_config_t dev_cfg = { .scl_speed_hz = 400000 };

    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = TWATCH_I2C_ADDR_PMU;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(twatch_i2c_bus, &dev_cfg, &twatch_pmu_dev), TAG, "pmu add failed");

    dev_cfg.device_address = TWATCH_I2C_ADDR_RTC;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(twatch_i2c_bus, &dev_cfg, &twatch_rtc_dev), TAG, "rtc add failed");

    dev_cfg.device_address = TWATCH_I2C_ADDR_IMU;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(twatch_i2c_bus, &dev_cfg, &twatch_imu_dev), TAG, "imu add failed");

    dev_cfg.device_address = TWATCH_I2C_ADDR_HAPTIC;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(twatch_i2c_bus, &dev_cfg, &twatch_haptic_dev), TAG, "haptic add failed");

    dev_cfg.device_address = TWATCH_I2C_ADDR_TOUCH;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(twatch_i2c_bus, &dev_cfg, &twatch_touch_dev), TAG, "touch add failed");

    dev_cfg.device_address = TWATCH_I2C_ADDR_XL9555;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(twatch_i2c_bus, &dev_cfg, &twatch_xl9555_dev), TAG, "xl9555 add failed");

    return ESP_OK;
}

static esp_err_t twatch_spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = TWATCH_PIN_SPI_MOSI,
        .miso_io_num = TWATCH_PIN_SPI_MISO,
        .sclk_io_num = TWATCH_PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(TWATCH_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "spi bus init failed");

    spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = 10 * 1000 * 1000,
        .queue_size = 7,
    };

    dev_cfg.spics_io_num = TWATCH_PIN_SD_CS;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TWATCH_SPI_HOST, &dev_cfg, &twatch_sd_spi_dev), TAG, "sd spi dev failed");

    dev_cfg.spics_io_num = TWATCH_PIN_LORA_CS;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TWATCH_SPI_HOST, &dev_cfg, &twatch_lora_spi_dev), TAG, "lora spi dev failed");

    dev_cfg.spics_io_num = TWATCH_PIN_NFC_CS;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TWATCH_SPI_HOST, &dev_cfg, &twatch_nfc_spi_dev), TAG, "nfc spi dev failed");

    return ESP_OK;
}

/* Push the RTC wall-clock time into the FreeRTOS/ESP-IDF system clock. */
static void sync_system_time(void)
{
    pcf85063a_time_t t;
    if (pcf85063a_get_time(twatch_rtc_dev, &t) != ESP_OK) {
        ESP_LOGW(TAG, "RTC time invalid, system clock not synced");
        return;
    }
    struct tm tm = { 0 };
    tm.tm_sec  = t.sec;
    tm.tm_min  = t.min;
    tm.tm_hour = t.hour;
    tm.tm_mday = t.day;
    tm.tm_mon  = t.month - 1;
    tm.tm_year = t.year - 1900;
    time_t now = mktime(&tm);
    if (now == (time_t)-1) {
        ESP_LOGW(TAG, "RTC time out of range, system clock not synced");
        return;
    }
    struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "system clock synced from RTC");
}

/* Public wrapper: the sensor cache calls this periodically so the ESP32
 * system clock (used by time()/mktime/crash timestamps) never drifts away
 * from the battery-backed RTC, which is the authoritative clock. */
void twatch_board_sync_system_time(void)
{
    sync_system_time();
}

esp_err_t twatch_board_init(void)
{
    esp_err_t err = ESP_OK;

    gpio_config_t boot_btn = {
        .pin_bit_mask = 1ULL << TWATCH_PIN_BOOT_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    err = gpio_config(&boot_btn);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "boot button config failed: %s", esp_err_to_name(err));
    }

    /* Buses are critical: if these fail, nothing on the board works. */
    err = twatch_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = twatch_spi_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi init failed: %s", esp_err_to_name(err));
    }

    /* 1. GPIO expander: drives haptic enable, display power, touch reset.
     *    Display power on, haptic off, touch released from reset. */
    err = xl9555_init(twatch_xl9555_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "xl9555 init failed: %s", esp_err_to_name(err));
    } else {
        err = xl9555_pin_mode(twatch_xl9555_dev, TWATCH_XL_GPIO_DISP_PWR, true);
        if (err != ESP_OK) ESP_LOGE(TAG, "disp pwr mode: %s", esp_err_to_name(err));
        err = xl9555_pin_mode(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, true);
        if (err != ESP_OK) ESP_LOGE(TAG, "haptic mode: %s", esp_err_to_name(err));
        err = xl9555_pin_mode(twatch_xl9555_dev, TWATCH_XL_GPIO_TOUCH_RST, true);
        if (err != ESP_OK) ESP_LOGE(TAG, "touch rst mode: %s", esp_err_to_name(err));
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_DISP_PWR, true);
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_TOUCH_RST, true);
    }

    /* 2. PMU: verify chip, then bring up the power tree (display, SD, LoRa,
     *    sensor, GNSS, speaker, NFC) and ADC channels. */
    err = axp2101_init(twatch_pmu_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "axp2101 init failed: %s", esp_err_to_name(err));
    } else {
        err = axp2101_set_default_power(twatch_pmu_dev);
        if (err != ESP_OK) ESP_LOGE(TAG, "axp2101 power tree failed: %s", esp_err_to_name(err));

        uint16_t batt_mv = 0;
        uint8_t batt_pct = 0;
        if (axp2101_get_battery_mv(twatch_pmu_dev, &batt_mv) == ESP_OK &&
            axp2101_get_battery_pct(twatch_pmu_dev, &batt_pct) == ESP_OK) {
            ESP_LOGI(TAG, "battery: %umV, %u%%", batt_mv, batt_pct);
        }
    }

    /* 3. Touch: pulse TP_RST (XL9555 P8) low->high so the CST9217 boots
     *    into a known state, then init its driver on the I2C bus. */
    xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_TOUCH_RST, false);
    vTaskDelay(pdMS_TO_TICKS(20));
    xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_TOUCH_RST, true);
    vTaskDelay(pdMS_TO_TICKS(60));
    cst9217_init(twatch_i2c_bus);

    /* 4. Peripheral driver init (skeletons). */
    pcf85063a_init(twatch_rtc_dev);
    sync_system_time();
    drv2605_init(twatch_haptic_dev);
    co5300_init();
    twatch_i2s_init();
    max98357a_init(twatch_audio_tx);
    t3902_init(twatch_audio_rx);
    m10q_init(twatch_pmu_dev, twatch_rtc_dev);
    sx1262_init(twatch_lora_spi_dev);
    st25r3916_init(twatch_nfc_spi_dev);

    ESP_LOGI(TAG, "T-Watch Ultra board init finished");
    return ESP_OK;
}
