#include "twatch_board.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s_common.h"
#include "driver/i2s_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "axp2101.h"
#include "xl9555.h"
#include "sx1262.h"
#include "spi2_power.h"

static const char *TAG = "twatch_board";

i2c_master_bus_handle_t twatch_i2c_bus;
spi_host_device_t       twatch_spi_bus;
spi_device_handle_t     twatch_lora_spi_dev;
spi_device_handle_t     twatch_nfc_spi_dev;

i2c_master_dev_handle_t twatch_pmu_dev;
i2c_master_dev_handle_t twatch_rtc_dev;
i2c_master_dev_handle_t twatch_imu_dev;
i2c_master_dev_handle_t twatch_haptic_dev;
i2c_master_dev_handle_t twatch_xl9555_dev;

i2s_chan_handle_t twatch_audio_tx;
i2s_chan_handle_t twatch_audio_rx;

/* BLDO2 is shared by audio and haptic clients. Serialize rail lease updates
 * so the first consumer enables it and the last consumer disables it. */
static SemaphoreHandle_t s_av_rail_mutex;
static portMUX_TYPE s_av_rail_init_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_av_rail_users;

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
                        TAG, "allocate audio TX channel");
    chan_cfg.id = I2S_NUM_0;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &twatch_audio_rx),
                        TAG, "allocate microphone RX channel");
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

    dev_cfg.device_address = TWATCH_I2C_ADDR_XL9555;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(twatch_i2c_bus, &dev_cfg, &twatch_xl9555_dev), TAG, "xl9555 add failed");

    return ESP_OK;
}

static SemaphoreHandle_t twatch_av_rail_mutex_get(void)
{
    portENTER_CRITICAL(&s_av_rail_init_mux);
    SemaphoreHandle_t mutex = s_av_rail_mutex;
    portEXIT_CRITICAL(&s_av_rail_init_mux);
    return mutex;
}

static esp_err_t twatch_av_rail_mutex_init(void)
{
    if (!twatch_av_rail_mutex_get()) {
        SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
        if (!mutex) return ESP_ERR_NO_MEM;
        portENTER_CRITICAL(&s_av_rail_init_mux);
        if (!s_av_rail_mutex) {
            s_av_rail_mutex = mutex;
            mutex = NULL;
        }
        portEXIT_CRITICAL(&s_av_rail_init_mux);
        if (mutex) vSemaphoreDelete(mutex);
    }
    return ESP_OK;
}

esp_err_t twatch_board_av_rail_acquire(void)
{
    SemaphoreHandle_t mutex = twatch_av_rail_mutex_get();
    if (!mutex || !twatch_pmu_dev) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(mutex, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (s_av_rail_users == 0) {
        err = axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, true);
    }
    if (err == ESP_OK) s_av_rail_users++;
    xSemaphoreGive(mutex);
    return err;
}

esp_err_t twatch_board_av_rail_release(void)
{
    SemaphoreHandle_t mutex = twatch_av_rail_mutex_get();
    if (!mutex || !twatch_pmu_dev) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (s_av_rail_users == 0) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = ESP_OK;
    if (s_av_rail_users == 1) {
        err = axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);
    }
    if (err == ESP_OK) s_av_rail_users--;
    xSemaphoreGive(mutex);
    return err;
}

bool twatch_sd_card_seated(void)
{
    bool level = false;
    if (xl9555_read_input(twatch_xl9555_dev, TWATCH_XL_GPIO_SD_DETECT, &level) != ESP_OK) {
        return true;   /* fail safe - see the header comment */
    }
    return !level;   /* active low: 0 = card seated */
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

    spi_device_interface_config_t lora_cfg = {
        .mode = 0,
        .clock_speed_hz = 10 * 1000 * 1000,
        .queue_size = 7,
        .spics_io_num = TWATCH_PIN_LORA_CS,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TWATCH_SPI_HOST, &lora_cfg, &twatch_lora_spi_dev), TAG, "lora spi dev failed");

    spi_device_interface_config_t nfc_cfg = {
        .mode = 1,
        .clock_speed_hz = 2 * 1000 * 1000,
        .input_delay_ns = 80,
        .queue_size = 7,
        .spics_io_num = TWATCH_PIN_NFC_CS,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(TWATCH_SPI_HOST, &nfc_cfg, &twatch_nfc_spi_dev), TAG, "nfc spi dev failed");
    return ESP_OK;
}

/* Initialize buses, board-control GPIO, PMU policy, and power ownership.
 * Peripheral drivers are initialized by the application as needed. */
esp_err_t twatch_board_init(void)
{
    esp_err_t err = twatch_av_rail_mutex_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AV rail mutex init failed: %s", esp_err_to_name(err));
        return err;
    }

    gpio_config_t boot_btn = {
        .pin_bit_mask = 1ULL << TWATCH_PIN_BOOT_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    err = gpio_config(&boot_btn);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "boot button config failed: %s", esp_err_to_name(err));
    }

    /* Bus creation is essential board plumbing; no device driver is probed
     * or started here. */
    err = twatch_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = twatch_spi_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = twatch_i2s_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* The GPIO expander establishes safe levels for board-owned enables and
     * routing signals. Start with display power held off. */
    err = xl9555_init(twatch_xl9555_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "xl9555 init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = xl9555_config_output(twatch_xl9555_dev, TWATCH_XL_GPIO_DISP_PWR, false);
    if (err != ESP_OK) return err;
    err = xl9555_config_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, false);
    if (err != ESP_OK) return err;
    err = xl9555_config_output(twatch_xl9555_dev, TWATCH_XL_GPIO_TOUCH_RST, true);
    if (err != ESP_OK) return err;
    err = xl9555_config_output(twatch_xl9555_dev, TWATCH_XL_GPIO_LORA_SEL, true);
    if (err != ESP_OK) return err;

    /* The XL9555 display-enable gate remains low until the application starts
     * its display stack; the PMU's default rail tree is configured below. */
    err = axp2101_init(twatch_pmu_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "axp2101 init failed: %s", esp_err_to_name(err));
        return err;
    }
    err = axp2101_set_default_power(twatch_pmu_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "axp2101 power tree failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Register shared SPI2 rail ownership after the PMU and expander are
     * initialized. SD presence protects the shared MISO line; the SX1262
     * callback protects its rail during active RX. NFC owns DLDO1. */
    err = spi2_power_init(twatch_pmu_dev);
    if (err != ESP_OK) return err;
    err = spi2_power_register(AXP2101_ALDO1, SPI2_POWER_SHARED, twatch_sd_card_seated);
    if (err != ESP_OK) return err;
    err = spi2_power_register(AXP2101_ALDO3, SPI2_POWER_SHARED, sx1262_spi2_power_needed);
    if (err != ESP_OK) return err;
    err = spi2_power_register(AXP2101_DLDO1, SPI2_POWER_OWNED, NULL);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "T-Watch Ultra board foundation initialized");
    return ESP_OK;
}

esp_err_t twatch_board_reset_touch(void)
{
    ESP_RETURN_ON_FALSE(twatch_xl9555_dev, ESP_ERR_INVALID_STATE, TAG,
                        "touch reset requested before expander init");
    ESP_RETURN_ON_ERROR(xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_TOUCH_RST, false),
                        TAG, "touch reset low");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_TOUCH_RST, true),
                        TAG, "touch reset high");
    vTaskDelay(pdMS_TO_TICKS(60));
    return ESP_OK;
}
