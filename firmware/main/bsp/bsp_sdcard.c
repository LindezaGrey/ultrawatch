#include "bsp_sdcard.h"
#include "bsp_twatch_ultra.h"
#include "bsp_axp2101.h"
#include "bsp_xl9555.h"

#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp_sdcard";

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;
static const char *s_last_error = "not attempted";

const char *bsp_sd_last_error(void)
{
    return s_last_error;
}

bool bsp_sd_detect(void)
{
    return bsp_xl9555_sd_detect();
}

esp_err_t bsp_sd_init(void)
{
    if (s_mounted) {
        s_last_error = "already mounted";
        return ESP_OK;
    }

    s_last_error = "aldo1 power";
    esp_err_t err = bsp_axp2101_set_aldo1(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "aldo1 (sd power) disable failed: %s", esp_err_to_name(err));
        return err;
    }
    /* Long enough for the SD_VDD rail capacitor to discharge so the card
     * actually power-cycles (not just browns out). */
    vTaskDelay(pdMS_TO_TICKS(250));
    err = bsp_axp2101_set_aldo1(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "aldo1 (sd power) enable failed: %s", esp_err_to_name(err));
        return err;
    }
    /* Let the card VDD rail (ALDO1 soft-start) and the card itself stabilize */
    vTaskDelay(pdMS_TO_TICKS(250));

    if (!bsp_sd_detect()) {
        s_last_error = "no card (detect pin high)";
        ESP_LOGW(TAG, "no SD card detected (detect pin high)");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "SD card detected");

    /* Deassert the chip selects of the other devices sharing SPI2 (LoRa,
     * NFC) so they can never drive MISO while the SD card is addressed. */
    const gpio_num_t spi2_cs_pins[] = {
        BSP_SPI2_NFC_CS_PIN,
        BSP_SPI2_LORA_CS_PIN,
        BSP_SPI2_LORA_RST_PIN,
    };
    for (int i = 0; i < (int)sizeof(spi2_cs_pins) / (int)sizeof(spi2_cs_pins[0]); ++i) {
        gpio_config_t cs_cfg = {
            .pin_bit_mask = 1ULL << spi2_cs_pins[i],
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&cs_cfg);
        gpio_set_level(spi2_cs_pins[i], 1);
    }

    /* The display owns SPI3 (QSPI), so the SD card uses SPI2 (as on the stock
     * T-Watch Ultra: display on SPI3, SD/NFC/LoRa shared on SPI2). */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = BSP_SD_SPI_FREQ_KHZ;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BSP_SD_MOSI_PIN,
        .miso_io_num = BSP_SD_MISO_PIN,
        .sclk_io_num = BSP_SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    s_last_error = "spi bus init";
    err = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.host_id = host.slot;
    slot_cfg.gpio_cs = BSP_SD_CS_PIN;

    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    s_last_error = "mount";
    for (int attempt = 0; attempt < 3; ++attempt) {
        err = esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, &host, &slot_cfg,
                                      &mount_cfg, &s_card);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "mount attempt %d/%d failed: %s", attempt + 1, 3,
                 esp_err_to_name(err));
        if (attempt + 1 < 3) {
            /* Card may have been left in a bad state; power-cycle it fully */
            bsp_axp2101_set_aldo1(false);
            vTaskDelay(pdMS_TO_TICKS(250));
            bsp_axp2101_set_aldo1(true);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount failed: %s (card must be FAT32/FAT16)", esp_err_to_name(err));
        return err;
    }

    s_mounted = true;
    s_last_error = "ok";
    ESP_LOGI(TAG, "mounted at %s, %u MB", BSP_SD_MOUNT_POINT, bsp_sd_capacity_mb());
    return ESP_OK;
}

esp_err_t bsp_sd_unmount(void)
{
    if (!s_mounted || s_card == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, s_card);
    if (err == ESP_OK) {
        s_card = NULL;
        s_mounted = false;
    }
    return err;
}

bool bsp_sd_mounted(void)
{
    return s_mounted;
}

uint32_t bsp_sd_capacity_mb(void)
{
    if (!s_mounted || s_card == NULL) {
        return 0;
    }
    return (uint32_t)((uint64_t)s_card->csd.capacity * s_card->csd.sector_size /
                      (1024 * 1024));
}

uint32_t bsp_sd_free_mb(void)
{
    FATFS *fs = NULL;
    DWORD fre_clust = 0, fre_sect = 0, tot_sect = 0;
    if (f_getfree(BSP_SD_MOUNT_POINT, &fre_clust, &fs) != FR_OK) {
        return 0;
    }
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;
    if (tot_sect == 0) {
        return 0;
    }
    return (uint32_t)((uint64_t)fre_sect * fs->ssize / (1024 * 1024));
}
