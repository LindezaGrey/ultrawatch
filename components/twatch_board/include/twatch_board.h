/*
 * twatch_board.h - LilyGO T-Watch Ultra board package.
 *
 * Owns the shared buses and the power-up sequence (XL9555 expander +
 * AXP2101 rails). Exposes handles to the on-bus devices so driver
 * components can be used without re-opening the buses.
 *
 * Built on Espressif native APIs only (driver/i2c_master.h,
 * driver/spi_master.h, driver/gpio.h).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/i2s_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Pin map (see docs/hardware.md) ---- */

/* I2C */
#define TWATCH_PIN_I2C_SDA         3
#define TWATCH_PIN_I2C_SCL         2
#define TWATCH_PIN_RTC_INT         1
#define TWATCH_PIN_NFC_IRQ         5
#define TWATCH_PIN_IMU_INT         8
#define TWATCH_PIN_AXP2101_IRQ     7
#define TWATCH_PIN_TOUCH_INT       12

/* SPI */
#define TWATCH_PIN_SPI_MOSI        34
#define TWATCH_PIN_SPI_MISO        33
#define TWATCH_PIN_SPI_SCK         35
#define TWATCH_PIN_SD_CS           21
#define TWATCH_PIN_LORA_CS         36
#define TWATCH_PIN_LORA_RESET      47
#define TWATCH_PIN_LORA_BUSY       48
#define TWATCH_PIN_LORA_IRQ        14
#define TWATCH_PIN_NFC_CS          4

/* Display QSPI */
#define TWATCH_PIN_DISP_CS         41
#define TWATCH_PIN_DISP_DATA0      38
#define TWATCH_PIN_DISP_DATA1      39
#define TWATCH_PIN_DISP_DATA2      42
#define TWATCH_PIN_DISP_DATA3      45
#define TWATCH_PIN_DISP_SCK        40
#define TWATCH_PIN_DISP_TE         6
#define TWATCH_PIN_DISP_RESET      37

/* Audio / mic */
#define TWATCH_PIN_AUDIO_BCLK      9
#define TWATCH_PIN_AUDIO_WCLK      10
#define TWATCH_PIN_AUDIO_DOUT      11
#define TWATCH_PIN_MIC_SCK         17
#define TWATCH_PIN_MIC_DAT         18

/* GNSS */
#define TWATCH_PIN_GNSS_TX         43
#define TWATCH_PIN_GNSS_RX         44
#define TWATCH_PIN_GNSS_PPS        13

/* Buttons */
#define TWATCH_PIN_BOOT_BUTTON     0

/* ---- XL9555 expander outputs ----
 * VERIFIED ON HARDWARE: DRV_EN=P6, VCI_EN=P7, TP_RST=P8 (driving P8 low
 * NACKs the CST9217 touch on I2C), SD_DET=P10 (0 with card seated, 1 without).
 * Matches the arduino-esp32 variant; the LilyGO doc/schematic (P10/P12) is
 * incorrect. See docs/hardware.md. */
#define TWATCH_XL_GPIO_HAPTIC_EN   6
#define TWATCH_XL_GPIO_DISP_PWR    7
#define TWATCH_XL_GPIO_TOUCH_RST   8
#define TWATCH_XL_GPIO_SD_DETECT   10

/* ---- I2C addresses ---- */
#define TWATCH_I2C_ADDR_TOUCH      0x1A
#define TWATCH_I2C_ADDR_XL9555     0x20
#define TWATCH_I2C_ADDR_IMU        0x28
#define TWATCH_I2C_ADDR_PMU        0x34
#define TWATCH_I2C_ADDR_RTC        0x51
#define TWATCH_I2C_ADDR_HAPTIC     0x5A

/* ---- SPI bus / hosts ----
 * Shared SD/LoRa/NFC bus on SPI2; the CO5300 QSPI display owns SPI3.
 * (This allocation was proven to drive the panel via esp_lcd's SH8601 driver.) */
#define TWATCH_SPI_HOST             SPI2_HOST
#define TWATCH_LCD_SPI_HOST         SPI3_HOST

/* Shared bus handles (valid after twatch_board_init). */
extern i2c_master_bus_handle_t twatch_i2c_bus;
extern spi_host_device_t       twatch_spi_bus;
extern spi_device_handle_t     twatch_sd_spi_dev;
extern spi_device_handle_t     twatch_lora_spi_dev;
extern spi_device_handle_t     twatch_nfc_spi_dev;

/* On-bus device handles (valid after twatch_board_init). */
extern i2c_master_dev_handle_t twatch_pmu_dev;
extern i2c_master_dev_handle_t twatch_rtc_dev;
extern i2c_master_dev_handle_t twatch_imu_dev;
extern i2c_master_dev_handle_t twatch_haptic_dev;
extern i2c_master_dev_handle_t twatch_touch_dev;
extern i2c_master_dev_handle_t twatch_xl9555_dev;

/* Shared I2S0 channels: TX = MAX98357A amp, RX = T3902 PDM mic (full-duplex). */
extern i2s_chan_handle_t twatch_audio_tx;
extern i2s_chan_handle_t twatch_audio_rx;

/**
 * @brief Bring up the board: GPIO, I2C + SPI buses, expander, power rails.
 *
 * @return ESP_OK on success, an error code otherwise.
 */
esp_err_t twatch_board_init(void);

#ifdef __cplusplus
}
#endif
