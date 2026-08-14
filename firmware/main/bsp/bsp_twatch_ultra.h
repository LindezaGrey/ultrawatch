#pragma once

#include "driver/gpio.h"
#include "driver/i2c_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C bus (shared by AXP2101, XL9555, CST9217, PCF85063A, ...) */
#define BSP_I2C_SDA_PIN         GPIO_NUM_3
#define BSP_I2C_SCL_PIN         GPIO_NUM_2
#define BSP_I2C_FREQ_HZ         (400 * 1000)
extern i2c_master_bus_handle_t bsp_i2c_bus_handle;

/* Display QSPI */
#define BSP_DISP_SCK_PIN        GPIO_NUM_40
#define BSP_DISP_D0_PIN         GPIO_NUM_38
#define BSP_DISP_D1_PIN         GPIO_NUM_39
#define BSP_DISP_D2_PIN         GPIO_NUM_42
#define BSP_DISP_D3_PIN         GPIO_NUM_45
#define BSP_DISP_CS_PIN         GPIO_NUM_41
#define BSP_DISP_RST_PIN        GPIO_NUM_37
#define BSP_DISP_TE_PIN         GPIO_NUM_6

#define BSP_LCD_H_RES           410
#define BSP_LCD_V_RES           502

/* Touch interrupt */
#define BSP_TOUCH_INT_PIN       GPIO_NUM_12

/* SD card: second SPI bus (SPI3), shared with LoRa/NFC on this board.
 * Power rail is AXP2101 ALDO1; card detect is XL9555 expander pin 10,
 * active low. */
#define BSP_SD_CS_PIN           GPIO_NUM_21
#define BSP_SD_SCK_PIN          GPIO_NUM_35
#define BSP_SD_MOSI_PIN         GPIO_NUM_34
#define BSP_SD_MISO_PIN         GPIO_NUM_33
#define BSP_SD_SPI_FREQ_KHZ     4000
#define BSP_SD_MOUNT_POINT      "/sdcard"
#define BSP_SD_DET_PIN          10

/* Other devices sharing the SD SPI bus (SPI2): keep their chip selects
 * deasserted so they never drive MISO while the SD card is being used. */
#define BSP_SPI2_NFC_CS_PIN     GPIO_NUM_4
#define BSP_SPI2_LORA_CS_PIN    GPIO_NUM_36
#define BSP_SPI2_LORA_RST_PIN   GPIO_NUM_47

/* I2C device addresses */
#define AXP2101_I2C_ADDR        0x34
#define XL9555_I2C_ADDR         0x20
#define CST9217_I2C_ADDR        0x5A
#define PCF85063_I2C_ADDR       0x51

/* XL9555 expander output pins (P0: 0-7, P1: 8-15) */
#define XL9555_PIN_DRV_EN       6
#define XL9555_PIN_DISP_EN      7
#define XL9555_PIN_TOUCH_RST    8

#ifdef __cplusplus
}
#endif
