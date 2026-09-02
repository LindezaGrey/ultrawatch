#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

#define BOARD_I2C_PORT              I2C_NUM_0
#define BOARD_I2C_SDA               GPIO_NUM_3
#define BOARD_I2C_SCL               GPIO_NUM_2
#define BOARD_I2C_HZ                400000

#define BOARD_AXP2101_ADDR          0x34
#define BOARD_AXP2101_LDO_ENABLE    0x90
#define BOARD_AXP2101_ALDO1_VOLTAGE 0x92
#define BOARD_AXP2101_ALDO1_BIT     0
#define BOARD_AXP2101_ALDO2_VOLTAGE 0x93
#define BOARD_AXP2101_ALDO2_BIT     1
#define BOARD_AXP2101_ALDO3_VOLTAGE 0x94
#define BOARD_AXP2101_ALDO3_BIT     2
#define BOARD_AXP2101_ALDO4_VOLTAGE 0x95
#define BOARD_AXP2101_ALDO4_BIT     3
#define BOARD_AXP2101_BLDO1_VOLTAGE 0x96
#define BOARD_AXP2101_BLDO1_BIT     4
#define BOARD_PMU_INTERRUPT         GPIO_NUM_7
#define BOARD_XL9555_ADDR           0x20
#define BOARD_XL9555_INPUT1         0x01
#define BOARD_XL9555_OUTPUT0        0x02
#define BOARD_XL9555_OUTPUT1        0x03
#define BOARD_XL9555_CONFIG0        0x06
#define BOARD_XL9555_CONFIG1        0x07
#define BOARD_XL9555_DRIVER_BIT     6
#define BOARD_XL9555_DISPLAY_BIT    7
#define BOARD_XL9555_TOUCH_RESET_BIT 0
#define BOARD_XL9555_SD_DETECT_BIT   2

#define BOARD_PCF85063_ADDR         0x51
#define BOARD_RTC_INTERRUPT         GPIO_NUM_1
#define BOARD_BHI260_ADDR           0x28
#define BOARD_BHI260_INTERRUPT      GPIO_NUM_8
#define BOARD_TOUCH_INTERRUPT       GPIO_NUM_12
#define BOARD_CST9217_ADDR_PRIMARY  0x1a
#define BOARD_DRV2605_ADDR          0x5a

#define BOARD_AUDIO_BCLK            GPIO_NUM_9
#define BOARD_AUDIO_WCLK            GPIO_NUM_10
#define BOARD_AUDIO_DOUT            GPIO_NUM_11
#define BOARD_AXP2101_BLDO2_VOLTAGE 0x97
#define BOARD_AXP2101_BLDO2_BIT     5

#define BOARD_GPS_UART              UART_NUM_1
#define BOARD_GPS_TX                GPIO_NUM_43
#define BOARD_GPS_RX                GPIO_NUM_44
#define BOARD_GPS_PPS               GPIO_NUM_13

#define BOARD_DISPLAY_SPI_HOST      SPI3_HOST
#define BOARD_DISPLAY_SCK           GPIO_NUM_40
#define BOARD_DISPLAY_D0            GPIO_NUM_38
#define BOARD_DISPLAY_D1            GPIO_NUM_39
#define BOARD_DISPLAY_D2            GPIO_NUM_42
#define BOARD_DISPLAY_D3            GPIO_NUM_45
#define BOARD_DISPLAY_CS            GPIO_NUM_41
#define BOARD_DISPLAY_RESET         GPIO_NUM_37
#define BOARD_DISPLAY_TE            GPIO_NUM_6

#define BOARD_DISPLAY_WIDTH         410
#define BOARD_DISPLAY_HEIGHT        502
#define BOARD_DISPLAY_COLUMN_OFFSET 22
#define BOARD_DISPLAY_QSPI_HZ       80000000

#define BOARD_SD_SPI_HOST           SPI2_HOST
#define BOARD_SD_SCK                GPIO_NUM_35
#define BOARD_SD_MOSI               GPIO_NUM_34
#define BOARD_SD_MISO               GPIO_NUM_33
#define BOARD_SD_CS                 GPIO_NUM_21
#define BOARD_NFC_CS                GPIO_NUM_4
#define BOARD_LORA_CS               GPIO_NUM_36
#define BOARD_LORA_RESET            GPIO_NUM_47
#define BOARD_LORA_BUSY             GPIO_NUM_48
#define BOARD_LORA_INTERRUPT        GPIO_NUM_14
#define BOARD_XL9555_LORA_SELECT_BIT 3
#define BOARD_SD_SPI_HZ             4000000

#define BOARD_ACTIVE_WIDTH_UM       33090
#define BOARD_ACTIVE_HEIGHT_UM      40510
#define BOARD_TOP_RADIUS_UM          8420
#define BOARD_BOTTOM_RADIUS_UM       9000
