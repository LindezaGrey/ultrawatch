#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

#define BOARD_I2C_PORT              I2C_NUM_0
#define BOARD_I2C_SDA               GPIO_NUM_3
#define BOARD_I2C_SCL               GPIO_NUM_2
#define BOARD_I2C_HZ                400000

#define BOARD_AXP2101_ADDR          0x34
#define BOARD_AXP2101_LDO_ENABLE    0x90
#define BOARD_AXP2101_ALDO2_VOLTAGE 0x93
#define BOARD_AXP2101_ALDO2_BIT     1
#define BOARD_AXP2101_ALDO4_VOLTAGE 0x95
#define BOARD_AXP2101_ALDO4_BIT     3
#define BOARD_XL9555_ADDR           0x20
#define BOARD_XL9555_OUTPUT0        0x02
#define BOARD_XL9555_OUTPUT1        0x03
#define BOARD_XL9555_CONFIG0        0x06
#define BOARD_XL9555_CONFIG1        0x07
#define BOARD_XL9555_DRIVER_BIT     6
#define BOARD_XL9555_DISPLAY_BIT    7
#define BOARD_XL9555_TOUCH_RESET_BIT 0

#define BOARD_PCF85063_ADDR         0x51
#define BOARD_BHI260_ADDR           0x28
#define BOARD_BHI260_INTERRUPT      GPIO_NUM_8
#define BOARD_TOUCH_INTERRUPT       GPIO_NUM_12
#define BOARD_CST9217_ADDR_PRIMARY  0x1a
#define BOARD_CST9217_ADDR_FALLBACK 0x5a

#define BOARD_DISPLAY_SPI_HOST      SPI2_HOST
#define BOARD_DISPLAY_SCK           GPIO_NUM_40
#define BOARD_DISPLAY_D0            GPIO_NUM_38
#define BOARD_DISPLAY_D1            GPIO_NUM_39
#define BOARD_DISPLAY_D2            GPIO_NUM_42
#define BOARD_DISPLAY_D3            GPIO_NUM_45
#define BOARD_DISPLAY_CS            GPIO_NUM_41
#define BOARD_DISPLAY_RESET         GPIO_NUM_37

#define BOARD_DISPLAY_WIDTH         410
#define BOARD_DISPLAY_HEIGHT        502
#define BOARD_DISPLAY_COLUMN_OFFSET 22
#define BOARD_DISPLAY_QSPI_HZ       80000000

#define BOARD_ACTIVE_WIDTH_UM       33090
#define BOARD_ACTIVE_HEIGHT_UM      40510
#define BOARD_TOP_RADIUS_UM          8420
#define BOARD_BOTTOM_RADIUS_UM       9000
