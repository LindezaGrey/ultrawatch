#include "bsp_pcf85063.h"
#include "bsp_i2c.h"
#include "bsp_twatch_ultra.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp_pcf85063";

#define PCF_REG_CONTROL1   0x00
/* PCF85063A: 0x02=offset, 0x03=RAM, time counters start at 0x04 */
#define PCF_REG_SECONDS    0x04
#define PCF_REG_YEARS      0x0A

#define PCF_CTRL1_24H_BIT   (5)
#define PCF_SECONDS_OS_BIT  (7)

static uint8_t bcd_to_dec(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) & 0x0F) * 10 + (bcd & 0x0F));
}

static uint8_t dec_to_bcd(uint8_t dec)
{
    return (uint8_t)(((dec / 10) << 4) | (dec % 10));
}

esp_err_t bsp_rtc_init(void)
{
    uint8_t ctrl = 0;

    /* Clear STOP, select 24-hour mode */
    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(PCF85063_I2C_ADDR, PCF_REG_CONTROL1, &ctrl, 1),
                        TAG, "read control1 failed");
    ctrl &= ~(1 << PCF_CTRL1_24H_BIT);
    return bsp_i2c_write_reg(PCF85063_I2C_ADDR, PCF_REG_CONTROL1, &ctrl, 1);
}

esp_err_t bsp_rtc_get_time(struct tm *tm)
{
    uint8_t buf[7] = { 0 }; /* seconds..years */
    ESP_RETURN_ON_ERROR(bsp_i2c_read_reg(PCF85063_I2C_ADDR, PCF_REG_SECONDS, buf, sizeof(buf)),
                        TAG, "read time failed");

    if (buf[0] & (1 << PCF_SECONDS_OS_BIT)) {
        return ESP_ERR_INVALID_STATE;
    }

    tm->tm_sec = bcd_to_dec(buf[0] & 0x7F);
    tm->tm_min = bcd_to_dec(buf[1] & 0x7F);
    tm->tm_hour = bcd_to_dec(buf[2] & 0x3F);
    tm->tm_mday = bcd_to_dec(buf[3] & 0x3F);
    tm->tm_wday = buf[4] & 0x07; /* PCF85063A: 0=Sunday..6=Saturday */
    tm->tm_mon = bcd_to_dec(buf[5] & 0x1F) - 1;
    tm->tm_year = bcd_to_dec(buf[6]) + 100;
    tm->tm_isdst = 0;

    return ESP_OK;
}

esp_err_t bsp_rtc_set_time(const struct tm *tm)
{
    uint8_t buf[7];

    buf[0] = dec_to_bcd(tm->tm_sec);
    buf[1] = dec_to_bcd(tm->tm_min);
    buf[2] = dec_to_bcd(tm->tm_hour);
    buf[3] = dec_to_bcd(tm->tm_mday);
    buf[4] = tm->tm_wday & 0x07; /* 0=Sunday..6=Saturday, stored as-is */
    buf[5] = dec_to_bcd(tm->tm_mon + 1);
    buf[6] = dec_to_bcd((tm->tm_year - 100) % 100);

    ESP_RETURN_ON_ERROR(bsp_i2c_write_reg(PCF85063_I2C_ADDR, PCF_REG_SECONDS, buf, sizeof(buf)),
                        TAG, "write time failed");

    return ESP_OK;
}
