#include "pcf85063a.h"
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pcf85063a";

#define REG_CTRL1       0x00
#define REG_CTRL2       0x01
#define REG_OFFSET      0x02
#define REG_RAM         0x03
#define REG_SEC         0x04   /* time/date base */
#define REG_ALM_SEC     0x0B   /* alarm base */
#define REG_TIMER_VALUE 0x10
#define REG_TIMER_MODE  0x11

#define CTRL1_STOP      (1 << 5)
#define CTRL2_AF        (1 << 6)
#define CTRL2_AIE       (1 << 7)
#define CTRL2_TF        (1 << 3)
#define CTRL2_MI        (1 << 5)   /* minute interrupt */
#define CTRL2_HMI       (1 << 4)   /* half-minute interrupt */
#define CTRL2_COF_OFF   0x07       /* COF[2:0] = 111 -> CLKOUT disabled */

#define SEC_OS          0x80
#define MON_CENTURY     0x80
#define ALM_AEN         0x80

/* Timer_mode (0x11): TCF[1:0] at bits 4:3 (00=4.096k, 01=64 Hz, 10=1 Hz,
 * 11=1/60 Hz), TE at bit 2, TIE at bit 1, TI_TP at bit 0. */
#define TIMER_TCF_1HZ   0x10    /* TCF = 10 -> 1 Hz clock (max 255 s) */
#define TIMER_TCF_1MIN  0x18    /* TCF = 11 -> 1/60 Hz clock (max 255 min) */
#define TIMER_TE        (1 << 2)
#define TIMER_TIE       (1 << 1)
#define TIMER_TI_TP     (1 << 0)   /* 0 = INT follows TF flag, 1 = pulse */

static uint8_t bcd_to_bin(uint8_t v) { return (uint8_t)((v & 0x0f) + ((v >> 4) * 10)); }
static uint8_t bin_to_bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

static esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, n, 100);
}

static esp_err_t write_regs(i2c_master_dev_handle_t dev, uint8_t reg, const uint8_t *buf, size_t n)
{
    uint8_t data[1 + 8];
    if (n > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    data[0] = reg;
    memcpy(&data[1], buf, n);
    return i2c_master_transmit(dev, data, 1 + n, 100);
}

static esp_err_t write_byte(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    return write_regs(dev, reg, &val, 1);
}

esp_err_t pcf85063a_init(i2c_master_dev_handle_t dev)
{
    uint8_t ctrl1 = 0;
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_CTRL1, &ctrl1, 1), TAG, "probe failed");
    ESP_LOGI(TAG, "PCF85063A found (ctrl1=0x%02x)", ctrl1);

    /* Ensure the oscillator runs (clear STOP bit). */
    if (ctrl1 & CTRL1_STOP) {
        ESP_RETURN_ON_ERROR(write_byte(dev, REG_CTRL1, ctrl1 & ~CTRL1_STOP), TAG, "clear stop");
        ESP_LOGI(TAG, "cleared oscillator STOP bit");
    }

    /* Normalize Control_2: clear the alarm/timer flags and the MI/HMI
     * minute-interrupt enables so a stale CTRL2 left over from older firmware
     * or from the battery-backed registers cannot assert INT/TF every minute.
     * AIE is re-armed by pcf85063a_set_alarm() when needed. CLKOUT off. */
    ESP_RETURN_ON_ERROR(write_byte(dev, REG_CTRL2, CTRL2_COF_OFF), TAG, "init ctrl2");

    pcf85063a_time_t t;
    if (pcf85063a_get_time(dev, &t) == ESP_OK) {
        ESP_LOGI(TAG, "RTC time: %04u-%02u-%02u %02u:%02u:%02u (wd=%u)",
                 t.year, t.month, t.day, t.hour, t.min, t.sec, t.weekday);
    } else {
        ESP_LOGW(TAG, "RTC time invalid (oscillator stopped / needs setting)");
    }
    return ESP_OK;
}

esp_err_t pcf85063a_get_time(i2c_master_dev_handle_t dev, pcf85063a_time_t *t)
{
    uint8_t r[7];
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_SEC, r, sizeof(r)), TAG, "read time");

    if (r[0] & SEC_OS) {
        /* Oscillator stopped (e.g. backup battery drained) -> invalid time. */
        return ESP_ERR_INVALID_STATE;
    }
    if (!t) {
        return ESP_ERR_INVALID_ARG;
    }

    t->sec     = bcd_to_bin(r[0] & 0x7f);
    t->min     = bcd_to_bin(r[1] & 0x7f);
    t->hour    = bcd_to_bin(r[2] & 0x3f);
    t->day     = bcd_to_bin(r[3] & 0x3f);
    t->weekday = (uint8_t)(r[4] & 0x07);
    t->month   = bcd_to_bin(r[5] & 0x1f);
    t->year    = (uint16_t)(2000 + ((r[5] & MON_CENTURY) ? 100 : 0) + bcd_to_bin(r[6]));
    return ESP_OK;
}

esp_err_t pcf85063a_set_time(i2c_master_dev_handle_t dev, const pcf85063a_time_t *t)
{
    uint8_t r[7];
    r[0] = bin_to_bcd(t->sec) & 0x7f;
    r[1] = bin_to_bcd(t->min) & 0x7f;
    r[2] = bin_to_bcd(t->hour) & 0x3f;
    r[3] = bin_to_bcd(t->day) & 0x3f;
    r[4] = (uint8_t)(t->weekday & 0x07);
    r[5] = (bin_to_bcd(t->month) & 0x1f) | ((t->year >= 2100) ? MON_CENTURY : 0);
    r[6] = bin_to_bcd((uint8_t)(t->year % 100));
    ESP_RETURN_ON_ERROR(write_regs(dev, REG_SEC, r, sizeof(r)), TAG, "write time");

    /* Clear the oscillator-stop flag so get_time accepts the new value. */
    uint8_t sec = 0;
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_SEC, &sec, 1), TAG, "read sec");
    return write_byte(dev, REG_SEC, (uint8_t)(sec & ~SEC_OS));
}

esp_err_t pcf85063a_get_second(i2c_master_dev_handle_t dev, uint8_t *sec)
{
    return read_regs(dev, REG_SEC, sec, 1);
}

esp_err_t pcf85063a_set_alarm(i2c_master_dev_handle_t dev, const pcf85063a_alarm_t *alarm)
{
    uint8_t r[5];
    r[0] = (bin_to_bcd(alarm->time.sec) & 0x7f) | (alarm->mask_sec ? ALM_AEN : 0);
    r[1] = (bin_to_bcd(alarm->time.min) & 0x7f) | (alarm->mask_min ? ALM_AEN : 0);
    r[2] = (bin_to_bcd(alarm->time.hour) & 0x3f) | (alarm->mask_hour ? ALM_AEN : 0);
    r[3] = (bin_to_bcd(alarm->time.day) & 0x3f) | (alarm->mask_day ? ALM_AEN : 0);
    r[4] = (uint8_t)((alarm->time.weekday & 0x07) | (alarm->mask_weekday ? ALM_AEN : 0));
    ESP_RETURN_ON_ERROR(write_regs(dev, REG_ALM_SEC, r, sizeof(r)), TAG, "write alarm");

    uint8_t ctrl2 = 0;
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_CTRL2, &ctrl2, 1), TAG, "read ctrl2");
    ctrl2 &= (uint8_t)~(CTRL2_AF);                 /* clear alarm flag */
    if (alarm->enabled) {
        ctrl2 |= CTRL2_AIE;
    } else {
        ctrl2 &= (uint8_t)~CTRL2_AIE;
    }
    return write_byte(dev, REG_CTRL2, ctrl2);
}

esp_err_t pcf85063a_clear_alarm(i2c_master_dev_handle_t dev)
{
    uint8_t ctrl2 = 0;
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_CTRL2, &ctrl2, 1), TAG, "read ctrl2");
    ctrl2 &= (uint8_t)~(CTRL2_AF | CTRL2_AIE);
    return write_byte(dev, REG_CTRL2, ctrl2);
}

esp_err_t pcf85063a_alarm_triggered(i2c_master_dev_handle_t dev, bool *triggered)
{
    uint8_t ctrl2 = 0;
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_CTRL2, &ctrl2, 1), TAG, "read ctrl2");
    *triggered = (ctrl2 & CTRL2_AF) != 0;
    return ESP_OK;
}

/* Clear the countdown-timer flag TF (Control_2 bit 3). */
esp_err_t pcf85063a_timer_flag_clear(i2c_master_dev_handle_t dev)
{
    uint8_t ctrl2 = 0;
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_CTRL2, &ctrl2, 1), TAG, "read ctrl2");
    ctrl2 &= (uint8_t)~CTRL2_TF;
    return write_byte(dev, REG_CTRL2, ctrl2);
}

/* Arm a countdown timer. value is written to Timer_value (0x10), the clock
 * source/control to Timer_mode (0x11). Order matters: write the value first,
 * then enable TE last so the counter starts from a known value. */
static esp_err_t timer_start(i2c_master_dev_handle_t dev, uint8_t value,
                             uint8_t tcf, bool enable_int)
{
    /* Stop first: changing T with TE=1 is not recommended by the datasheet. */
    ESP_RETURN_ON_ERROR(write_byte(dev, REG_TIMER_MODE, 0x00), TAG, "timer stop");
    ESP_RETURN_ON_ERROR(pcf85063a_timer_flag_clear(dev), TAG, "clear TF");
    ESP_RETURN_ON_ERROR(write_byte(dev, REG_TIMER_VALUE, value), TAG, "timer val");

    /* TI_TP stays 0: the INT line follows the TF flag, so it is held LOW
     * until TF is cleared (matches the LOW_LEVEL GPIO1 wake in power_mgmt). */
    uint8_t mode = tcf | TIMER_TE;
    if (enable_int) {
        mode |= TIMER_TIE;
    }
    return write_byte(dev, REG_TIMER_MODE, mode);
}

esp_err_t pcf85063a_set_timer_seconds(i2c_master_dev_handle_t dev, uint16_t seconds, bool enable_int)
{
    if (seconds == 0 || seconds > 255) {
        return ESP_ERR_INVALID_ARG;
    }
    return timer_start(dev, (uint8_t)seconds, TIMER_TCF_1HZ, enable_int);
}

esp_err_t pcf85063a_set_timer_minutes(i2c_master_dev_handle_t dev, uint8_t minutes, bool enable_int)
{
    if (minutes == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return timer_start(dev, minutes, TIMER_TCF_1MIN, enable_int);
}

esp_err_t pcf85063a_timer_stop(i2c_master_dev_handle_t dev)
{
    /* Disable the timer (TE=0) and its interrupt (TIE=0); clear TF too. */
    ESP_RETURN_ON_ERROR(write_byte(dev, REG_TIMER_MODE, 0x00), TAG, "timer off");
    return pcf85063a_timer_flag_clear(dev);
}

esp_err_t pcf85063a_timer_triggered(i2c_master_dev_handle_t dev, bool *triggered)
{
    uint8_t ctrl2 = 0;
    ESP_RETURN_ON_ERROR(read_regs(dev, REG_CTRL2, &ctrl2, 1), TAG, "read ctrl2");
    *triggered = (ctrl2 & CTRL2_TF) != 0;
    return ESP_OK;
}

esp_err_t pcf85063a_set_offset(i2c_master_dev_handle_t dev, uint8_t raw7)
{
    return write_byte(dev, REG_OFFSET, (uint8_t)(raw7 & 0x7f));
}

esp_err_t pcf85063a_write_ram(i2c_master_dev_handle_t dev, uint8_t val)
{
    return write_byte(dev, REG_RAM, val);
}

esp_err_t pcf85063a_read_ram(i2c_master_dev_handle_t dev, uint8_t *val)
{
    return read_regs(dev, REG_RAM, val, 1);
}

void pcf85063a_debug_dump(i2c_master_dev_handle_t dev)
{
    uint8_t buf[18];
    if (read_regs(dev, REG_CTRL1, buf, sizeof(buf)) != ESP_OK) {
        ESP_LOGE(TAG, "debug dump read failed");
        return;
    }
    ESP_LOGI(TAG,
             "CTRL1=0x%02x CTRL2=0x%02x OFF=0x%02x RAM=0x%02x "
             "TIME=%02x%02x%02x%02x%02x%02x%02x "
             "ALM=%02x%02x%02x%02x%02x "
             "TMRVAL=0x%02x TMRMODE=0x%02x",
             buf[0], buf[1], buf[2], buf[3],
             buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10],
             buf[11], buf[12], buf[13], buf[14], buf[15],
             buf[16], buf[17]);
}
