#include "bsp_cst9217.h"
#include "bsp_i2c.h"
#include "bsp_xl9555.h"
#include "bsp_twatch_ultra.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp_cst9217";

#define CST9217_REG_READ       0xD000
#define CST9217_ACK            0xAB
#define CST9217_MAX_POINTS     2
#define CST9217_POINT_EVENT_DOWN 0x06

/* Commands from TouchDrvCST92xx (LilyGo library for this panel) */
#define CST9217_CMD_ENTER_CMD     0x01
#define CST9217_CMD_NORMAL        0x09
#define CST9217_CMD_CHECKCODE     0xFC
#define CST9217_CMD_RESOLUTION    0xF8
#define CST9217_CMD_CHIP_TYPE     0x04
#define CST9217_CMD_CHIP_TYPE_REG 0xD2
#define CST9217_CMD_FIRST_BYTE    0xD1

static bool s_online = false;
static uint8_t s_touch_addr = 0x5A;

esp_err_t bsp_touch_init(void)
{
    /* Reset touch controller through the XL9555 expander.
     * Datasheet: RST pulse >= 0.1ms, re-init after reset = 100ms. */
    ESP_RETURN_ON_ERROR(bsp_xl9555_write_pin(XL9555_PIN_TOUCH_RST, false), TAG, "touch rst low failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(bsp_xl9555_write_pin(XL9555_PIN_TOUCH_RST, true), TAG, "touch rst high failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Scan for the touch controller: LilyGo probes 0x1A first, then 0x5A */
    static const uint8_t candidates[] = { 0x1A, 0x5A };
    uint8_t found = 0;
    for (size_t i = 0; i < sizeof(candidates); i++) {
        uint8_t probe = 0;
        if (bsp_i2c_read_reg(candidates[i], 0x00, &probe, 1) == ESP_OK) {
            found = candidates[i];
            ESP_LOGI(TAG, "touch controller found at 0x%02X", found);
            break;
        }
    }
    if (found == 0) {
        ESP_LOGE(TAG, "no touch controller found (tried 0x1A, 0x5A)");
        return ESP_ERR_NOT_FOUND;
    }
    s_touch_addr = found;

    uint8_t cmd[3];
    uint8_t rbuf[8];
    esp_err_t err;

    /* ESPHome reference init: write register 0xD101 = 0x01 (3-byte: 0xD1 0x01 0x01)
     * to enable touch reporting without leaving the chip's normal mode. */
    cmd[0] = CST9217_CMD_FIRST_BYTE;
    cmd[1] = CST9217_CMD_ENTER_CMD;
    cmd[2] = 0x01;
    err = bsp_i2c_write_raw(s_touch_addr, cmd, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enable reporting failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Read chip info while in command mode */
    cmd[0] = CST9217_CMD_FIRST_BYTE;
    cmd[1] = CST9217_CMD_CHECKCODE;
    ESP_RETURN_ON_ERROR(bsp_i2c_transmit_receive(s_touch_addr, cmd, 2, rbuf, 4),
                        TAG, "checkcode read failed");
    uint32_t checkcode = ((uint32_t)rbuf[3] << 24) | ((uint32_t)rbuf[2] << 16) |
                         ((uint32_t)rbuf[1] << 8) | rbuf[0];

    cmd[0] = CST9217_CMD_FIRST_BYTE;
    cmd[1] = CST9217_CMD_RESOLUTION;
    ESP_RETURN_ON_ERROR(bsp_i2c_transmit_receive(s_touch_addr, cmd, 2, rbuf, 4),
                        TAG, "resolution read failed");
    uint16_t res_x = (uint16_t)((rbuf[1] << 8) | rbuf[0]);
    uint16_t res_y = (uint16_t)((rbuf[3] << 8) | rbuf[2]);

    cmd[0] = CST9217_CMD_CHIP_TYPE_REG;
    cmd[1] = CST9217_CMD_CHIP_TYPE;
    ESP_RETURN_ON_ERROR(bsp_i2c_transmit_receive(s_touch_addr, cmd, 2, rbuf, 4),
                        TAG, "chip type read failed");
    uint32_t chip_type = ((uint32_t)rbuf[3] << 24) | ((uint32_t)rbuf[2] << 16) |
                         ((uint32_t)rbuf[1] << 8) | rbuf[0];

    ESP_LOGI(TAG, "chip checkcode=0x%08lX res=%ux%u type=0x%08lX",
             (unsigned long)checkcode, res_x, res_y, (unsigned long)chip_type);

    /* Return the chip to normal mode (TouchDrvCST92xx::set_work_mode) and
     * verify via the 0x0002 echo register: byte1 must read back 0x09. */
    cmd[0] = CST9217_CMD_FIRST_BYTE;
    cmd[1] = CST9217_CMD_NORMAL;
    err = bsp_i2c_write_raw(s_touch_addr, cmd, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "normal mode write failed: %s", esp_err_to_name(err));
        return err;
    }
    cmd[0] = 0x00;
    cmd[1] = 0x02;
    if (bsp_i2c_transmit_receive(s_touch_addr, cmd, 2, rbuf, 2) == ESP_OK) {
        if (rbuf[1] == CST9217_CMD_NORMAL) {
            ESP_LOGI(TAG, "normal mode confirmed (echo 0x09)");
        } else {
            ESP_LOGW(TAG, "normal mode echo mismatch: 0x%02X 0x%02X", rbuf[0], rbuf[1]);
        }
    } else {
        ESP_LOGW(TAG, "normal mode verify read failed");
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    s_online = true;
    ESP_LOGI(TAG, "CST9217 touch ready");
    return ESP_OK;
}

bool bsp_touch_get_point(uint16_t *x, uint16_t *y)
{
    uint8_t buf[CST9217_MAX_POINTS * 5 + 5] = { 0 };
    uint8_t cmd[2] = { (uint8_t)(CST9217_REG_READ >> 8), (uint8_t)(CST9217_REG_READ & 0xFF) };

    if (!s_online) {
        return false;
    }

    static uint32_t s_read_cnt;
    static uint32_t s_fail_cnt;
    esp_err_t rerr = bsp_i2c_transmit_receive(s_touch_addr, cmd, 2, buf, sizeof(buf));
    if (rerr != ESP_OK) {
        if ((s_fail_cnt++ % 20) == 0) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(rerr));
        }
        return false;
    }

    if ((s_read_cnt++ % 5) == 0 || buf[0] != 0xFF || (buf[5] & 0x7F) != 0) {
        ESP_LOGD(TAG, "raw[%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]",
                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
                 buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14]);
    }

    if (buf[0] == CST9217_ACK || buf[6] != CST9217_ACK || buf[0] == 0x00) {
        return false;
    }

    uint8_t num = buf[5] & 0x7F;
    if (num == 0 || num > CST9217_MAX_POINTS) {
        return false;
    }

    uint8_t *p = buf; /* first point at offset 0, second at offset 7 */
    uint8_t event = p[0] & 0x0F;
    uint8_t id = p[0] >> 4;

    if (event == CST9217_POINT_EVENT_DOWN && id < CST9217_MAX_POINTS) {
        *x = ((uint16_t)(p[1]) << 4) | (p[3] >> 4);
        *y = ((uint16_t)(p[2]) << 4) | (p[3] & 0x0F);
        ESP_LOGD(TAG, "point id=%u (%u,%u) num=%u", id, *x, *y, num);
        return true;
    }
    return false;
}
