/*
 * drv2605.c - TI DRV2605 haptic driver (I2C).
 *
 * DRV2605 register map (0x5A):
 *   0x01 Mode (MOD 2:0, RTPIN 6, ERM/LRA 5, STANDBY 7)
 *   0x02 Real-time playback control (input analog/PWM)
 *   0x03 Library selection
 *   0x04-0x0B Waveform sequencer (1..8)
 *   0x0C Go bit
 *   0x16 RATEDV (rated voltage), 0x17 CLAMPV (overdrive clamp)
 *   0x18 AUTOCALCOMP (drive compensation), 0x19 AUTOCALEMP (back-EMF)
 *   0x1D Data (real-time amplitude)
 *
 * The board drives M_EN on the XL9555 (haptic enable); set it high before
 * use. Default actuator type is ERM with library 1 (the T-Watch uses an ERM
 * motor).
 *
 * Auto-calibration: the chip has NO non-volatile storage, so calibration
 * results (AUTOCALCOMP/AUTOCALEMP) are volatile RAM and lost on power-down.
 * The host persists them in NVS ("drv2605" namespace) and restores them on
 * init, avoiding a ~1 s calibration buzz on every boot.
 */
#include "drv2605.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "drv2605";

#define DRV2605_I2C_TIMEOUT_MS 100

#define DRV2605_MODE_REG      0x01
#define DRV2605_LIBRARY_REG   0x03
#define DRV2605_WAVEFORM0     0x04
#define DRV2605_GO_REG        0x0C
#define DRV2605_RATEDV_REG    0x16
#define DRV2605_CLAMPV_REG    0x17
#define DRV2605_AUTOCALCOMP   0x18
#define DRV2605_AUTOCALEMP    0x19
#define DRV2605_DATA_REG      0x1D

#define DRV2605_MODE_INT_TRIGGER 0x00
#define DRV2605_MODE_STANDBY  (1u << 7)
#define DRV2605_MODE_RTPIN    (1u << 6)
#define DRV2605_MODE_ERM_LRA  (1u << 5)   /* 1=LRA, 0=ERM */
#define DRV2605_MODE_AUTOCAL  0x07        /* MOD bits = auto-calibration */

#define DRV2605_NVS_NS        "drv2605"
#define NVS_KEY_COMP          "cal_comp"
#define NVS_KEY_EMF           "cal_emf"

static esp_err_t drv2605_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    return i2c_bus_write(dev, reg, &val, 1, DRV2605_I2C_TIMEOUT_MS);
}

static esp_err_t drv2605_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val)
{
    return i2c_bus_read(dev, reg, val, 1, DRV2605_I2C_TIMEOUT_MS);
}

esp_err_t drv2605_set_waveform(i2c_master_dev_handle_t dev, uint8_t slot, uint8_t wave)
{
    if (!dev || slot > 7) {
        return ESP_ERR_INVALID_ARG;
    }
    return drv2605_write_reg(dev, DRV2605_WAVEFORM0 + slot, wave);
}

esp_err_t drv2605_go(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    return drv2605_write_reg(dev, DRV2605_GO_REG, 1);
}

esp_err_t drv2605_play(i2c_master_dev_handle_t dev, uint8_t wave)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(drv2605_set_waveform(dev, 0, wave), TAG, "set_waveform failed");
    /* Single-effect playback must terminate the whole eight-slot sequence;
     * higher slots may have been configured by an earlier sequencer user. */
    for (uint8_t slot = 1; slot < 8; ++slot) {
        ESP_RETURN_ON_ERROR(drv2605_set_waveform(dev, slot, 0), TAG, "clear sequence slot");
    }
    return drv2605_go(dev);
}

/* Save calibration values to NVS and report persistence failures to caller. */
static esp_err_t cal_save(uint8_t comp, uint8_t emf)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(DRV2605_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, NVS_KEY_COMP, comp);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, NVS_KEY_EMF, emf);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* Read stored calibration; returns true if both values exist. */
static bool cal_load(uint8_t *comp, uint8_t *emf)
{
    if (!comp || !emf) {
        return false;
    }
    nvs_handle_t h;
    if (nvs_open(DRV2605_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    uint8_t comp_value = 0, emf_value = 0;
    bool ok = (nvs_get_u8(h, NVS_KEY_COMP, &comp_value) == ESP_OK &&
               nvs_get_u8(h, NVS_KEY_EMF, &emf_value) == ESP_OK);
    nvs_close(h);
    if (ok) {
        *comp = comp_value;
        *emf = emf_value;
    }
    return ok;
}

/* Run the on-chip auto-calibration. Blocks until the GO bit self-clears
 * (~1 s); the motor vibrates during this. Returns the compensation value. */
esp_err_t drv2605_auto_calibrate(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_AUTOCAL);
    if (err != ESP_OK) {
        return err;
    }
    err = drv2605_go(dev);
    if (err != ESP_OK) {
        esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
        return restore == ESP_OK ? err : restore;
    }

    /* Wait for GO to self-clear (indicates calibration finished). */
    bool completed = false;
    for (int i = 0; i < 200; i++) {
        uint8_t go = 0;
        err = drv2605_read_reg(dev, DRV2605_GO_REG, &go);
        if (err != ESP_OK) {
            esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
            return restore == ESP_OK ? err : restore;
        }
        if ((go & 0x01) == 0) {
            completed = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!completed) {
        esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
        return restore == ESP_OK ? ESP_ERR_TIMEOUT : restore;
    }

    uint8_t comp = 0, emf = 0;
    err = drv2605_read_reg(dev, DRV2605_AUTOCALCOMP, &comp);
    if (err == ESP_OK) {
        err = drv2605_read_reg(dev, DRV2605_AUTOCALEMP, &emf);
    }

    /* Always leave calibration mode, including the read-error path. */
    esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
    if (err != ESP_OK) {
        return err;
    }
    if (restore != ESP_OK) {
        return restore;
    }

    err = cal_save(comp, emf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to save calibration: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "auto-calibration: comp=0x%02X emf=0x%02X (saved)", comp, emf);
    return ESP_OK;
}

/* Restore previously calibrated values (RATEDV/CLAMPV/AUTOCAL*). Returns true
 * if a stored calibration was applied. */
esp_err_t drv2605_calibrate_restore(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t comp = 0, emf = 0;
    if (!cal_load(&comp, &emf)) {
        return ESP_ERR_NOT_FOUND;
    }
    /* Calibration constants live in volatile device registers; these writes
     * don't drive the actuator and can safely restore on the common rail. */
    ESP_RETURN_ON_ERROR(drv2605_write_reg(dev, DRV2605_AUTOCALCOMP, comp), TAG, "restore comp");
    ESP_RETURN_ON_ERROR(drv2605_write_reg(dev, DRV2605_AUTOCALEMP, emf), TAG, "restore emf");
    ESP_LOGI(TAG, "restored calibration comp=0x%02X emf=0x%02X", comp, emf);
    return ESP_OK;
}

esp_err_t drv2605_init(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Take out of standby, internal trigger, ERM (default library 1). */
    ESP_RETURN_ON_ERROR(drv2605_write_reg(dev, DRV2605_MODE_REG,
                                          DRV2605_MODE_INT_TRIGGER),
                        TAG, "mode");
    ESP_RETURN_ON_ERROR(drv2605_write_reg(dev, DRV2605_LIBRARY_REG, 1), TAG, "library");
    /* Read back the mode register to confirm the chip is present. */
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(drv2605_read_reg(dev, DRV2605_MODE_REG, &val), TAG, "readback");
    ESP_LOGI(TAG, "DRV2605 initialized (mode=0x%02X, ERM, library 1)", val);

    /* Restore a previously saved calibration if available. Auto-calibration
     * itself is NOT run here: it needs the motor rail (M_EN) enabled, which
     * the board keeps off during init. Run it explicitly via "motor cal" or
     * a UI path that enables M_EN first. */
    if (drv2605_calibrate_restore(dev) == ESP_OK) {
        ESP_LOGI(TAG, "calibration restored from NVS");
    }
    return ESP_OK;
}
