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
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "drv2605";

#define DRV2605_I2C_TIMEOUT_MS 100
#define DRV2605_CAL_TIMEOUT_MS 1500
#define DRV2605_POLL_INTERVAL_MS 10
#define DRV2605_LIBRARY1_MAX_WAVE 123

#define DRV2605_MODE_REG      0x01
#define DRV2605_LIBRARY_REG   0x03
#define DRV2605_WAVEFORM0     0x04
#define DRV2605_GO_REG        0x0C
#define DRV2605_AUTOCALCOMP   0x18
#define DRV2605_AUTOCALEMP    0x19

#define DRV2605_MODE_INT_TRIGGER 0x00
#define DRV2605_MODE_AUTOCAL  0x07        /* MOD bits = auto-calibration */

#define DRV2605_NVS_NS        "drv2605"
#define NVS_KEY_COMP          "cal_comp"
#define NVS_KEY_EMF           "cal_emf"

static SemaphoreHandle_t s_dev_lock;
static portMUX_TYPE s_dev_lock_init_mux = portMUX_INITIALIZER_UNLOCKED;
static i2c_master_dev_handle_t s_dev_handle;

static esp_err_t drv2605_dev_lock(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_dev_lock_init_mux);
    SemaphoreHandle_t lock = s_dev_lock;
    portEXIT_CRITICAL(&s_dev_lock_init_mux);
    if (!lock) {
        SemaphoreHandle_t created = xSemaphoreCreateMutex();
        if (!created) {
            return ESP_ERR_NO_MEM;
        }
        portENTER_CRITICAL(&s_dev_lock_init_mux);
        if (!s_dev_lock) {
            s_dev_lock = created;
            lock = created;
            created = NULL;
        } else {
            lock = s_dev_lock;
        }
        portEXIT_CRITICAL(&s_dev_lock_init_mux);
        if (created) {
            vSemaphoreDelete(created);
        }
    }

    if (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_dev_lock_init_mux);
    bool wrong_dev = s_dev_handle && s_dev_handle != dev;
    if (!wrong_dev) {
        s_dev_handle = dev;
    }
    portEXIT_CRITICAL(&s_dev_lock_init_mux);
    if (wrong_dev) {
        xSemaphoreGive(lock);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static void drv2605_dev_unlock(void)
{
    xSemaphoreGive(s_dev_lock);
}

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
    if (!dev || slot > 7 || wave > DRV2605_LIBRARY1_MAX_WAVE) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = drv2605_dev_lock(dev);
    if (err != ESP_OK) {
        return err;
    }
    err = drv2605_write_reg(dev, DRV2605_WAVEFORM0 + slot, wave);
    drv2605_dev_unlock();
    return err;
}

static esp_err_t drv2605_go_locked(i2c_master_dev_handle_t dev)
{
    return drv2605_write_reg(dev, DRV2605_GO_REG, 1);
}

esp_err_t drv2605_go(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = drv2605_dev_lock(dev);
    if (err != ESP_OK) {
        return err;
    }
    err = drv2605_go_locked(dev);
    drv2605_dev_unlock();
    return err;
}

esp_err_t drv2605_play(i2c_master_dev_handle_t dev, uint8_t wave)
{
    if (!dev || wave == 0 || wave > DRV2605_LIBRARY1_MAX_WAVE) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(drv2605_dev_lock(dev), TAG, "device lock");

    esp_err_t err = drv2605_write_reg(dev, DRV2605_WAVEFORM0, wave);
    if (err == ESP_OK) {
        /* Single-effect playback must terminate the whole eight-slot sequence;
         * higher slots may have been configured by an earlier sequencer user. */
        for (uint8_t slot = 1; slot < 8 && err == ESP_OK; ++slot) {
            err = drv2605_write_reg(dev, DRV2605_WAVEFORM0 + slot, 0);
        }
    }
    if (err == ESP_OK) {
        err = drv2605_go_locked(dev);
    }
    drv2605_dev_unlock();
    return err;
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

/* Read stored calibration; return NOT_FOUND only if the namespace/key pair is
 * absent, while propagating storage errors so callers do not mask NVS failure. */
static esp_err_t cal_load(uint8_t *comp, uint8_t *emf)
{
    if (!comp || !emf) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(DRV2605_NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }
    uint8_t comp_value = 0, emf_value = 0;
    err = nvs_get_u8(h, NVS_KEY_COMP, &comp_value);
    if (err == ESP_OK) {
        err = nvs_get_u8(h, NVS_KEY_EMF, &emf_value);
    }
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err == ESP_OK) {
        *comp = comp_value;
        *emf = emf_value;
    }
    return err;
}

/* Run the on-chip auto-calibration. Blocks until GO self-clears (typically
 * about 1 s), bounded to 1.5 s; the motor vibrates during calibration. */
static esp_err_t drv2605_auto_calibrate_locked(i2c_master_dev_handle_t dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_AUTOCAL);
    if (err != ESP_OK) {
        esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
        return restore == ESP_OK ? err : restore;
    }
    err = drv2605_go_locked(dev);
    if (err != ESP_OK) {
        esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
        return restore == ESP_OK ? err : restore;
    }

    /* Wait for GO to self-clear (indicates calibration finished), bounded by
     * the datasheet-scale ~1 s calibration duration plus a scheduling margin. */
    bool completed = false;
    int64_t deadline_us = esp_timer_get_time() +
                          (int64_t)DRV2605_CAL_TIMEOUT_MS * 1000;
    while (esp_timer_get_time() < deadline_us) {
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
        vTaskDelay(pdMS_TO_TICKS(DRV2605_POLL_INTERVAL_MS));
    }

    if (!completed) {
        uint8_t go = 0;
        esp_err_t check = drv2605_read_reg(dev, DRV2605_GO_REG, &go);
        esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
        if (check != ESP_OK) {
            return restore == ESP_OK ? check : restore;
        }
        if ((go & 0x01) == 0) {
            completed = true;
        } else {
            return restore == ESP_OK ? ESP_ERR_TIMEOUT : restore;
        }
    }
    if (!completed) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t comp = 0, emf = 0;
    err = drv2605_read_reg(dev, DRV2605_AUTOCALCOMP, &comp);
    if (err == ESP_OK) {
        err = drv2605_read_reg(dev, DRV2605_AUTOCALEMP, &emf);
    }

    /* Always leave calibration mode, including the read-error path. */
    esp_err_t restore = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
    if (restore != ESP_OK) {
        return restore;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = cal_save(comp, emf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to save calibration: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "auto-calibration: comp=0x%02X emf=0x%02X (saved)", comp, emf);
    return ESP_OK;
}

/* Restore the stored volatile AUTOCALCOMP/AUTOCALEMP values. Returns
 * ESP_ERR_NOT_FOUND if a complete calibration pair is not stored. */
static esp_err_t drv2605_calibrate_restore_locked(i2c_master_dev_handle_t dev)
{
    uint8_t comp = 0, emf = 0;
    esp_err_t err = cal_load(&comp, &emf);
    if (err != ESP_OK) {
        return err;
    }
    /* Calibration constants live in volatile device registers; these writes
     * don't drive the actuator and can safely restore on the common rail. */
    err = drv2605_write_reg(dev, DRV2605_AUTOCALCOMP, comp);
    if (err == ESP_OK) {
        err = drv2605_write_reg(dev, DRV2605_AUTOCALEMP, emf);
    }
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "restored calibration comp=0x%02X emf=0x%02X", comp, emf);
    return ESP_OK;
}

esp_err_t drv2605_calibrate_restore(i2c_master_dev_handle_t dev)
{
    esp_err_t err = drv2605_dev_lock(dev);
    if (err != ESP_OK) {
        return err;
    }
    err = drv2605_calibrate_restore_locked(dev);
    drv2605_dev_unlock();
    return err;
}

esp_err_t drv2605_auto_calibrate(i2c_master_dev_handle_t dev)
{
    esp_err_t err = drv2605_dev_lock(dev);
    if (err != ESP_OK) {
        return err;
    }
    err = drv2605_auto_calibrate_locked(dev);
    drv2605_dev_unlock();
    return err;
}

esp_err_t drv2605_init(i2c_master_dev_handle_t dev)
{
    esp_err_t err = drv2605_dev_lock(dev);
    if (err != ESP_OK) {
        return err;
    }
    /* Take out of standby, internal trigger, ERM (default library 1). */
    err = drv2605_write_reg(dev, DRV2605_MODE_REG, DRV2605_MODE_INT_TRIGGER);
    if (err == ESP_OK) {
        err = drv2605_write_reg(dev, DRV2605_LIBRARY_REG, 1);
    }
    /* Read back the mode register to confirm the chip is present. */
    uint8_t val = 0;
    if (err == ESP_OK) {
        err = drv2605_read_reg(dev, DRV2605_MODE_REG, &val);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "DRV2605 initialized (mode=0x%02X, ERM, library 1)", val);
        /* Auto-calibration needs M_EN, which the board keeps off during init;
         * only restore persisted volatile values here. */
        esp_err_t restore = drv2605_calibrate_restore_locked(dev);
        if (restore == ESP_OK) {
            ESP_LOGI(TAG, "calibration restored from NVS");
        } else if (restore != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "failed to restore calibration: %s", esp_err_to_name(restore));
            err = restore;
        }
    }
    drv2605_dev_unlock();
    return err;
}
