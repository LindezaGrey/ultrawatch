/*
 * bhi260ap.c - Bosch BHI260AP smart sensor / IMU (I2C, Bosch FSC protocol).
 *
 * Uses the Bosch BHI2xy Sensor API (BSD-3, vendored in this component) to
 * bring the chip up: upload the RAM firmware over I2C, boot it, and stream
 * sensor data from the FIFO.
 *
 * The firmware blob is loaded from the SPIFFS "assets" partition
 * (/assets/bhi260/BHI260AP.fw) at init; call bhi260ap_init() only after the
 * assets partition is mounted. On failure the driver logs and continues
 * (the watch must keep booting even if the IMU is absent/faulty).
 */
#include "bhi260ap.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "bhi260ap";

#define BHI260_I2C_TIMEOUT_MS   1000
#define BHY2_RD_WR_LEN          256
#define BHY2_FIFO_BUFFER_SIZE   512

#define FIRMWARE_PATH           "/assets/bhi260/BHI260AP.fw"

static struct bhy2_dev s_bhy2;
static bool s_initialized;
static volatile bool s_ap_suspended;
static uint32_t s_step_count;
static uint32_t s_step_base;   /* persisted offset: total = base + count */
static uint32_t s_last_data_tick;   /* tick at last sensor event (freshness) */

/* Cached sensor state (written by FIFO parse callbacks in the polling task,
 * read by UI/debug accessors). No locking needed: fields are 32-bit or less
 * and the UI reads them from a lower-priority task. */
static struct bhy2_data_xyz s_accel;
static struct bhy2_data_xyz s_gyro;
static struct bhy2_data_quaternion s_rv;
static bhi260ap_activity_t s_activity = BHI260AP_ACTIVITY_UNKNOWN;
static volatile bool s_wrist_tilt;
static volatile bool s_wake_gesture;
static volatile bool s_glance_gesture;
static volatile bool s_pickup_gesture;
static volatile bool s_tilt_detector;

/* ---- Step counter persistence (NVS) ----
 * The BHI260AP's on-chip step counter resets on every RAM-firmware upload
 * (each boot). The reported total is
 *     s_step_base + (s_step_count - s_step_at_base)
 * where s_step_base is persisted to NVS and s_step_at_base is the chip count
 * when the base was last folded (RAM only, starts at 0 on boot). Steps between
 * folds are at most STEP_FOLD_STEP lost on sudden power loss. */
#define STEP_NVS_NS     "bhi260ap"
#define STEP_NVS_KEY    "step_base"
#define STEP_FOLD_STEP  100   /* fold chip steps into the base every ~100 */

static uint32_t s_step_at_base;

static void step_base_load(void)
{
    nvs_handle_t h;
    if (nvs_open(STEP_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        s_step_base = 0;
        return;
    }
    uint32_t v = 0;
    if (nvs_get_u32(h, STEP_NVS_KEY, &v) != ESP_OK) {
        v = 0;
    }
    nvs_close(h);
    s_step_base = v;
}

static void step_base_save(void)
{
    nvs_handle_t h;
    if (nvs_open(STEP_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u32(h, STEP_NVS_KEY, s_step_base);
    nvs_commit(h);
    nvs_close(h);
}

/* Fold the steps accumulated since s_step_at_base into the persisted base. */
static void step_fold(void)
{
    uint32_t delta = s_step_count - s_step_at_base;
    if (delta == 0) {
        return;
    }
    s_step_base += delta;
    s_step_at_base = s_step_count;
    step_base_save();
}

/* ---- I2C glue for the Bosch API ---- */

static int8_t bhi260ap_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t ret = i2c_master_transmit_receive(dev, &reg_addr, 1, reg_data, length, BHI260_I2C_TIMEOUT_MS);
    return (ret == ESP_OK) ? BHY2_OK : BHY2_E_IO;
}

static int8_t bhi260ap_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    uint8_t *buf = heap_caps_malloc(length + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) {
        return BHY2_E_IO;
    }
    buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, length);
    esp_err_t ret = i2c_master_transmit(dev, buf, length + 1, BHI260_I2C_TIMEOUT_MS);
    free(buf);
    return (ret == ESP_OK) ? BHY2_OK : BHY2_E_IO;
}

static void bhi260ap_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period_us);
}

/* ---- FIFO parse callbacks ---- */

/* Called on any real sensor sample; marks the data stream as alive. */
static void bhi260ap_mark_data(void)
{
    s_last_data_tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* Age of the newest sensor sample in ms, or a huge value if none ever arrived.
 * A live accel stream at 12.5 Hz keeps this well under 1 s. */
uint32_t bhi260ap_get_data_age_ms(void)
{
    if (!s_last_data_tick) {
        return UINT32_MAX;
    }
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    return now - s_last_data_tick;
}

bool bhi260ap_is_suspended(void)
{
    return s_ap_suspended;
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    ESP_LOGD(TAG, "meta event: sid=%u data_size=%u", callback_info->sensor_id, callback_info->data_size);
}

static void parse_step_counter(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size < 4) {
        return;
    }
    bhi260ap_mark_data();
    uint32_t steps = BHY2_LE2U32(callback_info->data_ptr);
    /* The chip counter restarts at 0 after a firmware upload / rail power-cycle.
     * Fold the accumulated chip steps into the base so the total is monotonic. */
    if (steps < s_step_count) {
        step_fold();
        s_step_count = steps;
        s_step_at_base = steps;
    } else {
        s_step_count = steps;
    }
    if (s_step_count - s_step_at_base >= STEP_FOLD_STEP) {
        step_fold();
    }
    ESP_LOGI(TAG, "step counter: %lu (total %lu)",
             (unsigned long)steps, (unsigned long)(s_step_base + (s_step_count - s_step_at_base)));
}

static void parse_accel(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 6) {
        bhi260ap_mark_data();
        bhy2_parse_xyz(callback_info->data_ptr, &s_accel);
    }
}

static void parse_gyro(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 6) {
        bhi260ap_mark_data();
        bhy2_parse_xyz(callback_info->data_ptr, &s_gyro);
    }
}

static void parse_rotation_vector(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 10) {
        bhy2_parse_quaternion(callback_info->data_ptr, &s_rv);
    }
}

static void parse_activity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size < 2) {
        return;
    }
    uint16_t activity = BHY2_LE2U16(callback_info->data_ptr);
    if (activity & BHY2_STILL_ACTIVITY_STARTED) {
        s_activity = BHI260AP_ACTIVITY_STILL;
    } else if (activity & BHY2_WALKING_ACTIVITY_STARTED) {
        s_activity = BHI260AP_ACTIVITY_WALKING;
    } else if (activity & BHY2_RUNNING_ACTIVITY_STARTED) {
        s_activity = BHI260AP_ACTIVITY_RUNNING;
    } else if (activity & BHY2_ON_BICYCLE_ACTIVITY_STARTED) {
        s_activity = BHI260AP_ACTIVITY_ON_BICYCLE;
    } else if (activity & BHY2_IN_VEHICLE_ACTIVITY_STARTED) {
        s_activity = BHI260AP_ACTIVITY_IN_VEHICLE;
    } else if (activity & BHY2_TILTING_ACTIVITY_STARTED) {
        s_activity = BHI260AP_ACTIVITY_TILTING;
    }
    ESP_LOGD(TAG, "activity: 0x%04x -> %d", activity, (int)s_activity);
}

static void parse_wrist_tilt(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 1 && callback_info->data_ptr[0]) {
        s_wrist_tilt = true;
        ESP_LOGI(TAG, "wrist tilt event (fifo=%d)", (int)callback_info->fifo_type);
    }
}

static void parse_wake_gesture(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 1 && callback_info->data_ptr[0]) {
        s_wake_gesture = true;
        ESP_LOGI(TAG, "wake gesture event (fifo=%d)", (int)callback_info->fifo_type);
    }
}

static void parse_glance_gesture(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 1 && callback_info->data_ptr[0]) {
        s_glance_gesture = true;
    }
}

static void parse_pickup_gesture(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 1 && callback_info->data_ptr[0]) {
        s_pickup_gesture = true;
    }
}

static void parse_tilt_detector(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size >= 1 && callback_info->data_ptr[0]) {
        s_tilt_detector = true;
    }
}

/* Load the RAM firmware blob from SPIFFS into a PSRAM buffer. */
static esp_err_t load_firmware(uint8_t **out, size_t *out_len)
{
    FILE *f = fopen(FIRMWARE_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "open %s failed", FIRMWARE_PATH);
        return ESP_ERR_NOT_FOUND;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return ESP_FAIL;
    }
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        fclose(f);
        return ESP_FAIL;
    }

    uint8_t *buf = heap_caps_malloc((size_t)size, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "fw buffer alloc failed (%ld B)", size);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }
    size_t rd = fread(buf, 1, (size_t)size, f);
    fclose(f);
    if (rd != (size_t)size) {
        ESP_LOGE(TAG, "fw read %u/%ld", (unsigned)rd, size);
        free(buf);
        return ESP_FAIL;
    }

    *out = buf;
    *out_len = (size_t)size;
    ESP_LOGI(TAG, "loaded firmware: %u bytes", (unsigned)size);
    return ESP_OK;
}

esp_err_t bhi260ap_init(i2c_master_dev_handle_t dev)
{
    int8_t rslt;
    uint8_t product_id = 0;
    uint8_t boot_status = 0;
    uint16_t kernel_version = 0;
    uint8_t *fw = NULL;
    size_t fw_len = 0;

    ESP_RETURN_ON_ERROR(load_firmware(&fw, &fw_len), TAG, "load firmware");

    rslt = bhy2_init(BHY2_I2C_INTERFACE, bhi260ap_i2c_read, bhi260ap_i2c_write,
                     bhi260ap_delay_us, BHY2_RD_WR_LEN, dev, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "bhy2_init failed: %d", rslt);
        free(fw);
        return ESP_FAIL;
    }

    rslt = bhy2_soft_reset(&s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "soft reset failed: %d", rslt);
        free(fw);
        return ESP_FAIL;
    }

    rslt = bhy2_get_product_id(&product_id, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "get product id failed: %d", rslt);
        free(fw);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "product id: 0x%02X (expected 0x%02X)", product_id, BHY2_PRODUCT_ID);

    /* Disable status/debug FIFO interrupts, enable normal FIFO. */
    /* Host interrupt only for the WAKE-UP FIFO (gesture wake). The
     * non-wakeup FIFO (continuous ACC/GYRO/etc. streams) must not assert INT
     * while the host is awake - the polling task drains it, and a live INT
     * line would flood the wake task. Status/debug FIFOs are unused. */
    bhy2_set_host_interrupt_ctrl(BHY2_ICTL_DISABLE_FIFO_NW |
                                 BHY2_ICTL_DISABLE_STATUS_FIFO |
                                 BHY2_ICTL_DISABLE_DEBUG, &s_bhy2);
    bhy2_set_host_intf_ctrl(0, &s_bhy2);

    rslt = bhy2_get_boot_status(&boot_status, &s_bhy2);
    if (rslt != BHY2_OK || !(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
        ESP_LOGE(TAG, "host interface not ready: rslt=%d status=0x%02X", rslt, boot_status);
        free(fw);
        return ESP_FAIL;
    }

    rslt = bhy2_upload_firmware_to_ram(fw, (uint32_t)fw_len, &s_bhy2);
    free(fw);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "firmware upload failed: %d", rslt);
        return ESP_FAIL;
    }

    rslt = bhy2_boot_from_ram(&s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGE(TAG, "boot from RAM failed: %d", rslt);
        return ESP_FAIL;
    }

    rslt = bhy2_get_kernel_version(&kernel_version, &s_bhy2);
    if (rslt != BHY2_OK || kernel_version == 0) {
        ESP_LOGE(TAG, "kernel version check failed: rslt=%d ver=%u", rslt, kernel_version);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "RAM firmware booted, kernel version %u", kernel_version);

    /* Register FIFO parse callbacks. */
    bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC, parse_step_counter, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_STC_WU, parse_step_counter, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, parse_accel, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC_WU, parse_accel, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO, parse_gyro, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO_WU, parse_gyro, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAMERV, parse_rotation_vector, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAMERV_WU, parse_rotation_vector, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_AR, parse_activity, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_WRIST_TILT_GESTURE, parse_wrist_tilt, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_WAKE_GESTURE, parse_wake_gesture, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GLANCE_GESTURE, parse_glance_gesture, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_PICKUP_GESTURE, parse_pickup_gesture, NULL, &s_bhy2);
    bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_TILT_DETECTOR, parse_tilt_detector, NULL, &s_bhy2);

    rslt = bhy2_update_virtual_sensor_list(&s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "update sensor list failed: %d", rslt);
    }

    /* Enable virtual sensors (non-fatal: the watch keeps running if one is
     * unavailable in the loaded firmware). */
    struct {
        uint8_t id;
        bhy2_float rate;
    } enable[] = {
        { BHY2_SENSOR_ID_STC, 1.0f },
        { BHY2_SENSOR_ID_ACC, 12.5f },
        { BHY2_SENSOR_ID_GYRO, 12.5f },
        { BHY2_SENSOR_ID_GAMERV, 5.0f },
        { BHY2_SENSOR_ID_AR, 5.0f },
        { BHY2_SENSOR_ID_WRIST_TILT_GESTURE, 1.0f },
        { BHY2_SENSOR_ID_WAKE_GESTURE, 1.0f },
        { BHY2_SENSOR_ID_GLANCE_GESTURE, 1.0f },
        { BHY2_SENSOR_ID_PICKUP_GESTURE, 1.0f },
        { BHY2_SENSOR_ID_TILT_DETECTOR, 1.0f },
    };
    for (size_t i = 0; i < sizeof(enable) / sizeof(enable[0]); i++) {
        rslt = bhy2_set_virt_sensor_cfg(enable[i].id, enable[i].rate, 0, &s_bhy2);
        if (rslt != BHY2_OK) {
            ESP_LOGW(TAG, "enable sensor %u @ %.1f Hz failed: %d", enable[i].id, enable[i].rate, rslt);
        }
    }

    /* Arm the host-interrupt wake path: the chip asserts its INT line (GPIO8)
     * when the wake-up FIFO fills to this watermark, so a gesture/wake event
     * wakes the sleeping host. The default watermark is 0 (never fires).
     * A single wake-gesture event is small, so keep the threshold low. */
    rslt = bhy2_set_fifo_wmark_wkup(4, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "set wake-up FIFO watermark failed: %d", rslt);
    }

    /* Start the staleness clock now, so the re-init logic doesn't see an
     * immediate "stale" before the first FIFO sample arrives. */
    bhi260ap_mark_data();
    step_base_load();
    s_step_at_base = 0;
    s_step_count = 0;
    s_initialized = true;
    ESP_LOGI(TAG, "BHI260AP ready (persisted steps %lu)",
             (unsigned long)s_step_base);
    return ESP_OK;
}

/* Poll the FIFO once; returns ESP_OK on success. */
esp_err_t bhi260ap_process_fifo(void)
{
    if (!s_initialized || s_ap_suspended) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t work[BHY2_FIFO_BUFFER_SIZE];
    int8_t rslt = bhy2_get_and_process_fifo(work, sizeof(work), &s_bhy2);
    return (rslt == BHY2_OK) ? ESP_OK : ESP_FAIL;
}

/* Tell the BHI260AP the host (ESP32) is going to sleep. It then runs only the
 * wake-up sensors (*_WU variants / gesture sensors) at low power and stops the
 * high-rate non-wakeup streams. Per the datasheet, flush the FIFO first so a
 * stale event doesn't immediately re-wake the host. ALDO4 must stay powered;
 * do not power-cycle the sensor rail while suspended. */
esp_err_t bhi260ap_ap_suspend(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    step_fold();   /* persist steps accumulated since the last fold */
    s_ap_suspended = true;
    bhy2_flush_fifo(0xFF, &s_bhy2);   /* 0xFF = flush all virtual sensors */
    uint8_t intr_ctrl = 0;
    bhy2_get_host_interrupt_ctrl(&intr_ctrl, &s_bhy2);
    int8_t rslt = bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_AP_SUSPENDED, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "AP suspend failed: %d", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "AP suspend (host sleeping) intr_ctrl=0x%02x gpio8=%d",
             (unsigned)intr_ctrl, (int)gpio_get_level(GPIO_NUM_8));
    return ESP_OK;
}

/* Host is awake again; resume the normal (non-wakeup) sensor streams. */
esp_err_t bhi260ap_ap_resume(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    int8_t rslt = bhy2_set_host_intf_ctrl(0, &s_bhy2);
    s_ap_suspended = false;
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "AP resume failed: %d", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "AP resume (host awake)");
    return ESP_OK;
}

/* Reset the driver to the uninitialized state. Called when the sensor rail is
 * power-cycled (auto-sleep), after which the chip must be brought up again. */
void bhi260ap_deinit(void)
{
    s_initialized = false;
    s_ap_suspended = false;
    s_step_count = 0;
    s_step_at_base = 0;
    s_last_data_tick = 0;
    memset(&s_accel, 0, sizeof(s_accel));
    memset(&s_gyro, 0, sizeof(s_gyro));
    memset(&s_rv, 0, sizeof(s_rv));
    s_activity = BHI260AP_ACTIVITY_UNKNOWN;
    s_wrist_tilt = false;
    s_wake_gesture = false;
    s_glance_gesture = false;
    s_pickup_gesture = false;
    s_tilt_detector = false;
}

esp_err_t bhi260ap_get_step_count(uint32_t *steps)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    *steps = s_step_base + (s_step_count - s_step_at_base);
    return ESP_OK;
}

esp_err_t bhi260ap_get_status(bool *ready, uint32_t *steps)
{
    if (ready) {
        *ready = s_initialized;
    }
    if (steps) {
        *steps = s_initialized ? (s_step_base + (s_step_count - s_step_at_base)) : 0;
    }
    return ESP_OK;
}

esp_err_t bhi260ap_get_accel(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x_mg) {
        *x_mg = s_accel.x;
    }
    if (y_mg) {
        *y_mg = s_accel.y;
    }
    if (z_mg) {
        *z_mg = s_accel.z;
    }
    return ESP_OK;
}

esp_err_t bhi260ap_get_gyro(int16_t *x_dps, int16_t *y_dps, int16_t *z_dps)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x_dps) {
        *x_dps = s_gyro.x;
    }
    if (y_dps) {
        *y_dps = s_gyro.y;
    }
    if (z_dps) {
        *z_dps = s_gyro.z;
    }
    return ESP_OK;
}

/* Orientation (heading/pitch/roll) computed from the accelerometer.
 *
 * The BHI260AP is a 6-DoF IMU (accel+gyro); the ORI/RV fusion sensors need an
 * external magnetometer to produce heading (absent on this board), so heading
 * is reported as 0. Pitch/roll come from the gravity vector and are reliable:
 *   pitch = atan2(-ax, sqrt(ay^2 + az^2))
 *   roll  = atan2(ay, az)
 * Accel is in mg (1 mg resolution); the ratios cancel the scale. */
esp_err_t bhi260ap_get_orientation(int16_t *heading, int16_t *pitch, int16_t *roll)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (heading) {
        *heading = 0;   /* no magnetometer on this board */
    }
    if (pitch || roll) {
        float ax = (float)s_accel.x;
        float ay = (float)s_accel.y;
        float az = (float)s_accel.z;
        if (pitch) {
            *pitch = (int16_t)(atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI);
        }
        if (roll) {
            *roll = (int16_t)(atan2f(ay, az) * 180.0f / M_PI);
        }
    }
    return ESP_OK;
}

esp_err_t bhi260ap_get_rotation(int16_t *x, int16_t *y, int16_t *z, int16_t *w,
                                uint16_t *accuracy)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x) {
        *x = s_rv.x;
    }
    if (y) {
        *y = s_rv.y;
    }
    if (z) {
        *z = s_rv.z;
    }
    if (w) {
        *w = s_rv.w;
    }
    if (accuracy) {
        *accuracy = s_rv.accuracy;
    }
    return ESP_OK;
}

esp_err_t bhi260ap_get_activity(uint8_t *activity)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    *activity = (uint8_t)s_activity;
    return ESP_OK;
}

esp_err_t bhi260ap_consume_gestures(bool *wrist_tilt, bool *wake_gesture, bool *glance,
                                    bool *pickup, bool *tilt)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (wrist_tilt) {
        *wrist_tilt = s_wrist_tilt;
        s_wrist_tilt = false;
    }
    if (wake_gesture) {
        *wake_gesture = s_wake_gesture;
        s_wake_gesture = false;
    }
    if (glance) {
        *glance = s_glance_gesture;
        s_glance_gesture = false;
    }
    if (pickup) {
        *pickup = s_pickup_gesture;
        s_pickup_gesture = false;
    }
    if (tilt) {
        *tilt = s_tilt_detector;
        s_tilt_detector = false;
    }
    return ESP_OK;
}
