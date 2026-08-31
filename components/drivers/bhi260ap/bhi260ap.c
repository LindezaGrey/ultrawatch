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
#include "bhy2_hif.h"
#include "bhy2_parse.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "i2c_bus.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "bhi260ap";

#define BHI260_I2C_TIMEOUT_MS   1000
#define BHY2_RD_WR_LEN          256
#define BHY2_FIFO_BUFFER_SIZE   512

#define FIRMWARE_PATH           "/assets/bhi260/BHI260AP.fw"

/* Meta-event payload histogram (debug): categorize what the chip keeps
 * reporting into the WU FIFO so spurious wake interrupts can be traced. */
#define META_HIST_MAX 16
static uint8_t s_meta_hist_keys[META_HIST_MAX];
static uint8_t s_meta_hist_counts[META_HIST_MAX];
static uint8_t s_meta_hist_len;
static volatile bool s_meta_hist_enabled;
static volatile uint32_t s_meta_print_first;   /* debug: print first N metas */
static volatile uint32_t s_meta_print_count;

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
/* Event-driven per-activity duration tally (today), credited in parse_activity()
 * as each transition is processed - see that function. Guarded by s_step_mux
 * (already exists for the day-scoped step counters; reused rather than adding
 * a second small mutex for the same "day-scoped BHI260AP counters" purpose). */
static uint32_t s_activity_ms[BHI260AP_ACTIVITY_COUNT];
static int64_t s_activity_since_us;
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
#define STEP_NVS_DAY    "day_start"
#define STEP_NVS_DAYLST "day_last"   /* lifetime at last daily sample */
#define STEP_NVS_DAYTOT "day_total"  /* accumulated daily steps */
#define STEP_NVS_DAYID  "day_id"     /* yyyymmdd the daily total applies to */
#define STEP_FOLD_STEP  1000  /* fold chip steps into the base every ~1000 */

/* Daily step counter (steps since midnight). Tracks the *increment* of the
 * monotonic lifetime total, so it is immune to on-chip counter resets and
 * reboots: the last-sampled lifetime, today's total and the day-id are all
 * persisted. A new calendar day (different day-id) resets the total to 0. */
static uint32_t s_daily_total;
static uint32_t s_daily_last_lifetime;
static uint32_t s_daily_id;

static uint32_t s_step_at_base;
static SemaphoreHandle_t s_step_mux;   /* guards step fold/total arithmetic */

/* Guards every call into the Bosch bhy2/bhy2_hif API (s_bhy2's HIF sequence
 * counters and internal read/parse buffers are not reentrant). Without this,
 * bhi260_task's own polling loop, power_mgmt.c's suspend/resume calls (a
 * different task, invoked from the sleep/wake path), and debug_bhi.c's
 * console commands (a third task) could all call into s_bhy2 concurrently -
 * a real hazard, not just a theoretical one, since bhi260ap_ap_suspend()
 * flips s_ap_suspended only partway through its own body, leaving a window
 * where bhi260_task's poll and a concurrent suspend can race on the same I2C
 * handshake. Recursive: bhi260ap_ap_suspend() calls bhi260ap_drain_wakeup_fifo(),
 * which also takes this same lock. */
static SemaphoreHandle_t s_bhy2_mux;

static void step_base_load(void)
{
    nvs_handle_t h;
    uint32_t v = 0;
    if (nvs_open(STEP_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        s_step_base = 0;
        s_daily_total = 0;
        s_daily_last_lifetime = 0;
        s_daily_id = 0;
        return;
    }
    if (nvs_get_u32(h, STEP_NVS_KEY, &v) != ESP_OK) {
        v = 0;
    }
    s_step_base = v;
    /* Load persisted daily-step state. First use: seed the baseline to the
     * current lifetime so the counter starts at 0 and counts forward. */
    if (nvs_get_u32(h, STEP_NVS_DAYID, &s_daily_id) != ESP_OK) {
        s_daily_id = 0;
    }
    if (nvs_get_u32(h, STEP_NVS_DAYTOT, &s_daily_total) != ESP_OK) {
        s_daily_total = 0;
    }
    if (nvs_get_u32(h, STEP_NVS_DAYLST, &s_daily_last_lifetime) != ESP_OK) {
        s_daily_last_lifetime = s_step_base;
    }
    nvs_close(h);
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

/* Fold the steps accumulated since s_step_at_base into the persisted base.
 * Called from the polling task (parse_step_counter) and from AP-suspend
 * (LVGL task context), so it is guarded by a mutex. */
static void step_fold(void)
{
    if (xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    uint32_t delta = s_step_count - s_step_at_base;
    if (delta == 0) {
        xSemaphoreGive(s_step_mux);
        return;
    }
    s_step_base += delta;
    s_step_at_base = s_step_count;
    xSemaphoreGive(s_step_mux);
    step_base_save();
}

/* ---- I2C glue for the Bosch API ---- */

static int8_t bhi260ap_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t ret = i2c_bus_read(dev, reg_addr, reg_data, length, BHI260_I2C_TIMEOUT_MS);
    return (ret == ESP_OK) ? BHY2_OK : BHY2_E_IO;
}

static int8_t bhi260ap_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t ret = i2c_bus_write(dev, reg_addr, reg_data, length, BHI260_I2C_TIMEOUT_MS);
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
    /* Decode the idle WAKE-UP-FIFO meta stream (debug): print only WU metas
     * (sid 248) while the metahist window is active. */
    if (s_meta_hist_enabled && callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU
            && s_meta_print_first) {
        s_meta_print_first--;
        printf("wumeta[%lu] sid=%u dat=", (unsigned long)s_meta_print_count++, callback_info->sensor_id);
        for (uint32_t i = 0; i < callback_info->data_size && i < 8; i++) {
            printf("%02x ", callback_info->data_ptr[i]);
        }
        printf("\n");
    }
    /* Histogram the meta payload to identify what the chip keeps reporting. */
    if (s_meta_hist_enabled && callback_info->data_size >= 1) {
        uint8_t key = callback_info->data_ptr[0];
        if (s_meta_hist_len == 0 || s_meta_hist_keys[s_meta_hist_len - 1] != key) {
            if (s_meta_hist_len < META_HIST_MAX) {
                s_meta_hist_keys[s_meta_hist_len] = key;
                s_meta_hist_counts[s_meta_hist_len] = 1;
                s_meta_hist_len++;
            }
        } else {
            s_meta_hist_counts[s_meta_hist_len - 1]++;
        }
    }
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
    if (xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    if (steps < s_step_count) {
        uint32_t delta = s_step_count - s_step_at_base;
        s_step_base += delta;
        s_step_at_base = steps;
        s_step_count = steps;
    } else {
        s_step_count = steps;
    }
    uint32_t total = s_step_base + (s_step_count - s_step_at_base);
    bool need_fold = (s_step_count - s_step_at_base) >= STEP_FOLD_STEP;
    xSemaphoreGive(s_step_mux);
    if (need_fold) {
        step_fold();   /* re-takes the mutex + NVS save */
    }
    ESP_LOGD(TAG, "step counter: %lu (total %lu)",
             (unsigned long)steps, (unsigned long)total);
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

/* Fires once per activity-change event found in the FIFO, in the order the
 * chip recorded them - bhy2_get_and_process_fifo() can deliver several in one
 * drain (e.g. Still->Walking->Still all between two polls of bhi260_task).
 * Crediting duration here, per event, means none of those transitions are
 * lost the way they would be if something instead just read "the current
 * activity" once after the whole drain finished. */
static void parse_activity(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    if (callback_info->data_size < 2) {
        return;
    }
    uint16_t activity = BHY2_LE2U16(callback_info->data_ptr);
    bhi260ap_activity_t new_activity = s_activity;
    if (activity & BHY2_STILL_ACTIVITY_STARTED) {
        new_activity = BHI260AP_ACTIVITY_STILL;
    } else if (activity & BHY2_WALKING_ACTIVITY_STARTED) {
        new_activity = BHI260AP_ACTIVITY_WALKING;
    } else if (activity & BHY2_RUNNING_ACTIVITY_STARTED) {
        new_activity = BHI260AP_ACTIVITY_RUNNING;
    } else if (activity & BHY2_ON_BICYCLE_ACTIVITY_STARTED) {
        new_activity = BHI260AP_ACTIVITY_ON_BICYCLE;
    } else if (activity & BHY2_IN_VEHICLE_ACTIVITY_STARTED) {
        new_activity = BHI260AP_ACTIVITY_IN_VEHICLE;
    } else if (activity & BHY2_TILTING_ACTIVITY_STARTED) {
        new_activity = BHI260AP_ACTIVITY_TILTING;
    } else {
        /* No STARTED bit in this event - per the datasheet, the chip can
         * also report an ENDED-only event (low-confidence transitional
         * motion between classes, nothing new confidently recognized yet).
         * Without this check the code below would keep crediting duration
         * to s_activity's class forever, even though it has actually
         * ended - fall back to UNKNOWN instead, but only if the ENDED bit
         * actually names the class that's currently active (an ENDED bit
         * for some other, already-inactive class is not a real transition
         * and should be ignored, same as an event with no relevant bits
         * at all - new_activity stays s_activity, a no-op). */
        static const struct { bhi260ap_activity_t cls; uint16_t ended_bit; } ended_map[] = {
            { BHI260AP_ACTIVITY_STILL,      BHY2_STILL_ACTIVITY_ENDED },
            { BHI260AP_ACTIVITY_WALKING,    BHY2_WALKING_ACTIVITY_ENDED },
            { BHI260AP_ACTIVITY_RUNNING,    BHY2_RUNNING_ACTIVITY_ENDED },
            { BHI260AP_ACTIVITY_ON_BICYCLE, BHY2_ON_BICYCLE_ACTIVITY_ENDED },
            { BHI260AP_ACTIVITY_IN_VEHICLE, BHY2_IN_VEHICLE_ACTIVITY_ENDED },
            { BHI260AP_ACTIVITY_TILTING,    BHY2_TILTING_ACTIVITY_ENDED },
        };
        for (size_t i = 0; i < sizeof(ended_map) / sizeof(ended_map[0]); i++) {
            if (ended_map[i].cls == s_activity && (activity & ended_map[i].ended_bit)) {
                new_activity = BHI260AP_ACTIVITY_UNKNOWN;
                break;
            }
        }
    }

    /* Blocking take (not a 0-timeout try): if this silently failed to take
     * the mutex, s_activity would never advance to new_activity, corrupting
     * every later comparison, not just losing one sample. Contention here is
     * brief arithmetic in bhi260ap_get_activity_ms()/bhi260ap_daily_sample(),
     * so this essentially always succeeds immediately. */
    int64_t now = esp_timer_get_time();
    if (s_step_mux != NULL && xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        int64_t elapsed_us = now - s_activity_since_us;
        if (elapsed_us > 0) {
            uint32_t elapsed_ms = (uint32_t)(elapsed_us / 1000);
            if (s_activity_ms[s_activity] <= UINT32_MAX - elapsed_ms) {
                s_activity_ms[s_activity] += elapsed_ms;
            }
        }
        s_activity = new_activity;
        s_activity_since_us = now;
        xSemaphoreGive(s_step_mux);
    } else {
        /* Mutex unavailable (or not yet created): still advance activity
         * state so recognition doesn't silently stick to a stale class, just
         * without a duration credit for this segment. */
        s_activity = new_activity;
        s_activity_since_us = now;
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

/* Virtual sensors enabled at init, and re-applied verbatim by
 * bhi260ap_reenable_sensors() (soft recovery - see that function). */
static const struct {
    uint8_t id;
    bhy2_float rate;
} s_sensor_enable[] = {
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

/* Applies s_sensor_enable[] to the chip (non-fatal per-sensor: the watch
 * keeps running if one is unavailable in the loaded firmware). Call under
 * s_bhy2_mux. */
static void enable_virtual_sensors_locked(void)
{
    for (size_t i = 0; i < sizeof(s_sensor_enable) / sizeof(s_sensor_enable[0]); i++) {
        int8_t rslt = bhy2_set_virt_sensor_cfg(s_sensor_enable[i].id, s_sensor_enable[i].rate, 0, &s_bhy2);
        if (rslt != BHY2_OK) {
            ESP_LOGW(TAG, "enable sensor %u @ %.1f Hz failed: %d",
                     s_sensor_enable[i].id, s_sensor_enable[i].rate, rslt);
        }
    }
}

/* Actual init body, run under s_bhy2_mux (see bhi260ap_init() below) - not
 * called directly by anything else. */
static esp_err_t bhi260ap_init_locked(i2c_master_dev_handle_t dev)
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
     * line would flood the wake task. Status/debug FIFOs are unused.
     *
     * IMPORTANT: the default firmware image configures the INT line ACTIVE
     * HIGH (idles low, pulses high). Everything downstream (GPIO8 NEGEDGE
     * ISR, LOW_LEVEL light-sleep wake, and the "arm only if line idles high"
     * guard in pm_arm_gpio_wakeup) assumes an ACTIVE-LOW INT (idles high,
     * pulses low). Override the polarity here so the whole wake path agrees. */
    bhy2_set_host_interrupt_ctrl(BHY2_ICTL_ACTIVE_LOW |
                                 BHY2_ICTL_DISABLE_FIFO_NW |
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

    /* UNVERIFIED ON HARDWARE (2026-08-27): candidate fix for the rotation-
     * sense mismatch previously worked around in lvgl_app.c's
     * bhi_cube_update() by conjugating the GAMERV quaternion app-side (only
     * fixed the cube widget, not gesture/activity detection, which also
     * consume the fusion output). Hypothesis: the gyroscope's angular-rate
     * sign convention is inverted relative to what the on-chip fusion
     * expects, producing a globally rotation-inverted (conjugated)
     * quaternion; negating all three gyro axes here, before fusion runs,
     * should correct GAMERV at the source. Needs physical verification
     * (rotate the watch on each axis, confirm the BHI screen's cube tracks
     * correctly; confirm wrist-tilt-wake still triggers correctly) - see
     * todo.md. If wrong, revert this and restore the app-side conjugate. */
    struct bhy2_orient_matrix gyro_orient = { .c = { -1, 0, 0, 0, -1, 0, 0, 0, -1 } };
    rslt = bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_GYROSCOPE, gyro_orient, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "set gyro orientation matrix failed: %d", rslt);
    }

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
    enable_virtual_sensors_locked();

    /* Arm the host-interrupt wake path: the chip asserts its INT line (GPIO8)
     * when the wake-up FIFO reaches this watermark, so a gesture/wake event
     * wakes the sleeping host. The default watermark is 0 (never fires).
     * One gesture = one FIFO event, so a threshold of 1 lets a single
     * wrist-tilt/wake-raise assert the line immediately; anything higher
     * would need several events before waking. */
    rslt = bhy2_set_fifo_wmark_wkup(1, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "set wake-up FIFO watermark failed: %d", rslt);
    }

    /* Start the staleness clock now, so the re-init logic doesn't see an
     * immediate "stale" before the first FIFO sample arrives. */
    bhi260ap_mark_data();
    if (!s_step_mux) {
        s_step_mux = xSemaphoreCreateMutex();
    }
    step_base_load();
    s_step_at_base = 0;
    s_step_count = 0;
    /* Start the activity-duration clock now too (deinit() also does this,
     * for re-init after a sensor power-cycle, but the very first boot never
     * calls deinit first). Without this the first parse_activity() call
     * would credit the boot-to-here gap to whatever s_activity starts as. */
    s_activity_since_us = esp_timer_get_time();
    s_initialized = true;
    ESP_LOGI(TAG, "BHI260AP ready (persisted steps %lu)",
             (unsigned long)s_step_base);
    return ESP_OK;
}

esp_err_t bhi260ap_init(i2c_master_dev_handle_t dev)
{
    if (!s_bhy2_mux) {
        s_bhy2_mux = xSemaphoreCreateRecursiveMutex();
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    esp_err_t err = bhi260ap_init_locked(dev);
    xSemaphoreGiveRecursive(s_bhy2_mux);
    return err;
}

bool bhi260ap_ping(void)
{
    if (!s_initialized) {
        return false;
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    uint8_t product_id = 0;
    int8_t rslt = bhy2_get_product_id(&product_id, &s_bhy2);
    xSemaphoreGiveRecursive(s_bhy2_mux);
    return rslt == BHY2_OK && product_id == BHY2_PRODUCT_ID;
}

void bhi260ap_reenable_sensors(void)
{
    if (!s_initialized) {
        return;
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    enable_virtual_sensors_locked();
    xSemaphoreGiveRecursive(s_bhy2_mux);
    /* Reset the staleness clock so this soft recovery doesn't immediately
     * re-trigger before the first post-recovery sample arrives. */
    bhi260ap_mark_data();
    ESP_LOGI(TAG, "sensors re-enabled (soft recovery)");
}

/* Poll the FIFO once; returns ESP_OK on success. */
esp_err_t bhi260ap_process_fifo(void)
{
    if (!s_initialized || s_ap_suspended) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    uint8_t work[BHY2_FIFO_BUFFER_SIZE];
    int8_t rslt = bhy2_get_and_process_fifo(work, sizeof(work), &s_bhy2);
    xSemaphoreGiveRecursive(s_bhy2_mux);
    return (rslt == BHY2_OK) ? ESP_OK : ESP_FAIL;
}

/* Consume every pending wake-up FIFO event until the INT line (GPIO8)
 * de-asserts back to its idle-high state. Used after AP-suspend and again at
 * wake-arm time: a leftover WU event (a real gesture landing during the
 * power-down window, glance/pickup disabling, or the FIFO flush) holds the
 * ACTIVE-LOW INT low, which would trip a LOW_LEVEL wake the instant the ESP32
 * goes to sleep. Returns ESP_OK once the line is high. */
esp_err_t bhi260ap_drain_wakeup_fifo(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    for (int attempt = 0; attempt < 5 && gpio_get_level(GPIO_NUM_8) == 0; attempt++) {
        uint8_t drain[BHY2_FIFO_BUFFER_SIZE];
        bhy2_get_and_process_fifo(drain, sizeof(drain), &s_bhy2);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    esp_err_t ret = (gpio_get_level(GPIO_NUM_8) == 1) ? ESP_OK : ESP_ERR_TIMEOUT;
    xSemaphoreGiveRecursive(s_bhy2_mux);
    return ret;
}

/* Tell the BHI260AP the host (ESP32) is going to sleep. It then runs only the
 * wake-up sensors (*_WU variants / gesture sensors) at low power and stops the
 * high-rate non-wakeup streams. Per the datasheet, flush the FIFO first so a
 * stale event doesn't immediately re-wake the host. ALDO4 must stay powered;
 * do not power-cycle the sensor rail while suspended.
 *
 * Glance and pickup gestures are disabled while sleeping: they are very
 * sensitive and fire spuriously (watch settling, arm brush), which re-asserts
 * the INT line and wakes the host without any real gesture. Wrist-tilt and
 * wake gestures are kept. */
esp_err_t bhi260ap_ap_suspend(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    step_fold();   /* persist steps accumulated since the last fold */
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GLANCE_GESTURE, 0.0f, 0, &s_bhy2);
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_PICKUP_GESTURE, 0.0f, 0, &s_bhy2);
    s_ap_suspended = true;
    bhy2_flush_fifo(0xFF, &s_bhy2);   /* 0xFF = flush all virtual sensors */
    uint8_t intr_ctrl = 0;
    bhy2_get_host_interrupt_ctrl(&intr_ctrl, &s_bhy2);
    int8_t rslt = bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_AP_SUSPENDED, &s_bhy2);
    if (rslt != BHY2_OK) {
        ESP_LOGW(TAG, "AP suspend failed: %d", rslt);
        xSemaphoreGiveRecursive(s_bhy2_mux);
        return ESP_FAIL;
    }

    /* Drain any event that slipped into the wake-up FIFO during the suspend
     * transition (disabling glance/pickup, the FIFO flush, or the AP-suspend
     * handshake can each leave one behind, and a real gesture can land at the
     * same time). With the WU watermark at 1 such a leftover holds the INT line
     * low while the host sleeps, and the arming gate in pm_arm_gpio_wakeup()
     * would then see GPIO8 low and disable gesture wake for the whole cycle.
     * Consume them now so the line de-asserts back to its idle-high state
     * before the ESP32 arms the level wake. */
    bhi260ap_drain_wakeup_fifo();

    ESP_LOGI(TAG, "AP suspend (host sleeping) intr_ctrl=0x%02x gpio8=%d",
             (unsigned)intr_ctrl, (int)gpio_get_level(GPIO_NUM_8));
    xSemaphoreGiveRecursive(s_bhy2_mux);
    return ESP_OK;
}

/* Host is awake again; resume the normal (non-wakeup) sensor streams.
 *
 * Called from power_mgmt.c's wake path, BEFORE the panel is woken - a
 * live incident showed the display stuck white after a GPIO wake, with the
 * rest of the system (console, scheduler) still responsive, and a full
 * panel-level recovery (power-cycle the display rail, resend wake commands)
 * unable to bring it back. Root cause wasn't conclusively pinned down (the
 * device had to be rebooted to restore it before more diagnostics could be
 * taken), but this function sitting ahead of co5300_wake() in the wake
 * sequence means anything that makes it block would delay the panel wake
 * indefinitely - so unlike every other bhy2 entry point here, this one
 * takes the lock with a bounded wait rather than portMAX_DELAY: if it can't
 * get in quickly (bhi260_task mid-poll, or worse, wedged), skip resuming
 * the IMU cleanly rather than risk blocking the caller (and therefore the
 * panel wake) forever. A skipped resume just means gesture sensitivity
 * stays at its suspended settings for one cycle - not user-visible the way
 * a stuck white screen is. See power_mgmt.c's wake function, which also
 * now wakes the panel first regardless. */
esp_err_t bhi260ap_ap_resume(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTakeRecursive(s_bhy2_mux, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "AP resume: bhy2 busy, skipping this cycle");
        return ESP_ERR_TIMEOUT;
    }
    /* No data flowed while the chip was in AP-suspend (by design), so the
     * staleness clock reads old. Reset it so the FIFO re-init logic doesn't
     * declare the resumed stream dead before its first sample arrives. */
    bhi260ap_mark_data();
    /* Re-enable the gestures that were paused during sleep. */
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GLANCE_GESTURE, 1.0f, 0, &s_bhy2);
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_PICKUP_GESTURE, 1.0f, 0, &s_bhy2);
    int8_t rslt = bhy2_set_host_intf_ctrl(0, &s_bhy2);
    s_ap_suspended = false;
    xSemaphoreGiveRecursive(s_bhy2_mux);
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
    /* Take the same lock every other entry point here takes. s_initialized is
     * checked *before* the lock by the public API, so a suspend/drain call
     * from power_mgmt.c can already be past its own check and mid-I2C on
     * s_bhy2 when this runs; clearing state underneath it would corrupt the
     * transfer in progress. Unbounded wait is fine (unlike
     * bhi260ap_ap_resume()'s bounded one): this runs on bhi260_task, which
     * nothing user-visible is waiting on. */
    bool bhy2_locked = (s_bhy2_mux != NULL) &&
                       (xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY) == pdTRUE);
    /* The step/activity counters below are the ones s_step_mux guards; take it
     * too so a concurrent parse_activity()/get_activity_ms() can't observe
     * s_activity and s_activity_since_us half-reset. Proceed even if the take
     * times out - leaving the driver marked initialized after a rail
     * power-cycle is the worse outcome. */
    bool step_locked = (s_step_mux != NULL) &&
                       (xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) == pdTRUE);

    s_initialized = false;
    s_ap_suspended = false;
    s_step_count = 0;
    s_step_at_base = 0;
    s_last_data_tick = 0;
    memset(&s_accel, 0, sizeof(s_accel));
    memset(&s_gyro, 0, sizeof(s_gyro));
    memset(&s_rv, 0, sizeof(s_rv));
    s_activity = BHI260AP_ACTIVITY_UNKNOWN;
    /* Restart the "since" clock cleanly so the next parse_activity() doesn't
     * credit whatever class is active with the entire power-cycle gap as
     * elapsed time. s_activity_ms (today's totals) is NOT reset here - it's
     * day-scoped, reset only on an actual calendar-day rollover in
     * bhi260ap_daily_sample(), and should survive a mid-day sensor
     * power-cycle. */
    s_activity_since_us = esp_timer_get_time();
    s_wrist_tilt = false;
    s_wake_gesture = false;
    s_glance_gesture = false;
    s_pickup_gesture = false;
    s_tilt_detector = false;

    if (step_locked) {
        xSemaphoreGive(s_step_mux);
    }
    if (bhy2_locked) {
        xSemaphoreGiveRecursive(s_bhy2_mux);
    }
}

esp_err_t bhi260ap_get_step_count(uint32_t *steps)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t total = 0;
    if (xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        total = s_step_base + (s_step_count - s_step_at_base);
        xSemaphoreGive(s_step_mux);
    }
    *steps = total;
    return ESP_OK;
}

esp_err_t bhi260ap_get_daily_steps(uint32_t *steps)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Seed the default before the take, not inside it: a timed-out take used
     * to leave *steps untouched while still returning ESP_OK, handing the
     * caller back whatever was in its own variable. Same shape as
     * bhi260ap_get_status(), which reports a known value either way. */
    *steps = 0;
    if (xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
        *steps = s_daily_total;
        xSemaphoreGive(s_step_mux);
    }
    return ESP_OK;
}

/* Feed the current lifetime step total into the daily counter. Called once per
 * minute (daily_log). `ymd` is the current calendar day (year*10000+month*100+day):
 * a change of day resets the daily total. Persists the running total + the
 * last-sampled lifetime so the count survives reboots without drift. */
void bhi260ap_daily_sample(uint32_t lifetime, uint32_t ymd)
{
    /* The mutex is created during BHI init; sample-from-boot (daily_log task)
     * can race ahead of that, so guard against a NULL handle (a take on NULL
     * asserts and reboots the watch). */
    if (s_step_mux == NULL ||
        xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    if (s_daily_id != ymd) {
        s_daily_id = ymd;
        s_daily_total = 0;
        s_daily_last_lifetime = lifetime;
        /* Same day-rollover trigger resets the activity-duration tally -
         * one place tracking "is it a new day", shared by both day-scoped
         * counters this driver owns. */
        memset(s_activity_ms, 0, sizeof(s_activity_ms));
        ESP_LOGI(TAG, "daily reset, day=%lu", (unsigned long)ymd);
    } else if (lifetime > s_daily_last_lifetime) {
        s_daily_total += (lifetime - s_daily_last_lifetime);
        s_daily_last_lifetime = lifetime;
    }
    uint32_t total = s_daily_total;
    uint32_t last = s_daily_last_lifetime;
    uint32_t id = s_daily_id;
    xSemaphoreGive(s_step_mux);

    nvs_handle_t h;
    if (nvs_open(STEP_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, STEP_NVS_DAYID, id);
        nvs_set_u32(h, STEP_NVS_DAYTOT, total);
        nvs_set_u32(h, STEP_NVS_DAYLST, last);
        nvs_commit(h);
        nvs_close(h);
    }
}

esp_err_t bhi260ap_get_status(bool *ready, uint32_t *steps)
{
    if (ready) {
        *ready = s_initialized;
    }
    if (steps) {
        uint32_t total = 0;
        if (s_initialized && xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
            total = s_step_base + (s_step_count - s_step_at_base);
            xSemaphoreGive(s_step_mux);
        }
        *steps = total;
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

esp_err_t bhi260ap_get_activity_ms(uint32_t out_ms[BHI260AP_ACTIVITY_COUNT])
{
    if (!out_ms) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        memset(out_ms, 0, BHI260AP_ACTIVITY_COUNT * sizeof(out_ms[0]));
        return ESP_ERR_INVALID_STATE;
    }
    if (s_step_mux == NULL || xSemaphoreTake(s_step_mux, pdMS_TO_TICKS(100)) != pdTRUE) {
        memset(out_ms, 0, BHI260AP_ACTIVITY_COUNT * sizeof(out_ms[0]));
        return ESP_FAIL;
    }
    for (int i = 0; i < BHI260AP_ACTIVITY_COUNT; i++) {
        out_ms[i] = s_activity_ms[i];
    }
    bhi260ap_activity_t cur = s_activity;
    int64_t since = s_activity_since_us;
    xSemaphoreGive(s_step_mux);

    /* Add the live elapsed time for the currently-active class: it hasn't
     * been credited to s_activity_ms yet, that only happens when it ends
     * (the next parse_activity() transition). Same pattern as
     * bhi260ap_get_step_count()'s base+live-delta combination. */
    int64_t elapsed_us = esp_timer_get_time() - since;
    if (elapsed_us > 0) {
        uint32_t elapsed_ms = (uint32_t)(elapsed_us / 1000);
        if (out_ms[cur] <= UINT32_MAX - elapsed_ms) {
            out_ms[cur] += elapsed_ms;
        }
    }
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

/* ---- Wake-up FIFO sample tracer (debug) ----
 * Counts every sensor/system id seen in the WU FIFO while enabled, so the
 * source of spurious wake events can be identified. Non-reentrant: call from
 * one task. */

void bhi260ap_meta_hist_start(void){
    s_meta_hist_len = 0;
    s_meta_hist_enabled = true;
}

void bhi260ap_meta_hist_stop(void)
{
    s_meta_hist_enabled = false;
}

uint8_t bhi260ap_meta_hist_len_get(void) { return s_meta_hist_len; }

void bhi260ap_meta_print(uint32_t n)
{
    s_meta_print_first = n;
}

void bhi260ap_meta_hist_get(uint8_t idx, uint8_t *key, uint8_t *count)
{
    /* idx comes from a console command, so it is not necessarily bounded by
     * the len this module last reported. Read out of range would walk past
     * the arrays into whatever follows them. */
    if (idx >= s_meta_hist_len || idx >= META_HIST_MAX) {
        if (key) {
            *key = 0;
        }
        if (count) {
            *count = 0;
        }
        return;
    }
    *key = s_meta_hist_keys[idx];
    *count = s_meta_hist_counts[idx];
}

extern volatile bool s_bhy2_wu_trace_enabled;
extern uint32_t s_bhy2_wu_trace_count[256];

void bhi260ap_wu_trace_start(void)
{
    memset((void *)s_bhy2_wu_trace_count, 0, sizeof(s_bhy2_wu_trace_count));
    s_bhy2_wu_trace_enabled = true;
}

void bhi260ap_wu_trace_stop(void)
{
    s_bhy2_wu_trace_enabled = false;
}

uint32_t bhi260ap_wu_trace_get_count(uint8_t id)
{
    return s_bhy2_wu_trace_count[id];
}

/* Debug: change one virtual sensor's sample rate (0 = disable). */
esp_err_t bhi260ap_set_sensor_rate(uint8_t id, float rate)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    int8_t rslt = bhy2_set_virt_sensor_cfg(id, rate, 0, &s_bhy2);
    xSemaphoreGiveRecursive(s_bhy2_mux);
    return (rslt == BHY2_OK) ? ESP_OK : ESP_FAIL;
}

/* GNSS data-injection readiness probe. Reports whether the firmware exposes
 * the GPS virtual sensor, accepts real-time sensor-data injection, and -- after
 * enabling injection -- whether BSX issues an Injected Sensor Configuration
 * Request naming the GPS physical sensor (which proves a live GPS injection
 * driver) and at what sample rate it wants GPS data. Returns the bhy2 code
 * from the injection-mode switch (BHY2_OK=0 on success). */
int8_t bhi260ap_gnss_inject_probe(void)
{
    if (!s_initialized) {
        return BHY2_E_NULL_PTR;
    }
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    uint8_t v_gps = bhy2_is_sensor_available(BHY2_SENSOR_ID_GPS, &s_bhy2);
    uint8_t p_gps = bhy2_is_sensor_available(BHY2_PHYS_SENSOR_ID_GPS, &s_bhy2);
    ESP_LOGI(TAG, "gnss probe: virtual_gps=%u physical_gps=%u",
             (unsigned)v_gps, (unsigned)p_gps);

    int8_t rslt = bhy2_set_data_injection_mode(BHY2_REAL_TIME_INJECTION, &s_bhy2);
    ESP_LOGI(TAG, "gnss probe: set inject mode rslt=%d", (int)rslt);
    if (rslt != BHY2_OK) {
        xSemaphoreGiveRecursive(s_bhy2_mux);
        return rslt;
    }

    /* Poll the status FIFO briefly for an injected-sensor-config-request: if
     * the firmware has a live GPS injection driver it will ask for physical
     * sensor 48 (GPS) at a required rate. */
    int found = 0;
    uint8_t status[32];
    for (int try = 0; try < 10; try++) {
        uint16_t code = 0;
        uint32_t remain = 0;
        memset(status, 0, sizeof(status));
        int8_t r = bhy2_hif_get_status_fifo(&code, status, sizeof(status), &remain,
                                            &s_bhy2.hif);
        if (r == BHY2_OK && code == BHY2_STATUS_INJECT_SENSOR_CONF_REQ && remain >= 9) {
            float rate = 0.0f;
            memcpy(&rate, &status[1], 4);   /* sample rate, Hz (LE float) */
            uint8_t sid = status[5];         /* physical sensor id */
            ESP_LOGI(TAG, "gnss probe: BSX requests sensor %u @ %.1f Hz",
                     (unsigned)sid, (double)rate);
            found = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(TAG, "gnss probe: %s", found ? "GPS injection driver active"
                                          : "no GPS injection request seen");

    bhy2_set_data_injection_mode(BHY2_NORMAL_MODE, &s_bhy2);
    xSemaphoreGiveRecursive(s_bhy2_mux);
    return found ? BHY2_OK : BHY2_E_INVALID_PARAM;
}

/* Print the virtual-sensor ids present in the loaded firmware's ACT subset.
 * Used to factually compare firmware feature sets (standard vs KLIO/etc.). */
void bhi260ap_dump_sensor_list(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "sensor list: not initialized");
        return;
    }
    /* Algorithm ids not defined in this driver's bhy2_defs.h (from the ACT). */
    enum { SENSOR_ID_KLIO = 112, SENSOR_ID_PDR = 113, SENSOR_ID_SWIM = 114,
           SENSOR_ID_LIGHT = 146, SENSOR_ID_PROX = 147 };
    static const struct { uint8_t id; const char *name; } known[] = {
        { BHY2_SENSOR_ID_ACC, "acc" },
        { BHY2_SENSOR_ID_GYRO, "gyro" },
        { BHY2_SENSOR_ID_GAMERV, "gamerv" },
        { BHY2_SENSOR_ID_ORI, "orientation" },
        { BHY2_SENSOR_ID_GEORV, "georv" },
        { BHY2_SENSOR_ID_TILT_DETECTOR, "tilt" },
        { BHY2_SENSOR_ID_STD, "step_det" },
        { BHY2_SENSOR_ID_STC, "step_cnt" },
        { BHY2_SENSOR_ID_SIG, "sig_motion" },
        { BHY2_SENSOR_ID_WAKE_GESTURE, "wake" },
        { BHY2_SENSOR_ID_GLANCE_GESTURE, "glance" },
        { BHY2_SENSOR_ID_PICKUP_GESTURE, "pickup" },
        { BHY2_SENSOR_ID_AR, "activity" },
        { BHY2_SENSOR_ID_WRIST_TILT_GESTURE, "wrist_tilt" },
        { BHY2_SENSOR_ID_DEVICE_ORI, "device_ori" },
        { BHY2_SENSOR_ID_STATIONARY_DET, "stationary" },
        { BHY2_SENSOR_ID_MOTION_DET, "motion" },
        { BHY2_SENSOR_ID_STC_LP, "step_cnt_lp" },
        { SENSOR_ID_KLIO, "klio_ai" },
        { BHY2_SENSOR_ID_GPS, "gps" },
        { SENSOR_ID_PDR, "pdr" },
        { SENSOR_ID_SWIM, "swim" },
        { BHY2_SENSOR_ID_TEMP, "temp" },
        { BHY2_SENSOR_ID_BARO, "baro" },
    };
    ESP_LOGI(TAG, "sensor list:");
    xSemaphoreTakeRecursive(s_bhy2_mux, portMAX_DELAY);
    for (size_t i = 0; i < sizeof(known) / sizeof(known[0]); i++) {
        if (bhy2_is_sensor_available(known[i].id, &s_bhy2)) {
            ESP_LOGI(TAG, "  present: %-14s id=%u", known[i].name, (unsigned)known[i].id);
        }
    }
    xSemaphoreGiveRecursive(s_bhy2_mux);
}
