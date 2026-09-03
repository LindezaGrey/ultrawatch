/*
 * services/alarm.c - multi-alarm clock + shared ring engine (see alarm.h for the API).
 *
 * Implementation notes:
 *  - Up to ALARM_MAX_COUNT alarm entries live in RAM and are persisted as one
 *    NVS blob ("alarm"/"cfg2"). The PCF85063A has exactly one hardware alarm
 *    register, so alarm_recompute_next() always finds whichever enabled
 *    entry's next (weekday, hour, min) occurrence is soonest and arms the
 *    RTC for exactly that one moment (mask_weekday=false, a specific UTC
 *    weekday) - re-run on every list edit and after every fire.
 *  - The ring runs on a dedicated task so it never blocks LVGL or the sensor
 *    cache, shared between alarms and main/cd_timer.c's countdown timer via
 *    alarm_ring_now(). Beep (MAX98357A) enables the BLDO2 amp rail;
 *    vibration (DRV2605) enables the haptic M_EN line; both pulse on the
 *    same cadence, using whichever ring_mode was passed to alarm_ring_now()
 *    for the current ring cycle (s_ring_mode_active).
 *  - Dismissing an alarm-sourced ring clears AF + re-arms the next earliest
 *    occurrence across the whole list. Snooze (alarms only) swaps the alarm
 *    interrupt for the RTC countdown timer (600 s, wakes on TF/TIE).
 *    Dismissing a timer-sourced ring does nothing RTC-related - the
 *    countdown that triggered it lives entirely in main/cd_timer.c.
 */
#include "alarm.h"
#include "audio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "twatch_board.h"
#include "xl9555.h"
#include "pcf85063a.h"
#include "drv2605.h"
#include "haptic.h"
#include "esp_lv_adapter.h"
#include "power_mgmt.h"
#include <string.h>
#include <time.h>

static const char *TAG = "alarm";

#define ALARM_NVS_NS      "alarm"
#define ALARM_NVS_KEY     "cfg2"   /* new key: not the old single-alarm "cfg" blob (different layout, no migration - see alarm.h) */

#define ALARM_SNOOZE_MIN  10

/* If a ring is left unanswered for this long, snooze automatically and re-arm
 * the countdown timer. A missed snooze retries every 10 min for one hour;
 * the final, loudest ring then continues until dismissed. 0 disables
 * auto-snooze. Alarms only - see ring_task(). */
#define ALARM_AUTO_SNOOZE_MS  60000
#define ALARM_AUTO_SNOOZE_MAX 6

#define RING_AMP          3276    /* initial level: ~10% of 16-bit FS */
#define RING_AMP_MAX       16380  /* final retry level: ~50%, below clipping */

/* ---- state ---- */
static alarm_entry_t s_entries[ALARM_MAX_COUNT];
static int s_armed_idx = -1;              /* which entry the RTC alarm register currently targets, -1 = none */
static uint32_t s_schedule_revision;      /* increments whenever the alarm list changes */
static bool s_ringing;
static bool s_ring_queued;
static bool s_snoozing;                   /* 10 min snooze timer armed */
static alarm_ring_source_t s_ring_source; /* which kind of ring is/was in progress */
static uint8_t s_ring_mode_active;        /* ALARM_RING_* used for the current/last ring cycle */
static TaskHandle_t s_ring_task;
typedef enum { RING_CMD_START, RING_CMD_RESUME, RING_CMD_DISMISS, RING_CMD_SNOOZE } ring_cmd_type_t;
typedef struct { ring_cmd_type_t type; alarm_ring_source_t source; uint8_t mode; } ring_cmd_t;
static QueueHandle_t s_ring_cmd;
static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_rtc_lock;      /* serializes this service's RTC programming */
static SemaphoreHandle_t s_nvs_lock;      /* serializes snapshots written to alarm NVS */
static alarm_ring_cb_t s_ring_cb;
static bool s_sound_enabled = true;       /* global mute, see alarm_set_sound_enabled() */
static uint8_t s_auto_snooze_count;       /* completed automatic retries in this alarm cycle */
static bool s_auto_snooze_exhausted;

static void alarm_recompute_next(void);

/* ---- NVS persistence ---- */

static void cfg_save(void)
{
    alarm_entry_t entries[ALARM_MAX_COUNT];
    xSemaphoreTake(s_nvs_lock, portMAX_DELAY);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(entries, s_entries, sizeof(entries));
    xSemaphoreGive(s_lock);

    nvs_handle_t h;
    if (nvs_open(ALARM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, ALARM_NVS_KEY, entries, sizeof(entries));
        nvs_commit(h);
        nvs_close(h);
    }
    xSemaphoreGive(s_nvs_lock);
}

static void cfg_load(void)
{
    memset(s_entries, 0, sizeof(s_entries));
    nvs_handle_t h;
    if (nvs_open(ALARM_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(s_entries);
        if (nvs_get_blob(h, ALARM_NVS_KEY, s_entries, &len) != ESP_OK || len != sizeof(s_entries)) {
            /* First boot (or old single-alarm blob under a different key/size
             * that we deliberately don't try to migrate): all slots empty. */
            memset(s_entries, 0, sizeof(s_entries));
        }
        uint8_t v = 1;
        if (nvs_get_u8(h, "sound_en", &v) == ESP_OK) {
            s_sound_enabled = (v != 0);
        }
        nvs_close(h);
    }
}

/* ---- RTC alarm scheduler ---- */

static void rtc_disarm_alarm(void)
{
    pcf85063a_clear_alarm(twatch_rtc_dev);
}

static void rtc_start_snooze(void)
{
    xSemaphoreTake(s_rtc_lock, portMAX_DELAY);
    rtc_disarm_alarm();
    pcf85063a_set_timer_minutes(twatch_rtc_dev, ALARM_SNOOZE_MIN, true);
    xSemaphoreGive(s_rtc_lock);
}

static void rtc_stop_snooze(void)
{
    xSemaphoreTake(s_rtc_lock, portMAX_DELAY);
    pcf85063a_timer_stop(twatch_rtc_dev);
    xSemaphoreGive(s_rtc_lock);
}

/* Scans all enabled entries, finds whichever (weekday, hour, min) occurrence
 * is soonest from now, and arms the RTC's one hardware alarm register for
 * exactly that moment (a specific UTC weekday, not "every day"). Re-run
 * after every list edit and after every alarm-sourced ring finishes. */
static void alarm_recompute_next(void)
{
    alarm_entry_t entries[ALARM_MAX_COUNT];
    xSemaphoreTake(s_rtc_lock, portMAX_DELAY);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(entries, s_entries, sizeof(entries));
    uint32_t revision = s_schedule_revision;
    xSemaphoreGive(s_lock);

    time_t now_epoch;
    pcf85063a_time_t rtc_now;
    if (pcf85063a_get_time(twatch_rtc_dev, &rtc_now) == ESP_OK) {
        now_epoch = pcf85063a_time_to_epoch(&rtc_now);
    } else {
        now_epoch = time(NULL);
    }
    struct tm today_lt;
    localtime_r(&now_epoch, &today_lt);

    int best_idx = -1;
    time_t best_epoch = 0;

    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        if (!entries[i].in_use || !entries[i].enabled) {
            continue;
        }
        uint8_t mask = entries[i].weekday_mask ? entries[i].weekday_mask : ALARM_WEEKDAY_ALL;
        for (int wday = 0; wday < 7; wday++) {
            if (!(mask & (1u << wday))) {
                continue;
            }
            int delta = (wday - today_lt.tm_wday + 7) % 7;
            struct tm cand = today_lt;
            cand.tm_mday += delta;
            cand.tm_hour = entries[i].hour;
            cand.tm_min = entries[i].min;
            cand.tm_sec = 0;
            cand.tm_isdst = -1;
            time_t cand_epoch = mktime(&cand);
            if (cand_epoch <= now_epoch) {
                /* Today's slot (delta==0) already passed, or is "now" -
                 * push a week out rather than firing immediately again. */
                cand.tm_mday += 7;
                cand.tm_isdst = -1;
                cand_epoch = mktime(&cand);
            }
            if (best_idx < 0 || cand_epoch < best_epoch) {
                best_idx = i;
                best_epoch = cand_epoch;
            }
        }
    }

    if (best_idx < 0) {
        rtc_disarm_alarm();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_schedule_revision == revision) s_armed_idx = -1;
        xSemaphoreGive(s_lock);
        xSemaphoreGive(s_rtc_lock);
        ESP_LOGI(TAG, "recompute: nothing enabled, disarmed");
        return;
    }

    struct tm utc;
    gmtime_r(&best_epoch, &utc);

    pcf85063a_alarm_t a;
    memset(&a, 0, sizeof(a));
    a.enabled = true;
    /* Match sec=0/min/hour exactly, on one specific UTC weekday; day-of-month
     * is masked (irrelevant once weekday pins the day). */
    a.mask_sec = false;
    a.time.sec = 0;
    a.mask_min = false;
    a.time.min = (uint8_t)utc.tm_min;
    a.mask_hour = false;
    a.time.hour = (uint8_t)utc.tm_hour;
    a.mask_day = true;
    a.mask_weekday = false;
    a.time.weekday = (uint8_t)utc.tm_wday;
    pcf85063a_set_alarm(twatch_rtc_dev, &a);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_schedule_revision == revision) s_armed_idx = best_idx;
    xSemaphoreGive(s_lock);
    xSemaphoreGive(s_rtc_lock);
    ESP_LOGI(TAG, "armed entry %d: local %02u:%02u -> RTC UTC wday=%u %02u:%02u",
             best_idx, (unsigned)entries[best_idx].hour, (unsigned)entries[best_idx].min,
             (unsigned)a.time.weekday, (unsigned)a.time.hour, (unsigned)a.time.min);
}

/* ---- ring task ---- */

static int16_t ring_amplitude(void)
{
    if (s_ring_source != ALARM_RING_SOURCE_ALARM || s_auto_snooze_count == 0) {
        return RING_AMP;
    }
    uint32_t steps = s_auto_snooze_count;
    if (steps > ALARM_AUTO_SNOOZE_MAX) {
        steps = ALARM_AUTO_SNOOZE_MAX;
    }
    return (int16_t)(RING_AMP +
                     steps * (RING_AMP_MAX - RING_AMP) / ALARM_AUTO_SNOOZE_MAX);
}

static void ring_set_outputs(bool on)
{
    if (s_ring_mode_active == ALARM_RING_VIB || s_ring_mode_active == ALARM_RING_BOTH) {
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, on);  /* M_EN */
    }
}

static void ring_vibrate(void)
{
    drv2605_play(twatch_haptic_dev, haptic_get_wave_id());
}

static esp_err_t ring_send(ring_cmd_t cmd)
{
    return xQueueSend(s_ring_cmd, &cmd, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void ring_task(void *arg)
{
    (void)arg;

    for (;;) {
        ring_cmd_t cmd;
        if (xQueueReceive(s_ring_cmd, &cmd, portMAX_DELAY) != pdTRUE ||
            (cmd.type != RING_CMD_START && cmd.type != RING_CMD_RESUME)) {
            continue;
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (cmd.type == RING_CMD_START) {
            s_ring_source = cmd.source;
            s_ring_mode_active = cmd.mode;
            s_auto_snooze_count = 0;
            s_auto_snooze_exhausted = false;
        }
        s_ringing = true;
        s_ring_queued = false;
        s_snoozing = false;
        bool sound_enabled = s_sound_enabled;
        int armed_idx = s_armed_idx;
        xSemaphoreGive(s_lock);

        if (sound_enabled &&
            (s_ring_mode_active == ALARM_RING_BEEP || s_ring_mode_active == ALARM_RING_BOTH)) {
            audio_alarm_start(ring_amplitude());
        }
        if (s_ring_cb) {
            s_ring_cb(true, s_ring_source);
        }
        ESP_LOGI(TAG, "ringing source=%d mode=%u armed_idx=%d sound=%d",
                 (int)s_ring_source, (unsigned)s_ring_mode_active, armed_idx,
                 (int)sound_enabled);
        /* Global mute: screen/dismiss/snooze/auto-snooze timing all still
         * work normally, just silently - a master off-switch, not a mode
         * change to the ringing entry's own Beep/Vib/Both. */
        if (sound_enabled) {
            ring_set_outputs(true);
        }

        bool vib  = sound_enabled &&
                    (s_ring_mode_active == ALARM_RING_VIB || s_ring_mode_active == ALARM_RING_BOTH);

        /* The audio task owns I2S TX and the speaker rail. This task only
         * coordinates alarm timing, haptics, and the UI lifecycle. */
        TickType_t ring_started = xTaskGetTickCount();
        int finished = -1;
        while (finished < 0) {
            if (vib) ring_vibrate();
            TickType_t wait = pdMS_TO_TICKS(vib ? 300 : 40);
            if (xQueueReceive(s_ring_cmd, &cmd, wait) == pdTRUE) {
                if (cmd.type == RING_CMD_DISMISS) finished = 0;
                else if (cmd.type == RING_CMD_SNOOZE)
                    finished = s_ring_source == ALARM_RING_SOURCE_TIMER ? 0 : 1;
            }
            /* Keep LVGL activity timestamp fresh so auto-light-sleep doesn't
             * cut the ring's rails under us. */
            esp_lv_adapter_report_activity();

            /* Unanswered for too long -> snooze automatically. Alarms only
             * (timers have no snooze concept - they just keep ringing until
             * the physical button stops them), and only when a real entry is
             * actually armed (a bare alarmring test with nothing configured
             * must not spin its own 10-min snooze cycle). */
            if (ALARM_AUTO_SNOOZE_MS > 0 && s_ring_source == ALARM_RING_SOURCE_ALARM &&
                armed_idx >= 0 &&
                pdTICKS_TO_MS(xTaskGetTickCount() - ring_started) >= ALARM_AUTO_SNOOZE_MS) {
                if (s_auto_snooze_count < ALARM_AUTO_SNOOZE_MAX) {
                    s_auto_snooze_count++;
                    ESP_LOGI(TAG, "auto-snoozed retry %u/%u (%lu ms unanswered)",
                             (unsigned)s_auto_snooze_count,
                             (unsigned)ALARM_AUTO_SNOOZE_MAX,
                             (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - ring_started));
                    finished = 1;
                } else if (!s_auto_snooze_exhausted) {
                    s_auto_snooze_exhausted = true;
                    ESP_LOGW(TAG, "auto-snooze limit reached; keeping final alarm ringing");
                }
            }
        }

        ring_set_outputs(false);
        audio_alarm_stop();

        /* Update snooze/dismiss state and RTC before the UI callback, so the
         * watch face can show the Zz icon on the very first update. */
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_ringing = false;
        alarm_ring_source_t ring_source = s_ring_source;
        xSemaphoreGive(s_lock);

        if (ring_source == ALARM_RING_SOURCE_TIMER) {
            /* No RTC interaction - main/cd_timer.c already cleared its own
             * active state the moment the countdown hit zero and this ring
             * started (a timer has nothing left to "re-arm"). */
            ESP_LOGI(TAG, "timer ring stopped");
        } else if (finished == 1) {
            /* Snooze: swap the armed alarm for a 10 min countdown timer. */
            rtc_start_snooze();
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_snoozing = true;
            xSemaphoreGive(s_lock);
            ESP_LOGI(TAG, "snoozed %u min", (unsigned)ALARM_SNOOZE_MIN);
        } else {
            /* Dismiss: clear AF, cancel any stray snooze countdown, and
             * re-arm for the next earliest occurrence across the whole list
             * (possibly a different entry than the one that just rang). */
            rtc_stop_snooze();
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_snoozing = false;
            xSemaphoreGive(s_lock);
            alarm_recompute_next();
            ESP_LOGI(TAG, "dismissed, recomputed");
        }

        if (s_ring_cb) {
            s_ring_cb(false, ring_source);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t alarm_ring_now(alarm_ring_source_t source, uint8_t ring_mode)
{
    if (ring_mode > ALARM_RING_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_ringing || s_ring_queued) {
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }
    s_ring_queued = true;
    xSemaphoreGive(s_lock);

    esp_err_t err = ring_send((ring_cmd_t){ .type = RING_CMD_START, .source = source, .mode = ring_mode });
    if (err != ESP_OK) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (!s_ringing) s_ring_queued = false;
        xSemaphoreGive(s_lock);
    }
    return err;
}

esp_err_t alarm_ring_test(void)
{
    return alarm_ring_now(ALARM_RING_SOURCE_ALARM, ALARM_RING_BOTH);
}

/* Physical-button dismiss (docs/application.md SS8.3: stopping is
 * button-only, Power or Boot, either one - snooze stays touch-only via the
 * ring screen). Registered with power_mgmt.c, which fires this on every
 * short PWRKEY press and every BOOT press; a no-op whenever nothing is
 * ringing, so it can't interfere with normal wake/sleep/shutdown behavior. */
static void alarm_button_cb(void)
{
    alarm_dismiss();
}

/* ---- public API ---- */

esp_err_t alarm_init(void)
{
    esp_err_t audio_err = audio_init();
    if (audio_err != ESP_OK) {
        ESP_LOGE(TAG, "audio init: %s", esp_err_to_name(audio_err));
        return audio_err;
    }
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    if (!s_rtc_lock) s_rtc_lock = xSemaphoreCreateMutex();
    if (!s_rtc_lock) return ESP_ERR_NO_MEM;
    if (!s_nvs_lock) s_nvs_lock = xSemaphoreCreateMutex();
    if (!s_nvs_lock) return ESP_ERR_NO_MEM;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    cfg_load();
    xSemaphoreGive(s_lock);
    if (!s_ring_cmd) {
        s_ring_cmd = xQueueCreate(8, sizeof(ring_cmd_t));
        if (!s_ring_cmd) {
            ESP_LOGE(TAG, "ring command semaphore allocation failed");
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_ring_task == NULL) {
        if (xTaskCreate(ring_task, "alarm_ring", 4096, NULL, 5, &s_ring_task) != pdPASS) {
            ESP_LOGE(TAG, "ring task allocation failed");
            return ESP_ERR_NO_MEM;
        }
    }

    power_mgmt_register_button_cb(alarm_button_cb);

    alarm_recompute_next();
    ESP_LOGI(TAG, "init complete");
    return ESP_OK;
}

int alarm_add(uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask)
{
    if (hour > 23 || min > 59 || ring_mode > ALARM_RING_BOTH) {
        return -1;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        if (!s_entries[i].in_use) {
            s_entries[i].in_use = true;
            s_entries[i].enabled = true;
            s_entries[i].hour = hour;
            s_entries[i].min = min;
            s_entries[i].ring_mode = ring_mode;
            s_entries[i].weekday_mask = weekday_mask ? weekday_mask : ALARM_WEEKDAY_ALL;
            s_schedule_revision++;
            ESP_LOGI(TAG, "add[%d]: %02u:%02u mode=%u wmask=0x%02x", i,
                     (unsigned)hour, (unsigned)min, (unsigned)ring_mode,
                     (unsigned)s_entries[i].weekday_mask);
            xSemaphoreGive(s_lock);
            cfg_save();
            alarm_recompute_next();
            return i;
        }
    }
    ESP_LOGW(TAG, "add: all %d slots full", ALARM_MAX_COUNT);
    xSemaphoreGive(s_lock);
    return -1;
}

esp_err_t alarm_update(int idx, uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_entries[idx].in_use ||
        hour > 23 || min > 59 || ring_mode > ALARM_RING_BOTH) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_ARG;
    }
    s_entries[idx].hour = hour;
    s_entries[idx].min = min;
    s_entries[idx].ring_mode = ring_mode;
    s_entries[idx].weekday_mask = weekday_mask ? weekday_mask : ALARM_WEEKDAY_ALL;
    s_schedule_revision++;
    xSemaphoreGive(s_lock);
    cfg_save();
    alarm_recompute_next();
    return ESP_OK;
}

esp_err_t alarm_remove(int idx)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_entries[idx].in_use) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s_entries[idx], 0, sizeof(s_entries[idx]));
    s_schedule_revision++;
    xSemaphoreGive(s_lock);
    cfg_save();
    alarm_recompute_next();
    return ESP_OK;
}

esp_err_t alarm_set_enabled(int idx, bool enabled)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_entries[idx].in_use) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_ARG;
    }
    s_entries[idx].enabled = enabled;
    s_schedule_revision++;
    xSemaphoreGive(s_lock);
    cfg_save();
    alarm_recompute_next();
    return ESP_OK;
}

size_t alarm_get_all(alarm_entry_t *out, size_t max)
{
    size_t n = (max < ALARM_MAX_COUNT) ? max : ALARM_MAX_COUNT;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(out, s_entries, n * sizeof(alarm_entry_t));
    xSemaphoreGive(s_lock);
    return n;
}

bool alarm_is_armed(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool armed = s_armed_idx >= 0;
    xSemaphoreGive(s_lock);
    return armed;
}

int alarm_get_ringing_index(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int idx = s_armed_idx;
    xSemaphoreGive(s_lock);
    return idx;
}

bool alarm_is_ringing(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ringing = s_ringing;
    xSemaphoreGive(s_lock);
    return ringing;
}

bool alarm_get_sound_enabled(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool enabled = s_sound_enabled;
    xSemaphoreGive(s_lock);
    return enabled;
}

void alarm_set_sound_enabled(bool enabled)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (enabled == s_sound_enabled) {
        xSemaphoreGive(s_lock);
        return;
    }
    s_sound_enabled = enabled;
    xSemaphoreGive(s_lock);

    xSemaphoreTake(s_nvs_lock, portMAX_DELAY);
    nvs_handle_t h;
    if (nvs_open(ALARM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "sound_en", enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
    xSemaphoreGive(s_nvs_lock);
}

bool alarm_is_snoozing(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool snoozing = s_snoozing;
    xSemaphoreGive(s_lock);
    return snoozing;
}

esp_err_t alarm_check(void)
{
    /* Alarm flag: start the ring if the configured time matched. */
    if (!twatch_rtc_dev) {
        ESP_LOGW(TAG, "alarm_check: RTC dev not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    /* Read both flags; a transient AF read failure must not be allowed to skip
     * the snooze-timer (TF) check, or a 10-min re-ring would silently die. */
    bool af = false;
    xSemaphoreTake(s_rtc_lock, portMAX_DELAY);
    esp_err_t af_err = pcf85063a_alarm_triggered(twatch_rtc_dev, &af);
    bool tf = false;
    esp_err_t tf_err = pcf85063a_timer_triggered(twatch_rtc_dev, &tf);
    xSemaphoreGive(s_rtc_lock);
    if (af_err != ESP_OK && tf_err != ESP_OK) {
        return ESP_FAIL;
    }

    ring_cmd_t cmd = { 0 };
    bool send = false;
    bool stop_timer = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if ((af || tf) && !s_ringing && !s_ring_queued) {
        if (tf) {
            /* The PCF85063A timer re-loads and loops by itself; stop it now so
             * the snooze is a one-shot (the next ring is started explicitly by
             * another Snooze press). Also clears TF -> INT line de-asserts.
             * source/ring_mode are already set from the alarm that snoozed -
             * don't touch them, just resume ringing. */
            s_ring_queued = true;
            cmd.type = RING_CMD_RESUME;
            send = true;
            stop_timer = true;
        } else {
            /* AF: a fresh alarm match. Ring using whichever entry is
             * currently armed; fall back to a safe default rather than
             * crashing if s_armed_idx is somehow stale. */
            uint8_t mode = (s_armed_idx >= 0 && s_entries[s_armed_idx].in_use)
                           ? s_entries[s_armed_idx].ring_mode : ALARM_RING_BEEP;
            s_ring_queued = true;
            cmd = (ring_cmd_t){ .type = RING_CMD_START,
                                .source = ALARM_RING_SOURCE_ALARM, .mode = mode };
            send = true;
        }
    }
    xSemaphoreGive(s_lock);
    if (!send) return ESP_OK;

    if (stop_timer) rtc_stop_snooze();
    esp_err_t err = ring_send(cmd);
    if (err != ESP_OK) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (!s_ringing) s_ring_queued = false;
        xSemaphoreGive(s_lock);
    }
    return err;
}

esp_err_t alarm_handle_wake(void)
{
    /* The RTC INT line was (or may have been) the wake source. Read the flags
     * directly; if either fired, ring. Called from the power wake task. */
    return alarm_check();
}

esp_err_t alarm_dismiss(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ringing = s_ringing;
    xSemaphoreGive(s_lock);
    return ringing ? ring_send((ring_cmd_t){ .type = RING_CMD_DISMISS }) : ESP_OK;
}

esp_err_t alarm_snooze(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ringing = s_ringing;
    xSemaphoreGive(s_lock);
    return ringing ? ring_send((ring_cmd_t){ .type = RING_CMD_SNOOZE }) : ESP_OK;
}

void alarm_register_ring_cb(alarm_ring_cb_t cb)
{
    s_ring_cb = cb;
}
