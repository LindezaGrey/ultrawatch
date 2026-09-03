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
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "twatch_board.h"
#include "axp2101.h"
#include "xl9555.h"
#include "pcf85063a.h"
#include "drv2605.h"
#include "haptic.h"
#include "max98357a.h"
#include "esp_lv_adapter.h"
#include "power_mgmt.h"
#include <math.h>
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

/* Ring melody: a repeating cycle of 3 soft beeps followed by a paced pause.
 * Full-scale raw sines hard-clip the small speaker and click on the hard
 * on/off edges, so each beep gets a gentle attack/release envelope and a
 * moderate amplitude. The whole cycle stays inside one buffer and is written
 * back-to-back, so the I2S DMA never underruns between writes (which made
 * the old burst + vTaskDelay cadence crackle). */
#define RING_TONE_MS      170
#define RING_GAP_MS       130
#define RING_BEEPS        3
#define RING_CYCLE_MS     2000   /* beep-beep-beep then this long silence */
#define RING_BEEP_HZ      880
#define RING_AMP          3276    /* initial level: ~10% of 16-bit FS */
#define RING_AMP_MAX       16380  /* final retry level: ~50%, below clipping */
#define RING_EDGE_MS      12     /* attack/release ramp length */
#define RING_CHUNK_SAMPLES (AUDIO_SAMPLE_RATE / 25)  /* ~40 ms per write */

/* ---- state ---- */
static alarm_entry_t s_entries[ALARM_MAX_COUNT];
static int s_armed_idx = -1;              /* which entry the RTC alarm register currently targets, -1 = none */
static bool s_ringing;
static bool s_snoozing;                   /* 10 min snooze timer armed */
static alarm_ring_source_t s_ring_source; /* which kind of ring is/was in progress */
static uint8_t s_ring_mode_active;        /* ALARM_RING_* used for the current/last ring cycle */
static TaskHandle_t s_ring_task;
static SemaphoreHandle_t s_ring_cmd;      /* binary: signals ring start */
static int s_ring_mode_pending;           /* -1 = none, 0=dismiss, 1=snooze */
static alarm_ring_cb_t s_ring_cb;
static bool s_sound_enabled = true;       /* global mute, see alarm_set_sound_enabled() */
static uint8_t s_auto_snooze_count;       /* completed automatic retries in this alarm cycle */
static bool s_auto_snooze_exhausted;

static void alarm_recompute_next(void);

/* ---- NVS persistence ---- */

static void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open(ALARM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, ALARM_NVS_KEY, s_entries, sizeof(s_entries));
        nvs_commit(h);
        nvs_close(h);
    }
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

/* Scans all enabled entries, finds whichever (weekday, hour, min) occurrence
 * is soonest from now, and arms the RTC's one hardware alarm register for
 * exactly that moment (a specific UTC weekday, not "every day"). Re-run
 * after every list edit and after every alarm-sourced ring finishes. */
static void alarm_recompute_next(void)
{
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
        if (!s_entries[i].in_use || !s_entries[i].enabled) {
            continue;
        }
        uint8_t mask = s_entries[i].weekday_mask ? s_entries[i].weekday_mask : ALARM_WEEKDAY_ALL;
        for (int wday = 0; wday < 7; wday++) {
            if (!(mask & (1u << wday))) {
                continue;
            }
            int delta = (wday - today_lt.tm_wday + 7) % 7;
            struct tm cand = today_lt;
            cand.tm_mday += delta;
            cand.tm_hour = s_entries[i].hour;
            cand.tm_min = s_entries[i].min;
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

    s_armed_idx = best_idx;
    if (best_idx < 0) {
        rtc_disarm_alarm();
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
    ESP_LOGI(TAG, "armed entry %d: local %02u:%02u -> RTC UTC wday=%u %02u:%02u",
             best_idx, (unsigned)s_entries[best_idx].hour, (unsigned)s_entries[best_idx].min,
             (unsigned)a.time.weekday, (unsigned)a.time.hour, (unsigned)a.time.min);
}

/* ---- ring task ---- */

/* Render one repeating cycle into buf: RING_BEEPS sine beeps (~RING_TONE_MS
 * on, ~RING_GAP_MS off) plus a paced silence tail up to RING_CYCLE_MS. Each
 * beep has a linear attack/release envelope so the speaker doesn't click or
 * distort. Writes back-to-back, the DMA never underruns -> no crackle.
 * Returns the number of samples rendered. */
static size_t ring_render_cycle(int16_t *buf, size_t cap, int16_t amplitude)
{
    size_t tone_n = (size_t)(AUDIO_SAMPLE_RATE * RING_TONE_MS / 1000);
    size_t gap_n  = (size_t)(AUDIO_SAMPLE_RATE * RING_GAP_MS / 1000);
    size_t unit_n = tone_n + gap_n;
    size_t n = unit_n * RING_BEEPS;
    size_t edge_n = (size_t)(AUDIO_SAMPLE_RATE * RING_EDGE_MS / 1000);
    if (edge_n > tone_n / 2) {
        edge_n = tone_n / 2;
    }
    for (int b = 0; b < RING_BEEPS; b++) {
        int16_t *tone = buf + b * unit_n;
        for (size_t i = 0; i < tone_n; i++) {
            /* Linear attack in the first edge_n samples, release at the end. */
            float env;
            if (i < edge_n) {
                env = (float)i / (float)edge_n;
            } else if (i > tone_n - edge_n) {
                env = (float)(tone_n - i) / (float)edge_n;
            } else {
                env = 1.0f;
            }
            tone[i] = (int16_t)(sinf(2.0f * 3.14159265f * RING_BEEP_HZ * i / AUDIO_SAMPLE_RATE)
                                * amplitude * env);
        }
        memset(tone + tone_n, 0, gap_n * sizeof(int16_t));
    }
    /* Silence tail so the melody repeats every RING_CYCLE_MS, all in-buffer. */
    size_t cycle_n = (size_t)(AUDIO_SAMPLE_RATE * RING_CYCLE_MS / 1000);
    if (cycle_n > cap) {
        cycle_n = cap;
    }
    if (cycle_n > n) {
        memset(buf + n, 0, (cycle_n - n) * sizeof(int16_t));
    }
    return cycle_n;
}

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
    if (s_ring_mode_active == ALARM_RING_BEEP || s_ring_mode_active == ALARM_RING_BOTH) {
        axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, on);   /* amp */
    }
    if (s_ring_mode_active == ALARM_RING_VIB || s_ring_mode_active == ALARM_RING_BOTH) {
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, on);  /* M_EN */
    }
}

static void ring_vibrate(void)
{
    drv2605_play(twatch_haptic_dev, haptic_get_wave_id());
}

static void ring_task(void *arg)
{
    (void)arg;
    size_t cycle_n = (size_t)(AUDIO_SAMPLE_RATE * RING_CYCLE_MS / 1000);
    int16_t *buf = heap_caps_malloc(cycle_n * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    for (;;) {
        if (xSemaphoreTake(s_ring_cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_ringing = true;
        if (buf) {
            ring_render_cycle(buf, cycle_n, ring_amplitude());
        }
        if (s_ring_cb) {
            s_ring_cb(true, s_ring_source);
        }
        ESP_LOGI(TAG, "ringing source=%d mode=%u armed_idx=%d sound=%d",
                 (int)s_ring_source, (unsigned)s_ring_mode_active, s_armed_idx,
                 (int)s_sound_enabled);
        /* Global mute: screen/dismiss/snooze/auto-snooze timing all still
         * work normally, just silently - a master off-switch, not a mode
         * change to the ringing entry's own Beep/Vib/Both. */
        if (s_sound_enabled) {
            ring_set_outputs(true);
        }

        bool beep = s_sound_enabled &&
                    (s_ring_mode_active == ALARM_RING_BEEP || s_ring_mode_active == ALARM_RING_BOTH);
        bool vib  = s_sound_enabled &&
                    (s_ring_mode_active == ALARM_RING_VIB || s_ring_mode_active == ALARM_RING_BOTH);

        /* Ring until dismiss or snooze. The melody cycle is written in a tight
         * loop so the I2S DMA stays continuously fed (no underrun crackle);
         * max98357a_write blocks at the sample rate, pacing the playback. */
        TickType_t ring_started = xTaskGetTickCount();
        while (s_ring_mode_pending < 0) {
            if (beep && buf) {
                size_t off = 0;
                while (off < cycle_n && s_ring_mode_pending < 0) {
                    size_t chunk = cycle_n - off;
                    if (chunk > RING_CHUNK_SAMPLES) {
                        chunk = RING_CHUNK_SAMPLES;
                    }
                    max98357a_write(buf + off, chunk);
                    off += chunk;
                    esp_lv_adapter_report_activity();
                }
            }
            if (vib) {
                ring_vibrate();
                vTaskDelay(pdMS_TO_TICKS(300));
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
                s_armed_idx >= 0 &&
                pdTICKS_TO_MS(xTaskGetTickCount() - ring_started) >= ALARM_AUTO_SNOOZE_MS) {
                if (s_auto_snooze_count < ALARM_AUTO_SNOOZE_MAX) {
                    s_auto_snooze_count++;
                    ESP_LOGI(TAG, "auto-snoozed retry %u/%u (%lu ms unanswered)",
                             (unsigned)s_auto_snooze_count,
                             (unsigned)ALARM_AUTO_SNOOZE_MAX,
                             (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - ring_started));
                    s_ring_mode_pending = 1;
                } else if (!s_auto_snooze_exhausted) {
                    s_auto_snooze_exhausted = true;
                    ESP_LOGW(TAG, "auto-snooze limit reached; keeping final alarm ringing");
                }
            }
        }

        ring_set_outputs(false);
        int finished = s_ring_mode_pending;   /* 0=dismiss, 1=snooze */
        s_ring_mode_pending = -1;
        s_ringing = false;

        /* Update snooze/dismiss state and RTC before the UI callback, so the
         * watch face can show the Zz icon on the very first update. */
        if (s_ring_source == ALARM_RING_SOURCE_TIMER) {
            /* No RTC interaction - main/cd_timer.c already cleared its own
             * active state the moment the countdown hit zero and this ring
             * started (a timer has nothing left to "re-arm"). */
            ESP_LOGI(TAG, "timer ring stopped");
        } else if (finished == 1) {
            /* Snooze: swap the armed alarm for a 10 min countdown timer. */
            rtc_disarm_alarm();
            pcf85063a_set_timer_minutes(twatch_rtc_dev, ALARM_SNOOZE_MIN, true);
            s_snoozing = true;
            ESP_LOGI(TAG, "snoozed %u min", (unsigned)ALARM_SNOOZE_MIN);
        } else {
            /* Dismiss: clear AF, cancel any stray snooze countdown, and
             * re-arm for the next earliest occurrence across the whole list
             * (possibly a different entry than the one that just rang). */
            pcf85063a_timer_stop(twatch_rtc_dev);
            s_snoozing = false;
            alarm_recompute_next();
            ESP_LOGI(TAG, "dismissed, recomputed (armed_idx=%d)", s_armed_idx);
        }

        if (s_ring_cb) {
            s_ring_cb(false, s_ring_source);
        }
    }

    /* Task termination path (not currently used): release the ring buffer. */
    if (buf) {
        heap_caps_free(buf);
    }
    vTaskDelete(NULL);
}

/* Notify the ring task to start with whatever s_ring_source/s_ring_mode_active
 * are already set to. Private - external callers go through
 * alarm_ring_now(), which sets those fields first. */
static void ring_start(void)
{
    s_ring_mode_pending = -1;
    s_snoozing = false;   /* a new ring means the snooze cycle is over */
    xSemaphoreGive(s_ring_cmd);
}

esp_err_t alarm_ring_now(alarm_ring_source_t source, uint8_t ring_mode)
{
    if (ring_mode > ALARM_RING_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }
    s_ring_source = source;
    s_ring_mode_active = ring_mode;
    s_auto_snooze_count = 0;
    s_auto_snooze_exhausted = false;
    ring_start();
    return ESP_OK;
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
    if (s_ringing) {
        alarm_dismiss();
    }
}

/* ---- public API ---- */

esp_err_t alarm_init(void)
{
    cfg_load();
    if (!s_ring_cmd) {
        s_ring_cmd = xSemaphoreCreateBinary();
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
    ESP_LOGI(TAG, "init: armed_idx=%d", s_armed_idx);
    return ESP_OK;
}

int alarm_add(uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask)
{
    if (hour > 23 || min > 59 || ring_mode > ALARM_RING_BOTH) {
        return -1;
    }
    for (int i = 0; i < ALARM_MAX_COUNT; i++) {
        if (!s_entries[i].in_use) {
            s_entries[i].in_use = true;
            s_entries[i].enabled = true;
            s_entries[i].hour = hour;
            s_entries[i].min = min;
            s_entries[i].ring_mode = ring_mode;
            s_entries[i].weekday_mask = weekday_mask ? weekday_mask : ALARM_WEEKDAY_ALL;
            cfg_save();
            alarm_recompute_next();
            ESP_LOGI(TAG, "add[%d]: %02u:%02u mode=%u wmask=0x%02x", i,
                     (unsigned)hour, (unsigned)min, (unsigned)ring_mode,
                     (unsigned)s_entries[i].weekday_mask);
            return i;
        }
    }
    ESP_LOGW(TAG, "add: all %d slots full", ALARM_MAX_COUNT);
    return -1;
}

esp_err_t alarm_update(int idx, uint8_t hour, uint8_t min, uint8_t ring_mode, uint8_t weekday_mask)
{
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_entries[idx].in_use ||
        hour > 23 || min > 59 || ring_mode > ALARM_RING_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }
    s_entries[idx].hour = hour;
    s_entries[idx].min = min;
    s_entries[idx].ring_mode = ring_mode;
    s_entries[idx].weekday_mask = weekday_mask ? weekday_mask : ALARM_WEEKDAY_ALL;
    cfg_save();
    alarm_recompute_next();
    return ESP_OK;
}

esp_err_t alarm_remove(int idx)
{
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_entries[idx].in_use) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s_entries[idx], 0, sizeof(s_entries[idx]));
    cfg_save();
    alarm_recompute_next();
    return ESP_OK;
}

esp_err_t alarm_set_enabled(int idx, bool enabled)
{
    if (idx < 0 || idx >= ALARM_MAX_COUNT || !s_entries[idx].in_use) {
        return ESP_ERR_INVALID_ARG;
    }
    s_entries[idx].enabled = enabled;
    cfg_save();
    alarm_recompute_next();
    return ESP_OK;
}

size_t alarm_get_all(alarm_entry_t *out, size_t max)
{
    size_t n = (max < ALARM_MAX_COUNT) ? max : ALARM_MAX_COUNT;
    memcpy(out, s_entries, n * sizeof(alarm_entry_t));
    return n;
}

bool alarm_is_armed(void)
{
    return s_armed_idx >= 0;
}

int alarm_get_ringing_index(void)
{
    return s_armed_idx;
}

bool alarm_is_ringing(void)
{
    return s_ringing;
}

bool alarm_get_sound_enabled(void)
{
    return s_sound_enabled;
}

void alarm_set_sound_enabled(bool enabled)
{
    if (enabled == s_sound_enabled) {
        return;
    }
    s_sound_enabled = enabled;
    nvs_handle_t h;
    if (nvs_open(ALARM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "sound_en", enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool alarm_is_snoozing(void)
{
    return s_snoozing;
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
    esp_err_t af_err = pcf85063a_alarm_triggered(twatch_rtc_dev, &af);
    bool tf = false;
    esp_err_t tf_err = pcf85063a_timer_triggered(twatch_rtc_dev, &tf);
    if (af_err != ESP_OK && tf_err != ESP_OK) {
        return ESP_FAIL;
    }

    if ((af || tf) && !s_ringing) {
        if (tf) {
            /* The PCF85063A timer re-loads and loops by itself; stop it now so
             * the snooze is a one-shot (the next ring is started explicitly by
             * another Snooze press). Also clears TF -> INT line de-asserts.
             * source/ring_mode are already set from the alarm that snoozed -
             * don't touch them, just resume ringing. */
            pcf85063a_timer_stop(twatch_rtc_dev);
            ring_start();
        } else {
            /* AF: a fresh alarm match. Ring using whichever entry is
             * currently armed; fall back to a safe default rather than
             * crashing if s_armed_idx is somehow stale. */
            uint8_t mode = (s_armed_idx >= 0 && s_entries[s_armed_idx].in_use)
                           ? s_entries[s_armed_idx].ring_mode : ALARM_RING_BEEP;
            alarm_ring_now(ALARM_RING_SOURCE_ALARM, mode);
        }
    }
    return ESP_OK;
}

esp_err_t alarm_handle_wake(void)
{
    /* The RTC INT line was (or may have been) the wake source. Read the flags
     * directly; if either fired, ring. Called from the power wake task. */
    return alarm_check();
}

esp_err_t alarm_dismiss(void)
{
    s_ring_mode_pending = 0;
    return ESP_OK;
}

esp_err_t alarm_snooze(void)
{
    if (s_ring_source == ALARM_RING_SOURCE_TIMER) {
        /* No snooze concept for timers - just stop it. */
        return alarm_dismiss();
    }
    s_ring_mode_pending = 1;
    return ESP_OK;
}

void alarm_register_ring_cb(alarm_ring_cb_t cb)
{
    s_ring_cb = cb;
}
