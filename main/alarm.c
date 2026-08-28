/*
 * alarm.c - daily alarm clock (see alarm.h for the API).
 *
 * Implementation notes:
 *  - Config is persisted in NVS ("alarm"/"cfg").
 *  - The RTC alarm matches hour+minute at second 0 (day/weekday masked), so it
 *    fires once per day at the configured time. Matching only hour+minute would
 *    keep AF asserted all minute long and re-ring after every dismissal.
 *    AIE (alarm interrupt) and GPIO1 wake are armed only while enabled.
 *  - The ring runs on a dedicated task so it never blocks LVGL or the sensor
 *    cache. Beep (MAX98357A) enables the BLDO2 amp rail; vibration (DRV2605)
 *    enables the haptic M_EN line; both pulse on the same cadence.
 *  - Dismiss clears AF + re-arms for the next day. Snooze swaps the alarm
 *    interrupt for the RTC countdown timer (600 s, wakes on TF/TIE).
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
#include "max98357a.h"
#include "esp_lv_adapter.h"
#include <math.h>
#include <string.h>
#include <time.h>

static const char *TAG = "alarm";

#define ALARM_NVS_NS      "alarm"
#define ALARM_NVS_KEY     "cfg"

#define ALARM_SNOOZE_MIN  10

/* If a ring is left unanswered for this long, snooze automatically and re-arm
 * the countdown timer. A missed snooze therefore keeps ringing every 10 min
 * instead of forever. 0 disables auto-snooze. */
#define ALARM_AUTO_SNOOZE_MS  60000

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
#define RING_AMP          3276    /* ~10% of 16-bit FS, for quiet testing */
#define RING_EDGE_MS      12     /* attack/release ramp length */
#define RING_CHUNK_SAMPLES (AUDIO_SAMPLE_RATE / 25)  /* ~40 ms per write */

/* ---- state ---- */
static alarm_config_t s_cfg;
static bool s_ringing;
static bool s_snoozing;            /* 10 min snooze timer armed */
static TaskHandle_t s_ring_task;
static SemaphoreHandle_t s_ring_cmd;    /* binary: signals ring start */
static int s_ring_mode_pending;         /* -1 = none, 0=dismiss, 1=snooze */
static alarm_ring_cb_t s_ring_cb;

/* ---- NVS persistence ---- */

static void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open(ALARM_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, ALARM_NVS_KEY, &s_cfg, sizeof(s_cfg));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open(ALARM_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(s_cfg);
        if (nvs_get_blob(h, ALARM_NVS_KEY, &s_cfg, &len) != ESP_OK) {
            /* First boot: sensible default, disabled. */
            s_cfg.enabled = false;
            s_cfg.hour = 7;
            s_cfg.min = 0;
            s_cfg.ring_mode = ALARM_RING_BEEP;
        }
        nvs_close(h);
    } else {
        s_cfg.enabled = false;
        s_cfg.hour = 7;
        s_cfg.min = 0;
        s_cfg.ring_mode = ALARM_RING_BEEP;
    }
}

/* ---- RTC alarm helper ---- */

static void rtc_arm_alarm(void)
{
    /* Re-arming the daily alarm also cancels a pending snooze countdown. */
    pcf85063a_timer_stop(twatch_rtc_dev);
    s_snoozing = false;

    /* s_cfg.hour/min are the user's LOCAL alarm time, but the RTC hardware
     * match registers hold UTC. Convert using today's date (from the live
     * RTC) so the DST offset in effect right now is the one baked into the
     * match registers. Accepted edge case: since day/weekday are masked (the
     * alarm fires at the same hh:mm UTC every day), a DST transition between
     * arming and the alarm next firing shifts it by the DST delta until the
     * alarm is re-armed (e.g. by changing the time, toggling it, or the
     * dismiss/re-arm each ring already does) - not re-computed live. */
    struct tm lt = {0};
    pcf85063a_time_t rtc_now;
    if (pcf85063a_get_time(twatch_rtc_dev, &rtc_now) == ESP_OK) {
        time_t epoch_now = pcf85063a_time_to_epoch(&rtc_now);
        localtime_r(&epoch_now, &lt);
    } else {
        time_t now = time(NULL);
        localtime_r(&now, &lt);
    }
    lt.tm_hour = s_cfg.hour;
    lt.tm_min = s_cfg.min;
    lt.tm_sec = 0;
    lt.tm_isdst = -1;
    time_t alarm_epoch = mktime(&lt);
    struct tm utc;
    gmtime_r(&alarm_epoch, &utc);

    pcf85063a_alarm_t a;
    memset(&a, 0, sizeof(a));
    a.enabled = true;
    /* Match hour+minute exactly at second 0 -> fires once daily at hh:mm:00.
     * Matching only hour+minute would keep AF asserted for the entire minute,
     * so a dismissal would be overridden a second later by a fresh AF. */
    a.mask_sec = false;
    a.time.sec = 0;
    a.mask_day = true;
    a.mask_weekday = true;
    a.time.hour = (uint8_t)utc.tm_hour;
    a.time.min = (uint8_t)utc.tm_min;
    pcf85063a_set_alarm(twatch_rtc_dev, &a);
    ESP_LOGI(TAG, "armed local %02u:%02u -> RTC UTC %02u:%02u",
             (unsigned)s_cfg.hour, (unsigned)s_cfg.min, a.time.hour, a.time.min);
}

static void rtc_disarm_alarm(void)
{
    pcf85063a_clear_alarm(twatch_rtc_dev);
}

/* ---- ring task ---- */

/* Render one repeating cycle into buf: RING_BEEPS sine beeps (~RING_TONE_MS
 * on, ~RING_GAP_MS off) plus a paced silence tail up to RING_CYCLE_MS. Each
 * beep has a linear attack/release envelope so the speaker doesn't click or
 * distort. Writes back-to-back, the DMA never underruns -> no crackle.
 * Returns the number of samples rendered. */
static size_t ring_render_cycle(int16_t *buf, size_t cap)
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
                                * RING_AMP * env);
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

static void ring_set_outputs(bool on)
{
    if (s_cfg.ring_mode == ALARM_RING_BEEP || s_cfg.ring_mode == ALARM_RING_BOTH) {
        axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, on);   /* amp */
    }
    if (s_cfg.ring_mode == ALARM_RING_VIB || s_cfg.ring_mode == ALARM_RING_BOTH) {
        xl9555_set_output(twatch_xl9555_dev, TWATCH_XL_GPIO_HAPTIC_EN, on);  /* M_EN */
    }
}

static void ring_vibrate(void)
{
    drv2605_play(twatch_haptic_dev, 47);   /* strong click */
}

static void ring_task(void *arg)
{
    (void)arg;
    size_t cycle_n = (size_t)(AUDIO_SAMPLE_RATE * RING_CYCLE_MS / 1000);
    int16_t *buf = heap_caps_malloc(cycle_n * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf) {
        ring_render_cycle(buf, cycle_n);
    }

    for (;;) {
        if (xSemaphoreTake(s_ring_cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_ringing = true;
        if (s_ring_cb) {
            s_ring_cb(true);
        }
        ESP_LOGI(TAG, "ringing %02u:%02u (mode %u)",
                 (unsigned)s_cfg.hour, (unsigned)s_cfg.min, (unsigned)s_cfg.ring_mode);
        ring_set_outputs(true);

        bool beep = (s_cfg.ring_mode == ALARM_RING_BEEP || s_cfg.ring_mode == ALARM_RING_BOTH);
        bool vib  = (s_cfg.ring_mode == ALARM_RING_VIB || s_cfg.ring_mode == ALARM_RING_BOTH);

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

            /* Unanswered for too long -> snooze automatically. Only when a
             * real alarm is configured (a bare `alarmring` test with the alarm
             * off must not spin its own 10-min snooze cycle). */
            if (ALARM_AUTO_SNOOZE_MS > 0 && alarm_is_armed() &&
                pdTICKS_TO_MS(xTaskGetTickCount() - ring_started) >= ALARM_AUTO_SNOOZE_MS) {
                ESP_LOGI(TAG, "auto-snoozed (%lu ms unanswered)",
                         (unsigned long)pdTICKS_TO_MS(xTaskGetTickCount() - ring_started));
                s_ring_mode_pending = 1;
            }
        }

        ring_set_outputs(false);
        int finished = s_ring_mode_pending;   /* 0=dismiss, 1=snooze */
        s_ring_mode_pending = -1;
        s_ringing = false;

        /* Update snooze/dismiss state and RTC before the UI callback, so the
         * watch face can show the Zz icon on the very first update. */
        if (finished == 1) {
            /* Snooze: swap the daily alarm for a 10 min countdown timer. */
            rtc_disarm_alarm();
            pcf85063a_set_timer_minutes(twatch_rtc_dev, ALARM_SNOOZE_MIN, true);
            s_snoozing = true;
            ESP_LOGI(TAG, "snoozed %u min", (unsigned)ALARM_SNOOZE_MIN);
        } else {
            /* Dismiss: clear AF + re-arm AIE for the next day (masked alarm
             * re-fires daily, so just re-enabling the interrupt suffices).
             * Also make sure no snooze countdown is still latched. */
            pcf85063a_timer_stop(twatch_rtc_dev);
            rtc_arm_alarm();
            s_snoozing = false;
            ESP_LOGI(TAG, "dismissed, re-armed for next day");
        }

        if (s_ring_cb) {
            s_ring_cb(false);
        }
    }

    /* Task termination path (not currently used): release the ring buffer. */
    if (buf) {
        heap_caps_free(buf);
    }
    vTaskDelete(NULL);
}

/* Notify the ring task to start. */
static void ring_start(void)
{
    s_ring_mode_pending = -1;
    s_snoozing = false;   /* a new ring means the snooze cycle is over */
    xSemaphoreGive(s_ring_cmd);
}

esp_err_t alarm_ring_test(void)
{
    ring_start();
    return ESP_OK;
}

/* ---- public API ---- */

esp_err_t alarm_init(void)
{
    cfg_load();
    if (!s_ring_cmd) {
        s_ring_cmd = xSemaphoreCreateBinary();
    }
    if (s_ring_task == NULL) {
        xTaskCreate(ring_task, "alarm_ring", 4096, NULL, 5, &s_ring_task);
    }

    if (s_cfg.enabled) {
        rtc_arm_alarm();
    } else {
        rtc_disarm_alarm();
    }
    ESP_LOGI(TAG, "init: %s %02u:%02u mode %u",
             s_cfg.enabled ? "armed" : "disabled",
             (unsigned)s_cfg.hour, (unsigned)s_cfg.min, (unsigned)s_cfg.ring_mode);
    return ESP_OK;
}

esp_err_t alarm_set(uint8_t hour, uint8_t min, bool enabled, uint8_t ring_mode)
{
    if (hour > 23 || min > 59 || ring_mode > ALARM_RING_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg.hour = hour;
    s_cfg.min = min;
    s_cfg.ring_mode = ring_mode;
    s_cfg.enabled = enabled;
    cfg_save();

    if (enabled) {
        rtc_arm_alarm();
    } else {
        rtc_disarm_alarm();
    }
    ESP_LOGI(TAG, "set: %s %02u:%02u mode %u",
             enabled ? "armed" : "disabled", (unsigned)hour, (unsigned)min,
             (unsigned)ring_mode);
    return ESP_OK;
}

esp_err_t alarm_arm(void)
{
    s_cfg.enabled = true;
    cfg_save();
    rtc_arm_alarm();
    return ESP_OK;
}

esp_err_t alarm_disarm(void)
{
    s_cfg.enabled = false;
    cfg_save();
    rtc_disarm_alarm();
    /* Also cancel a pending snooze timer. */
    pcf85063a_timer_stop(twatch_rtc_dev);
    s_snoozing = false;
    return ESP_OK;
}

bool alarm_is_armed(void)
{
    return s_cfg.enabled;
}

bool alarm_is_ringing(void)
{
    return s_ringing;
}

bool alarm_is_snoozing(void)
{
    return s_snoozing;
}

void alarm_get_config(alarm_config_t *cfg)
{
    if (cfg) {
        *cfg = s_cfg;
    }
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
             * another Snooze press). Also clears TF -> INT line de-asserts. */
            pcf85063a_timer_stop(twatch_rtc_dev);
        }
        ring_start();
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
    s_ring_mode_pending = 1;
    return ESP_OK;
}

void alarm_register_ring_cb(alarm_ring_cb_t cb)
{
    s_ring_cb = cb;
}