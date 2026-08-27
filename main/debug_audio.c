/*
 * debug_audio.c - audio debug console commands: tone/rec/playrec/tonerec/sweep.
 *
 * Moved out of uwatch_main.c's debug_process_cmd() dispatch (mechanical
 * refactor, no behavior change). Owns the shared recording buffer used by
 * rec/tonerec/sweep/playrec.
 */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "max98357a.h"
#include "t3902.h"
#include "axp2101.h"
#include "twatch_board.h"
#include "debug_audio.h"

/* Audio test: buffer holding the last recording (PSRAM), shared by rec/playrec. */
static int16_t *s_rec_buf;
static size_t s_rec_n;
static SemaphoreHandle_t s_rec_done;   /* signalled when a background rec finishes */

/* Ensure s_rec_buf can hold n samples; returns true on success. On alloc/realloc
 * failure the previous buffer is left intact (it may still be NULL), so the
 * caller can retry later without having s_rec_n outpace the actual buffer size.
 * When only shrinking, the existing larger buffer is reused as-is. */
static bool rec_ensure_buf(size_t n)
{
    if (!s_rec_buf) {
        s_rec_buf = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    } else if (n > s_rec_n) {
        int16_t *nb = heap_caps_realloc(s_rec_buf, n * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!nb) {
            return false;   /* old (smaller) buffer kept, s_rec_n unchanged */
        }
        s_rec_buf = nb;
    }
    return s_rec_buf != NULL;
}

/* Background recorder: fills s_rec_buf while the speaker plays (the amp and
 * mic are on separate I2S controllers, so TX + RX can run concurrently). */
static void rec_task(void *arg)
{
    (void)arg;
    size_t n = s_rec_n;
    t3902_read(s_rec_buf, n);
    if (s_rec_done) {
        xSemaphoreGive(s_rec_done);
    }
    vTaskDelete(NULL);
}

/* Shared by tonerec/sweep, which differ only in how `buf` (n samples) was
 * generated: play it back while recording concurrently into the shared rec
 * buffer, then report the peak recorded amplitude. `tag` prefixes every
 * printed line and names the recording task; `info_line` is printed once
 * the rec buffer is confirmed available, matching where each caller used to
 * print its own "Hz/ms" announcement. */
static void play_and_record(const int16_t *buf, size_t n, const char *tag, const char *info_line)
{
    if (!s_rec_done) {
        s_rec_done = xSemaphoreCreateBinary();
    }
    xSemaphoreTake(s_rec_done, 0);
    if (!rec_ensure_buf(n)) {
        printf("%s: no mem for rec buffer\n", tag);
        return;
    }
    s_rec_n = n;
    printf("%s", info_line);
    xTaskCreate(rec_task, tag, 2048, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t err = max98357a_write(buf, n);
    printf("%s: playback %s\n", tag, (err == ESP_OK) ? "ok" : esp_err_to_name(err));
    if (xSemaphoreTake(s_rec_done, pdMS_TO_TICKS(n * 1000 / AUDIO_SAMPLE_RATE + 5000)) != pdTRUE) {
        printf("%s: rec timed out\n", tag);
    } else {
        int32_t peak = 0;
        for (size_t i = 0; i < n; i++) {
            int32_t v = s_rec_buf[i];
            if (v < 0) v = -v;
            if (v > peak) peak = v;
        }
        printf("%s: recorded, peak amp %ld (play 'playrec' to hear)\n", tag, (long)peak);
    }
}

void debug_audio_tone(const char *args)
{
    /* Play a sine tone. Usage: "tone" (440 Hz, 500 ms, near-max vol)
     * or "tone <hz> <ms>" or "tone <hz> <ms> <amp 0-32767>". */
    int hz = 440, ms = 500, amp = 30000;
    if (args[0] != '\0') {
        hz = atoi(args);
        const char *sp = strchr(args, ' ');
        if (sp) {
            ms = atoi(sp + 1);
            sp = strchr(sp + 1, ' ');
            if (sp) {
                amp = atoi(sp + 1);
            }
        }
    }
    if (hz <= 0) {
        hz = 440;
    }
    if (ms <= 0) {
        ms = 500;
    }
    if (amp <= 0 || amp > 32767) {
        amp = 30000;
    }
    size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);
    int16_t *buf = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        printf("tone: no mem for %u samples\n", (unsigned)n);
    } else {
        for (size_t i = 0; i < n; i++) {
            buf[i] = (int16_t)(sinf(2.0f * 3.14159265f * hz * i / AUDIO_SAMPLE_RATE) * amp);
        }
        printf("tone: %d Hz, %d ms, amp %d (%u samples)\n", hz, ms, amp, (unsigned)n);
        axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, true);   /* amp */
        esp_err_t err = max98357a_write(buf, n);
        axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);
        printf("tone: %s\n", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
        heap_caps_free(buf);
    }
}

void debug_audio_rec(const char *args)
{
    /* Record a short mono clip to a PSRAM buffer.
     * Usage: "rec" (2 s) or "rec <ms>". */
    int ms = 2000;
    if (args[0] != '\0') {
        ms = atoi(args);
    }
    if (ms <= 0) {
        ms = 2000;
    }
    if (ms > 10000) {
        ms = 10000;
    }
    size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);
    if (!rec_ensure_buf(n)) {
        printf("rec: no mem for %u samples\n", (unsigned)n);
    } else {
        s_rec_n = n;
        printf("rec: recording %d ms (%u samples)...\n", ms, (unsigned)n);
        esp_err_t err = t3902_read(s_rec_buf, n);
        if (err == ESP_OK) {
            /* Report the peak amplitude so we can tell if the mic
             * captured real audio vs silence. */
            int32_t peak = 0;
            for (size_t i = 0; i < n; i++) {
                int32_t v = s_rec_buf[i];
                if (v < 0) v = -v;
                if (v > peak) peak = v;
            }
            printf("rec: done, peak amp %ld\n", (long)peak);
        } else {
            printf("rec: %s\n", esp_err_to_name(err));
        }
    }
}

void debug_audio_playrec(const char *args)
{
    (void)args;
    /* Play back the last recording (loops 3x so it's audible). */
    if (!s_rec_buf || s_rec_n == 0) {
        printf("playrec: nothing recorded yet (use rec first)\n");
    } else {
        printf("playrec: playing %u samples x3\n", (unsigned)s_rec_n);
        for (int r = 0; r < 3; r++) {
            esp_err_t err = max98357a_write(s_rec_buf, s_rec_n);
            if (err != ESP_OK) {
                printf("playrec: %s\n", esp_err_to_name(err));
                break;
            }
        }
        printf("playrec: done\n");
    }
}

void debug_audio_tonerec(const char *args)
{
    /* Play a single tone and record it with the mic (like sweep,
     * but fixed frequency). Usage: "tonerec" (440 Hz, 3 s) or
     * "tonerec <hz> <ms> <amp>". */
    int hz = 440, ms = 3000, amp = 30000;
    if (args[0] != '\0') {
        hz = atoi(args);
        const char *sp = strchr(args, ' ');
        if (sp) {
            ms = atoi(sp + 1);
            sp = strchr(sp + 1, ' ');
            if (sp) {
                amp = atoi(sp + 1);
            }
        }
    }
    if (hz < 20) hz = 440;
    if (ms <= 0) ms = 3000;
    if (ms > 10000) ms = 10000;
    if (amp <= 0 || amp > 32767) amp = 30000;
    size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);

    int16_t *tone = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tone) {
        printf("tonerec: no mem for %u samples\n", (unsigned)n);
        return;
    }
    for (size_t i = 0; i < n; i++) {
        tone[i] = (int16_t)(sinf(2.0f * 3.14159265f * hz * i / AUDIO_SAMPLE_RATE) * amp);
    }
    char info[96];
    snprintf(info, sizeof(info), "tonerec: %d Hz, %d ms, amp %d (playing + recording)\n", hz, ms, amp);
    play_and_record(tone, n, "tonerec", info);
    heap_caps_free(tone);
}

void debug_audio_sweep(const char *args)
{
    /* Play a log-frequency sweep and record it with the mic.
     * Usage: "sweep" (200..8000 Hz, 3 s) or "sweep <f0> <f1> <ms>". */
    int f0 = 200, f1 = 8000, ms = 3000, amp = 30000;
    if (args[0] != '\0') {
        f0 = atoi(args);
        const char *sp = strchr(args, ' ');
        if (sp) {
            f1 = atoi(sp + 1);
            sp = strchr(sp + 1, ' ');
            if (sp) {
                ms = atoi(sp + 1);
            }
        }
    }
    if (f0 < 20) f0 = 20;
    if (f1 <= f0) f1 = f0 + 100;
    if (ms <= 0) ms = 3000;
    if (ms > 10000) ms = 10000;
    size_t n = (size_t)(AUDIO_SAMPLE_RATE * ms / 1000);
    int f0_start = f0, f1_end = f1;

    /* Generate a logarithmic sine sweep f0 -> f1 over the duration.
     * The running frequency must be a double: incrementing an int by
     * the per-sample factor (~1.00008) truncates and never moves. */
    int16_t *sweep = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!sweep) {
        printf("sweep: no mem for %u samples\n", (unsigned)n);
        return;
    }
    double k = exp(log((double)f1 / f0) / n);
    double f = f0;
    double ph = 0.0;
    for (size_t i = 0; i < n; i++) {
        sweep[i] = (int16_t)(sinf(ph) * amp);
        ph += 2.0 * 3.14159265358979 * f / AUDIO_SAMPLE_RATE;
        f *= k;
    }
    char info[96];
    snprintf(info, sizeof(info), "sweep: %d -> %d Hz, %d ms (playing + recording)\n", f0_start, f1_end, ms);
    play_and_record(sweep, n, "sweep", info);
    heap_caps_free(sweep);
}
