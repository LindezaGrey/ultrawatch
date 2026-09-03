#include "audio.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "axp2101.h"
#include "max98357a.h"
#include "twatch_board.h"

#define AUDIO_CMD_QUEUE_LEN 8
#define AUDIO_CHUNK_SAMPLES (AUDIO_SAMPLE_RATE / 25)
#define ALARM_TONE_MS 170
#define ALARM_GAP_MS 130
#define ALARM_BEEPS 3
#define ALARM_CYCLE_MS 2000
#define ALARM_BEEP_HZ 880
#define ALARM_EDGE_MS 12

typedef enum { AUDIO_CMD_ALARM_START, AUDIO_CMD_ALARM_STOP, AUDIO_CMD_PCM } audio_cmd_type_t;

typedef struct {
    SemaphoreHandle_t done;
    esp_err_t result;
    int refs;
} audio_completion_t;

typedef struct {
    audio_cmd_type_t type;
    int16_t *samples;
    size_t sample_count;
    int16_t amplitude;
    audio_completion_t *completion;
} audio_cmd_t;

static const char *TAG = "audio";
static QueueHandle_t s_cmd_q;
static TaskHandle_t s_task;

static void audio_completion_release(audio_completion_t *completion)
{
    if (__atomic_sub_fetch(&completion->refs, 1, __ATOMIC_ACQ_REL) == 0) {
        vSemaphoreDelete(completion->done);
        free(completion);
    }
}

static void audio_reply(audio_cmd_t *cmd, esp_err_t result)
{
    if (cmd->samples) {
        heap_caps_free(cmd->samples);
        cmd->samples = NULL;
    }
    if (cmd->completion) {
        cmd->completion->result = result;
        xSemaphoreGive(cmd->completion->done);
        audio_completion_release(cmd->completion);
        cmd->completion = NULL;
    }
}

static size_t render_alarm_cycle(int16_t *buf, size_t cap, int16_t amplitude)
{
    size_t tone_n = (size_t)(AUDIO_SAMPLE_RATE * ALARM_TONE_MS / 1000);
    size_t gap_n = (size_t)(AUDIO_SAMPLE_RATE * ALARM_GAP_MS / 1000);
    size_t unit_n = tone_n + gap_n;
    size_t edge_n = (size_t)(AUDIO_SAMPLE_RATE * ALARM_EDGE_MS / 1000);
    if (edge_n > tone_n / 2) edge_n = tone_n / 2;
    for (int beep = 0; beep < ALARM_BEEPS; beep++) {
        int16_t *tone = buf + beep * unit_n;
        for (size_t i = 0; i < tone_n; i++) {
            float env = i < edge_n ? (float)i / edge_n :
                        i > tone_n - edge_n ? (float)(tone_n - i) / edge_n : 1.0f;
            tone[i] = (int16_t)(sinf(2.0f * 3.14159265f * ALARM_BEEP_HZ * i / AUDIO_SAMPLE_RATE)
                                * amplitude * env);
        }
        memset(tone + tone_n, 0, gap_n * sizeof(*buf));
    }
    size_t cycle_n = (size_t)(AUDIO_SAMPLE_RATE * ALARM_CYCLE_MS / 1000);
    if (cycle_n > cap) cycle_n = cap;
    size_t used = unit_n * ALARM_BEEPS;
    if (cycle_n > used) memset(buf + used, 0, (cycle_n - used) * sizeof(*buf));
    return cycle_n;
}

static void audio_task(void *arg)
{
    (void)arg;
    const size_t alarm_cap = (size_t)(AUDIO_SAMPLE_RATE * ALARM_CYCLE_MS / 1000);
    int16_t *alarm = heap_caps_malloc(alarm_cap * sizeof(*alarm), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    bool alarm_active = false;
    size_t alarm_n = 0;

    for (;;) {
        audio_cmd_t cmd;
        if (!alarm_active) {
            if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) != pdTRUE) continue;
            if (cmd.type == AUDIO_CMD_ALARM_START) {
                if (!alarm) {
                    ESP_LOGE(TAG, "no memory for alarm tone");
                    continue;
                }
                alarm_n = render_alarm_cycle(alarm, alarm_cap, cmd.amplitude);
                alarm_active = true;
                axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, true);
            } else if (cmd.type == AUDIO_CMD_PCM) {
                axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, true);
                esp_err_t result = ESP_OK;
                for (size_t off = 0; off < cmd.sample_count;) {
                    audio_cmd_t pending;
                    if (xQueueReceive(s_cmd_q, &pending, 0) == pdTRUE) {
                        if (pending.type == AUDIO_CMD_ALARM_START && alarm) {
                            alarm_n = render_alarm_cycle(alarm, alarm_cap, pending.amplitude);
                            alarm_active = true;
                            result = ESP_ERR_INVALID_STATE;
                            break;
                        }
                        audio_reply(&pending, pending.type == AUDIO_CMD_PCM ? ESP_ERR_INVALID_STATE : ESP_OK);
                        continue;
                    }
                    size_t n = cmd.sample_count - off;
                    if (n > AUDIO_CHUNK_SAMPLES) n = AUDIO_CHUNK_SAMPLES;
                    result = max98357a_write(cmd.samples + off, n);
                    if (result != ESP_OK) break;
                    off += n;
                }
                audio_reply(&cmd, result);
                if (!alarm_active) axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);
            }
            continue;
        }

        if (xQueueReceive(s_cmd_q, &cmd, 0) == pdTRUE) {
            if (cmd.type == AUDIO_CMD_ALARM_STOP) {
                alarm_active = false;
                axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);
            } else if (cmd.type == AUDIO_CMD_ALARM_START && alarm) {
                alarm_n = render_alarm_cycle(alarm, alarm_cap, cmd.amplitude);
            } else {
                audio_reply(&cmd, ESP_ERR_INVALID_STATE);
            }
            continue;
        }

        for (size_t off = 0; off < alarm_n && alarm_active; off += AUDIO_CHUNK_SAMPLES) {
            size_t n = alarm_n - off;
            if (n > AUDIO_CHUNK_SAMPLES) n = AUDIO_CHUNK_SAMPLES;
            esp_err_t err = max98357a_write(alarm + off, n);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "alarm playback: %s", esp_err_to_name(err));
                alarm_active = false;
                axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);
                break;
            }
            if (xQueueReceive(s_cmd_q, &cmd, 0) == pdTRUE) {
                if (cmd.type == AUDIO_CMD_ALARM_STOP) {
                    alarm_active = false;
                    axp2101_enable_rail(twatch_pmu_dev, AXP2101_BLDO2, false);
                } else if (cmd.type == AUDIO_CMD_ALARM_START && alarm) {
                    alarm_n = render_alarm_cycle(alarm, alarm_cap, cmd.amplitude);
                } else {
                    audio_reply(&cmd, ESP_ERR_INVALID_STATE);
                }
            }
        }
    }
}

esp_err_t audio_init(void)
{
    if (s_task) return ESP_OK;
    s_cmd_q = xQueueCreate(AUDIO_CMD_QUEUE_LEN, sizeof(audio_cmd_t));
    if (!s_cmd_q) return ESP_ERR_NO_MEM;
    if (xTaskCreate(audio_task, "audio", 4096, NULL, 5, &s_task) != pdPASS) {
        vQueueDelete(s_cmd_q);
        s_cmd_q = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t send_command(audio_cmd_t cmd, TickType_t timeout)
{
    if (!s_cmd_q) return ESP_ERR_INVALID_STATE;
    return xQueueSend(s_cmd_q, &cmd, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t audio_alarm_start(int16_t amplitude)
{
    return send_command((audio_cmd_t){ .type = AUDIO_CMD_ALARM_START, .amplitude = amplitude }, 0);
}

esp_err_t audio_alarm_stop(void)
{
    return send_command((audio_cmd_t){ .type = AUDIO_CMD_ALARM_STOP }, 0);
}

esp_err_t audio_play_pcm(const int16_t *samples, size_t sample_count)
{
    if (!samples || sample_count == 0) return ESP_ERR_INVALID_ARG;
    if (sample_count > SIZE_MAX / sizeof(*samples)) return ESP_ERR_INVALID_SIZE;

    int16_t *copy = heap_caps_malloc(sample_count * sizeof(*copy),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    audio_completion_t *completion = calloc(1, sizeof(*completion));
    if (!copy || !completion) {
        heap_caps_free(copy);
        free(completion);
        return ESP_ERR_NO_MEM;
    }
    completion->done = xSemaphoreCreateBinary();
    if (!completion->done) {
        heap_caps_free(copy);
        free(completion);
        return ESP_ERR_NO_MEM;
    }
    memcpy(copy, samples, sample_count * sizeof(*copy));
    completion->refs = 2; /* caller and the audio task each release one. */
    audio_cmd_t cmd = { .type = AUDIO_CMD_PCM, .samples = copy, .sample_count = sample_count,
                        .completion = completion };
    esp_err_t err = send_command(cmd, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        audio_completion_release(completion);
        audio_completion_release(completion);
        heap_caps_free(copy);
        return err;
    }
    TickType_t timeout = pdMS_TO_TICKS((sample_count * 1000 / AUDIO_SAMPLE_RATE) * 10 + 2000);
    if (xSemaphoreTake(completion->done, timeout) != pdTRUE) {
        audio_completion_release(completion);
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t result = completion->result;
    audio_completion_release(completion);
    return result;
}
