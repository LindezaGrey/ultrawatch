#include "screenshot.h"

#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lvgl.h"

static const char *TAG = "screenshot";

#define SHOT_MAGIC "TWSHOT01"
#define SHOT_END   "TWSHOTEND"

#define SHOT_HEX_CHUNK_BYTES 1024

/* Write a chunk of the screenshot dump. With the console VFS in blocking
 * driver mode (see app_main.c) a full USB TX buffer makes the write wait for
 * the host to catch up instead of silently dropping bytes, so the dump is
 * lossless as long as the host keeps reading. Retry on EAGAIN just in case. */
static void write_all(const char *s, size_t n)
{
    while (n > 0) {
        ssize_t w = write(STDOUT_FILENO, s, n);
        if (w > 0) {
            s += w;
            n -= (size_t)w;
            continue;
        }
        if (w < 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        return;
    }
}

static SemaphoreHandle_t s_shot_sem;
static lv_draw_buf_t *s_shot_buf;

static void hex_encode(char *out, const uint8_t *in, size_t n)
{
    static const char hexd[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) {
        out[2 * i] = hexd[in[i] >> 4];
        out[2 * i + 1] = hexd[in[i] & 0x0F];
    }
    out[2 * n] = '\0';
}

static void shot_task(void *arg)
{
    char hexbuf[SHOT_HEX_CHUNK_BYTES * 2 + 2];

    for (;;) {
        if (xSemaphoreTake(s_shot_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        lv_draw_buf_t *buf = s_shot_buf;
        if (buf == NULL) {
            continue;
        }

        /* Suspend the console REPL task while dumping: its "esp> " prompt
         * interleaves with the hex output and corrupts the stream. */
        TaskHandle_t repl = xTaskGetHandle("console_repl");
        if (repl != NULL) {
            vTaskSuspend(repl);
        }

        ESP_LOGI(TAG, "dump start %ux%u stride=%u", buf->header.w, buf->header.h, buf->header.stride);
        esp_log_level_set("*", ESP_LOG_NONE);

        printf(SHOT_MAGIC " %u %u %u\n",
               (unsigned)buf->header.w, (unsigned)buf->header.h, (unsigned)buf->header.stride);

        const uint8_t *d = buf->data;
        size_t remaining = buf->header.stride * buf->header.h;
        while (remaining > 0) {
            size_t chunk = remaining > SHOT_HEX_CHUNK_BYTES ? SHOT_HEX_CHUNK_BYTES : remaining;
            hex_encode(hexbuf, d, chunk);
            write_all(hexbuf, chunk * 2);
            write_all("\n", 1);
            d += chunk;
            remaining -= chunk;
            /* The USB-Serial-JTAG TX is the bottleneck and this task mostly
             * spins in non-blocking writes, starving the idle task on this
             * core. The idle task is what feeds the task WDT; without a
             * yield a 5 s dump trips it and its backtrace lands in the hex
             * stream, corrupting the capture. */
            vTaskDelay(1);
        }

        write_all(SHOT_END "\n", strlen(SHOT_END) + 1);
        esp_log_level_set("*", ESP_LOG_DEBUG);
        ESP_LOGI(TAG, "dump done");

        if (repl != NULL) {
            vTaskResume(repl);
        }
        lv_draw_buf_destroy(buf);
        s_shot_buf = NULL;
    }
}

void screenshot_queue(lv_draw_buf_t *buf)
{
    if (s_shot_buf != NULL) {
        ESP_LOGW(TAG, "previous capture not dumped yet");
        lv_draw_buf_destroy(buf);
        return;
    }

    s_shot_buf = buf;
    xSemaphoreGive(s_shot_sem);
}

void screenshot_init(void)
{
    s_shot_sem = xSemaphoreCreateBinary();
    if (s_shot_sem == NULL) {
        ESP_LOGE(TAG, "semaphore alloc failed");
        return;
    }

    TaskHandle_t task = NULL;
    if (xTaskCreate(shot_task, "shot", 16384, NULL, 5, &task) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
    }
}
