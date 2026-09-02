#include "display_controller.h"
#include "co5300.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl_app.h"

static const char *TAG = "display_ctrl";

typedef enum {
    DISPLAY_CMD_INIT,
    DISPLAY_CMD_VISIBLE,
    DISPLAY_CMD_DRAW_COMPLETE,
    DISPLAY_CMD_BRIGHTNESS,
    DISPLAY_CMD_SLEEP,
} display_cmd_type_t;

typedef struct {
    display_cmd_type_t type;
    TaskHandle_t reply;
    SemaphoreHandle_t done;
    esp_err_t *result;
    uint8_t brightness;
} display_cmd_t;

static QueueHandle_t s_queue;
static volatile display_state_t s_state = DISPLAY_STATE_INITIALIZING;
static bool s_lvgl_attached;
static uint8_t s_brightness = 0x80;

static void display_reply(TaskHandle_t reply, esp_err_t err)
{
    if (reply) {
        xTaskNotify(reply, (uint32_t)err, eSetValueWithOverwrite);
    }
}

static void display_complete(const display_cmd_t *cmd, esp_err_t err)
{
    if (cmd->result) {
        *cmd->result = err;
    }
    if (cmd->done) {
        xSemaphoreGive(cmd->done);
    }
}

static void display_controller_task(void *arg)
{
    (void)arg;
    display_cmd_t cmd;
    for (;;) {
        xQueueReceive(s_queue, &cmd, portMAX_DELAY);
        if (cmd.type == DISPLAY_CMD_INIT) {
            esp_err_t err = co5300_init(true);
            if (err == ESP_OK) {
                err = co5300_blank();
            }
            s_state = (err == ESP_OK) ? DISPLAY_STATE_READY : DISPLAY_STATE_FAULT;
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "panel init: %s", esp_err_to_name(err));
            }
            display_reply(cmd.reply, err);
            continue;
        }

        if (cmd.type == DISPLAY_CMD_VISIBLE) {
            if ((s_state == DISPLAY_STATE_READY || s_state == DISPLAY_STATE_SLEEPING) && s_lvgl_attached) {
                esp_err_t err = ESP_OK;
                if (s_state == DISPLAY_STATE_READY) {
                    if (esp_lv_adapter_lock(1000) != ESP_OK) {
                        ESP_LOGW(TAG, "panel show: LVGL lock timeout");
                        continue;
                    }
                    err = co5300_set_brightness(s_brightness);
                    esp_lv_adapter_unlock();
                } else {
                    if (esp_lv_adapter_lock(1000) != ESP_OK) {
                        ESP_LOGW(TAG, "panel wake: LVGL lock timeout");
                        continue;
                    }
                    err = co5300_reinit(true, true);
                    if (err == ESP_OK) {
                        err = co5300_blank();
                    }
                    if (err == ESP_OK) {
                        err = co5300_set_brightness(s_brightness);
                    }
                    esp_lv_adapter_unlock();
                }
                if (err != ESP_OK) {
                    s_state = DISPLAY_STATE_FAULT;
                    ESP_LOGE(TAG, "panel wake: %s", esp_err_to_name(err));
                    continue;
                }
                s_state = DISPLAY_STATE_AWAITING_DRAW;
                lvgl_force_redraw();
            }
            continue;
        }

        if (cmd.type == DISPLAY_CMD_DRAW_COMPLETE && s_state == DISPLAY_STATE_AWAITING_DRAW) {
            if (esp_lv_adapter_lock(1000) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));
                xQueueSend(s_queue, &cmd, 0);
                continue;
            }
            esp_err_t err = co5300_display_on();
            esp_lv_adapter_unlock();
            if (err == ESP_OK) {
                s_state = DISPLAY_STATE_VISIBLE;
            } else {
                s_state = DISPLAY_STATE_FAULT;
                ESP_LOGE(TAG, "display on: %s", esp_err_to_name(err));
            }
            continue;
        }

        if (cmd.type == DISPLAY_CMD_BRIGHTNESS) {
            s_brightness = cmd.brightness;
            if (s_state == DISPLAY_STATE_VISIBLE) {
                if (esp_lv_adapter_lock(1000) == ESP_OK) {
                    esp_err_t err = co5300_set_brightness(s_brightness);
                    esp_lv_adapter_unlock();
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "brightness: %s", esp_err_to_name(err));
                    }
                }
            }
            continue;
        }

        if (cmd.type == DISPLAY_CMD_SLEEP) {
            esp_err_t err = ESP_OK;
            if (s_state == DISPLAY_STATE_VISIBLE) {
                if (esp_lv_adapter_lock(1000) != ESP_OK) {
                    err = ESP_ERR_TIMEOUT;
                } else {
                    err = co5300_display_off();
                    if (err == ESP_OK) err = co5300_blank();
                    if (err == ESP_OK) err = co5300_sleep();
                    esp_lv_adapter_unlock();
                }
                if (err == ESP_OK) {
                    s_state = DISPLAY_STATE_SLEEPING;
                }
            }
            display_complete(&cmd, err);
        }
    }
}

esp_err_t display_controller_init(void)
{
    if (s_queue) {
        return s_state == DISPLAY_STATE_READY ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    s_queue = xQueueCreate(8, sizeof(display_cmd_t));
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(display_controller_task, "display_ctrl", 4096, NULL, 5, NULL) != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    display_cmd_t cmd = {
        .type = DISPLAY_CMD_INIT,
        .reply = xTaskGetCurrentTaskHandle(),
    };
    if (xQueueSend(s_queue, &cmd, portMAX_DELAY) != pdPASS) {
        return ESP_FAIL;
    }
    uint32_t result = ESP_FAIL;
    if (xTaskNotifyWait(0, UINT32_MAX, &result, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return (esp_err_t)result;
}

void display_controller_attach_lvgl(void)
{
    s_lvgl_attached = true;
}

void display_controller_request_visible(display_reason_t reason)
{
    (void)reason;
    if (!s_queue) {
        return;
    }
    const display_cmd_t cmd = { .type = DISPLAY_CMD_VISIBLE };
    xQueueSend(s_queue, &cmd, 0);
}

void display_controller_note_draw_complete(void)
{
    if (!s_queue) {
        return;
    }
    const display_cmd_t cmd = { .type = DISPLAY_CMD_DRAW_COMPLETE };
    xQueueSend(s_queue, &cmd, 0);
}

void display_controller_set_brightness(uint8_t level)
{
    if (!s_queue) {
        return;
    }
    const display_cmd_t cmd = {
        .type = DISPLAY_CMD_BRIGHTNESS,
        .brightness = level,
    };
    xQueueSend(s_queue, &cmd, 0);
}

esp_err_t display_controller_sleep(TickType_t timeout)
{
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    if (!done) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t result = ESP_FAIL;
    const display_cmd_t cmd = {
        .type = DISPLAY_CMD_SLEEP,
        .done = done,
        .result = &result,
    };
    if (xQueueSend(s_queue, &cmd, timeout) != pdPASS) {
        vSemaphoreDelete(done);
        return ESP_ERR_TIMEOUT;
    }
    xSemaphoreTake(done, portMAX_DELAY);
    vSemaphoreDelete(done);
    return result;
}

display_state_t display_controller_get_state(void)
{
    return s_state;
}
