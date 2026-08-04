#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

void screenshot_init(void);

void screenshot_queue(lv_draw_buf_t *buf);

#ifdef __cplusplus
}
#endif
