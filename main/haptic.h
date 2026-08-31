/*
 * haptic.h - global vibration pattern selection (Settings > Ton & Vibration).
 * One NVS-persisted DRV2605 library waveform id, shared by every haptic
 * trigger in the app (alarm/timer ring, LoRa message notification) - see
 * docs/application.md and drv2605.h. The gating around the actual I2C call
 * (enabling XL9555's HAPTIC_EN/M_EN line before playback, disabling after)
 * stays with each caller, since alarm.c already owns a longer-lived
 * enable/disable around its whole ring cycle - haptic_play_test() is the
 * one exception, self-contained for the Settings screen's tap-to-preview.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *name;
    uint8_t wave_id;   /* DRV2605 waveform library effect id (datasheet 11.2) */
} haptic_pattern_t;

extern const haptic_pattern_t HAPTIC_PATTERNS[];
#define HAPTIC_PATTERN_COUNT 6

/* Call once at boot: loads the persisted pattern index from NVS (default 0,
 * "Strong Click", if nothing stored yet). */
void haptic_init(void);

size_t haptic_get_pattern_index(void);
/* Out-of-range idx is ignored. Persists to NVS ("haptic"/"pattern"). */
void haptic_set_pattern_index(size_t idx);

/* The currently-selected pattern's DRV2605 waveform id - what alarm.c's
 * ring_vibrate() and mesh_log.c's notification tap should actually play. */
uint8_t haptic_get_wave_id(void);

/* Self-contained one-shot preview: enables HAPTIC_EN, plays `wave_id`,
 * waits for it to finish, disables HAPTIC_EN again - same sequence as the
 * debug "motor" command. Ignores the persisted selection so the Settings
 * screen can preview any pattern in the list, not just the current one. */
void haptic_play_test(uint8_t wave_id);

/* Same as haptic_play_test(), but fires on a short-lived background task
 * instead of blocking the caller for the ~420ms enable/play/disable
 * sequence - use this from any LVGL event callback (they run on the LVGL
 * task under esp_lv_adapter_lock(), so blocking there stalls the whole
 * UI, not just the caller). */
void haptic_play_test_async(uint8_t wave_id);

#ifdef __cplusplus
}
#endif
