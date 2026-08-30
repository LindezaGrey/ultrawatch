/*
 * status_icons.h - two small hand-drawn placeholder glyphs for the status
 * bar's RF row (see status_bar.c): a satellite icon for GNSS and a 3-node
 * mesh-network icon for LoRa/Meshtastic. Plain LVGL image data, no ESP-IDF
 * dependency, so this compiles unchanged in both the firmware and the sim.
 *
 * These are NOT the real Meshtastic logo - deliberately generic placeholder
 * art (per explicit request) to sidestep the licensing question a real
 * trademarked logo would raise. Swap in real assets later by replacing the
 * two lv_image_dsc_t below; every call site just references these symbols
 * by name.
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_image_dsc_t status_icon_satellite;
extern const lv_image_dsc_t status_icon_mesh;

#ifdef __cplusplus
}
#endif
