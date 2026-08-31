/*
 * status_icons.h - two small glyphs for the status bar's RF row (see
 * status_bar.c): a satellite icon for GNSS and a mesh-network node icon
 * for LoRa/Meshtastic. Plain LVGL image data, no ESP-IDF dependency, so
 * this compiles unchanged in both the firmware and the sim.
 *
 * Generated (see status_icons.c) from user-provided source art -
 * assets/ui/satellite.png and assets/ui/network.png, both transparent-
 * background black line icons, not the official Meshtastic logo. Swap in
 * different source art later by re-running the same conversion against a
 * new PNG; every call site just references these symbols by name.
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
