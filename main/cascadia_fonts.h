/*
 * cascadia_fonts.h - Embedded Cascadia Code bitmap fonts (LVGL fmt_txt).
 *
 * Generated with lv_font_conv 1.5.3 from CascadiaCode.ttf (SIL Open Font
 * License 1.1) - the original assets/fonts/CascadiaCode.ttf this project
 * generated the first four sizes from isn't in the repo/on every machine;
 * cascadia_88 was regenerated from the system-installed copy
 * (/usr/share/fonts/TTF/CascadiaCode.ttf, ttf-cascadia-code package) with
 * the same options (--bpp 4 --format lvgl --range 0x20-0x7E --no-compress
 * --no-kerning), just --size 88 --lv-font-name cascadia_88 - see the
 * .c file's own header comment for the exact command. 100px was tried
 * first and actually overflowed the 410px panel width with "HH:MM:SS" at
 * the watch face's -6 letter-spacing - 88px is the largest size that
 * still fits with a small margin either side ("nearly spans the whole
 * screen", per the request that prompted this). Pre-rendered sizes
 * replace the runtime FreeType font, removing the SPIFFS font dependency.
 * Range 0x20-0x7E.
 */
#ifndef CASCADIA_FONTS_H
#define CASCADIA_FONTS_H

#include "lvgl.h"

extern const lv_font_t cascadia_88;   /* watch face HH:MM:SS - the hero element */
extern const lv_font_t cascadia_72;   /* (no longer used by the watch face; kept for any future use) */
extern const lv_font_t cascadia_36;   /* UTC / alarm time */
extern const lv_font_t cascadia_22;   /* body text */
extern const lv_font_t cascadia_18;   /* GPS diag line */

#endif
