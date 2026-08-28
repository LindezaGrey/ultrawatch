/*
 * bhi_screen.c - the real BHI260AP sensor screen from main/lvgl_app.c (lines
 * ~691-912 as of the porting pass), copied verbatim (activity_name /
 * bhi_cube_update / bhi_screen_update / bhi_text_row / lvgl_build_bhi_screen,
 * unmodified widget layout and logic - including the pure quaternion math in
 * bhi_cube_update) and run against mock_hw.c instead of real drivers.
 *
 * Deviations from the firmware original (mechanical only):
 *   - screen_new() copied in here too, same as every non-watch-face screen.
 *   - No esp_lv_adapter_lock()/unlock(): none was present in this range of
 *     the original anyway (lvgl_show_bhi_screen(), which does lock, is out
 *     of scope here - navigation goes through nav.c instead).
 *
 * Keep this in sync with main/lvgl_app.c by hand: there's no build-time
 * link between the two.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lvgl.h"
#include "cascadia_fonts.h"
#include "mock_hw.h"
#include "screens.h"

static const lv_font_t *s_font_small = &cascadia_22;
static const lv_font_t *s_font_micro = &cascadia_18;

/* Not static: declared extern in screens.h. */
lv_obj_t *s_bhi_screen;

static lv_obj_t *s_bhi_status_label;
static lv_obj_t *s_bhi_rv_acc_label;
static lv_obj_t *s_bhi_activity_label;
static lv_obj_t *s_bhi_gesture_label;
static lv_obj_t *s_daily_act_label[DAILY_ACT_COUNT];
static lv_obj_t *s_cube_line[12];
static lv_point_precise_t s_cube_pts[12][2];
static lv_obj_t *s_axis_line[3];
static lv_point_precise_t s_axis_pts[3][2];

static lv_obj_t *screen_new(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    return scr;
}

static const char *activity_name(uint8_t activity)
{
    switch (activity) {
    case BHI260AP_ACTIVITY_STILL: return "still";
    case BHI260AP_ACTIVITY_WALKING: return "walking";
    case BHI260AP_ACTIVITY_RUNNING: return "running";
    case BHI260AP_ACTIVITY_ON_BICYCLE: return "cycling";
    case BHI260AP_ACTIVITY_IN_VEHICLE: return "in vehicle";
    case BHI260AP_ACTIVITY_TILTING: return "tilting";
    default: return "unknown";
    }
}

/* Project an 8-vertex unit cube rotated by the GAMERV quaternion (Q14 fixed
 * point) and move the 12 wireframe edge lines. Orthographic projection, so the
 * cube keeps constant size. */
static void bhi_cube_update(int16_t qx, int16_t qy, int16_t qz, int16_t qw)
{
    if (!s_cube_line[0]) {
        return;
    }
    /* Q14 -> float, renormalize. */
    double x = qx / 16384.0, y = qy / 16384.0, z = qz / 16384.0, w = qw / 16384.0;
    double n = sqrt(x * x + y * y + z * z + w * w);
    if (n < 1e-6) {
        x = 0; y = 0; z = 0; w = 1;
    } else {
        x /= n; y /= n; z /= n; w /= n;
    }

    /* Use the conjugate (negate the vector part) so the cube rotates WITH the
     * device instead of mirroring it: otherwise rotating the watch one way
     * appears to counter-rotate the cube and it looks like it holds still. */
    x = -x; y = -y; z = -z;

    /* Rotation matrix from the quaternion (column-vector convention). */
    double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
    double r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x);
    double r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);

    const double S = 42.0;   /* half cube size in px */
    const double A = 1.45;   /* axis length as a multiple of the half-size */
    const double cx = 300.0, cy = 135.0;   /* cube centre on screen */

    /* Unit cube corners (8). */
    const double corners[8][3] = {
        { -1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 },
        { -1, -1, 1 },  { 1, -1, 1 },  { 1, 1, 1 },  { -1, 1, 1 },
    };
    double px[8], py[8];
    for (int i = 0; i < 8; i++) {
        double vx = corners[i][0], vy = corners[i][1], vz = corners[i][2];
        px[i] = cx + S * (r00 * vx + r01 * vy + r02 * vz);
        py[i] = cy + S * (r10 * vx + r11 * vy + r12 * vz);
    }

    /* 12 cube edges. */
    const int edges[12][2] = {
        { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
        { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
        { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
    };
    for (int e = 0; e < 12; e++) {
        s_cube_pts[e][0].x = (lv_coord_t)px[edges[e][0]];
        s_cube_pts[e][0].y = (lv_coord_t)py[edges[e][0]];
        s_cube_pts[e][1].x = (lv_coord_t)px[edges[e][1]];
        s_cube_pts[e][1].y = (lv_coord_t)py[edges[e][1]];
        lv_line_set_points(s_cube_line[e], s_cube_pts[e], 2);
    }

    /* Origin cross: rotated unit axes from the cube centre. Rows of the
     * rotation matrix are the X/Y/Z axes in world x/y/z (orthographic: the
     * axes use world X=row0, Y=row1, Z=row2 -> screen x/y). */
    const double axes[3][3] = {
        { r00, r01, r02 },   /* X */
        { r10, r11, r12 },   /* Y */
        { r20, r21, r22 },   /* Z */
    };
    for (int a = 0; a < 3; a++) {
        s_axis_pts[a][0].x = (lv_coord_t)cx;
        s_axis_pts[a][0].y = (lv_coord_t)cy;
        s_axis_pts[a][1].x = (lv_coord_t)(cx + A * S * axes[a][0]);
        s_axis_pts[a][1].y = (lv_coord_t)(cy + A * S * axes[a][1]);
        lv_line_set_points(s_axis_line[a], s_axis_pts[a], 2);
    }
}

static void bhi_screen_update(lv_timer_t *timer)
{
    (void)timer;
    if (!s_bhi_status_label) {
        return;
    }
    /* Skip when the BHI screen is not active (avoids gesture consumption and
     * needless sensor reads while on another screen). */
    if (lv_screen_active() != s_bhi_screen) {
        return;
    }
    char buf[64];

    bool ready = false;
    bhi260ap_get_status(&ready, NULL);
    lv_label_set_text(s_bhi_status_label, ready ? "BHI260AP: ready" : "BHI260AP: not ready");

    int16_t rx = 0, ry = 0, rz = 0, rw = 0;
    uint16_t racc = 0;
    bhi260ap_get_rotation(&rx, &ry, &rz, &rw, &racc);
    bhi_cube_update(rx, ry, rz, rw);
    snprintf(buf, sizeof(buf), "RV acc: %u", (unsigned)racc);
    lv_label_set_text(s_bhi_rv_acc_label, buf);

    uint8_t activity = BHI260AP_ACTIVITY_UNKNOWN;
    bhi260ap_get_activity(&activity);
    snprintf(buf, sizeof(buf), "Activity: %s", activity_name(activity));
    lv_label_set_text(s_bhi_activity_label, buf);

    /* Per-activity minutes logged today (from daily_log). */
    const uint32_t *secs[DAILY_ACT_COUNT];
    static const char *act_names[DAILY_ACT_COUNT] = {
        "Still", "Walk", "Run", "Cycle", "Vehicle", "Tilt", "Other" };
    if (daily_log_get_activity_seconds(secs) == ESP_OK) {
        for (int i = 0; i < DAILY_ACT_COUNT; i++) {
            if (s_daily_act_label[i]) {
                uint32_t s = *secs[i];
                char dur[16];
                if (s >= 3600) {
                    snprintf(dur, sizeof(dur), "%uh %um", (unsigned)(s / 3600), (unsigned)((s % 3600) / 60));
                } else if (s >= 60) {
                    snprintf(dur, sizeof(dur), "%um %us", (unsigned)(s / 60), (unsigned)(s % 60));
                } else {
                    snprintf(dur, sizeof(dur), "%us", (unsigned)s);
                }
                snprintf(buf, sizeof(buf), "%s %s", act_names[i], dur);
                lv_label_set_text(s_daily_act_label[i], buf);
            }
        }
    }

    /* Keep the last gesture shown until a new one fires (otherwise the text
     * would clear on the next 1 s refresh). */
    bool tilt = false, wake = false, glance = false, pickup = false, tdet = false;
    if (bhi260ap_consume_gestures(&tilt, &wake, &glance, &pickup, &tdet) == ESP_OK) {
        snprintf(buf, sizeof(buf), "Gesture: %s%s%s%s%s",
                 tilt ? "wristtilt " : "", wake ? "wake " : "", glance ? "glance " : "",
                 pickup ? "pickup " : "", tdet ? "tiltdet " : "");
        if (strcmp(buf, "Gesture: ") != 0) {
            lv_label_set_text(s_bhi_gesture_label, buf);
        }
    }
}

static lv_obj_t *bhi_text_row(lv_obj_t *parent, const char *text, lv_obj_t **label)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_font(l, s_font_small, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(0xE0E0E0), 0);
    *label = l;
    return l;
}

static void lvgl_build_bhi_screen(void)
{
    s_bhi_screen = screen_new();
    lv_obj_set_style_bg_color(s_bhi_screen, lv_color_hex(0x002030), 0);

    lv_obj_t *title = lv_label_create(s_bhi_screen);
    lv_label_set_text(title, "BHI260AP");
    lv_obj_set_style_text_font(title, s_font_small, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    /* Status + rotation accuracy as text rows. */
    lv_obj_t *l;
    l = bhi_text_row(s_bhi_screen, "", &s_bhi_status_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 50);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_rv_acc_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 80);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_activity_label);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 110);

    l = bhi_text_row(s_bhi_screen, "", &s_bhi_gesture_label);
    lv_obj_set_style_text_color(l, lv_color_hex(0xFFD54D), 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 44, 140);

    /* 3D orientation wireframe cube from the GAMERV quaternion. */
    const lv_color_t cube_col = lv_color_hex(0x4FC3F7);
    for (int e = 0; e < 12; e++) {
        lv_obj_t *ln = lv_line_create(s_bhi_screen);
        lv_obj_set_style_line_color(ln, cube_col, 0);
        lv_obj_set_style_line_width(ln, 2, 0);
        s_cube_line[e] = ln;
    }

    /* Origin cross (x/y/z pointer) from the cube centre, RGB colors. */
    const lv_color_t axis_col[3] = {
        lv_color_hex(0xFF5252),   /* X red */
        lv_color_hex(0x00E676),   /* Y green */
        lv_color_hex(0x40C4FF),   /* Z blue */
    };
    for (int a = 0; a < 3; a++) {
        lv_obj_t *al = lv_line_create(s_bhi_screen);
        lv_obj_set_style_line_color(al, axis_col[a], 0);
        lv_obj_set_style_line_width(al, 3, 0);
        s_axis_line[a] = al;
    }

    /* Today's per-activity minutes, two compact columns to save height. */
    for (int i = 0; i < DAILY_ACT_COUNT; i++) {
        lv_obj_t *al = lv_label_create(s_bhi_screen);
        lv_label_set_text(al, "");
        lv_obj_set_style_text_font(al, s_font_micro, 0);
        lv_obj_set_style_text_color(al, lv_color_hex(0x9ECBE0), 0);
        int col = (i < 4) ? 0 : 1;
        int row = (i < 4) ? i : (i - 4);
        lv_obj_align(al, LV_ALIGN_TOP_LEFT, 44 + col * 190, 300 + row * 26);
        s_daily_act_label[i] = al;
    }

    lv_obj_t *hint = lv_label_create(s_bhi_screen);
    lv_label_set_text(hint, "swipe right to go back");
    lv_obj_set_style_text_font(hint, s_font_small, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -90);

    bhi_screen_update(NULL);
    lv_timer_create(bhi_screen_update, 200, NULL);   /* 5 Hz: smooth orientation cube */
}

void sim_bhi_screen_build(void)
{
    if (!s_bhi_screen) {
        lvgl_build_bhi_screen();
    }
    lv_scr_load(s_bhi_screen);
}
