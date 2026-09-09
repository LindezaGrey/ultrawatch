#include "screens.h"
#include "cascadia_fonts.h"

lv_obj_t *s_nfc_screen;
lv_obj_t *s_nfc_status_label;
lv_obj_t *s_nfc_uid_label;
lv_obj_t *s_nfc_start_btn;

void nfc_screen_create(lv_event_cb_t start_cb)
{
    s_nfc_screen = screen_new();
    lv_obj_set_style_bg_color(s_nfc_screen, lv_color_hex(0x201030), 0);

    /* High-DPI sizing (AGENT.md "Display density & UI sizing"): title and
     * the Start button promote to cascadia_36 (both short, fixed text
     * with room to spare). Status/UID text stay at their current sizes -
     * status can be a variable-length error string and the UID label
     * uses WRAP mode with unpredictable line count, both risk pushing
     * into the button below at a bigger font. */
    lv_obj_t *title = lv_label_create(s_nfc_screen);
    lv_label_set_text(title, "NFC");
    lv_obj_set_style_text_font(title, &cascadia_36, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    s_nfc_status_label = lv_label_create(s_nfc_screen);
    lv_label_set_text(s_nfc_status_label, "");
    lv_obj_set_style_text_font(s_nfc_status_label, &cascadia_22, 0);
    lv_obj_set_style_text_color(s_nfc_status_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(s_nfc_status_label, LV_ALIGN_CENTER, 0, -60);

    s_nfc_uid_label = lv_label_create(s_nfc_screen);
    lv_label_set_text(s_nfc_uid_label, "");
    lv_obj_set_style_text_font(s_nfc_uid_label, &cascadia_18, 0);
    lv_label_set_long_mode(s_nfc_uid_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_nfc_uid_label, 360);
    lv_obj_set_style_text_align(s_nfc_uid_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_nfc_uid_label, LV_ALIGN_CENTER, 0, -10);

    s_nfc_start_btn = lv_btn_create(s_nfc_screen);
    lv_obj_set_size(s_nfc_start_btn, 180, 64);
    lv_obj_align(s_nfc_start_btn, LV_ALIGN_CENTER, 0, 80);
    lv_obj_add_event_cb(s_nfc_start_btn, start_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_lbl = lv_label_create(s_nfc_start_btn);
    lv_label_set_text(btn_lbl, "Start");
    lv_obj_set_style_text_font(btn_lbl, &cascadia_36, 0);
    lv_obj_center(btn_lbl);

    lv_obj_t *hint = lv_label_create(s_nfc_screen);
    lv_label_set_text(hint, "Swipe right: Apps");
    lv_obj_set_style_text_font(hint, &cascadia_18, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -30);

}
