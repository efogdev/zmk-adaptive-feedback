#pragma once

#include <stdint.h>
#include <stdbool.h>

#define ZAF_ANIM_SOLID   0
#define ZAF_ANIM_BLINK   1
#define ZAF_ANIM_BREATHE 2
#define ZAF_ANIM_FLASH   3

#define ZAF_EVTIDX_LAYER       0
#define ZAF_EVTIDX_BATT_WARN   1
#define ZAF_EVTIDX_BATT_CRIT   2
#define ZAF_EVTIDX_USB_CONN    3
#define ZAF_EVTIDX_USB_DISCONN 4
#define ZAF_EVTIDX_BLE_PROFILE  5
#define ZAF_EVTIDX_IDLE         6
#define ZAF_EVTIDX_NO_ENDPOINT  7

#define ZAF_INFO_MAX_COLORS 4
#define ZAF_BATT_LEVEL_COUNT 3

struct zaf_rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct zaf_event_info {
    struct zaf_rgb colors[ZAF_INFO_MAX_COLORS];
    uint8_t color_count;
    uint8_t anim;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint16_t flash_dur_ms;
    uint16_t feedback_dur_ms;
    bool persistent;
};

int zaf_on(void);
int zaf_off(void);
bool zaf_is_on(void);

uint8_t zaf_layer_count(void);

int zaf_event_get(uint8_t event_idx, uint8_t sub_idx, struct zaf_event_info *out);
int zaf_event_set_color(uint8_t event_idx, uint8_t sub_idx, struct zaf_rgb color);
int zaf_event_set_color_at(uint8_t event_idx, uint8_t sub_idx, uint8_t color_idx, struct zaf_rgb color);
int zaf_event_set_anim(uint8_t event_idx, uint8_t sub_idx, uint8_t anim);
int zaf_event_set_blink(uint8_t event_idx, uint8_t sub_idx, uint16_t on_ms, uint16_t off_ms);
int zaf_event_set_flash(uint8_t event_idx, uint8_t sub_idx, uint16_t dur_ms);
int zaf_event_set_feedback(uint8_t event_idx, uint8_t sub_idx, uint16_t dur_ms);
int zaf_clear_persisted(void);
