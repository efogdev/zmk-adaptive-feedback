#pragma once

#include <stdint.h>
#include <stdbool.h>

#define ZAR_ANIM_SOLID   0
#define ZAR_ANIM_BLINK   1
#define ZAR_ANIM_BREATHE 2
#define ZAR_ANIM_FLASH   3

#define ZAR_EVTIDX_LAYER       0
#define ZAR_EVTIDX_BATT_WARN   1
#define ZAR_EVTIDX_BATT_CRIT   2
#define ZAR_EVTIDX_USB_CONN    3
#define ZAR_EVTIDX_USB_DISCONN 4
#define ZAR_EVTIDX_BLE_PROFILE  5
#define ZAR_EVTIDX_IDLE         6
#define ZAR_EVTIDX_NO_ENDPOINT  7

#define ZAR_INFO_MAX_COLORS 4

struct zar_rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct zar_event_info {
    struct zar_rgb colors[ZAR_INFO_MAX_COLORS];
    uint8_t color_count;
    uint8_t anim;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint16_t flash_dur_ms;
    uint16_t feedback_dur_ms;
    bool persistent;
};

int zar_on(void);
int zar_off(void);
bool zar_is_on(void);

uint8_t zar_layer_count(void);
uint8_t zar_batt_level_count(void);

int zar_event_get(uint8_t event_idx, uint8_t sub_idx, struct zar_event_info *out);
int zar_event_set_color(uint8_t event_idx, uint8_t sub_idx, struct zar_rgb color);
int zar_event_set_color_at(uint8_t event_idx, uint8_t sub_idx, uint8_t color_idx, struct zar_rgb color);
int zar_event_set_anim(uint8_t event_idx, uint8_t sub_idx, uint8_t anim);
int zar_event_set_blink(uint8_t event_idx, uint8_t sub_idx, uint16_t on_ms, uint16_t off_ms);
int zar_event_set_flash(uint8_t event_idx, uint8_t sub_idx, uint16_t dur_ms);
int zar_event_set_feedback(uint8_t event_idx, uint8_t sub_idx, uint16_t dur_ms);
