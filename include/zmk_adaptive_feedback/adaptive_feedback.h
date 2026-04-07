#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/iterable_sections.h>

#define ZAF_ANIM_SOLID   0
#define ZAF_ANIM_BLINK   1
#define ZAF_ANIM_BREATHE 2
#define ZAF_ANIM_FLASH   3

#define ZAF_EASE_LINEAR      0
#define ZAF_EASE_QUAD_IN     1
#define ZAF_EASE_QUAD_OUT    2
#define ZAF_EASE_QUAD_INOUT  3
#define ZAF_EASE_CUBIC_IN    4
#define ZAF_EASE_CUBIC_OUT   5
#define ZAF_EASE_CUBIC_INOUT 6
#define ZAF_EASE_QUART_IN    7
#define ZAF_EASE_QUART_OUT   8
#define ZAF_EASE_QUART_INOUT 9
#define ZAF_EASE_EXPO_IN     10
#define ZAF_EASE_EXPO_OUT    11
#define ZAF_EASE_BOUNCE_OUT  12
#define ZAF_EASE_BOUNCE_IN   13

#define ZAF_EVTIDX_LAYER          0
#define ZAF_EVTIDX_BATT_WARN      1
#define ZAF_EVTIDX_BATT_CRIT      2
#define ZAF_EVTIDX_USB_CONN       3
#define ZAF_EVTIDX_USB_DISCONN    4
#define ZAF_EVTIDX_BLE_PROFILE    5
#define ZAF_EVTIDX_IDLE           6
#define ZAF_EVTIDX_NO_ENDPOINT    7
#define ZAF_EVTIDX_STUDIO_UNLOCK  8
#define ZAF_EVTIDX_STUDIO_LOCK    9
#define ZAF_EVTIDX_CUSTOM         10
#define ZAF_EVTIDX_ERROR          11

#ifndef ZAF_INFO_MAX_COLORS
#define ZAF_INFO_MAX_COLORS CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS
#endif

#ifndef ZAF_BATT_LEVEL_COUNT
#define ZAF_BATT_LEVEL_COUNT CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT
#endif

#ifndef ZAF_MAX_ERROR_SLOTS
#define ZAF_MAX_ERROR_SLOTS CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_ERROR_SLOTS
#endif

#ifndef ZAF_FEEDBACK_PATTERN_MAX_LEN
#define ZAF_FEEDBACK_PATTERN_MAX_LEN CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN
#endif

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
    uint16_t flash_ease_in_ms;
    uint8_t  flash_ease_in_fn;
    uint16_t flash_ease_out_ms;
    uint8_t  flash_ease_out_fn;
    uint16_t feedback_pattern[ZAF_FEEDBACK_PATTERN_MAX_LEN];
    uint8_t  feedback_pattern_len;
    uint16_t breathe_dur_ms;
    bool persistent;
    const char *label;
};

int zaf_on(void);
int zaf_off(void);
bool zaf_is_on(void);
void zaf_set_rgb_not_supported(void);

uint8_t zaf_layer_count(void);

int zaf_event_get(uint8_t event_idx, uint8_t sub_idx, struct zaf_event_info *out);
int zaf_event_set_color(uint8_t event_idx, uint8_t sub_idx, struct zaf_rgb color);
int zaf_event_set_color_at(uint8_t event_idx, uint8_t sub_idx, uint8_t color_idx, struct zaf_rgb color);
int zaf_event_set_anim(uint8_t event_idx, uint8_t sub_idx, uint8_t anim);
int zaf_event_set_blink(uint8_t event_idx, uint8_t sub_idx, uint16_t on_ms, uint16_t off_ms);
int zaf_event_set_flash(uint8_t event_idx, uint8_t sub_idx, uint16_t dur_ms);
int zaf_event_set_flash_ease_in(uint8_t event_idx, uint8_t sub_idx, uint16_t ms, uint8_t fn);
int zaf_event_set_flash_ease_out(uint8_t event_idx, uint8_t sub_idx, uint16_t ms, uint8_t fn);
int zaf_event_set_feedback(uint8_t event_idx, uint8_t sub_idx, const uint16_t *pattern, uint8_t len);
int zaf_event_set_breathe(uint8_t event_idx, uint8_t sub_idx, uint16_t dur_ms);
int zaf_clear_persisted(void);

struct zaf_custom_event {
    const char *name;
    struct zaf_event_info info;
    bool pending;
    uint16_t ticks;
};

#define ZAF_CUSTOM_EVENT_DEFINE(sym, _name) \
    STRUCT_SECTION_ITERABLE(zaf_custom_event, sym) = { .name = (_name) }

int zaf_custom_event_trigger(struct zaf_custom_event *evt);
int zaf_custom_event_get(const struct zaf_custom_event *evt, struct zaf_event_info *out);
int zaf_custom_event_set_color(struct zaf_custom_event *evt, struct zaf_rgb color);
int zaf_custom_event_set_color_at(struct zaf_custom_event *evt, uint8_t color_idx,
                                  struct zaf_rgb color);
int zaf_custom_event_set_anim(struct zaf_custom_event *evt, uint8_t anim);
int zaf_custom_event_set_blink(struct zaf_custom_event *evt, uint16_t on_ms, uint16_t off_ms);
int zaf_custom_event_set_flash(struct zaf_custom_event *evt, uint16_t dur_ms);
int zaf_custom_event_set_flash_ease_in(struct zaf_custom_event *evt, uint16_t ms, uint8_t fn);
int zaf_custom_event_set_flash_ease_out(struct zaf_custom_event *evt, uint16_t ms, uint8_t fn);
int zaf_custom_event_set_feedback(struct zaf_custom_event *evt, const uint16_t *pattern, uint8_t len);
int zaf_custom_event_set_breathe(struct zaf_custom_event *evt, uint16_t dur_ms);

uint8_t zaf_error_slots_count(void);
int zaf_error_get(uint8_t slot_idx, struct zaf_event_info *out);
int zaf_error_set_color(uint8_t slot_idx, struct zaf_rgb color);
int zaf_error_set_color_at(uint8_t slot_idx, uint8_t color_idx, struct zaf_rgb color);
int zaf_error_set_anim(uint8_t slot_idx, uint8_t anim);
int zaf_error_set_blink(uint8_t slot_idx, uint16_t on_ms, uint16_t off_ms);
int zaf_error_set_flash(uint8_t slot_idx, uint16_t dur_ms);
int zaf_error_set_flash_ease_in(uint8_t slot_idx, uint16_t ms, uint8_t fn);
int zaf_error_set_flash_ease_out(uint8_t slot_idx, uint16_t ms, uint8_t fn);
int zaf_error_set_feedback(uint8_t slot_idx, const uint16_t *pattern, uint8_t len);
int zaf_error_set_breathe(uint8_t slot_idx, uint16_t dur_ms);
int zaf_error_trigger(uint8_t slot_idx);
int zaf_error_clear(uint8_t slot_idx);
int zaf_error_clear_all(void);
