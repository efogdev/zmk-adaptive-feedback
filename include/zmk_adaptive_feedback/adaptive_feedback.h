#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/iterable_sections.h>

/*
 * ZAF_ANIM_TABLE — animation type table
 * Columns: (TOKEN, cli_string, constant_value)
 * Row order MUST match the "animation" enum in dts/bindings/zmk,adaptive-feedback.yaml.
 * Do NOT reorder rows without updating the YAML file simultaneously.
 */
#define ZAF_ANIM_TABLE(_) \
    _(SOLID,   "solid",   0) \
    _(BLINK,   "blink",   1) \
    _(BREATHE, "breathe", 2) \
    _(FLASH,   "flash",   3)

#define ZAF_ANIM_SOLID   0
#define ZAF_ANIM_BLINK   1
#define ZAF_ANIM_BREATHE 2
#define ZAF_ANIM_FLASH   3

#define _ZAF_ANIM_CHECK(tok, str, val) \
    _Static_assert(ZAF_ANIM_##tok == (val), \
        "ZAF_ANIM_" #tok " constant does not match ZAF_ANIM_TABLE");
ZAF_ANIM_TABLE(_ZAF_ANIM_CHECK)
#undef _ZAF_ANIM_CHECK

/*
 * ZAF_EASE_TABLE — easing function table
 * Columns: (TOKEN, cli_and_yaml_string, constant_value)
 * Row order MUST match flash-ease-in-fn / flash-ease-out-fn enum in YAML.
 * Do NOT reorder rows without updating the YAML file simultaneously.
 */
#define ZAF_EASE_TABLE(_) \
    _(LINEAR,      "linear",       0)  \
    _(QUAD_IN,     "quad-in",      1)  \
    _(QUAD_OUT,    "quad-out",     2)  \
    _(QUAD_INOUT,  "quad-in-out",  3)  \
    _(CUBIC_IN,    "cubic-in",     4)  \
    _(CUBIC_OUT,   "cubic-out",    5)  \
    _(CUBIC_INOUT, "cubic-in-out", 6)  \
    _(QUART_IN,    "quart-in",     7)  \
    _(QUART_OUT,   "quart-out",    8)  \
    _(QUART_INOUT, "quart-in-out", 9)  \
    _(EXPO_IN,     "expo-in",      10) \
    _(EXPO_OUT,    "expo-out",     11) \
    _(BOUNCE_OUT,  "bounce-out",   12) \
    _(BOUNCE_IN,   "bounce-in",    13)

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

#define _ZAF_EASE_CHECK(tok, str, val) \
    _Static_assert(ZAF_EASE_##tok == (val), \
        "ZAF_EASE_" #tok " constant does not match ZAF_EASE_TABLE");
ZAF_EASE_TABLE(_ZAF_EASE_CHECK)
#undef _ZAF_EASE_CHECK

/*
 * ZAF_EVT_TABLE — built-in event type table
 * Columns: (TOKEN, string, constant_value)
 * string is used both in device tree overlays (event-type enum) and as the CLI token
 * accepted by "argb evt <string>".
 * Row order MUST match the "event-type" enum in dts/bindings/zmk,adaptive-feedback.yaml.
 * Do NOT reorder rows without updating the YAML file simultaneously.
 */
#define ZAF_EVT_TABLE(_) \
    _(LAYER,         "layer",        0)  \
    _(BATT_WARN,     "batt-warn",    1)  \
    _(BATT_CRIT,     "batt-crit",    2)  \
    _(USB_CONN,      "usb-conn",     3)  \
    _(USB_DISCONN,   "usb-disconn",  4)  \
    _(BLE_PROFILE,   "ble-profile",  5)  \
    _(IDLE,          "idle",         6)  \
    _(NO_ENDPOINT,   "no-endpoint",  7)  \
    _(STUDIO_UNLOCK, "studio-unlock",8)  \
    _(STUDIO_LOCK,   "studio-lock",  9)  \
    _(CUSTOM,        "custom",       10) \
    _(ERROR,         "error",        11)

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

#define _ZAF_EVT_CHECK(tok, str, val) \
    _Static_assert(ZAF_EVTIDX_##tok == (val), \
        "ZAF_EVTIDX_" #tok " constant does not match ZAF_EVT_TABLE");
ZAF_EVT_TABLE(_ZAF_EVT_CHECK)
#undef _ZAF_EVT_CHECK

/*
 * ZAF_SIMPLE_EVT_TABLE — subset of ZAF_EVT_TABLE for events that take no sub-index argument.
 * Parameterized events (layer, batt-warn, batt-crit, ble-profile, error, custom) are handled
 * explicitly in parse_event_spec() and are NOT listed here.
 */
#define ZAF_SIMPLE_EVT_TABLE(_) \
    _(IDLE,          "idle",          6)  \
    _(USB_CONN,      "usb-conn",      3)  \
    _(USB_DISCONN,   "usb-disconn",   4)  \
    _(NO_ENDPOINT,   "no-endpoint",   7)  \
    _(STUDIO_UNLOCK, "studio-unlock", 8)  \
    _(STUDIO_LOCK,   "studio-lock",   9)

struct zaf_rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct __attribute__((__packed__)) zaf_event_info {
    struct zaf_rgb colors[CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS];
    uint8_t color_count;
    uint8_t animation;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint16_t flash_duration_ms;
    uint16_t flash_ease_in_ms;
    uint8_t  flash_ease_in_fn;
    uint16_t flash_ease_out_ms;
    uint8_t  flash_ease_out_fn;
    uint16_t breathe_duration_ms;
    uint16_t feedback_pattern[CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN];
    uint8_t  feedback_pattern_len;
};

enum zaf_field {
    ZAF_FIELD_COLOR,         /* .color */
    ZAF_FIELD_COLOR_AT,      /* .color_at */
    ZAF_FIELD_ANIM,          /* .anim */
    ZAF_FIELD_BLINK,         /* .blink */
    ZAF_FIELD_FLASH,         /* .flash_dur_ms */
    ZAF_FIELD_FLASH_EASE_IN, /* .ease */
    ZAF_FIELD_FLASH_EASE_OUT,/* .ease */
    ZAF_FIELD_FEEDBACK,      /* .feedback */
    ZAF_FIELD_BREATHE,       /* .breathe_dur_ms */
};

union zaf_field_value {
    struct zaf_rgb color;
    struct { uint8_t idx; struct zaf_rgb color; } color_at;
    uint8_t anim;
    struct { uint16_t on_ms; uint16_t off_ms; } blink;
    uint16_t flash_dur_ms;
    struct { uint16_t ms; uint8_t fn; } ease;
    struct { const uint16_t *pattern; uint8_t len; } feedback;
    uint16_t breathe_dur_ms;
};

int zaf_on(void);
int zaf_off(void);
bool zaf_is_on(void);
uint8_t zaf_layer_count(void);
void zaf_set_rgb_not_supported(void);
bool zaf_event_is_headless(uint8_t event_idx, uint8_t sub_idx);
bool zaf_event_is_persistent(uint8_t event_idx, uint8_t sub_idx);
const char *zaf_event_get_label(uint8_t event_idx, uint8_t sub_idx);

int zaf_event_get(uint8_t event_idx, uint8_t sub_idx, struct zaf_event_info *out);
int zaf_event_set(uint8_t event_idx, uint8_t sub_idx, enum zaf_field field, union zaf_field_value val);
int zaf_clear_persisted(void);

struct zaf_custom_event {
    struct zaf_event_info info;
    uint16_t ticks;
    bool pending;
    const char *name;
    bool headless;
};

#define ZAF_CUSTOM_EVENT_DEFINE(sym, _name) \
    STRUCT_SECTION_ITERABLE(zaf_custom_event, sym) = { .name = (_name) }

int zaf_custom_event_trigger(struct zaf_custom_event *evt);
int zaf_custom_event_get(const struct zaf_custom_event *evt, struct zaf_event_info *out);
bool zaf_custom_event_is_persistent(const struct zaf_custom_event *evt);
const char *zaf_custom_event_get_label(const struct zaf_custom_event *evt);
int zaf_custom_event_set(struct zaf_custom_event *evt, enum zaf_field field, union zaf_field_value val);

uint8_t zaf_error_slots_count(void);
int zaf_error_get(uint8_t slot_idx, struct zaf_event_info *out);
int zaf_error_set(uint8_t slot_idx, enum zaf_field field, union zaf_field_value val);
int zaf_error_trigger(uint8_t slot_idx);
int zaf_error_clear(uint8_t slot_idx);
int zaf_error_clear_all(void);
