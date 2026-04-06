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
    uint16_t flash_ease_in_ms;
    uint8_t  flash_ease_in_fn;
    uint16_t flash_ease_out_ms;
    uint8_t  flash_ease_out_fn;
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
int zaf_event_set_flash_ease_in(uint8_t event_idx, uint8_t sub_idx, uint16_t ms, uint8_t fn);
int zaf_event_set_flash_ease_out(uint8_t event_idx, uint8_t sub_idx, uint16_t ms, uint8_t fn);
int zaf_event_set_feedback(uint8_t event_idx, uint8_t sub_idx, uint16_t dur_ms);
int zaf_clear_persisted(void);

/**
 * A custom event that external modules can register and trigger.
 * Declare a module-level instance with ZAF_CUSTOM_EVENT_DEFINE.
 * Configure it with zaf_custom_event_set_* and fire it with
 * zaf_custom_event_trigger.  Custom events are displayed after built-in
 * transient events (USB/BLE/studio) but before the persistent layer and
 * idle states.
 */
struct zaf_custom_event {
    const char *name;
    struct zaf_event_info info; /* color_count == 0 means not yet configured */
    bool pending;
    uint16_t ticks;
};

/**
 * Declare a custom event in the current translation unit.
 * @param sym   C symbol name for the event variable.
 * @param _name Human-readable event name (string literal).
 */
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
int zaf_custom_event_set_feedback(struct zaf_custom_event *evt, uint16_t dur_ms);
