#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>

#include <drivers/ext_power.h>

#include <zmk/event_manager.h>
#include <zmk/workqueue.h>
#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/ble.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>

#if IS_ENABLED(CONFIG_ZMK_STUDIO)
#include <zmk/studio/core.h>
#endif

#include <zmk_adaptive_feedback/adaptive_feedback.h>

#include "zmk/keymap.h"

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
#include <zmk_runtime_config/runtime_config.h>
#else
#define ZRC_GET(key, default_val) (default_val)
#endif

#define DT_DRV_COMPAT zmk_adaptive_feedback

LOG_MODULE_REGISTER(zmk_adaptive_feedback, CONFIG_ZMK_LOG_LEVEL);

static inline uint16_t zaf_ticks_from_ms(const uint32_t ms) {
    const int32_t tick = ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS);
    return (tick > 0) ? (uint16_t)(ms / (uint32_t)tick) : 1;
}

struct zaf_device_config {
    const struct device *led_strip;
    const struct device *ext_power;
    uint8_t chain_length;
    uint8_t error_slots;
    uint16_t error_section_size;
    bool error_slots_reverse;
    bool feedback_enabled;
    uint16_t feedback_delay_ms;
    const struct gpio_dt_spec *feedback_gpio;
    const struct gpio_dt_spec *feedback_extra_gpio;
};

struct zaf_evt_override {
    struct zaf_event_info cfg;
    bool valid;
    bool cleared;
};

struct zaf_evt_state {
    struct zaf_evt_override rt_layers[ZMK_KEYMAP_LAYERS_LEN];
    struct zaf_evt_override rt_idle;
    struct zaf_evt_override rt_usb_conn;
    struct zaf_evt_override rt_usb_disconn;
    struct zaf_evt_override rt_ble_profile[CONFIG_BT_MAX_PAIRED];
    struct zaf_evt_override rt_no_endpoint;
    struct zaf_evt_override rt_batt_warn[CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT];
    struct zaf_evt_override rt_batt_crit[CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT];
    struct zaf_evt_override rt_studio_unlock;
    struct zaf_evt_override rt_studio_lock;
    struct zaf_evt_override rt_error_slots[CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_ERROR_SLOTS];
};

struct __attribute__((__packed__)) zaf_runtime_state {
    bool on;
    bool override_active;
    bool feedback_active;
    struct zaf_event_info override_cfg;
    struct zaf_evt_state evts;
    uint8_t active_layer;
    uint8_t batt_level;
    uint8_t prev_batt_level;
    uint16_t batt_warn_ticks[CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT];
    uint16_t batt_crit_ticks[CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT];
    uint16_t usb_event_ticks;
    bool usb_connected;
    bool ble_connected;
    bool no_endpoint;
    uint16_t ble_event_ticks[CONFIG_BT_MAX_PAIRED];
    bool studio_unlocked;
    uint16_t studio_event_ticks;
    bool idle;
    uint16_t anim_step;
    uint8_t color_idx;
    bool blink_phase;
    uint16_t blink_phase_ticks;
    bool flash_active;
    uint16_t flash_ticks;
    uint16_t flash_total_ticks;
    uint16_t flash_ease_in_ticks;
    uint16_t flash_ease_out_ticks;
    uint8_t  flash_ease_in_fn;
    uint8_t  flash_ease_out_fn;
};

struct zaf_persisted {
    bool on;
    bool override_active;
    struct zaf_event_info override_cfg;
};


struct __attribute__((__packed__)) zaf_dt_child {
    uint8_t event_type_idx;
    uint8_t layer_index;
    uint8_t batt_level;
    uint8_t ble_profile_index;
    uint8_t error_slot_index;
    uint8_t color_bytes[CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS * 3];
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
    bool persistent;
    bool headless;
    const char *custom_name;
    const char *label;
};

#define ZAF_CHILD_ENTRY(child)                                                              \
    {                                                                                       \
        .event_type_idx    = DT_ENUM_IDX(child, event_type),                               \
        .custom_name       = COND_CODE_1(DT_NODE_HAS_PROP(child, custom_event_name),       \
                                         (DT_PROP(child, custom_event_name)), (NULL)),      \
        .label             = COND_CODE_1(DT_NODE_HAS_PROP(child, label),                   \
                                         (DT_PROP(child, label)), (NULL)),                  \
        .layer_index       = DT_PROP_OR(child, layer_index, 0),                            \
        .batt_level        = DT_PROP_OR(child, battery_level, 1),                          \
        .ble_profile_index = DT_PROP_OR(child, ble_profile_index, 0),                      \
        .error_slot_index  = DT_PROP_OR(child, error_slot_index, 0),                       \
        .color_bytes       = DT_PROP(child, colors),                                       \
        .color_count       = DT_PROP_LEN(child, colors) / 3,                               \
        .animation         = DT_ENUM_IDX_OR(child, animation, 0),                          \
        .persistent        = DT_PROP(child, persistent),                                   \
        .blink_on_ms       = DT_PROP_OR(child, blink_on_ms, 0),                            \
        .blink_off_ms      = DT_PROP_OR(child, blink_off_ms, 0),                           \
        .breathe_duration_ms = DT_PROP_OR(child, breathe_duration_ms, 0),                  \
        .flash_duration_ms = DT_PROP_OR(child, flash_duration_ms, 0),                      \
        .flash_ease_in_ms  = DT_PROP_OR(child, flash_ease_in_ms, 0),                       \
        .flash_ease_in_fn  = DT_ENUM_IDX_OR(child, flash_ease_in_fn, 0),                   \
        .flash_ease_out_ms = DT_PROP_OR(child, flash_ease_out_ms, 0),                      \
        .flash_ease_out_fn = DT_ENUM_IDX_OR(child, flash_ease_out_fn, 0),                  \
        .feedback_pattern     = COND_CODE_1(DT_NODE_HAS_PROP(child, feedback_pattern),      \
                                    (DT_PROP(child, feedback_pattern)), ({})),             \
        .feedback_pattern_len = COND_CODE_1(DT_NODE_HAS_PROP(child, feedback_pattern),    \
                                    (MIN(DT_PROP_LEN(child, feedback_pattern),             \
                                         CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN)), (0)), \
        .headless             = DT_PROP(child, headless),                                  \
    },

#define _ZAF_COUNT_CHILD(child) +1
#define ZAF_DT_CHILD_COUNT (0 DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(0), _ZAF_COUNT_CHILD))

static const struct zaf_dt_child zaf_dt_children[ZAF_DT_CHILD_COUNT] = {
    DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(0), ZAF_CHILD_ENTRY)
};

#if DT_INST_NODE_HAS_PROP(0, feedback_gpios)
static const struct gpio_dt_spec zaf_feedback_gpio_spec = GPIO_DT_SPEC_INST_GET(0, feedback_gpios);
#endif

#if DT_INST_NODE_HAS_PROP(0, feedback_extra_gpios)
static const struct gpio_dt_spec zaf_feedback_extra_gpio_spec = GPIO_DT_SPEC_INST_GET(0, feedback_extra_gpios);
#endif

static struct zaf_device_config zaf_config;
static struct zaf_runtime_state zaf_state;
static struct led_rgb zaf_pixels[DT_INST_PROP(0, chain_length)];

static struct k_work_delayable zaf_save_work;
static struct k_work_delayable zaf_save_evt_work;
static struct k_work_delayable zaf_save_custom_work;

static bool zaf_prev_extra_gpio_state;
static uint16_t zaf_feedback_pattern[CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN];
static uint8_t  zaf_feedback_pattern_len;
static uint8_t  zaf_feedback_pattern_idx;
static struct k_work_delayable zaf_feedback_step_work;

static bool rgb_supported = true;
void zaf_set_rgb_not_supported() {
    rgb_supported = false;
}

static void zaf_fill(const struct zaf_rgb color) {
    const int32_t bri = ZRC_GET("argb/brt", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BRIGHTNESS);
    const struct led_rgb c = {
        .r = (uint8_t)((color.r * bri) / 100),
        .g = (uint8_t)((color.g * bri) / 100),
        .b = (uint8_t)((color.b * bri) / 100),
    };
    for (int i = 0; i < zaf_config.chain_length; i++) {
        zaf_pixels[i] = c;
    }
}

static void zaf_clear_pixels(void) {
    const struct zaf_rgb off = {0, 0, 0};
    zaf_fill(off);
    if (!zaf_state.feedback_active && zaf_config.ext_power != NULL) {
        ext_power_disable(zaf_config.ext_power);
    }
}

static struct zaf_evt_override *zaf_evt_rt_slot(uint8_t event_idx, uint8_t sub_idx);
static const struct zaf_event_info *zaf_evt_eff_cfg(uint8_t event_idx, uint8_t sub_idx);
static const struct zaf_event_info *zaf_error_eff_cfg(uint8_t slot_idx);
static void zaf_apply_error_sections(void);
static void zaf_copy_animation_fields(struct zaf_event_info *dst, const struct zaf_event_info *src);
static void zaf_copy_feedback_pattern(struct zaf_event_info *dst, const struct zaf_event_info *src);
static void zaf_copy_colors(struct zaf_event_info *dst, const struct zaf_event_info *src);

static const struct zaf_event_info *zaf_resolve(void) {
    if (zaf_state.override_active) {
        return &zaf_state.override_cfg;
    }

    for (int i = CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT - 1; i >= 0; i--) {
        if (zaf_state.batt_crit_ticks[i] != 0xFFFF) {
            return zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_CRIT, (uint8_t)i);
        }
    }
    for (int i = CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT - 1; i >= 0; i--) {
        if (zaf_state.batt_warn_ticks[i] != 0xFFFF) {
            return zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_WARN, (uint8_t)i);
        }
    }

    if (zaf_state.usb_event_ticks != 0xFFFF) {
        return zaf_evt_eff_cfg(zaf_state.usb_connected
                               ? ZAF_EVTIDX_USB_CONN : ZAF_EVTIDX_USB_DISCONN, 0);
    }

    for (int i = CONFIG_BT_MAX_PAIRED - 1; i >= 0; i--) {
        if (zaf_state.ble_event_ticks[i] != 0xFFFF) {
            return zaf_evt_eff_cfg(ZAF_EVTIDX_BLE_PROFILE, (uint8_t)i);
        }
    }

    if (zaf_state.studio_event_ticks != 0xFFFF) {
        return zaf_evt_eff_cfg(zaf_state.studio_unlocked
                               ? ZAF_EVTIDX_STUDIO_UNLOCK : ZAF_EVTIDX_STUDIO_LOCK, 0);
    }

    if (zaf_state.no_endpoint) {
        return zaf_evt_eff_cfg(ZAF_EVTIDX_NO_ENDPOINT, 0);
    }

    STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
        if (cevt->pending && cevt->info.color_count > 0) {
            static struct zaf_event_info zaf_custom_resolve_buf;
            const struct zaf_event_info *ci = &cevt->info;
            zaf_custom_resolve_buf = (struct zaf_event_info){
                .color_count          = MIN(ci->color_count, CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS),
                .animation            = ci->animation,
                .feedback_pattern_len = ci->feedback_pattern_len,
            };
            zaf_copy_animation_fields(&zaf_custom_resolve_buf, ci);
            zaf_copy_colors(&zaf_custom_resolve_buf, ci);
            zaf_copy_feedback_pattern(&zaf_custom_resolve_buf, ci);
            return &zaf_custom_resolve_buf;
        }
    }

    const uint8_t layer = zaf_state.active_layer;
    if (layer < ZMK_KEYMAP_LAYERS_LEN) {
        const struct zaf_event_info *lcfg = zaf_evt_eff_cfg(ZAF_EVTIDX_LAYER, layer);
        if (lcfg != NULL && zaf_event_is_persistent(ZAF_EVTIDX_LAYER, layer)) {
            return lcfg;
        }
    }

    if (zaf_state.idle) {
        return zaf_evt_eff_cfg(ZAF_EVTIDX_IDLE, 0);
    }

    return NULL;
}

#define _ZAF_EASE_COUNT(tok, str, val) +1
_Static_assert((0 ZAF_EASE_TABLE(_ZAF_EASE_COUNT)) == 14,
    "ZAF_EASE_TABLE changed — add the new entry to the zaf_ease() switch");
#undef _ZAF_EASE_COUNT

#define _ZAF_ANIM_COUNT(tok, str, val) +1
_Static_assert((0 ZAF_ANIM_TABLE(_ZAF_ANIM_COUNT)) == 4,
    "ZAF_ANIM_TABLE changed — update animation dispatch in adaptive_feedback.c");
#undef _ZAF_ANIM_COUNT

static uint8_t zaf_ease(const uint8_t fn, const uint8_t t) {
    switch (fn) {
    default:
    case ZAF_EASE_LINEAR:
        return t;

    case ZAF_EASE_QUAD_IN:
        return (uint8_t)((uint16_t)t * t >> 8);

    case ZAF_EASE_QUAD_OUT: {
        const uint8_t u = 255 - t;
        return (uint8_t)(255 - ((uint16_t)u * u >> 8));
    }

    case ZAF_EASE_QUAD_INOUT:
        if (t < 128) {
            const uint8_t t2 = (uint8_t)(t << 1);
            return (uint8_t)((uint16_t)t2 * t2 >> 9);
        } else {
            const uint8_t u = (uint8_t)((255 - t) << 1);
            return (uint8_t)(255 - ((uint16_t)u * u >> 9));
        }

    case ZAF_EASE_CUBIC_IN: {
        const uint8_t t2 = (uint8_t)((uint16_t)t * t >> 8);
        return (uint8_t)((uint16_t)t2 * t >> 8);
    }

    case ZAF_EASE_CUBIC_OUT: {
        const uint8_t u  = 255 - t;
        const uint8_t u2 = (uint8_t)((uint16_t)u * u >> 8);
        return (uint8_t)(255 - ((uint16_t)u2 * u >> 8));
    }

    case ZAF_EASE_CUBIC_INOUT: {
        if (t < 128) {
            const uint8_t t2   = (uint8_t)(t << 1);
            const uint8_t t2sq = (uint8_t)((uint16_t)t2 * t2 >> 8);
            return (uint8_t)((uint16_t)t2sq * t2 >> 9);
        } else {
            const uint8_t u    = (uint8_t)((255 - t) << 1);
            const uint8_t usq  = (uint8_t)((uint16_t)u * u >> 8);
            return (uint8_t)(255 - ((uint16_t)usq * u >> 9));
        }
    }

    case ZAF_EASE_QUART_IN: {
        const uint8_t t2 = (uint8_t)((uint16_t)t * t >> 8);
        return (uint8_t)((uint16_t)t2 * t2 >> 8);
    }

    case ZAF_EASE_QUART_OUT: {
        const uint8_t u  = 255 - t;
        const uint8_t u2 = (uint8_t)((uint16_t)u * u >> 8);
        return (uint8_t)(255 - ((uint16_t)u2 * u2 >> 8));
    }

    case ZAF_EASE_QUART_INOUT: {
        if (t < 128) {
            const uint8_t t2   = (uint8_t)(t << 1);
            const uint8_t t2sq = (uint8_t)((uint16_t)t2 * t2 >> 8);
            return (uint8_t)((uint16_t)t2sq * t2sq >> 9);
        } else {
            const uint8_t u    = (uint8_t)((255 - t) << 1);
            const uint8_t usq  = (uint8_t)((uint16_t)u * u >> 8);
            return (uint8_t)(255 - ((uint16_t)usq * usq >> 9));
        }
    }

    case ZAF_EASE_EXPO_IN: {
        const uint8_t t2 = (uint8_t)((uint16_t)t * t >> 8);
        const uint8_t t3 = (uint8_t)((uint16_t)t2 * t >> 8);
        const uint8_t t4 = (uint8_t)((uint16_t)t3 * t >> 8);
        return (uint8_t)((uint16_t)t4 * t >> 8);
    }

    case ZAF_EASE_EXPO_OUT: {
        const uint8_t u  = 255 - t;
        const uint8_t u2 = (uint8_t)((uint16_t)u * u >> 8);
        const uint8_t u3 = (uint8_t)((uint16_t)u2 * u >> 8);
        const uint8_t u4 = (uint8_t)((uint16_t)u3 * u >> 8);
        return (uint8_t)(255 - ((uint16_t)u4 * u >> 8));
    }

    case ZAF_EASE_BOUNCE_OUT:
        if (t < 93) {
            return (uint8_t)((uint32_t)t * t * 15u / 510u);
        } else if (t < 185) {
            const int16_t d = (int16_t)t - 139;
            return (uint8_t)(191u + (uint32_t)((int32_t)d * d) * 15u / 510u);
        } else if (t < 232) {
            const int16_t d = (int16_t)t - 209;
            return (uint8_t)(239u + (uint32_t)((int32_t)d * d) * 15u / 510u);
        } else {
            const int16_t d = (int16_t)t - 244;
            return (uint8_t)(251u + (uint32_t)((int32_t)d * d) * 15u / 510u);
        }

    case ZAF_EASE_BOUNCE_IN:
        return (uint8_t)(255u - zaf_ease(ZAF_EASE_BOUNCE_OUT, (uint8_t)(255u - t)));
    }
}

static void zaf_copy_animation_fields(struct zaf_event_info *dst, const struct zaf_event_info *src) {
    dst->blink_on_ms = src->blink_on_ms;
    dst->blink_off_ms = src->blink_off_ms;
    dst->flash_duration_ms = src->flash_duration_ms;
    dst->flash_ease_in_ms = src->flash_ease_in_ms;
    dst->flash_ease_in_fn = src->flash_ease_in_fn;
    dst->flash_ease_out_ms = src->flash_ease_out_ms;
    dst->flash_ease_out_fn = src->flash_ease_out_fn;
    dst->breathe_duration_ms = src->breathe_duration_ms;
}

static void zaf_copy_feedback_pattern(struct zaf_event_info *dst, const struct zaf_event_info *src) {
    const uint8_t len = MIN(src->feedback_pattern_len, CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN);
    dst->feedback_pattern_len = len;
    for (uint8_t i = 0; i < len; i++) {
        dst->feedback_pattern[i] = src->feedback_pattern[i];
    }
}

static void zaf_copy_colors(struct zaf_event_info *dst, const struct zaf_event_info *src) {
    dst->color_count = MIN(src->color_count, CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS);
    for (uint8_t i = 0; i < dst->color_count; i++) {
        dst->colors[i] = src->colors[i];
    }
}

static void zaf_activate_flash_state(const uint16_t total_ticks,
                                      const uint16_t ease_in_ms, const uint8_t ease_in_fn,
                                      const uint16_t ease_out_ms, const uint8_t ease_out_fn) {
    zaf_state.flash_active        = true;
    zaf_state.flash_ticks         = total_ticks;
    zaf_state.flash_total_ticks   = total_ticks;
    zaf_state.flash_ease_in_ticks  = zaf_ticks_from_ms(ease_in_ms);
    zaf_state.flash_ease_out_ticks = zaf_ticks_from_ms(ease_out_ms);
    zaf_state.flash_ease_in_fn    = ease_in_fn;
    zaf_state.flash_ease_out_fn   = ease_out_fn;
}

static void __noinline zaf_fill_section(const uint16_t start, const uint16_t end,
                                        const struct led_rgb color) {
    for (uint16_t i = start; i < end && i < zaf_config.chain_length; i++) {
        zaf_pixels[i] = color;
    }
}

static struct zaf_rgb __noinline zaf_breathe_color(const struct zaf_event_info *cfg) {
    const uint32_t dur = cfg->breathe_duration_ms;
    const int32_t tick_ms = ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS);
    const uint16_t breathe_steps = (tick_ms > 0) ? (uint16_t)(dur / (uint32_t)tick_ms) : 60;
    const int step       = (int)zaf_state.anim_step - (int)(breathe_steps / 2);
    const int abs_step   = (step < 0) ? -step : step;
    const int half_steps = (int)(breathe_steps / 2);
    const uint8_t scale  = (half_steps > 0) ? (uint8_t)((100 * (half_steps - abs_step)) / half_steps) : 100;
    const uint8_t idx    = zaf_state.color_idx % cfg->color_count;
    const struct zaf_rgb base = cfg->colors[idx];
    return (struct zaf_rgb){
        .r = (uint8_t)((base.r * scale) / 100),
        .g = (uint8_t)((base.g * scale) / 100),
        .b = (uint8_t)((base.b * scale) / 100),
    };
}

static uint8_t __noinline zaf_compute_flash_scale(
    const uint16_t ease_in_ticks,  const uint8_t ease_in_fn,
    const uint16_t ease_out_ticks, const uint8_t ease_out_fn)
{
    if (!zaf_state.flash_active) {
        return 0;
    }
    const uint16_t elapsed = zaf_state.flash_total_ticks - zaf_state.flash_ticks;
    if (ease_in_ticks > 0 && elapsed < ease_in_ticks) {
        const uint8_t t = (uint8_t)((uint32_t)elapsed * 255u / ease_in_ticks);
        return zaf_ease(ease_in_fn, t);
    }
    if (ease_out_ticks > 0 && zaf_state.flash_ticks <= ease_out_ticks) {
        const uint16_t out = ease_out_ticks - zaf_state.flash_ticks;
        const uint8_t t = (uint8_t)((uint32_t)out * 255u / ease_out_ticks);
        return (uint8_t)(255u - zaf_ease(ease_out_fn, t));
    }
    return 255;
}

static void zaf_apply_animation(const struct zaf_event_info *cfg) {
    if (cfg->color_count == 0) {
        zaf_clear_pixels();
        return;
    }

    const struct zaf_rgb c0 = cfg->colors[0];
    switch (cfg->animation) {
    case ZAF_ANIM_SOLID:
        zaf_fill(c0);
        break;

    case ZAF_ANIM_BLINK: {
        const uint16_t phase_ms = zaf_state.blink_phase ? cfg->blink_on_ms : cfg->blink_off_ms;
        const uint16_t phase_ticks = zaf_ticks_from_ms(phase_ms);
        const uint8_t idx = zaf_state.color_idx % cfg->color_count;
        const struct zaf_rgb c_cur = cfg->colors[idx];
        const struct zaf_rgb off = {0, 0, 0};
        zaf_fill(zaf_state.blink_phase ? c_cur : off);
        zaf_state.blink_phase_ticks++;
        if (zaf_state.blink_phase_ticks >= phase_ticks) {
            zaf_state.blink_phase = !zaf_state.blink_phase;
            zaf_state.blink_phase_ticks = 0;
            if (zaf_state.blink_phase) {
                zaf_state.color_idx = (zaf_state.color_idx + 1) % cfg->color_count;
            }
        }
        break;
    }

    case ZAF_ANIM_BREATHE: {
        const uint32_t dur = cfg->breathe_duration_ms;
        const int32_t tick_ms = ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS);
        const uint16_t breathe_steps = (tick_ms > 0) ? (uint16_t)(dur / (uint32_t)tick_ms) : 60;
        const struct zaf_rgb bc = zaf_breathe_color(cfg);
        zaf_fill(bc);
        zaf_state.anim_step++;
        if (zaf_state.anim_step >= breathe_steps) {
            zaf_state.anim_step = 0;
            zaf_state.color_idx = (zaf_state.color_idx + 1) % cfg->color_count;
        }
        break;
    }

    case ZAF_ANIM_FLASH: {
        const uint8_t scale = zaf_compute_flash_scale(
            zaf_state.flash_ease_in_ticks,  zaf_state.flash_ease_in_fn,
            zaf_state.flash_ease_out_ticks, zaf_state.flash_ease_out_fn);
        if (scale == 0) {
            zaf_clear_pixels();
            break;
        }
        const struct zaf_rgb scaled = {
            .r = (uint8_t)(((uint16_t)c0.r * scale) >> 8),
            .g = (uint8_t)(((uint16_t)c0.g * scale) >> 8),
            .b = (uint8_t)(((uint16_t)c0.b * scale) >> 8),
        };
        zaf_fill(scaled);
        break;
    }
    default: break;
    }
}

static void zaf_feedback_step(void);

static void zaf_feedback_step_work_fn(struct k_work *work) {
    zaf_feedback_step();
}

static void zaf_feedback_step(void) {
    while (true) {
        const uint8_t idx = zaf_feedback_pattern_idx;
        if (idx >= zaf_feedback_pattern_len) {
            if (zaf_config.feedback_gpio != NULL) {
                gpio_pin_set_dt(zaf_config.feedback_gpio, 0);
            }
            if (zaf_config.feedback_extra_gpio != NULL) {
                gpio_pin_set_dt(zaf_config.feedback_extra_gpio, zaf_prev_extra_gpio_state);
            }
            zaf_state.feedback_active = false;
            return;
        }

        zaf_state.feedback_active = true;
        gpio_pin_set_dt(zaf_config.feedback_gpio, idx % 2 == 0);
        zaf_feedback_pattern_idx++;
        const uint16_t dur = zaf_feedback_pattern[idx];
        if (dur > 0) {
            k_work_reschedule(&zaf_feedback_step_work, K_MSEC(dur));
            return;
        }
    }
}

static void zaf_trigger_feedback(const uint16_t *pattern, const uint8_t len) {
    if (!zaf_config.feedback_enabled || zaf_config.feedback_gpio == NULL || len == 0) {
        return;
    }
    k_work_cancel_delayable(&zaf_feedback_step_work);
    if (zaf_state.on && zaf_config.ext_power != NULL) {
        ext_power_enable(zaf_config.ext_power);
    }
    if (zaf_state.feedback_active && zaf_config.feedback_gpio != NULL) {
        gpio_pin_set_dt(zaf_config.feedback_gpio, 0);
    }
    const uint8_t copy_len = MIN(len, CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN);
    for (uint8_t i = 0; i < copy_len; i++) {
        zaf_feedback_pattern[i] = pattern[i];
    }
    zaf_feedback_pattern_len = copy_len;
    zaf_feedback_pattern_idx = 0;
    zaf_state.feedback_active = true;
    if (zaf_config.feedback_extra_gpio != NULL) {
        zaf_prev_extra_gpio_state = gpio_pin_get_dt(zaf_config.feedback_extra_gpio);
        gpio_pin_set_dt(zaf_config.feedback_extra_gpio, 1);
    }
    if (zaf_config.feedback_delay_ms > 0) {
        k_work_reschedule(&zaf_feedback_step_work, K_MSEC(zaf_config.feedback_delay_ms));
    } else {
        zaf_feedback_step();
    }
}

extern struct k_timer zaf_timer;

static void zaf_tick_reset_blink(void) {
    zaf_state.blink_phase = true;
    zaf_state.blink_phase_ticks = 0;
}

static bool __noinline zaf_tick_decrement(uint16_t *ticks) {
    if (*ticks != 0xFFFF && *ticks > 0 && --(*ticks) == 0) {
        *ticks = 0xFFFF;
        return true;
    }
    return false;
}

static void zaf_tick(struct k_work *work) {
    if (!zaf_state.on) {
        return;
    }

    if (zaf_tick_decrement(&zaf_state.usb_event_ticks)) {
        zaf_tick_reset_blink();
    }

    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        if (zaf_tick_decrement(&zaf_state.ble_event_ticks[i])) {
            zaf_tick_reset_blink();
        }
    }

    if (zaf_tick_decrement(&zaf_state.studio_event_ticks)) {
        zaf_tick_reset_blink();
    }

    STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
        if (cevt->pending && cevt->ticks > 0) {
            if (--cevt->ticks == 0) {
                cevt->pending = false;
                zaf_tick_reset_blink();
            }
        }
    }

    for (int i = 0; i < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; i++) {
        if (zaf_tick_decrement(&zaf_state.batt_warn_ticks[i])) {
            zaf_tick_reset_blink();
        }
        if (zaf_tick_decrement(&zaf_state.batt_crit_ticks[i])) {
            zaf_tick_reset_blink();
        }
    }

    if (zaf_state.flash_active && zaf_state.flash_ticks > 0) {
        if (--zaf_state.flash_ticks == 0) {
            zaf_state.flash_active = false;
        }
    }

    const struct zaf_event_info *cfg = zaf_resolve();
    bool output_is_static_dark = false;

    if (cfg == NULL) {
        bool any_error_active = false;
        for (uint8_t s = 0; s < zaf_config.error_slots; s++) {
            const struct zaf_event_info *ecfg = zaf_error_eff_cfg(s);
            if (ecfg != NULL && ecfg->color_count > 0) {
                any_error_active = true;
                break;
            }
        }
        if (!any_error_active) {
            zaf_clear_pixels();
            output_is_static_dark = true;
        } else {
            zaf_apply_error_sections();
        }
    } else {
        zaf_apply_animation(cfg);
        zaf_apply_error_sections();
        bool all_zero = true;
        for (int i = 0; i < zaf_config.chain_length; i++) {
            if (zaf_pixels[i].r || zaf_pixels[i].g || zaf_pixels[i].b) {
                all_zero = false;
                break;
            }
        }
        if (all_zero) {
            if (!zaf_state.feedback_active) {
                ext_power_disable(zaf_config.ext_power);
            }
            if (cfg->animation == ZAF_ANIM_SOLID) {
                output_is_static_dark = true;
            }
        } else {
            ext_power_enable(zaf_config.ext_power);
        }
    }

    if (rgb_supported) {
        led_strip_update_rgb(zaf_config.led_strip, zaf_pixels, zaf_config.chain_length);
    }

    if (output_is_static_dark) {
        bool error_anim_needs_timer = false;
        for (uint8_t s = 0; s < zaf_config.error_slots; s++) {
            const struct zaf_event_info *ecfg = zaf_error_eff_cfg(s);
            if (ecfg && ecfg->color_count > 0 &&
                ecfg->animation != ZAF_ANIM_SOLID) {
                error_anim_needs_timer = true;
                break;
            }
        }
        if (!error_anim_needs_timer && !zaf_state.feedback_active) {
            k_timer_stop(&zaf_timer);
        }
    }
}

K_WORK_DEFINE(zaf_tick_work, zaf_tick);

static void zaf_timer_handler(struct k_timer *timer) {
    if (!zaf_state.on) {
        return;
    }
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &zaf_tick_work);
}

K_TIMER_DEFINE(zaf_timer, zaf_timer_handler, NULL);

static void zaf_off_work_fn(struct k_work *work) {
    zaf_clear_pixels();
    led_strip_update_rgb(zaf_config.led_strip, zaf_pixels, zaf_config.chain_length);
}

K_WORK_DEFINE(zaf_off_work, zaf_off_work_fn);

static void zaf_kick_timer(void) {
    if (!zaf_state.on) {
        return;
    }
    const uint32_t tick_ms = (uint32_t)ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS);
    k_timer_start(&zaf_timer, K_NO_WAIT, K_MSEC(tick_ms));
}

static void zaf_save_work_fn(struct k_work *work) {
    const struct zaf_persisted p = {
        .on              = zaf_state.on,
        .override_active = zaf_state.override_active,
        .override_cfg    = zaf_state.override_cfg,
    };
    settings_save_one("argb/state", &p, sizeof(p));
}

static void zaf_save_evt_work_fn(struct k_work *work) {
    settings_save_one("argb/evtcfg", &zaf_state.evts, sizeof(zaf_state.evts));
}

static void zaf_save_custom_work_fn(struct k_work *work) {
    char key[32];
    STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
        snprintf(key, sizeof(key), "argb/ce/%s", cevt->name);
        if (cevt->info.color_count > 0 || cevt->info.feedback_pattern_len > 0) {
            settings_save_one(key, &cevt->info, sizeof(cevt->info));
        } else {
            settings_delete(key);
        }
    }
}

static int zaf_save_custom_cfg(void) {
    const int ret = k_work_reschedule(&zaf_save_custom_work,
                                      K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
}

static int zaf_settings_set(const char *name, const size_t len, const settings_read_cb read_cb, void *cb_arg) {
    const char *next;

    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(struct zaf_persisted)) {
            return -EINVAL;
        }
        struct zaf_persisted p;
        const int rc = read_cb(cb_arg, &p, sizeof(p));
        if (rc < 0) {
            return rc;
        }
        zaf_state.on              = p.on;
        zaf_state.override_active = p.override_active;
        zaf_state.override_cfg    = p.override_cfg;
        return 0;
    }

    static bool loaded_evtcfg = false;
    if (settings_name_steq(name, "evtcfg", &next) && !next && !loaded_evtcfg) {
        if (len != sizeof(struct zaf_evt_state)) {
            LOG_WRN("config size mismatch (%zu vs %zu), ignoring",
                    len, sizeof(struct zaf_evt_state));
            return 0;
        }
        const int rc = read_cb(cb_arg, &zaf_state.evts, sizeof(zaf_state.evts));
        if (rc < 0) {
            return rc;
        }
        loaded_evtcfg = true;
        return 0;
    }

    if (settings_name_steq(name, "ce", &next) && next) {
        if (len != sizeof(struct zaf_event_info)) {
            LOG_WRN("ce/%s size mismatch (%zu vs %zu), ignoring",
                    next, len, sizeof(struct zaf_event_info));
            return 0;
        }
        STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
            if (strcmp(next, cevt->name) == 0) {
                struct zaf_event_info info;
                const int rc = read_cb(cb_arg, &info, sizeof(info));
                if (rc < 0) {
                    return rc;
                }
                cevt->info = info;
                return 0;
            }
        }
        return -ENOENT;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(zaf_settings, "argb", NULL, zaf_settings_set, NULL, NULL);

static int zaf_save_state(void) {
    const int ret = k_work_reschedule(&zaf_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
}

static int zaf_save_evt_cfg(void) {
    const int ret = k_work_reschedule(&zaf_save_evt_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
}

static bool zaf_dt_child_matches(const struct zaf_dt_child *c,
                                  const uint8_t event_idx, const uint8_t sub_idx) {
    if (c->event_type_idx != event_idx) { return false; }
    switch (event_idx) {
    case ZAF_EVTIDX_LAYER:       return c->layer_index == sub_idx;
    case ZAF_EVTIDX_BATT_WARN:   return (c->batt_level - 1) == sub_idx;
    case ZAF_EVTIDX_BATT_CRIT:   return (c->batt_level - 1) == sub_idx;
    case ZAF_EVTIDX_BLE_PROFILE: return c->ble_profile_index == sub_idx;
    case ZAF_EVTIDX_ERROR:       return c->error_slot_index == sub_idx;
    default:                     return true;
    }
}

static const struct zaf_dt_child *zaf_dt_child_find(const uint8_t event_idx, const uint8_t sub_idx) {
    for (uint8_t i = 0; i < ZAF_DT_CHILD_COUNT; i++) {
        if (zaf_dt_child_matches(&zaf_dt_children[i], event_idx, sub_idx)) {
            return &zaf_dt_children[i];
        }
    }
    return NULL;
}

static const struct zaf_dt_child *zaf_dt_child_find_custom(const char *name) {
    for (uint8_t i = 0; i < ZAF_DT_CHILD_COUNT; i++) {
        const struct zaf_dt_child *c = &zaf_dt_children[i];
        if (c->event_type_idx == ZAF_EVTIDX_CUSTOM &&
            c->custom_name != NULL && strcmp(c->custom_name, name) == 0) {
            return c;
        }
    }
    return NULL;
}

static struct zaf_event_info zaf_dt_evt_buf;
static struct zaf_event_info zaf_dt_err_buf;

static void zaf_dt_child_to_event_info(const struct zaf_dt_child *c, struct zaf_event_info *out) {
    out->color_count = MIN(c->color_count, CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS);
    for (uint8_t i = 0; i < out->color_count; i++) {
        out->colors[i].r = c->color_bytes[i * 3];
        out->colors[i].g = c->color_bytes[i * 3 + 1];
        out->colors[i].b = c->color_bytes[i * 3 + 2];
    }
    out->animation           = c->animation;
    out->blink_on_ms         = c->blink_on_ms;
    out->blink_off_ms        = c->blink_off_ms;
    out->flash_duration_ms   = c->flash_duration_ms;
    out->flash_ease_in_ms    = c->flash_ease_in_ms;
    out->flash_ease_in_fn    = c->flash_ease_in_fn;
    out->flash_ease_out_ms   = c->flash_ease_out_ms;
    out->flash_ease_out_fn   = c->flash_ease_out_fn;
    out->breathe_duration_ms = c->breathe_duration_ms;
    out->feedback_pattern_len = MIN(c->feedback_pattern_len,
                                    CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN);
    for (uint8_t i = 0; i < out->feedback_pattern_len; i++) {
        out->feedback_pattern[i] = c->feedback_pattern[i];
    }
}

static void zaf_apply_dt_children(void) {
    for (uint8_t i = 0; i < ZAF_DT_CHILD_COUNT; i++) {
        const struct zaf_dt_child *c = &zaf_dt_children[i];
        if (c->event_type_idx != ZAF_EVTIDX_CUSTOM || c->custom_name == NULL) {
            continue;
        }
        STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
            if (strcmp(c->custom_name, cevt->name) != 0) { continue; }
            zaf_dt_child_to_event_info(c, &cevt->info);
            cevt->headless = c->headless;
            break;
        }
    }
}

static void zaf_eval_no_endpoint(void) {
    zaf_state.no_endpoint = !zaf_state.usb_connected && !zaf_state.ble_connected;
}

static int zaf_init(void) {
    zaf_config.led_strip    = DEVICE_DT_GET(DT_INST_PHANDLE(0, led_strip));
    zaf_config.chain_length = DT_INST_PROP(0, chain_length);

    zaf_config.error_slots = DT_INST_PROP_OR(0, error_slots, 0);
    if (zaf_config.error_slots > CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_ERROR_SLOTS) {
        zaf_config.error_slots = CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_ERROR_SLOTS;
    }
    if (zaf_config.error_slots > 0 && zaf_config.chain_length > 0) {
        zaf_config.error_section_size = zaf_config.chain_length / zaf_config.error_slots;
    } else {
        zaf_config.error_section_size = 0;
    }
    zaf_config.error_slots_reverse = DT_INST_PROP(0, error_slots_reverse);

#if DT_INST_NODE_HAS_PROP(0, ext_power)
    zaf_config.ext_power = DEVICE_DT_GET(DT_INST_PHANDLE(0, ext_power));
    if (!device_is_ready(zaf_config.ext_power)) {
        zaf_config.ext_power = NULL;
    }
#else
    zaf_config.ext_power = NULL;
#endif

    if (!device_is_ready(zaf_config.led_strip)) {
        return -ENODEV;
    }

    zaf_apply_dt_children();

    zaf_state.on           = true;
    zaf_state.batt_level   = 100;
    zaf_state.active_layer = 0;

    zaf_state.usb_event_ticks   = 0xFFFF;
    zaf_state.studio_event_ticks = 0xFFFF;
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        zaf_state.ble_event_ticks[i] = 0xFFFF;
    }
    for (int i = 0; i < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; i++) {
        zaf_state.batt_warn_ticks[i] = 0xFFFF;
        zaf_state.batt_crit_ticks[i] = 0xFFFF;
    }

    k_work_init_delayable(&zaf_save_work, zaf_save_work_fn);
    k_work_init_delayable(&zaf_save_evt_work, zaf_save_evt_work_fn);
    k_work_init_delayable(&zaf_save_custom_work, zaf_save_custom_work_fn);
    k_work_init_delayable(&zaf_feedback_step_work, zaf_feedback_step_work_fn);

    zaf_config.feedback_enabled  = DT_INST_PROP(0, feedback_enabled);
    zaf_config.feedback_delay_ms = DT_INST_PROP_OR(0, feedback_delay, 0);

#if DT_INST_NODE_HAS_PROP(0, feedback_gpios)
    zaf_config.feedback_gpio = &zaf_feedback_gpio_spec;
    if (device_is_ready(zaf_feedback_gpio_spec.port)) {
        gpio_pin_configure_dt(&zaf_feedback_gpio_spec, GPIO_OUTPUT);
    } else {
        zaf_config.feedback_gpio = NULL;
    }
#endif

#if DT_INST_NODE_HAS_PROP(0, feedback_extra_gpios)
    zaf_config.feedback_extra_gpio = &zaf_feedback_extra_gpio_spec;
    if (device_is_ready(zaf_feedback_extra_gpio_spec.port)) {
        gpio_pin_configure_dt(&zaf_feedback_extra_gpio_spec, GPIO_OUTPUT);
    } else {
        zaf_config.feedback_extra_gpio = NULL;
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
    zrc_register("argb/bw1",  CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_1, 0,   100);
    zrc_register("argb/bw2",  CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_2, 0,   100);
    zrc_register("argb/bw3",  CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_3, 0,   100);
    zrc_register("argb/bc1",  CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_1, 0,   100);
    zrc_register("argb/bc2",  CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_2, 0,   100);
    zrc_register("argb/bc3",  CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_3, 0,   100);
    zrc_register("argb/tick", CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS,            1,   100);
    zrc_register("argb/brt",  CONFIG_ZMK_ADAPTIVE_FEEDBACK_BRIGHTNESS,         0,   100);
#endif

    settings_load_subtree("argb");
    for (uint8_t i = 0; i < zaf_config.error_slots && i < CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_ERROR_SLOTS; i++) {
        zaf_state.evts.rt_error_slots[i].cleared = true;
    }
    if (zaf_state.on) {
        if (zaf_config.ext_power != NULL) {
            ext_power_enable(zaf_config.ext_power);
        }
        k_timer_start(&zaf_timer, K_NO_WAIT, K_MSEC(CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS));
    }

    return 0;
}

SYS_INIT(zaf_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int zaf_on(void) {
    if (zaf_state.on) {
        return 0;
    }

    if (zaf_config.ext_power != NULL) {
        ext_power_enable(zaf_config.ext_power);
    }

    zaf_state.on = true;
    zaf_state.anim_step = 0;
    zaf_state.color_idx = 0;
    k_timer_start(&zaf_timer, K_NO_WAIT, K_MSEC((uint32_t)ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS)));
    return zaf_save_state();
}

int zaf_off(void) {
    if (!zaf_state.on) {
        return 0;
    }

    k_timer_stop(&zaf_timer);
    zaf_state.on = false;

    if (zaf_config.ext_power != NULL) {
        ext_power_disable(zaf_config.ext_power);
    }

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &zaf_off_work);
    return zaf_save_state();
}

bool zaf_is_on(void) {
    return zaf_state.on;
}

uint8_t zaf_layer_count(void) {
    return (uint8_t)ZMK_KEYMAP_LAYERS_LEN;
}

/* Returns the runtime override slot for this event (write path). */
static struct zaf_evt_override *zaf_evt_rt_slot(const uint8_t event_idx, const uint8_t sub_idx) {
    switch (event_idx) {
    case ZAF_EVTIDX_LAYER:
        if (sub_idx < ZMK_KEYMAP_LAYERS_LEN) return &zaf_state.evts.rt_layers[sub_idx];
        return NULL;
    case ZAF_EVTIDX_BATT_WARN:
        if (sub_idx < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT) return &zaf_state.evts.rt_batt_warn[sub_idx];
        return NULL;
    case ZAF_EVTIDX_BATT_CRIT:
        if (sub_idx < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT) return &zaf_state.evts.rt_batt_crit[sub_idx];
        return NULL;
    case ZAF_EVTIDX_USB_CONN:       return &zaf_state.evts.rt_usb_conn;
    case ZAF_EVTIDX_USB_DISCONN:    return &zaf_state.evts.rt_usb_disconn;
    case ZAF_EVTIDX_BLE_PROFILE:
        if (sub_idx < CONFIG_BT_MAX_PAIRED) return &zaf_state.evts.rt_ble_profile[sub_idx];
        return NULL;
    case ZAF_EVTIDX_IDLE:           return &zaf_state.evts.rt_idle;
    case ZAF_EVTIDX_NO_ENDPOINT:    return &zaf_state.evts.rt_no_endpoint;
    case ZAF_EVTIDX_STUDIO_UNLOCK:  return &zaf_state.evts.rt_studio_unlock;
    case ZAF_EVTIDX_STUDIO_LOCK:    return &zaf_state.evts.rt_studio_lock;
    case ZAF_EVTIDX_ERROR:
        if (sub_idx < zaf_config.error_slots) return &zaf_state.evts.rt_error_slots[sub_idx];
        return NULL;
    default:                         return NULL;
    }
}

/* Returns the effective config: runtime override if set, else DTS default (read path). */
static const struct zaf_event_info *zaf_evt_eff_cfg(const uint8_t event_idx, const uint8_t sub_idx) {
    if (event_idx == ZAF_EVTIDX_ERROR) {
        /* Error path: check valid (not cleared) first, then DT fallback without cleared check
         * so copy-on-write in zaf_evt_set_impl() can still seed from the DT default. */
        if (sub_idx >= zaf_config.error_slots) return NULL;
        if (zaf_state.evts.rt_error_slots[sub_idx].valid) return &zaf_state.evts.rt_error_slots[sub_idx].cfg;
        const struct zaf_dt_child *ec = zaf_dt_child_find(ZAF_EVTIDX_ERROR, sub_idx);
        if (!ec) return NULL;
        zaf_dt_child_to_event_info(ec, &zaf_dt_err_buf);
        return &zaf_dt_err_buf;
    }
    const struct zaf_evt_override *rt = zaf_evt_rt_slot(event_idx, sub_idx);
    if (rt && rt->valid) return &rt->cfg;
    const struct zaf_dt_child *c = zaf_dt_child_find(event_idx, sub_idx);
    if (!c) return NULL;
    zaf_dt_child_to_event_info(c, &zaf_dt_evt_buf);
    return &zaf_dt_evt_buf;
}

static void zaf_copy_event_info_to_out(struct zaf_event_info *out, const struct zaf_event_info *cfg) {
    const uint8_t n = MIN(cfg->color_count, CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS);
    out->color_count = n;
    for (uint8_t i = 0; i < n; i++) {
        out->colors[i] = cfg->colors[i];
    }
    out->animation = cfg->animation;
    zaf_copy_animation_fields(out, cfg);
    zaf_copy_feedback_pattern(out, cfg);
}

bool zaf_event_is_headless(const uint8_t event_idx, const uint8_t sub_idx) {
    const struct zaf_dt_child *c = zaf_dt_child_find(event_idx, sub_idx);
    return c ? c->headless : false;
}

bool zaf_event_is_persistent(const uint8_t event_idx, const uint8_t sub_idx) {
    const struct zaf_dt_child *c = zaf_dt_child_find(event_idx, sub_idx);
    return c ? c->persistent : false;
}

const char *zaf_event_get_label(const uint8_t event_idx, const uint8_t sub_idx) {
    const struct zaf_dt_child *c = zaf_dt_child_find(event_idx, sub_idx);
    return c ? c->label : NULL;
}

const char *zaf_custom_event_get_label(const struct zaf_custom_event *evt) {
    if (!evt) { return NULL; }
    const struct zaf_dt_child *c = zaf_dt_child_find_custom(evt->name);
    return c ? c->label : NULL;
}

bool zaf_custom_event_is_persistent(const struct zaf_custom_event *evt) {
    if (!evt) { return false; }
    const struct zaf_dt_child *c = zaf_dt_child_find_custom(evt->name);
    return c ? c->persistent : false;
}

int zaf_event_get(const uint8_t event_idx, const uint8_t sub_idx, struct zaf_event_info *out) {
    const struct zaf_event_info *cfg = zaf_evt_eff_cfg(event_idx, sub_idx);
    if (!cfg) {
        return -EINVAL;
    }
    zaf_copy_event_info_to_out(out, cfg);
    return 0;
}

static void zaf_apply_field(struct zaf_event_info *cfg, const enum zaf_field field,
                            const union zaf_field_value v) {
    switch (field) {
    case ZAF_FIELD_COLOR:
        cfg->colors[0]   = v.color;
        cfg->color_count = 1;
        break;
    case ZAF_FIELD_COLOR_AT:
        cfg->colors[v.color_at.idx] = v.color_at.color;
        if (v.color_at.idx >= cfg->color_count) {
            cfg->color_count = v.color_at.idx + 1;
        }
        break;
    case ZAF_FIELD_ANIM:
        cfg->animation = v.anim;
        break;
    case ZAF_FIELD_BLINK:
        cfg->blink_on_ms  = v.blink.on_ms;
        cfg->blink_off_ms = v.blink.off_ms;
        break;
    case ZAF_FIELD_FLASH:
        cfg->flash_duration_ms = v.flash_dur_ms;
        break;
    case ZAF_FIELD_FLASH_EASE_IN:
        cfg->flash_ease_in_ms = v.ease.ms;
        cfg->flash_ease_in_fn = v.ease.fn;
        break;
    case ZAF_FIELD_FLASH_EASE_OUT:
        cfg->flash_ease_out_ms = v.ease.ms;
        cfg->flash_ease_out_fn = v.ease.fn;
        break;
    case ZAF_FIELD_FEEDBACK: {
        const uint8_t n = MIN(v.feedback.len, CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN);
        cfg->feedback_pattern_len = n;
        for (uint8_t i = 0; i < n; i++) {
            cfg->feedback_pattern[i] = v.feedback.pattern[i];
        }
        break;
    }
    case ZAF_FIELD_BREATHE:
        cfg->breathe_duration_ms = v.breathe_dur_ms;
        break;
    }
}

static int zaf_evt_set_impl(const uint8_t event_idx, const uint8_t sub_idx,
                             const enum zaf_field field, const union zaf_field_value val) {
    struct zaf_evt_override *slot = zaf_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zaf_event_info *dts = zaf_evt_eff_cfg(event_idx, sub_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    zaf_apply_field(&slot->cfg, field, val);
    slot->valid = true;
    zaf_kick_timer();
    return zaf_save_evt_cfg();
}

int zaf_event_set(const uint8_t event_idx, const uint8_t sub_idx,
                  const enum zaf_field field, const union zaf_field_value val) {
    if (field == ZAF_FIELD_COLOR_AT &&
        val.color_at.idx >= CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS) {
        return -EINVAL;
    }
    return zaf_evt_set_impl(event_idx, sub_idx, field, val);
}

int zaf_clear_persisted(void) {
    zaf_state.on              = true;
    zaf_state.override_active = false;
    memset(&zaf_state.override_cfg, 0, sizeof(zaf_state.override_cfg));

    zaf_state.evts.rt_idle.valid          = false;
    zaf_state.evts.rt_usb_conn.valid      = false;
    zaf_state.evts.rt_usb_disconn.valid   = false;
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        zaf_state.evts.rt_ble_profile[i].valid = false;
    }
    zaf_state.evts.rt_no_endpoint.valid   = false;
    zaf_state.evts.rt_studio_unlock.valid = false;
    zaf_state.evts.rt_studio_lock.valid   = false;

    for (int i = 0; i < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; i++) {
        zaf_state.evts.rt_batt_warn[i].valid = false;
        zaf_state.evts.rt_batt_crit[i].valid = false;
    }
    for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
        zaf_state.evts.rt_layers[i].valid = false;
    }
    for (uint8_t i = 0; i < zaf_config.error_slots && i < CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_ERROR_SLOTS; i++) {
        zaf_state.evts.rt_error_slots[i].valid = false;
        zaf_state.evts.rt_error_slots[i].cleared = true;
    }

    char key[32];
    STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
        cevt->info = (struct zaf_event_info){0};
        snprintf(key, sizeof(key), "argb/ce/%s", cevt->name);
        settings_delete(key);
    }

    settings_delete("argb/state");
    settings_delete("argb/evtcfg");
    return 0;
}

static void zaf_evt_activate(const struct zaf_event_info *ecfg, uint16_t *ticks_out) {
    const uint16_t ticks = ecfg ? zaf_ticks_from_ms(ecfg->flash_duration_ms) : 1;
    *ticks_out = ticks > 0 ? ticks : 1;
    zaf_tick_reset_blink();
    if (ecfg) {
        if (ecfg->animation == ZAF_ANIM_FLASH) {
            zaf_activate_flash_state(*ticks_out,
                ecfg->flash_ease_in_ms, ecfg->flash_ease_in_fn,
                ecfg->flash_ease_out_ms, ecfg->flash_ease_out_fn);
        }
        zaf_trigger_feedback(ecfg->feedback_pattern, ecfg->feedback_pattern_len);
    }
}

static int zaf_event_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *batt = as_zmk_battery_state_changed(eh);
    if (batt) {
        const uint8_t prev = zaf_state.batt_level;
        const uint8_t cur  = batt->state_of_charge;
        zaf_state.prev_batt_level = prev;
        zaf_state.batt_level      = cur;

        const uint8_t warn_thresh[CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT] = {
            (uint8_t)ZRC_GET("argb/bw1", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_1),
            (uint8_t)ZRC_GET("argb/bw2", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_2),
            (uint8_t)ZRC_GET("argb/bw3", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_3),
        };
        const uint8_t crit_thresh[CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT] = {
            (uint8_t)ZRC_GET("argb/bc1", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_1),
            (uint8_t)ZRC_GET("argb/bc2", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_2),
            (uint8_t)ZRC_GET("argb/bc3", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_3),
        };

        for (int i = 0; i < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; i++) {
            if (prev > warn_thresh[i] && cur <= warn_thresh[i]) {
                zaf_evt_activate(zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_WARN, (uint8_t)i), &zaf_state.batt_warn_ticks[i]);
            }
            if (prev > crit_thresh[i] && cur <= crit_thresh[i]) {
                zaf_evt_activate(zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_CRIT, (uint8_t)i), &zaf_state.batt_crit_ticks[i]);
            }
        }
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);
    if (act) {
        const bool now_idle = (act->state != ZMK_ACTIVITY_ACTIVE);
        if (now_idle && !zaf_state.idle) {
            const struct zaf_event_info *icfg = zaf_evt_eff_cfg(ZAF_EVTIDX_IDLE, 0);
            if (icfg != NULL) {
                zaf_tick_reset_blink();
                if (icfg->animation == ZAF_ANIM_FLASH) {
                    const uint16_t ticks = zaf_ticks_from_ms(icfg->flash_duration_ms);
                    zaf_activate_flash_state(ticks > 0 ? ticks : 1,
                        icfg->flash_ease_in_ms, icfg->flash_ease_in_fn,
                        icfg->flash_ease_out_ms, icfg->flash_ease_out_fn);
                }
                zaf_trigger_feedback(icfg->feedback_pattern, icfg->feedback_pattern_len);
            }
        }
        zaf_state.idle = now_idle;
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb) {
        const bool now_connected = (usb->conn_state != ZMK_USB_CONN_NONE);
        if (now_connected != zaf_state.usb_connected) {
            zaf_state.usb_connected     = now_connected;
            zaf_eval_no_endpoint();
            zaf_evt_activate(zaf_evt_eff_cfg(
                now_connected ? ZAF_EVTIDX_USB_CONN : ZAF_EVTIDX_USB_DISCONN, 0), &zaf_state.usb_event_ticks);
        }
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_layer_state_changed *layer = as_zmk_layer_state_changed(eh);
    if (layer) {
        const uint8_t l = layer->state
            ? layer->layer
            : (uint8_t)zmk_keymap_highest_layer_active();
        zaf_state.active_layer = l;
        zaf_state.anim_step     = 0;
        zaf_state.color_idx    = 0;
        zaf_tick_reset_blink();
        if (layer->state && l < ZMK_KEYMAP_LAYERS_LEN) {
            const struct zaf_event_info *lcfg = zaf_evt_eff_cfg(ZAF_EVTIDX_LAYER, l);
            if (lcfg != NULL) {
                if (lcfg->animation == ZAF_ANIM_FLASH) {
                    const uint16_t ticks = zaf_ticks_from_ms(lcfg->flash_duration_ms);
                    zaf_activate_flash_state(ticks > 0 ? ticks : 1,
                        lcfg->flash_ease_in_ms, lcfg->flash_ease_in_fn,
                        lcfg->flash_ease_out_ms, lcfg->flash_ease_out_fn);
                }
                zaf_trigger_feedback(lcfg->feedback_pattern, lcfg->feedback_pattern_len);
            }
        }
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_ble_active_profile_changed *ble = as_zmk_ble_active_profile_changed(eh);
    if (ble) {
        zaf_state.ble_connected = zmk_ble_active_profile_is_connected();
        zaf_eval_no_endpoint();
        const uint8_t profile_idx = ble->index;
        if (profile_idx < CONFIG_BT_MAX_PAIRED) {
            zaf_evt_activate(zaf_evt_eff_cfg(ZAF_EVTIDX_BLE_PROFILE, profile_idx),
                             &zaf_state.ble_event_ticks[profile_idx]);
        }
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_STUDIO)
    const struct zmk_studio_core_lock_state_changed *studio =
        as_zmk_studio_core_lock_state_changed(eh);
    if (studio) {
        const bool now_unlocked =
            (studio->state == ZMK_STUDIO_CORE_LOCK_STATE_UNLOCKED);
        if (now_unlocked != zaf_state.studio_unlocked) {
            zaf_state.studio_unlocked      = now_unlocked;
            zaf_evt_activate(zaf_evt_eff_cfg(
                now_unlocked ? ZAF_EVTIDX_STUDIO_UNLOCK : ZAF_EVTIDX_STUDIO_LOCK, 0),
                &zaf_state.studio_event_ticks);
        }
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif /* CONFIG_ZMK_STUDIO */

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zaf, zaf_event_listener);
ZMK_SUBSCRIPTION(zaf, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_usb_conn_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_ble_active_profile_changed);
#if IS_ENABLED(CONFIG_ZMK_STUDIO)
ZMK_SUBSCRIPTION(zaf, zmk_studio_core_lock_state_changed);
#endif

int zaf_custom_event_trigger(struct zaf_custom_event *evt) {
    if (!evt || (evt->info.color_count == 0 && evt->info.feedback_pattern_len == 0)) {
        return -EINVAL;
    }
    const uint16_t ticks = zaf_ticks_from_ms(evt->info.flash_duration_ms);
    evt->ticks   = ticks > 0 ? ticks : 1;
    evt->pending = true;
    zaf_tick_reset_blink();
    if (evt->info.animation == ZAF_ANIM_FLASH) {
        zaf_activate_flash_state(evt->ticks,
            evt->info.flash_ease_in_ms, evt->info.flash_ease_in_fn,
            evt->info.flash_ease_out_ms, evt->info.flash_ease_out_fn);
    }
    zaf_trigger_feedback(evt->info.feedback_pattern, evt->info.feedback_pattern_len);
    zaf_kick_timer();
    return 0;
}

int zaf_custom_event_get(const struct zaf_custom_event *evt, struct zaf_event_info *out) {
    if (!evt || !out) {
        return -EINVAL;
    }
    *out = evt->info;
    return 0;
}

int zaf_custom_event_set(struct zaf_custom_event *evt, const enum zaf_field field,
                         const union zaf_field_value val) {
    if (!evt) {
        return -EINVAL;
    }
    if (field == ZAF_FIELD_COLOR_AT && val.color_at.idx >= CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS) {
        return -EINVAL;
    }
    zaf_apply_field(&evt->info, field, val);
    return zaf_save_custom_cfg();
}

uint8_t zaf_error_slots_count(void) {
    return zaf_config.error_slots;
}

static struct zaf_evt_override *zaf_error_rt_slot(const uint8_t slot_idx) {
    if (slot_idx >= zaf_config.error_slots) {
        return NULL;
    }
    return &zaf_state.evts.rt_error_slots[slot_idx];
}

static const struct zaf_event_info *zaf_error_eff_cfg(const uint8_t slot_idx) {
    if (slot_idx >= zaf_config.error_slots) return NULL;
    if (zaf_state.evts.rt_error_slots[slot_idx].cleared) return NULL;
    if (zaf_state.evts.rt_error_slots[slot_idx].valid) return &zaf_state.evts.rt_error_slots[slot_idx].cfg;
    const struct zaf_dt_child *c = zaf_dt_child_find(ZAF_EVTIDX_ERROR, slot_idx);
    if (!c) return NULL;
    zaf_dt_child_to_event_info(c, &zaf_dt_err_buf);
    return &zaf_dt_err_buf;
}

int zaf_error_get(const uint8_t slot_idx, struct zaf_event_info *out) {
    const struct zaf_event_info *cfg = zaf_error_eff_cfg(slot_idx);
    if (!cfg) {
        return -EINVAL;
    }
    zaf_copy_event_info_to_out(out, cfg);
    return 0;
}

int zaf_error_set(const uint8_t slot_idx, const enum zaf_field field,
                  const union zaf_field_value val) {
    if (field == ZAF_FIELD_COLOR_AT && val.color_at.idx >= CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS) {
        return -EINVAL;
    }
    struct zaf_evt_override *slot = zaf_error_rt_slot(slot_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zaf_event_info *dts = zaf_error_eff_cfg(slot_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    zaf_apply_field(&slot->cfg, field, val);
    slot->valid = true;
    zaf_kick_timer();
    return zaf_save_evt_cfg();
}

int zaf_error_trigger(const uint8_t slot_idx) {
    if (slot_idx >= zaf_config.error_slots) {
        return -EINVAL;
    }

    zaf_state.evts.rt_error_slots[slot_idx].cleared = false;

    const struct zaf_event_info *ecfg = zaf_error_eff_cfg(slot_idx);
    if (!ecfg || (ecfg->color_count == 0 && ecfg->feedback_pattern_len == 0)) {
        return -EINVAL;
    }

    zaf_state.evts.rt_error_slots[slot_idx].cfg = *ecfg;
    zaf_state.evts.rt_error_slots[slot_idx].valid = true;
    zaf_trigger_feedback(ecfg->feedback_pattern, ecfg->feedback_pattern_len);
    zaf_kick_timer();
    return 0;
}

int zaf_error_clear(const uint8_t slot_idx) {
    if (slot_idx >= zaf_config.error_slots) {
        return -EINVAL;
    }

    zaf_state.evts.rt_error_slots[slot_idx].cleared = true;
    zaf_kick_timer();
    return zaf_save_evt_cfg();
}

int zaf_error_clear_all(void) {
    for (uint8_t i = 0; i < zaf_config.error_slots; i++) {
        zaf_state.evts.rt_error_slots[i].cleared = true;
    }
    zaf_kick_timer();
    return zaf_save_evt_cfg();
}

static void zaf_apply_error_sections(void) {
    if (zaf_config.error_slots == 0 || zaf_config.error_section_size == 0) {
        return;
    }

    /* Always advance blink phase once before the rendering loop, using the
     * first active BLINK error section's timing. This ensures consistent blink
     * speed regardless of how many error sections are active. */
    for (uint8_t s = 0; s < zaf_config.error_slots; s++) {
        const struct zaf_event_info *ecfg = zaf_error_eff_cfg(s);
        if (ecfg && ecfg->color_count > 0 && ecfg->animation == ZAF_ANIM_BLINK) {
            const uint16_t phase_ms = zaf_state.blink_phase ? ecfg->blink_on_ms : ecfg->blink_off_ms;
            const uint16_t phase_ticks = zaf_ticks_from_ms(phase_ms);
            zaf_state.blink_phase_ticks++;
            if (zaf_state.blink_phase_ticks >= phase_ticks) {
                zaf_state.blink_phase = !zaf_state.blink_phase;
                zaf_state.blink_phase_ticks = 0;
                if (zaf_state.blink_phase) {
                    zaf_state.color_idx = (zaf_state.color_idx + 1) % ecfg->color_count;
                }
            }
            break;
        }
    }

    for (uint8_t s = 0; s < zaf_config.error_slots; s++) {
        const struct zaf_event_info *cfg = zaf_error_eff_cfg(s);
        if (cfg == NULL || cfg->color_count == 0) {
            continue;
        }

        const uint16_t start = zaf_config.error_slots_reverse
            ? zaf_config.chain_length - ((uint16_t)s + 1) * zaf_config.error_section_size
            : (uint16_t)s * zaf_config.error_section_size;
        const uint16_t end   = start + zaf_config.error_section_size;
        if (start >= zaf_config.chain_length || end > zaf_config.chain_length) {
            continue;
        }

        switch (cfg->animation) {
        case ZAF_ANIM_SOLID: {
            const struct zaf_rgb c0 = cfg->colors[0];
            const struct led_rgb sc = { .r = c0.r, .g = c0.g, .b = c0.b };
            zaf_fill_section(start, end, sc);
            break;
        }

        case ZAF_ANIM_BLINK: {
            const uint8_t idx = zaf_state.color_idx % cfg->color_count;
            const struct zaf_rgb c_cur = cfg->colors[idx];
            const struct led_rgb sc = zaf_state.blink_phase
                ? (struct led_rgb){ .r = c_cur.r, .g = c_cur.g, .b = c_cur.b }
                : (struct led_rgb){ 0, 0, 0 };
            zaf_fill_section(start, end, sc);
            break;
        }

        case ZAF_ANIM_BREATHE: {
            const uint32_t dur = cfg->breathe_duration_ms;
            const int32_t tick_ms = ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_FEEDBACK_TICK_MS);
            const uint16_t breathe_steps = (tick_ms > 0) ? (uint16_t)(dur / (uint32_t)tick_ms) : 60;
            const struct zaf_rgb bc = zaf_breathe_color(cfg);
            const struct led_rgb lbc = { .r = bc.r, .g = bc.g, .b = bc.b };
            zaf_state.anim_step++;
            if (zaf_state.anim_step >= breathe_steps) {
                zaf_state.anim_step = 0;
                zaf_state.color_idx = (zaf_state.color_idx + 1) % cfg->color_count;
            }
            zaf_fill_section(start, end, lbc);
            break;
        }

        case ZAF_ANIM_FLASH: {
            const uint16_t ei_ticks = zaf_ticks_from_ms(cfg->flash_ease_in_ms);
            const uint16_t eo_ticks = zaf_ticks_from_ms(cfg->flash_ease_out_ms);
            const uint8_t scale = zaf_compute_flash_scale(
                ei_ticks, cfg->flash_ease_in_fn,
                eo_ticks, cfg->flash_ease_out_fn);
            const struct zaf_rgb c0 = cfg->colors[0];
            const struct led_rgb sc = {
                .r = (uint8_t)(((uint16_t)c0.r * scale) >> 8),
                .g = (uint8_t)(((uint16_t)c0.g * scale) >> 8),
                .b = (uint8_t)(((uint16_t)c0.b * scale) >> 8),
            };
            zaf_fill_section(start, end, sc);
            break;
        }

        default: break;
        }
    }
}
