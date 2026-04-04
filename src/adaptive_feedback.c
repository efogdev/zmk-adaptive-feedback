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

struct zaf_led_config {
    struct zaf_rgb colors[CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS];
    uint8_t color_count;
    uint8_t animation;
    bool persistent;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint16_t flash_duration_ms;
    uint16_t feedback_duration_ms;
};

struct zaf_layer_entry {
    struct zaf_led_config cfg;
    bool valid;
};

struct zaf_device_config {
    const struct device *led_strip;
    const struct device *ext_power;
    uint8_t chain_length;
    bool feedback_enabled;
    uint16_t feedback_delay_ms;
    const struct gpio_dt_spec *feedback_gpio;
    const struct gpio_dt_spec *feedback_extra_gpio;
    struct zaf_led_config evt_idle;
    struct zaf_led_config evt_usb_conn;
    struct zaf_led_config evt_usb_disconn;
    struct zaf_led_config evt_ble_profile;
    struct zaf_led_config evt_batt_warn[ZAF_BATT_LEVEL_COUNT];
    struct zaf_led_config evt_batt_crit[ZAF_BATT_LEVEL_COUNT];
    struct zaf_led_config evt_no_endpoint;
    struct zaf_layer_entry dt_layers[ZMK_KEYMAP_LAYERS_LEN];
};

struct zaf_evt_override {
    struct zaf_led_config cfg;
    bool valid;
};

struct zaf_runtime_state {
    bool on;
    bool override_active;
    bool feedback_active;
    struct zaf_led_config override_cfg;
    struct zaf_evt_override rt_layers[ZMK_KEYMAP_LAYERS_LEN];
    struct zaf_evt_override rt_idle;
    struct zaf_evt_override rt_usb_conn;
    struct zaf_evt_override rt_usb_disconn;
    struct zaf_evt_override rt_ble_profile;
    struct zaf_evt_override rt_no_endpoint;
    struct zaf_evt_override rt_batt_warn[ZAF_BATT_LEVEL_COUNT];
    struct zaf_evt_override rt_batt_crit[ZAF_BATT_LEVEL_COUNT];
    uint8_t active_layer;
    uint8_t batt_level;
    uint8_t prev_batt_level;
    bool batt_warn_pending[ZAF_BATT_LEVEL_COUNT];
    uint16_t batt_warn_ticks[ZAF_BATT_LEVEL_COUNT];
    bool batt_crit_pending[ZAF_BATT_LEVEL_COUNT];
    uint16_t batt_crit_ticks[ZAF_BATT_LEVEL_COUNT];
    bool usb_event_pending;
    uint16_t usb_event_ticks;
    bool usb_connected;
    bool ble_connected;
    bool no_endpoint;
    bool ble_event_pending;
    uint16_t ble_event_ticks;
    bool idle;
    uint16_t anim_step;
    uint8_t color_idx;
    bool blink_phase;
    uint16_t blink_phase_ticks;
    bool flash_active;
    uint16_t flash_ticks;
};

struct zaf_persisted {
    bool on;
    bool override_active;
    struct zaf_led_config override_cfg;
};

struct zaf_evt_persisted {
    struct zaf_evt_override idle;
    struct zaf_evt_override usb_conn;
    struct zaf_evt_override usb_disconn;
    struct zaf_evt_override ble_profile;
    struct zaf_evt_override no_endpoint;
    struct zaf_evt_override batt_warn[ZAF_BATT_LEVEL_COUNT];
    struct zaf_evt_override batt_crit[ZAF_BATT_LEVEL_COUNT];
    struct zaf_evt_override layers[ZMK_KEYMAP_LAYERS_LEN];
};

struct zaf_dt_child {
    uint8_t event_type_idx;
    uint8_t layer_index;
    uint8_t batt_level;
    uint8_t color_bytes[CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS * 3];
    uint8_t color_count;
    uint8_t animation;
    bool persistent;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint16_t flash_duration_ms;
    uint16_t feedback_duration_ms;
};

#define ZAF_CHILD_ENTRY(child)                                                         \
    {                                                                                  \
        .event_type_idx  = DT_ENUM_IDX(child, event_type),                            \
        .layer_index     = DT_PROP_OR(child, layer_index, 0),                         \
        .batt_level      = DT_PROP_OR(child, battery_level, 1),                       \
        .color_bytes     = DT_PROP(child, colors),                                    \
        .color_count     = DT_PROP_LEN(child, colors) / 3,                            \
        .animation       = DT_ENUM_IDX_OR(child, animation, 0),                       \
        .persistent      = DT_PROP(child, persistent),                                \
        .blink_on_ms     = DT_PROP_OR(child, blink_on_ms, 500),                       \
        .blink_off_ms    = DT_PROP_OR(child, blink_off_ms, 500),                      \
        .flash_duration_ms    = DT_PROP_OR(child, flash_duration_ms, 300),            \
        .feedback_duration_ms = DT_PROP_OR(child, feedback_duration, 0),              \
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

static int zaf_prev_extra_gpio_state;
static uint16_t zaf_pending_feedback_duration_ms;
static struct k_work_delayable zaf_feedback_on_work;
static struct k_work_delayable zaf_feedback_off_work;

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
static const struct zaf_led_config *zaf_evt_eff_cfg(uint8_t event_idx, uint8_t sub_idx);

static const struct zaf_led_config *zaf_resolve(void) {
    if (zaf_state.override_active) {
        return &zaf_state.override_cfg;
    }

    for (int i = ZAF_BATT_LEVEL_COUNT - 1; i >= 0; i--) {
        if (zaf_state.batt_crit_pending[i]) {
            return zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_CRIT, (uint8_t)i);
        }
    }
    for (int i = ZAF_BATT_LEVEL_COUNT - 1; i >= 0; i--) {
        if (zaf_state.batt_warn_pending[i]) {
            return zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_WARN, (uint8_t)i);
        }
    }

    if (zaf_state.usb_event_pending) {
        return zaf_evt_eff_cfg(zaf_state.usb_connected
                               ? ZAF_EVTIDX_USB_CONN : ZAF_EVTIDX_USB_DISCONN, 0);
    }

    if (zaf_state.ble_event_pending) {
        return zaf_evt_eff_cfg(ZAF_EVTIDX_BLE_PROFILE, 0);
    }

    if (zaf_state.no_endpoint) {
        return zaf_evt_eff_cfg(ZAF_EVTIDX_NO_ENDPOINT, 0);
    }

    const uint8_t layer = zaf_state.active_layer;
    if (layer < ZMK_KEYMAP_LAYERS_LEN) {
        const struct zaf_led_config *lcfg = zaf_evt_eff_cfg(ZAF_EVTIDX_LAYER, layer);
        if (lcfg != NULL && lcfg->persistent) {
            return lcfg;
        }
    }

    if (zaf_state.idle) {
        return zaf_evt_eff_cfg(ZAF_EVTIDX_IDLE, 0);
    }

    return NULL;
}

static void zaf_apply_animation(const struct zaf_led_config *cfg) {
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
        const uint16_t breathe_steps = (uint16_t)ZRC_GET("argb/bstp", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BREATHE_STEPS);
        const int step = (int)zaf_state.anim_step - (int)(breathe_steps / 2);
        const int abs_step = (step < 0) ? -step : step;
        const int half_steps = (int)(breathe_steps / 2);
        const uint8_t scale = (half_steps > 0) ? (uint8_t)((100 * (half_steps - abs_step)) / half_steps) : 100;
        const uint8_t idx = zaf_state.color_idx % cfg->color_count;
        const struct zaf_rgb base = cfg->colors[idx];
        const struct zaf_rgb bc = {
            .r = (uint8_t)((base.r * scale) / 100),
            .g = (uint8_t)((base.g * scale) / 100),
            .b = (uint8_t)((base.b * scale) / 100),
        };
        zaf_fill(bc);
        zaf_state.anim_step += 10;
        if (zaf_state.anim_step >= breathe_steps) {
            zaf_state.anim_step = 0;
            zaf_state.color_idx = (zaf_state.color_idx + 1) % cfg->color_count;
        }
        break;
    }

    case ZAF_ANIM_FLASH:
        if (zaf_state.flash_active) {
            zaf_fill(c0);
        } else {
            zaf_clear_pixels();
        }
        break;
    default: break;
    }
}

static void zaf_feedback_off_work_fn(struct k_work *work) {
    if (zaf_config.feedback_gpio != NULL) {
        gpio_pin_set_dt(zaf_config.feedback_gpio, 0);
    }
    if (zaf_config.feedback_extra_gpio != NULL) {
        gpio_pin_set_dt(zaf_config.feedback_extra_gpio, zaf_prev_extra_gpio_state);
    }
    zaf_state.feedback_active = false;
}

static void zaf_feedback_on_work_fn(struct k_work *work) {
    if (zaf_config.feedback_gpio == NULL) {
        return;
    }
    gpio_pin_set_dt(zaf_config.feedback_gpio, 1);
    k_work_reschedule(&zaf_feedback_off_work, K_MSEC(zaf_pending_feedback_duration_ms));
}

static void zaf_trigger_feedback(const uint16_t duration_ms) {
    if (!zaf_config.feedback_enabled || zaf_config.feedback_gpio == NULL || duration_ms == 0) {
        return;
    }
    zaf_state.feedback_active = true;
    if (zaf_config.feedback_extra_gpio != NULL) {
        zaf_prev_extra_gpio_state = gpio_pin_get_dt(zaf_config.feedback_extra_gpio);
        gpio_pin_set_dt(zaf_config.feedback_extra_gpio, 1);
    }
    if (zaf_config.feedback_delay_ms > 0) {
        zaf_pending_feedback_duration_ms = duration_ms;
        k_work_reschedule(&zaf_feedback_on_work, K_MSEC(zaf_config.feedback_delay_ms));
    } else {
        gpio_pin_set_dt(zaf_config.feedback_gpio, 1);
        k_work_reschedule(&zaf_feedback_off_work, K_MSEC(duration_ms));
    }
}

extern struct k_timer zaf_timer;

static void zaf_tick_reset_blink(void) {
    zaf_state.blink_phase = true;
    zaf_state.blink_phase_ticks = 0;
}

static void zaf_tick(struct k_work *work) {
    if (!zaf_state.on) {
        return;
    }

    if (zaf_state.usb_event_pending && zaf_state.usb_event_ticks > 0) {
        if (--zaf_state.usb_event_ticks == 0) {
            zaf_state.usb_event_pending = false;
            zaf_tick_reset_blink();
        }
    }

    if (zaf_state.ble_event_pending && zaf_state.ble_event_ticks > 0) {
        if (--zaf_state.ble_event_ticks == 0) {
            zaf_state.ble_event_pending = false;
            zaf_tick_reset_blink();
        }
    }

    for (int i = 0; i < ZAF_BATT_LEVEL_COUNT; i++) {
        if (zaf_state.batt_warn_pending[i] && zaf_state.batt_warn_ticks[i] > 0) {
            if (--zaf_state.batt_warn_ticks[i] == 0) {
                zaf_state.batt_warn_pending[i] = false;
                zaf_tick_reset_blink();
            }
        }
        if (zaf_state.batt_crit_pending[i] && zaf_state.batt_crit_ticks[i] > 0) {
            if (--zaf_state.batt_crit_ticks[i] == 0) {
                zaf_state.batt_crit_pending[i] = false;
                zaf_tick_reset_blink();
            }
        }
    }

    if (zaf_state.flash_active && zaf_state.flash_ticks > 0) {
        if (--zaf_state.flash_ticks == 0) {
            zaf_state.flash_active = false;
        }
    }

    const struct zaf_led_config *cfg = zaf_resolve();
    bool output_is_static_dark = false;

    if (cfg == NULL) {
        zaf_clear_pixels();
        output_is_static_dark = true;
    } else {
        zaf_apply_animation(cfg);
        bool all_zero = true;
        for (int i = 0; i < zaf_config.chain_length; i++) {
            if (zaf_pixels[i].r || zaf_pixels[i].g || zaf_pixels[i].b) {
                all_zero = false;
                break;
            }
        }
        if (all_zero) {
            ext_power_disable(zaf_config.ext_power);
            if (cfg->animation == ZAF_ANIM_SOLID) {
                output_is_static_dark = true;
            }
        } else {
            ext_power_enable(zaf_config.ext_power);
        }
    }

    const int err = led_strip_update_rgb(zaf_config.led_strip, zaf_pixels, zaf_config.chain_length);
    if (err < 0) {
        LOG_ERR("led_strip_update_rgb failed: %d", err);
    }

    if (output_is_static_dark) {
        k_timer_stop(&zaf_timer);
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
    static struct zaf_evt_persisted p;
    p = (struct zaf_evt_persisted) {
        .idle        = zaf_state.rt_idle,
        .usb_conn    = zaf_state.rt_usb_conn,
        .usb_disconn = zaf_state.rt_usb_disconn,
        .ble_profile = zaf_state.rt_ble_profile,
        .no_endpoint = zaf_state.rt_no_endpoint,
    };
    for (int i = 0; i < ZAF_BATT_LEVEL_COUNT; i++) {
        p.batt_warn[i] = zaf_state.rt_batt_warn[i];
        p.batt_crit[i] = zaf_state.rt_batt_crit[i];
    }
    for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
        p.layers[i] = zaf_state.rt_layers[i];
    }
    settings_save_one("argb/evtcfg", &p, sizeof(p));
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

    if (settings_name_steq(name, "evtcfg", &next) && !next) {
        if (len != sizeof(struct zaf_evt_persisted)) {
            return -EINVAL;
        }
        static struct zaf_evt_persisted p;
        const int rc = read_cb(cb_arg, &p, sizeof(p));
        if (rc < 0) {
            return rc;
        }
        zaf_state.rt_idle        = p.idle;
        zaf_state.rt_usb_conn    = p.usb_conn;
        zaf_state.rt_usb_disconn = p.usb_disconn;
        zaf_state.rt_ble_profile = p.ble_profile;
        zaf_state.rt_no_endpoint = p.no_endpoint;
        for (int i = 0; i < ZAF_BATT_LEVEL_COUNT; i++) {
            zaf_state.rt_batt_warn[i] = p.batt_warn[i];
            zaf_state.rt_batt_crit[i] = p.batt_crit[i];
        }
        for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
            zaf_state.rt_layers[i] = p.layers[i];
        }
        return 0;
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

static void zaf_apply_dt_children(void) {
    for (uint8_t i = 0; i < ZAF_DT_CHILD_COUNT; i++) {
        const struct zaf_dt_child *c = &zaf_dt_children[i];
        struct zaf_led_config cfg = {
            .animation            = c->animation,
            .persistent           = c->persistent,
            .blink_on_ms          = c->blink_on_ms,
            .blink_off_ms         = c->blink_off_ms,
            .flash_duration_ms    = c->flash_duration_ms,
            .feedback_duration_ms = c->feedback_duration_ms,
            .color_count          = MIN(c->color_count, CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS),
        };
        for (uint8_t j = 0; j < c->color_count && j < CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS; j++) {
            cfg.colors[j].r = c->color_bytes[j * 3];
            cfg.colors[j].g = c->color_bytes[j * 3 + 1];
            cfg.colors[j].b = c->color_bytes[j * 3 + 2];
        }
        switch (c->event_type_idx) {
        case ZAF_EVTIDX_LAYER:
            if (c->layer_index < ZMK_KEYMAP_LAYERS_LEN) {
                zaf_config.dt_layers[c->layer_index].cfg   = cfg;
                zaf_config.dt_layers[c->layer_index].valid = true;
            }
            break;
        case ZAF_EVTIDX_BATT_WARN:
            if (c->batt_level >= 1 && c->batt_level <= 3) {
                zaf_config.evt_batt_warn[c->batt_level - 1] = cfg;
            }
            break;
        case ZAF_EVTIDX_BATT_CRIT:
            if (c->batt_level >= 1 && c->batt_level <= 3) {
                zaf_config.evt_batt_crit[c->batt_level - 1] = cfg;
            }
            break;
        case ZAF_EVTIDX_USB_CONN:
            zaf_config.evt_usb_conn = cfg;
            break;
        case ZAF_EVTIDX_USB_DISCONN:
            zaf_config.evt_usb_disconn = cfg;
            break;
        case ZAF_EVTIDX_BLE_PROFILE:
            zaf_config.evt_ble_profile = cfg;
            break;
        case ZAF_EVTIDX_IDLE:
            zaf_config.evt_idle = cfg;
            break;
        case ZAF_EVTIDX_NO_ENDPOINT:
            zaf_config.evt_no_endpoint = cfg;
            break;
        default: break;
        }
    }
}

static void zaf_eval_no_endpoint(void) {
    zaf_state.no_endpoint = !zaf_state.usb_connected && !zaf_state.ble_connected;
}

static int zaf_init(void) {
    zaf_config.led_strip    = DEVICE_DT_GET(DT_INST_PHANDLE(0, led_strip));
    zaf_config.chain_length = DT_INST_PROP(0, chain_length);

#if DT_INST_NODE_HAS_PROP(0, ext_power)
    zaf_config.ext_power = DEVICE_DT_GET(DT_INST_PHANDLE(0, ext_power));
    if (!device_is_ready(zaf_config.ext_power)) {
        LOG_WRN("ext-power device not ready");
        zaf_config.ext_power = NULL;
    }
#else
    zaf_config.ext_power = NULL;
#endif

    if (!device_is_ready(zaf_config.led_strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    zaf_apply_dt_children();

    zaf_state.on           = true;
    zaf_state.batt_level   = 100;
    zaf_state.active_layer = 0;

    k_work_init_delayable(&zaf_save_work, zaf_save_work_fn);
    k_work_init_delayable(&zaf_save_evt_work, zaf_save_evt_work_fn);
    k_work_init_delayable(&zaf_feedback_on_work, zaf_feedback_on_work_fn);
    k_work_init_delayable(&zaf_feedback_off_work, zaf_feedback_off_work_fn);

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
    zrc_register("argb/bstp", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BREATHE_STEPS,      100, 10000);
#endif

    settings_load_subtree("argb");
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
        if (sub_idx < ZMK_KEYMAP_LAYERS_LEN) return &zaf_state.rt_layers[sub_idx];
        return NULL;
    case ZAF_EVTIDX_BATT_WARN:
        if (sub_idx < ZAF_BATT_LEVEL_COUNT) return &zaf_state.rt_batt_warn[sub_idx];
        return NULL;
    case ZAF_EVTIDX_BATT_CRIT:
        if (sub_idx < ZAF_BATT_LEVEL_COUNT) return &zaf_state.rt_batt_crit[sub_idx];
        return NULL;
    case ZAF_EVTIDX_USB_CONN:    return &zaf_state.rt_usb_conn;
    case ZAF_EVTIDX_USB_DISCONN: return &zaf_state.rt_usb_disconn;
    case ZAF_EVTIDX_BLE_PROFILE: return &zaf_state.rt_ble_profile;
    case ZAF_EVTIDX_IDLE:        return &zaf_state.rt_idle;
    case ZAF_EVTIDX_NO_ENDPOINT: return &zaf_state.rt_no_endpoint;
    default:                      return NULL;
    }
}

/* Returns the effective config: runtime override if set, else DTS default (read path). */
static const struct zaf_led_config *zaf_evt_eff_cfg(const uint8_t event_idx, const uint8_t sub_idx) {
    switch (event_idx) {
    case ZAF_EVTIDX_LAYER:
        if (sub_idx < ZMK_KEYMAP_LAYERS_LEN) {
            if (zaf_state.rt_layers[sub_idx].valid) return &zaf_state.rt_layers[sub_idx].cfg;
            return zaf_config.dt_layers[sub_idx].valid ? &zaf_config.dt_layers[sub_idx].cfg : NULL;
        }
        return NULL;
    case ZAF_EVTIDX_BATT_WARN:
        if (sub_idx < ZAF_BATT_LEVEL_COUNT) {
            if (zaf_state.rt_batt_warn[sub_idx].valid) return &zaf_state.rt_batt_warn[sub_idx].cfg;
            return &zaf_config.evt_batt_warn[sub_idx];
        }
        return NULL;
    case ZAF_EVTIDX_BATT_CRIT:
        if (sub_idx < ZAF_BATT_LEVEL_COUNT) {
            if (zaf_state.rt_batt_crit[sub_idx].valid) return &zaf_state.rt_batt_crit[sub_idx].cfg;
            return &zaf_config.evt_batt_crit[sub_idx];
        }
        return NULL;
    case ZAF_EVTIDX_USB_CONN:
        return zaf_state.rt_usb_conn.valid ? &zaf_state.rt_usb_conn.cfg : &zaf_config.evt_usb_conn;
    case ZAF_EVTIDX_USB_DISCONN:
        return zaf_state.rt_usb_disconn.valid ? &zaf_state.rt_usb_disconn.cfg : &zaf_config.evt_usb_disconn;
    case ZAF_EVTIDX_BLE_PROFILE:
        return zaf_state.rt_ble_profile.valid ? &zaf_state.rt_ble_profile.cfg : &zaf_config.evt_ble_profile;
    case ZAF_EVTIDX_IDLE:
        return zaf_state.rt_idle.valid ? &zaf_state.rt_idle.cfg : &zaf_config.evt_idle;
    case ZAF_EVTIDX_NO_ENDPOINT:
        return zaf_state.rt_no_endpoint.valid ? &zaf_state.rt_no_endpoint.cfg : &zaf_config.evt_no_endpoint;
    default:
        return NULL;
    }
}

int zaf_event_get(const uint8_t event_idx, const uint8_t sub_idx, struct zaf_event_info *out) {
    const struct zaf_led_config *cfg = zaf_evt_eff_cfg(event_idx, sub_idx);
    if (!cfg) {
        return -EINVAL;
    }
    const uint8_t n = MIN(cfg->color_count, ZAF_INFO_MAX_COLORS);
    out->color_count = n;
    for (uint8_t i = 0; i < n; i++) {
        out->colors[i] = cfg->colors[i];
    }
    out->anim             = cfg->animation;
    out->blink_on_ms      = cfg->blink_on_ms;
    out->blink_off_ms     = cfg->blink_off_ms;
    out->flash_dur_ms     = cfg->flash_duration_ms;
    out->feedback_dur_ms  = cfg->feedback_duration_ms;
    out->persistent       = cfg->persistent;
    return 0;
}

static int zaf_evt_set_impl(const uint8_t event_idx, const uint8_t sub_idx,
    void (*set_field)(struct zaf_led_config *, void *), void *arg, const bool force_persist
) {
    struct zaf_evt_override *slot = zaf_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zaf_led_config *dts = zaf_evt_eff_cfg(event_idx, sub_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    set_field(&slot->cfg, arg);
    slot->valid = true;
    if (force_persist || event_idx == ZAF_EVTIDX_LAYER) {
        slot->cfg.persistent = true;
    }
    zaf_kick_timer();
    return zaf_save_evt_cfg();
}

static void set_color_single(struct zaf_led_config *cfg, void *arg) {
    const struct zaf_rgb *color = arg;
    cfg->colors[0]   = *color;
    cfg->color_count = 1;
}

static void set_color_at(struct zaf_led_config *cfg, void *arg) {
    const struct { uint8_t idx; struct zaf_rgb color; } *p = arg;
    cfg->colors[p->idx] = p->color;
    if (p->idx >= cfg->color_count) {
        cfg->color_count = p->idx + 1;
    }
}

static void set_anim(struct zaf_led_config *cfg, void *arg) {
    cfg->animation = *(uint8_t *)arg;
}

static void set_blink(struct zaf_led_config *cfg, void *arg) {
    const struct { uint16_t on_ms; uint16_t off_ms; } *p = arg;
    cfg->blink_on_ms  = p->on_ms;
    cfg->blink_off_ms = p->off_ms;
}

static void set_flash(struct zaf_led_config *cfg, void *arg) {
    cfg->flash_duration_ms = *(uint16_t *)arg;
}

static void set_feedback(struct zaf_led_config *cfg, void *arg) {
    cfg->feedback_duration_ms = *(uint16_t *)arg;
}

int zaf_event_set_color(const uint8_t event_idx, const uint8_t sub_idx, const struct zaf_rgb color) {
    return zaf_evt_set_impl(event_idx, sub_idx, set_color_single, (void *)&color, false);
}

int zaf_event_set_color_at(const uint8_t event_idx, const uint8_t sub_idx,
                           const uint8_t color_idx, const struct zaf_rgb color) {
    if (color_idx >= CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS) {
        return -EINVAL;
    }
    struct { uint8_t idx; struct zaf_rgb color; } p = { color_idx, color };
    return zaf_evt_set_impl(event_idx, sub_idx, set_color_at, &p, false);
}

int zaf_event_set_anim(const uint8_t event_idx, const uint8_t sub_idx, const uint8_t anim) {
    return zaf_evt_set_impl(event_idx, sub_idx, set_anim, (void *)&anim, false);
}

int zaf_event_set_blink(const uint8_t event_idx, const uint8_t sub_idx, const uint16_t on_ms, const uint16_t off_ms) {
    struct { uint16_t on_ms; uint16_t off_ms; } p = { on_ms, off_ms };
    return zaf_evt_set_impl(event_idx, sub_idx, set_blink, &p, false);
}

int zaf_event_set_flash(const uint8_t event_idx, const uint8_t sub_idx, const uint16_t dur_ms) {
    return zaf_evt_set_impl(event_idx, sub_idx, set_flash, (void *)&dur_ms, false);
}

int zaf_event_set_feedback(const uint8_t event_idx, const uint8_t sub_idx, const uint16_t dur_ms) {
    return zaf_evt_set_impl(event_idx, sub_idx, set_feedback, (void *)&dur_ms, false);
}

int zaf_clear_persisted(void) {
    zaf_state.on              = true;
    zaf_state.override_active = false;
    memset(&zaf_state.override_cfg, 0, sizeof(zaf_state.override_cfg));

    zaf_state.rt_idle.valid        = false;
    zaf_state.rt_usb_conn.valid    = false;
    zaf_state.rt_usb_disconn.valid = false;
    zaf_state.rt_ble_profile.valid = false;
    zaf_state.rt_no_endpoint.valid = false;

    for (int i = 0; i < ZAF_BATT_LEVEL_COUNT; i++) {
        zaf_state.rt_batt_warn[i].valid = false;
        zaf_state.rt_batt_crit[i].valid = false;
    }
    for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
        zaf_state.rt_layers[i].valid = false;
    }

    settings_delete("argb/state");
    settings_delete("argb/evtcfg");

    return 0;
}

static void zaf_evt_activate(const struct zaf_led_config *ecfg, uint16_t *ticks_out) {
    const uint16_t ticks = ecfg ? zaf_ticks_from_ms(ecfg->flash_duration_ms) : 1;
    *ticks_out = ticks > 0 ? ticks : 1;
    zaf_tick_reset_blink();
    if (ecfg) {
        if (ecfg->animation == ZAF_ANIM_FLASH) {
            zaf_state.flash_active = true;
            zaf_state.flash_ticks = *ticks_out;
        }
        zaf_trigger_feedback(ecfg->feedback_duration_ms);
    }
}

static int zaf_event_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *batt = as_zmk_battery_state_changed(eh);
    if (batt) {
        const uint8_t prev = zaf_state.batt_level;
        const uint8_t cur  = batt->state_of_charge;
        zaf_state.prev_batt_level = prev;
        zaf_state.batt_level      = cur;

        const uint8_t warn_thresh[ZAF_BATT_LEVEL_COUNT] = {
            (uint8_t)ZRC_GET("argb/bw1", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_1),
            (uint8_t)ZRC_GET("argb/bw2", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_2),
            (uint8_t)ZRC_GET("argb/bw3", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_WARN_THRESH_3),
        };
        const uint8_t crit_thresh[ZAF_BATT_LEVEL_COUNT] = {
            (uint8_t)ZRC_GET("argb/bc1", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_1),
            (uint8_t)ZRC_GET("argb/bc2", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_2),
            (uint8_t)ZRC_GET("argb/bc3", CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_CRIT_THRESH_3),
        };

        for (int i = 0; i < ZAF_BATT_LEVEL_COUNT; i++) {
            if (prev > warn_thresh[i] && cur <= warn_thresh[i]) {
                zaf_evt_activate(zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_WARN, (uint8_t)i), &zaf_state.batt_warn_ticks[i]);
                zaf_state.batt_warn_pending[i] = true;
            }
            if (prev > crit_thresh[i] && cur <= crit_thresh[i]) {
                zaf_evt_activate(zaf_evt_eff_cfg(ZAF_EVTIDX_BATT_CRIT, (uint8_t)i), &zaf_state.batt_crit_ticks[i]);
                zaf_state.batt_crit_pending[i] = true;
            }
        }
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);
    if (act) {
        const bool now_idle = (act->state != ZMK_ACTIVITY_ACTIVE);
        if (now_idle && !zaf_state.idle) {
            const struct zaf_led_config *icfg = zaf_evt_eff_cfg(ZAF_EVTIDX_IDLE, 0);
            if (icfg != NULL) {
                zaf_tick_reset_blink();
                if (icfg->animation == ZAF_ANIM_FLASH) {
                    zaf_state.flash_active = true;
                    zaf_state.flash_ticks = zaf_ticks_from_ms(icfg->flash_duration_ms);
                }
                zaf_trigger_feedback(icfg->feedback_duration_ms);
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
            zaf_state.usb_event_pending = true;
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
            const struct zaf_led_config *lcfg = zaf_evt_eff_cfg(ZAF_EVTIDX_LAYER, l);
            if (lcfg != NULL) {
                if (lcfg->animation == ZAF_ANIM_FLASH) {
                    zaf_state.flash_active = true;
                    zaf_state.flash_ticks = zaf_ticks_from_ms(lcfg->flash_duration_ms);
                }
                zaf_trigger_feedback(lcfg->feedback_duration_ms);
            }
        }
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_ble_active_profile_changed *ble = as_zmk_ble_active_profile_changed(eh);
    if (ble) {
        zaf_state.ble_connected    = zmk_ble_active_profile_is_connected();
        zaf_eval_no_endpoint();
        zaf_state.ble_event_pending = true;
        zaf_evt_activate(zaf_evt_eff_cfg(ZAF_EVTIDX_BLE_PROFILE, 0), &zaf_state.ble_event_ticks);
        zaf_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zaf, zaf_event_listener);
ZMK_SUBSCRIPTION(zaf, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_usb_conn_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(zaf, zmk_ble_active_profile_changed);
