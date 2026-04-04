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

#include <zmk_adaptive_rgb/adaptive_rgb.h>

#include "zmk/keymap.h"

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
#include <zmk_runtime_config/runtime_config.h>
#else
#define ZRC_GET(key, default_val) (default_val)
#endif

#define DT_DRV_COMPAT zmk_adaptive_rgb

LOG_MODULE_REGISTER(zmk_adaptive_rgb, CONFIG_ZMK_LOG_LEVEL);

#define ZAR_BREATHE_STEPS    2400
#define ZAR_BATT_LEVEL_COUNT DT_INST_PROP_OR(0, battery_levels, 3)

static inline uint16_t zar_ticks_from_ms(const uint32_t ms) {
    const int32_t tick = ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_RGB_TICK_MS);
    return (tick > 0) ? (uint16_t)(ms / (uint32_t)tick) : 1;
}

struct zar_led_config {
    struct zar_rgb colors[CONFIG_ZMK_ADAPTIVE_RGB_MAX_COLORS];
    uint8_t color_count;
    uint8_t animation;
    bool persistent;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint16_t flash_duration_ms;
    uint16_t feedback_duration_ms;
};

struct zar_layer_entry {
    struct zar_led_config cfg;
    bool valid;
};

struct zar_device_config {
    const struct device *led_strip;
    const struct device *ext_power;
    uint8_t chain_length;
    bool feedback_enabled;
    uint16_t feedback_delay_ms;
    const struct gpio_dt_spec *feedback_gpio;
    const struct gpio_dt_spec *feedback_extra_gpio;
    struct zar_led_config evt_idle;
    struct zar_led_config evt_usb_conn;
    struct zar_led_config evt_usb_disconn;
    struct zar_led_config evt_ble_profile;
    struct zar_led_config evt_batt_warn[ZAR_BATT_LEVEL_COUNT];
    struct zar_led_config evt_batt_crit[ZAR_BATT_LEVEL_COUNT];
    struct zar_led_config evt_no_endpoint;
    struct zar_layer_entry dt_layers[ZMK_KEYMAP_LAYERS_LEN];
};

struct zar_evt_override {
    struct zar_led_config cfg;
    bool valid;
};

struct zar_runtime_state {
    bool on;
    bool override_active;
    struct zar_led_config override_cfg;
    struct zar_evt_override rt_layers[ZMK_KEYMAP_LAYERS_LEN];
    struct zar_evt_override rt_idle;
    struct zar_evt_override rt_usb_conn;
    struct zar_evt_override rt_usb_disconn;
    struct zar_evt_override rt_ble_profile;
    struct zar_evt_override rt_no_endpoint;
    struct zar_evt_override rt_batt_warn[ZAR_BATT_LEVEL_COUNT];
    struct zar_evt_override rt_batt_crit[ZAR_BATT_LEVEL_COUNT];
    uint8_t active_layer;
    uint8_t batt_level;
    uint8_t prev_batt_level;
    bool batt_warn_pending[ZAR_BATT_LEVEL_COUNT];
    uint16_t batt_warn_ticks[ZAR_BATT_LEVEL_COUNT];
    bool batt_crit_pending[ZAR_BATT_LEVEL_COUNT];
    uint16_t batt_crit_ticks[ZAR_BATT_LEVEL_COUNT];
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
};

struct zar_persisted {
    bool on;
    bool override_active;
    struct zar_led_config override_cfg;
};

struct zar_evt_persisted {
    struct zar_evt_override idle;
    struct zar_evt_override usb_conn;
    struct zar_evt_override usb_disconn;
    struct zar_evt_override ble_profile;
    struct zar_evt_override no_endpoint;
    struct zar_evt_override batt_warn[ZAR_BATT_LEVEL_COUNT];
    struct zar_evt_override batt_crit[ZAR_BATT_LEVEL_COUNT];
    struct zar_evt_override layers[ZMK_KEYMAP_LAYERS_LEN];
};

struct zar_dt_child {
    uint8_t event_type_idx;
    uint8_t layer_index;
    uint8_t batt_level;
    uint8_t color_bytes[CONFIG_ZMK_ADAPTIVE_RGB_MAX_COLORS * 3];
    uint8_t color_count;
    uint8_t animation;
    bool persistent;
    uint16_t blink_on_ms;
    uint16_t blink_off_ms;
    uint16_t flash_duration_ms;
    uint16_t feedback_duration_ms;
};

#define ZAR_CHILD_ENTRY(child)                                                         \
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

#define _ZAR_COUNT_CHILD(child) +1
#define ZAR_DT_CHILD_COUNT (0 DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(0), _ZAR_COUNT_CHILD))

static const struct zar_dt_child zar_dt_children[ZAR_DT_CHILD_COUNT] = {
    DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(0), ZAR_CHILD_ENTRY)
};

#if DT_INST_NODE_HAS_PROP(0, feedback_gpios)
static const struct gpio_dt_spec zar_feedback_gpio_spec = GPIO_DT_SPEC_INST_GET(0, feedback_gpios);
#endif

#if DT_INST_NODE_HAS_PROP(0, feedback_extra_gpios)
static const struct gpio_dt_spec zar_feedback_extra_gpio_spec = GPIO_DT_SPEC_INST_GET(0, feedback_extra_gpios);
#endif

static struct zar_device_config zar_config;
static struct zar_runtime_state zar_state;
static struct led_rgb zar_pixels[DT_INST_PROP(0, chain_length)];

static struct k_work_delayable zar_save_work;
static struct k_work_delayable zar_save_evt_work;

static int zar_prev_extra_gpio_state;
static uint16_t zar_pending_feedback_duration_ms;
static struct k_work_delayable zar_feedback_on_work;
static struct k_work_delayable zar_feedback_off_work;

static void zar_fill(const struct zar_rgb color) {
    const int32_t bri = ZRC_GET("argb/brt", CONFIG_ZMK_ADAPTIVE_RGB_BRIGHTNESS);
    const struct led_rgb c = {
        .r = (uint8_t)((color.r * bri) / 100),
        .g = (uint8_t)((color.g * bri) / 100),
        .b = (uint8_t)((color.b * bri) / 100),
    };
    for (int i = 0; i < zar_config.chain_length; i++) {
        zar_pixels[i] = c;
    }
}

static void zar_clear_pixels(void) {
    const struct zar_rgb off = {0, 0, 0};
    zar_fill(off);
    ext_power_disable(zar_config.ext_power);
}

static struct zar_evt_override *zar_evt_rt_slot(uint8_t event_idx, uint8_t sub_idx);
static const struct zar_led_config *zar_evt_eff_cfg(uint8_t event_idx, uint8_t sub_idx);

static const struct zar_led_config *zar_resolve(void) {
    if (zar_state.override_active) {
        return &zar_state.override_cfg;
    }

    for (int i = ZAR_BATT_LEVEL_COUNT - 1; i >= 0; i--) {
        if (zar_state.batt_crit_pending[i]) {
            return zar_evt_eff_cfg(ZAR_EVTIDX_BATT_CRIT, (uint8_t)i);
        }
    }
    for (int i = ZAR_BATT_LEVEL_COUNT - 1; i >= 0; i--) {
        if (zar_state.batt_warn_pending[i]) {
            return zar_evt_eff_cfg(ZAR_EVTIDX_BATT_WARN, (uint8_t)i);
        }
    }

    if (zar_state.usb_event_pending) {
        return zar_evt_eff_cfg(zar_state.usb_connected
                               ? ZAR_EVTIDX_USB_CONN : ZAR_EVTIDX_USB_DISCONN, 0);
    }

    if (zar_state.ble_event_pending) {
        return zar_evt_eff_cfg(ZAR_EVTIDX_BLE_PROFILE, 0);
    }

    if (zar_state.no_endpoint) {
        return zar_evt_eff_cfg(ZAR_EVTIDX_NO_ENDPOINT, 0);
    }

    const uint8_t layer = zar_state.active_layer;
    if (layer < ZMK_KEYMAP_LAYERS_LEN) {
        const struct zar_led_config *lcfg = zar_evt_eff_cfg(ZAR_EVTIDX_LAYER, layer);
        if (lcfg != NULL && lcfg->persistent) {
            return lcfg;
        }
    }

    if (zar_state.idle) {
        return zar_evt_eff_cfg(ZAR_EVTIDX_IDLE, 0);
    }

    return NULL;
}

static void zar_apply_animation(const struct zar_led_config *cfg) {
    if (cfg->color_count == 0) {
        zar_clear_pixels();
        return;
    }

    const struct zar_rgb c0 = cfg->colors[0];
    const struct zar_rgb off = {0, 0, 0};
    const struct zar_rgb c1 = (cfg->color_count > 1) ? cfg->colors[1] : off;

    switch (cfg->animation) {
    case ZAR_ANIM_SOLID:
        zar_fill(c0);
        break;

    case ZAR_ANIM_BLINK: {
        const uint16_t phase_ms = zar_state.blink_phase ? cfg->blink_on_ms : cfg->blink_off_ms;
        const uint16_t phase_ticks = zar_ticks_from_ms(phase_ms);
        zar_fill(zar_state.blink_phase ? c0 : c1);
        zar_state.blink_phase_ticks++;
        if (zar_state.blink_phase_ticks >= phase_ticks) {
            zar_state.blink_phase = !zar_state.blink_phase;
            zar_state.blink_phase_ticks = 0;
        }
        break;
    }

    case ZAR_ANIM_BREATHE: {
        const int step = (int)zar_state.anim_step - 1200;
        const uint8_t scale = (uint8_t)((step < 0 ? -step : step) / 12);
        const uint8_t idx = zar_state.color_idx % cfg->color_count;
        const struct zar_rgb base = cfg->colors[idx];
        const struct zar_rgb bc = {
            .r = (uint8_t)((base.r * scale) / 100),
            .g = (uint8_t)((base.g * scale) / 100),
            .b = (uint8_t)((base.b * scale) / 100),
        };
        zar_fill(bc);
        zar_state.anim_step += 10;
        if (zar_state.anim_step > ZAR_BREATHE_STEPS) {
            zar_state.anim_step = 0;
            zar_state.color_idx = (zar_state.color_idx + 1) % cfg->color_count;
        }
        break;
    }

    case ZAR_ANIM_FLASH:
        zar_fill(c0);
        break;
    default: break;
    }
}

static void zar_feedback_off_work_fn(struct k_work *work) {
    if (zar_config.feedback_gpio != NULL) {
        gpio_pin_set_dt(zar_config.feedback_gpio, 0);
    }
    if (zar_config.feedback_extra_gpio != NULL) {
        gpio_pin_set_dt(zar_config.feedback_extra_gpio, zar_prev_extra_gpio_state);
    }
}

static void zar_feedback_on_work_fn(struct k_work *work) {
    if (zar_config.feedback_gpio == NULL) {
        return;
    }
    gpio_pin_set_dt(zar_config.feedback_gpio, 1);
    k_work_reschedule(&zar_feedback_off_work, K_MSEC(zar_pending_feedback_duration_ms));
}

static void zar_trigger_feedback(const uint16_t duration_ms) {
    if (!zar_config.feedback_enabled || zar_config.feedback_gpio == NULL || duration_ms == 0) {
        return;
    }
    if (zar_config.feedback_extra_gpio != NULL) {
        zar_prev_extra_gpio_state = gpio_pin_get_dt(zar_config.feedback_extra_gpio);
        gpio_pin_set_dt(zar_config.feedback_extra_gpio, 1);
    }
    if (zar_config.feedback_delay_ms > 0) {
        zar_pending_feedback_duration_ms = duration_ms;
        k_work_reschedule(&zar_feedback_on_work, K_MSEC(zar_config.feedback_delay_ms));
    } else {
        gpio_pin_set_dt(zar_config.feedback_gpio, 1);
        k_work_reschedule(&zar_feedback_off_work, K_MSEC(duration_ms));
    }
}

extern struct k_timer zar_timer;

static void zar_tick(struct k_work *work) {
    if (!zar_state.on) {
        return;
    }

    if (zar_state.usb_event_pending && zar_state.usb_event_ticks > 0) {
        zar_state.usb_event_ticks--;
        if (zar_state.usb_event_ticks == 0) {
            zar_state.usb_event_pending = false;
            zar_state.blink_phase = false;
            zar_state.blink_phase_ticks = 0;
        }
    }

    if (zar_state.ble_event_pending && zar_state.ble_event_ticks > 0) {
        zar_state.ble_event_ticks--;
        if (zar_state.ble_event_ticks == 0) {
            zar_state.ble_event_pending = false;
            zar_state.blink_phase = false;
            zar_state.blink_phase_ticks = 0;
        }
    }

    for (int i = 0; i < 3; i++) {
        if (zar_state.batt_warn_pending[i] && zar_state.batt_warn_ticks[i] > 0) {
            zar_state.batt_warn_ticks[i]--;
            if (zar_state.batt_warn_ticks[i] == 0) {
                zar_state.batt_warn_pending[i] = false;
                zar_state.blink_phase = false;
                zar_state.blink_phase_ticks = 0;
            }
        }
        if (zar_state.batt_crit_pending[i] && zar_state.batt_crit_ticks[i] > 0) {
            zar_state.batt_crit_ticks[i]--;
            if (zar_state.batt_crit_ticks[i] == 0) {
                zar_state.batt_crit_pending[i] = false;
                zar_state.blink_phase = false;
                zar_state.blink_phase_ticks = 0;
            }
        }
    }

    const struct zar_led_config *cfg = zar_resolve();
    bool output_is_static_dark = false;

    if (cfg == NULL) {
        zar_clear_pixels();
        output_is_static_dark = true;
    } else {
        zar_apply_animation(cfg);
        bool all_zero = true;
        for (int i = 0; i < zar_config.chain_length; i++) {
            if (zar_pixels[i].r || zar_pixels[i].g || zar_pixels[i].b) {
                all_zero = false;
                break;
            }
        }
        if (all_zero) {
            ext_power_disable(zar_config.ext_power);
            if (cfg->animation == ZAR_ANIM_SOLID) {
                output_is_static_dark = true;
            }
        } else {
            ext_power_enable(zar_config.ext_power);
        }
    }

    const int err = led_strip_update_rgb(zar_config.led_strip, zar_pixels, zar_config.chain_length);
    if (err < 0) {
        LOG_ERR("led_strip_update_rgb failed: %d", err);
    }

    if (output_is_static_dark) {
        k_timer_stop(&zar_timer);
    }
}

K_WORK_DEFINE(zar_tick_work, zar_tick);

static void zar_timer_handler(struct k_timer *timer) {
    if (!zar_state.on) {
        return;
    }
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &zar_tick_work);
}

K_TIMER_DEFINE(zar_timer, zar_timer_handler, NULL);

static void zar_off_work_fn(struct k_work *work) {
    zar_clear_pixels();
    led_strip_update_rgb(zar_config.led_strip, zar_pixels, zar_config.chain_length);
}

K_WORK_DEFINE(zar_off_work, zar_off_work_fn);

static void zar_kick_timer(void) {
    if (!zar_state.on) {
        return;
    }
    const uint32_t tick_ms = (uint32_t)ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_RGB_TICK_MS);
    k_timer_start(&zar_timer, K_NO_WAIT, K_MSEC(tick_ms));
}

static void zar_save_work_fn(struct k_work *work) {
    const struct zar_persisted p = {
        .on              = zar_state.on,
        .override_active = zar_state.override_active,
        .override_cfg    = zar_state.override_cfg,
    };
    settings_save_one("argb/state", &p, sizeof(p));
}

static void zar_save_evt_work_fn(struct k_work *work) {
    static struct zar_evt_persisted p;
    p = (struct zar_evt_persisted) {
        .idle        = zar_state.rt_idle,
        .usb_conn    = zar_state.rt_usb_conn,
        .usb_disconn = zar_state.rt_usb_disconn,
        .ble_profile = zar_state.rt_ble_profile,
        .no_endpoint = zar_state.rt_no_endpoint,
    };
    for (int i = 0; i < ZAR_BATT_LEVEL_COUNT; i++) {
        p.batt_warn[i] = zar_state.rt_batt_warn[i];
        p.batt_crit[i] = zar_state.rt_batt_crit[i];
    }
    for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
        p.layers[i] = zar_state.rt_layers[i];
    }
    settings_save_one("argb/evtcfg", &p, sizeof(p));
}

static int zar_settings_set(const char *name, const size_t len, const settings_read_cb read_cb, void *cb_arg) {
    const char *next;

    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(struct zar_persisted)) {
            return -EINVAL;
        }
        struct zar_persisted p;
        const int rc = read_cb(cb_arg, &p, sizeof(p));
        if (rc < 0) {
            return rc;
        }
        zar_state.on              = p.on;
        zar_state.override_active = p.override_active;
        zar_state.override_cfg    = p.override_cfg;
        return 0;
    }

    if (settings_name_steq(name, "evtcfg", &next) && !next) {
        if (len != sizeof(struct zar_evt_persisted)) {
            return -EINVAL;
        }
        static struct zar_evt_persisted p;
        const int rc = read_cb(cb_arg, &p, sizeof(p));
        if (rc < 0) {
            return rc;
        }
        zar_state.rt_idle        = p.idle;
        zar_state.rt_usb_conn    = p.usb_conn;
        zar_state.rt_usb_disconn = p.usb_disconn;
        zar_state.rt_ble_profile = p.ble_profile;
        zar_state.rt_no_endpoint = p.no_endpoint;
        for (int i = 0; i < ZAR_BATT_LEVEL_COUNT; i++) {
            zar_state.rt_batt_warn[i] = p.batt_warn[i];
            zar_state.rt_batt_crit[i] = p.batt_crit[i];
        }
        for (int i = 0; i < ZMK_KEYMAP_LAYERS_LEN; i++) {
            zar_state.rt_layers[i] = p.layers[i];
        }
        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(zar_settings, "argb", NULL, zar_settings_set, NULL, NULL);

static int zar_save_state(void) {
    const int ret = k_work_reschedule(&zar_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
}

static int zar_save_evt_cfg(void) {
    const int ret = k_work_reschedule(&zar_save_evt_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
}

static void zar_apply_dt_children(void) {
    for (uint8_t i = 0; i < ZAR_DT_CHILD_COUNT; i++) {
        const struct zar_dt_child *c = &zar_dt_children[i];
        struct zar_led_config cfg = {
            .animation            = c->animation,
            .persistent           = c->persistent,
            .blink_on_ms          = c->blink_on_ms,
            .blink_off_ms         = c->blink_off_ms,
            .flash_duration_ms    = c->flash_duration_ms,
            .feedback_duration_ms = c->feedback_duration_ms,
            .color_count          = MIN(c->color_count, CONFIG_ZMK_ADAPTIVE_RGB_MAX_COLORS),
        };
        for (uint8_t j = 0; j < c->color_count && j < CONFIG_ZMK_ADAPTIVE_RGB_MAX_COLORS; j++) {
            cfg.colors[j].r = c->color_bytes[j * 3];
            cfg.colors[j].g = c->color_bytes[j * 3 + 1];
            cfg.colors[j].b = c->color_bytes[j * 3 + 2];
        }
        switch (c->event_type_idx) {
        case ZAR_EVTIDX_LAYER:
            if (c->layer_index < ZMK_KEYMAP_LAYERS_LEN) {
                zar_config.dt_layers[c->layer_index].cfg   = cfg;
                zar_config.dt_layers[c->layer_index].valid = true;
            }
            break;
        case ZAR_EVTIDX_BATT_WARN:
            if (c->batt_level >= 1 && c->batt_level <= 3) {
                zar_config.evt_batt_warn[c->batt_level - 1] = cfg;
            }
            break;
        case ZAR_EVTIDX_BATT_CRIT:
            if (c->batt_level >= 1 && c->batt_level <= 3) {
                zar_config.evt_batt_crit[c->batt_level - 1] = cfg;
            }
            break;
        case ZAR_EVTIDX_USB_CONN:
            zar_config.evt_usb_conn = cfg;
            break;
        case ZAR_EVTIDX_USB_DISCONN:
            zar_config.evt_usb_disconn = cfg;
            break;
        case ZAR_EVTIDX_BLE_PROFILE:
            zar_config.evt_ble_profile = cfg;
            break;
        case ZAR_EVTIDX_IDLE:
            zar_config.evt_idle = cfg;
            break;
        case ZAR_EVTIDX_NO_ENDPOINT:
            zar_config.evt_no_endpoint = cfg;
            break;
        default: break;
        }
    }
}

static void zar_eval_no_endpoint(void) {
    zar_state.no_endpoint = !zar_state.usb_connected && !zar_state.ble_connected;
}

static int zar_init(void) {
    zar_config.led_strip    = DEVICE_DT_GET(DT_INST_PHANDLE(0, led_strip));
    zar_config.chain_length = DT_INST_PROP(0, chain_length);

#if DT_INST_NODE_HAS_PROP(0, ext_power)
    zar_config.ext_power = DEVICE_DT_GET(DT_INST_PHANDLE(0, ext_power));
    if (!device_is_ready(zar_config.ext_power)) {
        LOG_WRN("ext-power device not ready");
        zar_config.ext_power = NULL;
    }
#else
    zar_config.ext_power = NULL;
#endif

    if (!device_is_ready(zar_config.led_strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    zar_apply_dt_children();

    zar_state.on           = true;
    zar_state.batt_level   = 100;
    zar_state.active_layer = 0;

    k_work_init_delayable(&zar_save_work, zar_save_work_fn);
    k_work_init_delayable(&zar_save_evt_work, zar_save_evt_work_fn);
    k_work_init_delayable(&zar_feedback_on_work, zar_feedback_on_work_fn);
    k_work_init_delayable(&zar_feedback_off_work, zar_feedback_off_work_fn);

    zar_config.feedback_enabled  = DT_INST_PROP(0, feedback_enabled);
    zar_config.feedback_delay_ms = DT_INST_PROP_OR(0, feedback_delay, 0);

#if DT_INST_NODE_HAS_PROP(0, feedback_gpios)
    zar_config.feedback_gpio = &zar_feedback_gpio_spec;
    if (device_is_ready(zar_feedback_gpio_spec.port)) {
        gpio_pin_configure_dt(&zar_feedback_gpio_spec, GPIO_OUTPUT);
    } else {
        zar_config.feedback_gpio = NULL;
    }
#endif

#if DT_INST_NODE_HAS_PROP(0, feedback_extra_gpios)
    zar_config.feedback_extra_gpio = &zar_feedback_extra_gpio_spec;
    if (device_is_ready(zar_feedback_extra_gpio_spec.port)) {
        gpio_pin_configure_dt(&zar_feedback_extra_gpio_spec, GPIO_OUTPUT);
    } else {
        zar_config.feedback_extra_gpio = NULL;
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
    zrc_register("argb/bw1",  CONFIG_ZMK_ADAPTIVE_RGB_BATT_WARN_THRESH_1, 0,  100);
    zrc_register("argb/bw2",  CONFIG_ZMK_ADAPTIVE_RGB_BATT_WARN_THRESH_2, 0,  100);
    zrc_register("argb/bw3",  CONFIG_ZMK_ADAPTIVE_RGB_BATT_WARN_THRESH_3, 0,  100);
    zrc_register("argb/bc1",  CONFIG_ZMK_ADAPTIVE_RGB_BATT_CRIT_THRESH_1, 0,  100);
    zrc_register("argb/bc2",  CONFIG_ZMK_ADAPTIVE_RGB_BATT_CRIT_THRESH_2, 0,  100);
    zrc_register("argb/bc3",  CONFIG_ZMK_ADAPTIVE_RGB_BATT_CRIT_THRESH_3, 0,  100);
    zrc_register("argb/tick", CONFIG_ZMK_ADAPTIVE_RGB_TICK_MS,            16,  500);
    zrc_register("argb/brt",  CONFIG_ZMK_ADAPTIVE_RGB_BRIGHTNESS,         0,   100);
#endif

    settings_load_subtree("argb");
    if (zar_state.on) {
        if (zar_config.ext_power != NULL) {
            ext_power_enable(zar_config.ext_power);
        }
        k_timer_start(&zar_timer, K_NO_WAIT, K_MSEC(CONFIG_ZMK_ADAPTIVE_RGB_TICK_MS));
    }

    return 0;
}

SYS_INIT(zar_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int zar_on(void) {
    if (zar_state.on) {
        return 0;
    }

    if (zar_config.ext_power != NULL) {
        ext_power_enable(zar_config.ext_power);
    }

    zar_state.on = true;
    zar_state.anim_step = 0;
    zar_state.color_idx = 0;
    k_timer_start(&zar_timer, K_NO_WAIT, K_MSEC((uint32_t)ZRC_GET("argb/tick", CONFIG_ZMK_ADAPTIVE_RGB_TICK_MS)));
    return zar_save_state();
}

int zar_off(void) {
    if (!zar_state.on) {
        return 0;
    }

    k_timer_stop(&zar_timer);
    zar_state.on = false;

    if (zar_config.ext_power != NULL) {
        ext_power_disable(zar_config.ext_power);
    }

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &zar_off_work);
    return zar_save_state();
}

bool zar_is_on(void) {
    return zar_state.on;
}

uint8_t zar_layer_count(void) {
    return (uint8_t)ZMK_KEYMAP_LAYERS_LEN;
}

uint8_t zar_batt_level_count(void) {
    return (uint8_t)ZAR_BATT_LEVEL_COUNT;
}

/* Returns the runtime override slot for this event (write path). */
static struct zar_evt_override *zar_evt_rt_slot(const uint8_t event_idx, const uint8_t sub_idx) {
    switch (event_idx) {
    case ZAR_EVTIDX_LAYER:
        if (sub_idx < ZMK_KEYMAP_LAYERS_LEN) return &zar_state.rt_layers[sub_idx];
        return NULL;
    case ZAR_EVTIDX_BATT_WARN:
        if (sub_idx < ZAR_BATT_LEVEL_COUNT) return &zar_state.rt_batt_warn[sub_idx];
        return NULL;
    case ZAR_EVTIDX_BATT_CRIT:
        if (sub_idx < ZAR_BATT_LEVEL_COUNT) return &zar_state.rt_batt_crit[sub_idx];
        return NULL;
    case ZAR_EVTIDX_USB_CONN:    return &zar_state.rt_usb_conn;
    case ZAR_EVTIDX_USB_DISCONN: return &zar_state.rt_usb_disconn;
    case ZAR_EVTIDX_BLE_PROFILE: return &zar_state.rt_ble_profile;
    case ZAR_EVTIDX_IDLE:        return &zar_state.rt_idle;
    case ZAR_EVTIDX_NO_ENDPOINT: return &zar_state.rt_no_endpoint;
    default:                      return NULL;
    }
}

/* Returns the effective config: runtime override if set, else DTS default (read path). */
static const struct zar_led_config *zar_evt_eff_cfg(const uint8_t event_idx, const uint8_t sub_idx) {
    switch (event_idx) {
    case ZAR_EVTIDX_LAYER:
        if (sub_idx < ZMK_KEYMAP_LAYERS_LEN) {
            if (zar_state.rt_layers[sub_idx].valid) return &zar_state.rt_layers[sub_idx].cfg;
            return zar_config.dt_layers[sub_idx].valid ? &zar_config.dt_layers[sub_idx].cfg : NULL;
        }
        return NULL;
    case ZAR_EVTIDX_BATT_WARN:
        if (sub_idx < ZAR_BATT_LEVEL_COUNT) {
            if (zar_state.rt_batt_warn[sub_idx].valid) return &zar_state.rt_batt_warn[sub_idx].cfg;
            return &zar_config.evt_batt_warn[sub_idx];
        }
        return NULL;
    case ZAR_EVTIDX_BATT_CRIT:
        if (sub_idx < ZAR_BATT_LEVEL_COUNT) {
            if (zar_state.rt_batt_crit[sub_idx].valid) return &zar_state.rt_batt_crit[sub_idx].cfg;
            return &zar_config.evt_batt_crit[sub_idx];
        }
        return NULL;
    case ZAR_EVTIDX_USB_CONN:
        return zar_state.rt_usb_conn.valid ? &zar_state.rt_usb_conn.cfg : &zar_config.evt_usb_conn;
    case ZAR_EVTIDX_USB_DISCONN:
        return zar_state.rt_usb_disconn.valid ? &zar_state.rt_usb_disconn.cfg : &zar_config.evt_usb_disconn;
    case ZAR_EVTIDX_BLE_PROFILE:
        return zar_state.rt_ble_profile.valid ? &zar_state.rt_ble_profile.cfg : &zar_config.evt_ble_profile;
    case ZAR_EVTIDX_IDLE:
        return zar_state.rt_idle.valid ? &zar_state.rt_idle.cfg : &zar_config.evt_idle;
    case ZAR_EVTIDX_NO_ENDPOINT:
        return zar_state.rt_no_endpoint.valid ? &zar_state.rt_no_endpoint.cfg : &zar_config.evt_no_endpoint;
    default:
        return NULL;
    }
}

int zar_event_get(const uint8_t event_idx, const uint8_t sub_idx, struct zar_event_info *out) {
    const struct zar_led_config *cfg = zar_evt_eff_cfg(event_idx, sub_idx);
    if (!cfg) {
        return -EINVAL;
    }
    const uint8_t n = MIN(cfg->color_count, ZAR_INFO_MAX_COLORS);
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

int zar_event_set_color(const uint8_t event_idx, const uint8_t sub_idx, const struct zar_rgb color) {
    struct zar_evt_override *slot = zar_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    slot->cfg.colors[0]   = color;
    slot->cfg.color_count = 1;
    slot->valid           = true;
    if (event_idx == ZAR_EVTIDX_LAYER) {
        slot->cfg.persistent = true;
    }
    zar_kick_timer();
    return zar_save_evt_cfg();
}

int zar_event_set_color_at(const uint8_t event_idx, const uint8_t sub_idx,
                           const uint8_t color_idx, const struct zar_rgb color) {
    if (color_idx >= CONFIG_ZMK_ADAPTIVE_RGB_MAX_COLORS) {
        return -EINVAL;
    }
    struct zar_evt_override *slot = zar_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zar_led_config *dts = zar_evt_eff_cfg(event_idx, sub_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    slot->cfg.colors[color_idx] = color;
    if (color_idx >= slot->cfg.color_count) {
        slot->cfg.color_count = color_idx + 1;
    }
    slot->valid = true;
    if (event_idx == ZAR_EVTIDX_LAYER) {
        slot->cfg.persistent = true;
    }
    zar_kick_timer();
    return zar_save_evt_cfg();
}

int zar_event_set_anim(const uint8_t event_idx, const uint8_t sub_idx, const uint8_t anim) {
    struct zar_evt_override *slot = zar_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zar_led_config *dts = zar_evt_eff_cfg(event_idx, sub_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    slot->cfg.animation = anim;
    slot->valid         = true;
    zar_kick_timer();
    return zar_save_evt_cfg();
}

int zar_event_set_blink(const uint8_t event_idx, const uint8_t sub_idx, const uint16_t on_ms, const uint16_t off_ms) {
    struct zar_evt_override *slot = zar_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zar_led_config *dts = zar_evt_eff_cfg(event_idx, sub_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    slot->cfg.blink_on_ms  = on_ms;
    slot->cfg.blink_off_ms = off_ms;
    slot->valid            = true;
    zar_kick_timer();
    return zar_save_evt_cfg();
}

int zar_event_set_flash(const uint8_t event_idx, const uint8_t sub_idx, const uint16_t dur_ms) {
    struct zar_evt_override *slot = zar_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zar_led_config *dts = zar_evt_eff_cfg(event_idx, sub_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    slot->cfg.flash_duration_ms = dur_ms;
    slot->valid                 = true;
    zar_kick_timer();
    return zar_save_evt_cfg();
}

int zar_event_set_feedback(const uint8_t event_idx, const uint8_t sub_idx, const uint16_t dur_ms) {
    struct zar_evt_override *slot = zar_evt_rt_slot(event_idx, sub_idx);
    if (!slot) {
        return -EINVAL;
    }
    if (!slot->valid) {
        const struct zar_led_config *dts = zar_evt_eff_cfg(event_idx, sub_idx);
        if (dts) {
            slot->cfg = *dts;
        }
    }
    slot->cfg.feedback_duration_ms = dur_ms;
    slot->valid                    = true;
    zar_kick_timer();
    return zar_save_evt_cfg();
}

static int zar_event_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *batt = as_zmk_battery_state_changed(eh);
    if (batt) {
        const uint8_t prev = zar_state.batt_level;
        const uint8_t cur  = batt->state_of_charge;
        zar_state.prev_batt_level = prev;
        zar_state.batt_level      = cur;

        const uint8_t warn_thresh[ZAR_BATT_LEVEL_COUNT] = {
            (uint8_t)ZRC_GET("argb/bw1", CONFIG_ZMK_ADAPTIVE_RGB_BATT_WARN_THRESH_1),
            (uint8_t)ZRC_GET("argb/bw2", CONFIG_ZMK_ADAPTIVE_RGB_BATT_WARN_THRESH_2),
            (uint8_t)ZRC_GET("argb/bw3", CONFIG_ZMK_ADAPTIVE_RGB_BATT_WARN_THRESH_3),
        };
        const uint8_t crit_thresh[ZAR_BATT_LEVEL_COUNT] = {
            (uint8_t)ZRC_GET("argb/bc1", CONFIG_ZMK_ADAPTIVE_RGB_BATT_CRIT_THRESH_1),
            (uint8_t)ZRC_GET("argb/bc2", CONFIG_ZMK_ADAPTIVE_RGB_BATT_CRIT_THRESH_2),
            (uint8_t)ZRC_GET("argb/bc3", CONFIG_ZMK_ADAPTIVE_RGB_BATT_CRIT_THRESH_3),
        };

        for (int i = 0; i < ZAR_BATT_LEVEL_COUNT; i++) {
            if (prev > warn_thresh[i] && cur <= warn_thresh[i]) {
                const struct zar_led_config *ecfg = zar_evt_eff_cfg(ZAR_EVTIDX_BATT_WARN, (uint8_t)i);
                const uint16_t ticks = ecfg ? zar_ticks_from_ms(ecfg->flash_duration_ms) : 1;
                zar_state.batt_warn_ticks[i]   = ticks > 0 ? ticks : 1;
                zar_state.batt_warn_pending[i] = true;
                if (ecfg) zar_trigger_feedback(ecfg->feedback_duration_ms);
            }
            if (prev > crit_thresh[i] && cur <= crit_thresh[i]) {
                const struct zar_led_config *ecfg = zar_evt_eff_cfg(ZAR_EVTIDX_BATT_CRIT, (uint8_t)i);
                const uint16_t ticks = ecfg ? zar_ticks_from_ms(ecfg->flash_duration_ms) : 1;
                zar_state.batt_crit_ticks[i]   = ticks > 0 ? ticks : 1;
                zar_state.batt_crit_pending[i] = true;
                if (ecfg) zar_trigger_feedback(ecfg->feedback_duration_ms);
            }
        }
        zar_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_activity_state_changed *act = as_zmk_activity_state_changed(eh);
    if (act) {
        zar_state.idle = (act->state != ZMK_ACTIVITY_ACTIVE);
        zar_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb) {
        const bool now_connected = (usb->conn_state != ZMK_USB_CONN_NONE);
        if (now_connected != zar_state.usb_connected) {
            zar_state.usb_connected     = now_connected;
            zar_eval_no_endpoint();
            zar_state.usb_event_pending = true;
            const struct zar_led_config *ecfg = zar_evt_eff_cfg(
                now_connected ? ZAR_EVTIDX_USB_CONN : ZAR_EVTIDX_USB_DISCONN, 0);
            const uint16_t ticks = ecfg ? zar_ticks_from_ms(ecfg->flash_duration_ms) : 1;
            zar_state.usb_event_ticks   = ticks > 0 ? ticks : 1;
            zar_state.blink_phase       = false;
            zar_state.blink_phase_ticks = 0;
            if (ecfg) zar_trigger_feedback(ecfg->feedback_duration_ms);
        }
        zar_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_layer_state_changed *layer = as_zmk_layer_state_changed(eh);
    if (layer) {
        const uint8_t l = layer->state
            ? layer->layer
            : (uint8_t)zmk_keymap_highest_layer_active();
        zar_state.active_layer      = l;
        zar_state.anim_step         = 0;
        zar_state.color_idx         = 0;
        zar_state.blink_phase       = false;
        zar_state.blink_phase_ticks = 0;
        if (layer->state && l < ZMK_KEYMAP_LAYERS_LEN) {
            const struct zar_led_config *lcfg = zar_evt_eff_cfg(ZAR_EVTIDX_LAYER, l);
            if (lcfg != NULL) {
                zar_trigger_feedback(lcfg->feedback_duration_ms);
            }
        }
        zar_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_ble_active_profile_changed *ble = as_zmk_ble_active_profile_changed(eh);
    if (ble) {
        zar_state.ble_connected = zmk_ble_active_profile_is_connected();
        zar_eval_no_endpoint();
        zar_state.ble_event_pending = true;
        const struct zar_led_config *ecfg = zar_evt_eff_cfg(ZAR_EVTIDX_BLE_PROFILE, 0);
        const uint16_t ticks = ecfg ? zar_ticks_from_ms(ecfg->flash_duration_ms) : 1;
        zar_state.ble_event_ticks   = ticks > 0 ? ticks : 1;
        zar_state.blink_phase       = false;
        zar_state.blink_phase_ticks = 0;
        if (ecfg) zar_trigger_feedback(ecfg->feedback_duration_ms);
        zar_kick_timer();
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zar, zar_event_listener);
ZMK_SUBSCRIPTION(zar, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(zar, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(zar, zmk_usb_conn_state_changed);
ZMK_SUBSCRIPTION(zar, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(zar, zmk_ble_active_profile_changed);
