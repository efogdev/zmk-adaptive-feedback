#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

#include <zmk_adaptive_feedback/adaptive_feedback.h>

LOG_MODULE_DECLARE(zmk_adaptive_feedback, CONFIG_ZMK_LOG_LEVEL);

#define shprint(_sh, _fmt, ...)                      \
    do {                                             \
        if ((_sh) != NULL)                           \
            shell_print((_sh), _fmt, ##__VA_ARGS__); \
    } while (0)

static const char * const zaf_anim_names[] = {"solid", "blink", "breathe", "flash"};

static int parse_anim(const char *s, uint8_t *out) {
    for (uint8_t i = 0; i < ARRAY_SIZE(zaf_anim_names); i++) {
        if (strcmp(s, zaf_anim_names[i]) == 0) {
            *out = i;
            return 0;
        }
    }
    return -EINVAL;
}

static const char * const zaf_ease_fn_names[] = {
    "linear",
    "quad-in", "quad-out", "quad-in-out",
    "cubic-in", "cubic-out", "cubic-in-out",
    "quart-in", "quart-out", "quart-in-out",
    "expo-in", "expo-out",
    "bounce-out", "bounce-in",
};

static int parse_ease_fn(const char *s, uint8_t *out) {
    for (uint8_t i = 0; i < ARRAY_SIZE(zaf_ease_fn_names); i++) {
        if (strcmp(s, zaf_ease_fn_names[i]) == 0) {
            *out = i;
            return 0;
        }
    }
    return -EINVAL;
}

static int parse_event_spec(const struct shell *sh,
                             const int argc, char **argv,
                             uint8_t *event_idx, uint8_t *sub_idx,
                             int *consumed,
                             struct zaf_custom_event **custom_out) {
    if (argc < 1) {
        shprint(sh, "missing event type");
        return -EINVAL;
    }

    const char *type = argv[0];
    *sub_idx    = 0;
    *consumed   = 1;
    *custom_out = NULL;

    if (strcmp(type, "layer") == 0) {
        if (argc < 2) {
            shprint(sh, "layer requires an index");

            return -EINVAL;
        }
        *event_idx = ZAF_EVTIDX_LAYER;
        *sub_idx   = (uint8_t)strtol(argv[1], NULL, 10);
        *consumed  = 2;
    } else if (strcmp(type, "batt-warn") == 0) {
        if (argc < 2) {
            shprint(sh, "batt-warn requires level 1-3");
            return -EINVAL;
        }
        const int lvl = (int)strtol(argv[1], NULL, 10);
        if (lvl < 1 || lvl > 3) {
            shprint(sh, "batt-warn level must be 1-3");
            return -EINVAL;
        }
        *event_idx = ZAF_EVTIDX_BATT_WARN;
        *sub_idx   = (uint8_t)(lvl - 1);
        *consumed  = 2;
    } else if (strcmp(type, "batt-crit") == 0) {
        if (argc < 2) {
            shprint(sh, "batt-crit requires level 1-3");
            return -EINVAL;
        }
        const int lvl = (int)strtol(argv[1], NULL, 10);
        if (lvl < 1 || lvl > 3) {
            shprint(sh, "batt-crit level must be 1-3");
            return -EINVAL;
        }
        *event_idx = ZAF_EVTIDX_BATT_CRIT;
        *sub_idx   = (uint8_t)(lvl - 1);
        *consumed  = 2;
    } else if (strcmp(type, "idle") == 0) {
        *event_idx = ZAF_EVTIDX_IDLE;
    } else if (strcmp(type, "usb-conn") == 0) {
        *event_idx = ZAF_EVTIDX_USB_CONN;
    } else if (strcmp(type, "usb-disconn") == 0) {
        *event_idx = ZAF_EVTIDX_USB_DISCONN;
    } else if (strcmp(type, "ble-profile") == 0) {
        *event_idx = ZAF_EVTIDX_BLE_PROFILE;
    } else if (strcmp(type, "no-endpoint") == 0) {
        *event_idx = ZAF_EVTIDX_NO_ENDPOINT;
    } else if (strcmp(type, "studio-unlock") == 0) {
        *event_idx = ZAF_EVTIDX_STUDIO_UNLOCK;
    } else if (strcmp(type, "studio-lock") == 0) {
        *event_idx = ZAF_EVTIDX_STUDIO_LOCK;
    } else {
        STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
            if (strcmp(type, cevt->name) == 0) {
                *custom_out = cevt;
                return 0;
            }
        }
        shprint(sh, "unknown event type '%s'", type);
        return -EINVAL;
    }

    return 0;
}

static void print_event_info(const struct shell *sh, const struct zaf_event_info *info) {
    shprint(sh, "  anim:       %s",
            info->anim < ARRAY_SIZE(zaf_anim_names) ? zaf_anim_names[info->anim] : "?");
    shprint(sh, "  colors:     %d", info->color_count);
    for (uint8_t i = 0; i < info->color_count; i++) {
        shprint(sh, "    [%d] r=%-3d g=%-3d b=%-3d",
                i, info->colors[i].r, info->colors[i].g, info->colors[i].b);
    }
    shprint(sh, "  blink:      on=%dms off=%dms", info->blink_on_ms, info->blink_off_ms);
    shprint(sh, "  flash-dur:  %dms", info->flash_dur_ms);
    shprint(sh, "  flash-ease-in:  %dms fn=%s", info->flash_ease_in_ms,
            info->flash_ease_in_fn < ARRAY_SIZE(zaf_ease_fn_names)
                ? zaf_ease_fn_names[info->flash_ease_in_fn] : "?");
    shprint(sh, "  flash-ease-out: %dms fn=%s", info->flash_ease_out_ms,
            info->flash_ease_out_fn < ARRAY_SIZE(zaf_ease_fn_names)
                ? zaf_ease_fn_names[info->flash_ease_out_fn] : "?");
    shprint(sh, "  feedback:   %dms", info->feedback_dur_ms);
    shprint(sh, "  persistent: %s", info->persistent ? "yes" : "no");
}

static int cmd_evt_parse_color(const struct shell *sh, const int nvals, char **vals,
                                uint8_t *idx_out, struct zaf_rgb *color_out,
                                bool *has_idx) {
    if (nvals < 3) {
        shprint(sh, "usage: color [<idx 0-%d>] <r 0-255> <g 0-255> <b 0-255>",
                ZAF_INFO_MAX_COLORS - 1);
        return -EINVAL;
    }
    int r, g, b;
    if (nvals >= 4) {
        const int idx = (int)strtol(vals[0], NULL, 10);
        if (idx < 0 || idx >= ZAF_INFO_MAX_COLORS) {
            shprint(sh, "error: idx must be 0-%d", ZAF_INFO_MAX_COLORS - 1);
            return -EINVAL;
        }
        r = (int)strtol(vals[1], NULL, 10);
        g = (int)strtol(vals[2], NULL, 10);
        b = (int)strtol(vals[3], NULL, 10);
        *idx_out  = (uint8_t)idx;
        *has_idx  = true;
    } else {
        r = (int)strtol(vals[0], NULL, 10);
        g = (int)strtol(vals[1], NULL, 10);
        b = (int)strtol(vals[2], NULL, 10);
        *has_idx  = false;
    }
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
        shprint(sh, "error: r/g/b must be 0-255");
        return -EINVAL;
    }
    color_out->r = (uint8_t)r;
    color_out->g = (uint8_t)g;
    color_out->b = (uint8_t)b;
    return 0;
}

static int cmd_evt(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 3) {
        shprint(sh,
            "usage: argb evt <layer <n>|batt-warn <1-3>|batt-crit <1-3>|idle|usb-conn|"
            "usb-disconn|ble-profile|no-endpoint|studio-unlock|studio-lock|<custom-name>>"
            " <show|color [<idx>] <r> <g> <b>|anim <solid|blink|breathe|flash>"
            "|blink <on_ms> <off_ms>|flash <dur_ms>"
            "|flash-ease-in <ms> <fn>|flash-ease-out <ms> <fn>|feedback <dur_ms>>");
        return -EINVAL;
    }

    uint8_t event_idx = 0, sub_idx = 0;
    int consumed;
    struct zaf_custom_event *custom = NULL;
    const int rc_spec = parse_event_spec(sh, (int)argc - 1, &argv[1],
                                         &event_idx, &sub_idx, &consumed, &custom);
    if (rc_spec) {
        return rc_spec;
    }

    const int prop_off = 1 + consumed; /* index of property token in argv */
    if ((size_t)prop_off >= argc) {
        shprint(sh, "missing property");
        return -EINVAL;
    }

    const char *prop  = argv[prop_off];
    const int val_off = prop_off + 1;
    const int nvals   = (int)argc - val_off;
    char    **vals    = &argv[val_off];

    int rc = 0;
    if (custom != NULL) {
        if (strcmp(prop, "show") == 0) {
            struct zaf_event_info info;
            rc = zaf_custom_event_get(custom, &info);
            if (rc == 0) {
                print_event_info(sh, &info);
            }
        } else if (strcmp(prop, "color") == 0) {
            uint8_t idx = 0;
            struct zaf_rgb color;
            bool has_idx;
            rc = cmd_evt_parse_color(sh, nvals, vals, &idx, &color, &has_idx);
            if (rc == 0) {
                rc = has_idx
                    ? zaf_custom_event_set_color_at(custom, idx, color)
                    : zaf_custom_event_set_color(custom, color);
            }
        } else if (strcmp(prop, "anim") == 0) {
            if (nvals < 1) { shprint(sh, "usage: anim <solid|blink|breathe|flash>"); return -EINVAL; }
            uint8_t anim;
            if (parse_anim(vals[0], &anim) != 0) {
                shprint(sh, "unknown animation '%s'", vals[0]);
                return -EINVAL;
            }
            rc = zaf_custom_event_set_anim(custom, anim);
        } else if (strcmp(prop, "blink") == 0) {
            if (nvals < 2) { shprint(sh, "usage: blink <on_ms> <off_ms>"); return -EINVAL; }
            rc = zaf_custom_event_set_blink(custom,
                    (uint16_t)strtol(vals[0], NULL, 10),
                    (uint16_t)strtol(vals[1], NULL, 10));
        } else if (strcmp(prop, "flash") == 0) {
            if (nvals < 1) { shprint(sh, "usage: flash <dur_ms>"); return -EINVAL; }
            rc = zaf_custom_event_set_flash(custom, (uint16_t)strtol(vals[0], NULL, 10));
        } else if (strcmp(prop, "flash-ease-in") == 0) {
            if (nvals < 2) { shprint(sh, "usage: flash-ease-in <ms> <fn>"); return -EINVAL; }
            uint8_t fn;
            if (parse_ease_fn(vals[1], &fn) != 0) {
                shprint(sh, "unknown easing '%s'", vals[1]);
                return -EINVAL;
            }
            rc = zaf_custom_event_set_flash_ease_in(custom,
                    (uint16_t)strtol(vals[0], NULL, 10), fn);
        } else if (strcmp(prop, "flash-ease-out") == 0) {
            if (nvals < 2) { shprint(sh, "usage: flash-ease-out <ms> <fn>"); return -EINVAL; }
            uint8_t fn;
            if (parse_ease_fn(vals[1], &fn) != 0) {
                shprint(sh, "unknown easing '%s'", vals[1]);
                return -EINVAL;
            }
            rc = zaf_custom_event_set_flash_ease_out(custom,
                    (uint16_t)strtol(vals[0], NULL, 10), fn);
        } else if (strcmp(prop, "feedback") == 0) {
            if (nvals < 1) { shprint(sh, "usage: feedback <dur_ms>"); return -EINVAL; }
            rc = zaf_custom_event_set_feedback(custom, (uint16_t)strtol(vals[0], NULL, 10));
        } else {
            shprint(sh, "unknown property '%s'", prop);
            return -EINVAL;
        }
        if (rc) { shprint(sh, "error: %d", rc); }
        return rc;
    }

    /* ---- Built-in event path ---- */
    if (strcmp(prop, "show") == 0) {
        struct zaf_event_info info;
        rc = zaf_event_get(event_idx, sub_idx, &info);
        if (rc) {
            shprint(sh, "error: %d", rc);
            return rc;
        }
        print_event_info(sh, &info);
        return 0;
    }

    if (strcmp(prop, "color") == 0) {
        uint8_t idx = 0;
        struct zaf_rgb color;
        bool has_idx;
        rc = cmd_evt_parse_color(sh, nvals, vals, &idx, &color, &has_idx);
        if (rc == 0) {
            rc = has_idx
                ? zaf_event_set_color_at(event_idx, sub_idx, idx, color)
                : zaf_event_set_color(event_idx, sub_idx, color);
        }
    } else if (strcmp(prop, "anim") == 0) {
        if (nvals < 1) {
            shprint(sh, "usage: anim <solid|blink|breathe|flash>");
            return -EINVAL;
        }
        uint8_t anim;
        if (parse_anim(vals[0], &anim) != 0) {
            shprint(sh, "unknown animation '%s'", vals[0]);
            return -EINVAL;
        }
        rc = zaf_event_set_anim(event_idx, sub_idx, anim);
    } else if (strcmp(prop, "blink") == 0) {
        if (nvals < 2) {
            shprint(sh, "usage: blink <on_ms> <off_ms>");
            return -EINVAL;
        }
        rc = zaf_event_set_blink(event_idx, sub_idx,
                                 (uint16_t)strtol(vals[0], NULL, 10),
                                 (uint16_t)strtol(vals[1], NULL, 10));
    } else if (strcmp(prop, "flash") == 0) {
        if (nvals < 1) {
            shprint(sh, "usage: flash <dur_ms>");
            return -EINVAL;
        }
        rc = zaf_event_set_flash(event_idx, sub_idx,
                                 (uint16_t)strtol(vals[0], NULL, 10));
    } else if (strcmp(prop, "flash-ease-in") == 0) {
        if (nvals < 2) {
            shprint(sh, "usage: flash-ease-in <ms> <fn>");
            return -EINVAL;
        }
        uint8_t fn;
        if (parse_ease_fn(vals[1], &fn) != 0) {
            shprint(sh, "unknown easing '%s'", vals[1]);
            return -EINVAL;
        }
        rc = zaf_event_set_flash_ease_in(event_idx, sub_idx,
                                          (uint16_t)strtol(vals[0], NULL, 10), fn);
    } else if (strcmp(prop, "flash-ease-out") == 0) {
        if (nvals < 2) {
            shprint(sh, "usage: flash-ease-out <ms> <fn>");
            return -EINVAL;
        }
        uint8_t fn;
        if (parse_ease_fn(vals[1], &fn) != 0) {
            shprint(sh, "unknown easing '%s'", vals[1]);
            return -EINVAL;
        }
        rc = zaf_event_set_flash_ease_out(event_idx, sub_idx,
                                           (uint16_t)strtol(vals[0], NULL, 10), fn);
    } else if (strcmp(prop, "feedback") == 0) {
        if (nvals < 1) {
            shprint(sh, "usage: feedback <dur_ms>");
            return -EINVAL;
        }
        rc = zaf_event_set_feedback(event_idx, sub_idx,
                                    (uint16_t)strtol(vals[0], NULL, 10));
    } else {
        shprint(sh, "unknown property '%s'", prop);
        return -EINVAL;
    }

    if (rc) {
        shprint(sh, "error: %d", rc);
    }
    return rc;
}

static int cmd_on(const struct shell *sh, size_t argc, char **argv) {
    const int rc = zaf_on();
    if (rc) shprint(sh, "error: %d", rc);
    return rc;
}

static int cmd_off(const struct shell *sh, size_t argc, char **argv) {
    const int rc = zaf_off();
    if (rc) shprint(sh, "error: %d", rc);
    return rc;
}

static int cmd_clear(const struct shell *sh, size_t argc, char **argv) {
    const int rc = zaf_clear_persisted();
    if (rc) {
        shprint(sh, "error: %d", rc);
    } else {
        shprint(sh, "persisted settings cleared");
    }
    return rc;
}

static void print_named_event(const struct shell *sh, const char *name,
                              const uint8_t event_idx, const uint8_t sub_idx) {
    struct zaf_event_info info;
    if (zaf_event_get(event_idx, sub_idx, &info) != 0) {
        return;
    }
    shprint(sh, "[%s]", name);
    print_event_info(sh, &info);
}

static void print_custom_event(const struct shell *sh, const struct zaf_custom_event *evt) {
    struct zaf_event_info info;
    zaf_custom_event_get(evt, &info);
    shprint(sh, "[%s]", evt->name);
    print_event_info(sh, &info);
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv) {
    shprint(sh, "state: %s", zaf_is_on() ? "on" : "off");

    print_named_event(sh, "idle",           ZAF_EVTIDX_IDLE,          0);
    print_named_event(sh, "usb-conn",       ZAF_EVTIDX_USB_CONN,      0);
    print_named_event(sh, "usb-disconn",    ZAF_EVTIDX_USB_DISCONN,   0);
    print_named_event(sh, "ble-profile",    ZAF_EVTIDX_BLE_PROFILE,   0);
    print_named_event(sh, "no-endpoint",    ZAF_EVTIDX_NO_ENDPOINT,   0);
    print_named_event(sh, "studio-unlock",  ZAF_EVTIDX_STUDIO_UNLOCK, 0);
    print_named_event(sh, "studio-lock",    ZAF_EVTIDX_STUDIO_LOCK,   0);

    for (uint8_t lvl = 0; lvl < ZAF_BATT_LEVEL_COUNT; lvl++) {
        char name[16];
        snprintf(name, sizeof(name), "batt-warn %d", lvl + 1);
        print_named_event(sh, name, ZAF_EVTIDX_BATT_WARN, lvl);
    }
    for (uint8_t lvl = 0; lvl < ZAF_BATT_LEVEL_COUNT; lvl++) {
        char name[16];
        snprintf(name, sizeof(name), "batt-crit %d", lvl + 1);
        print_named_event(sh, name, ZAF_EVTIDX_BATT_CRIT, lvl);
    }

    const uint8_t layers = zaf_layer_count();
    for (uint8_t layer = 0; layer < layers; layer++) {
        char name[16];
        snprintf(name, sizeof(name), "layer %d", layer);
        print_named_event(sh, name, ZAF_EVTIDX_LAYER, layer);
    }

    STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
        print_custom_event(sh, cevt);
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_argb,
    SHELL_CMD(on,     NULL, "Enable feedback",  cmd_on),
    SHELL_CMD(off,    NULL, "Disable feedback", cmd_off),
    SHELL_CMD(status, NULL, "Show state and all event configs", cmd_status),
    SHELL_CMD(clear,  NULL, "Clear all persisted settings", cmd_clear),
    SHELL_CMD_ARG(evt, NULL,
        "Inspect or modify an event's LED config.\n"
        "Usage: argb evt <layer <n>|batt-warn <1-3>|batt-crit <1-3>|idle|usb-conn|usb-disconn|"
        "ble-profile|no-endpoint|studio-unlock|studio-lock|<custom-name>>"
        " <show|color [<idx>] <r> <g> <b>|anim <solid|blink|breathe|flash>"
        "|blink <on_ms> <off_ms>|flash <dur_ms>"
        "|flash-ease-in <ms> <fn>|flash-ease-out <ms> <fn>|feedback <dur_ms>>\n"
        "  color <r> <g> <b>         -- set colors[0], reset color count to 1\n"
        "  color <idx> <r> <g> <b>   -- set colors[idx] (0-based), expand count if needed\n"
        "  flash-ease-in <ms> <fn>   -- fade-in duration and curve for flash animation\n"
        "  flash-ease-out <ms> <fn>  -- fade-out duration and curve for flash animation\n"
        "  fn: linear|quad-in|quad-out|quad-in-out|cubic-in|cubic-out|cubic-in-out\n"
        "      quart-in|quart-out|quart-in-out|expo-in|expo-out|bounce-out|bounce-in\n"
        "  feedback <dur_ms>         -- haptic/LED feedback pulse duration for this event",
        cmd_evt, 3, 8),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(argb, &sub_argb, "Adaptive Feedback control", NULL);
