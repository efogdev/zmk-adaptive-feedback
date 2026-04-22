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

#define _ZAF_ANIM_STR(tok, str, val) str,
static const char * const zaf_anim_names[] = { ZAF_ANIM_TABLE(_ZAF_ANIM_STR) };
#undef _ZAF_ANIM_STR

static int parse_anim(const char *s, uint8_t *out) {
    for (uint8_t i = 0; i < ARRAY_SIZE(zaf_anim_names); i++) {
        if (strcmp(s, zaf_anim_names[i]) == 0) {
            *out = i;
            return 0;
        }
    }
    return -EINVAL;
}

#define _ZAF_EASE_STR(tok, str, val) str,
static const char * const zaf_ease_fn_names[] = { ZAF_EASE_TABLE(_ZAF_EASE_STR) };
#undef _ZAF_EASE_STR

#define _ZAF_EVT_STR(tok, str, val) str,
static const char * const zaf_evt_names[] = { ZAF_EVT_TABLE(_ZAF_EVT_STR) };
#undef _ZAF_EVT_STR

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

    if (strcmp(type, zaf_evt_names[ZAF_EVTIDX_LAYER]) == 0) {
        if (argc < 2) {
            shprint(sh, "layer requires an index");

            return -EINVAL;
        }
        *event_idx = ZAF_EVTIDX_LAYER;
        *sub_idx   = (uint8_t)strtol(argv[1], NULL, 10);
        *consumed  = 2;
    } else if (strcmp(type, zaf_evt_names[ZAF_EVTIDX_BATT_WARN]) == 0) {
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
    } else if (strcmp(type, zaf_evt_names[ZAF_EVTIDX_BATT_CRIT]) == 0) {
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
    } else if (strcmp(type, zaf_evt_names[ZAF_EVTIDX_BLE_PROFILE]) == 0) {
        if (argc < 2) {
            shprint(sh, "ble-profile requires a profile index 1-%d", CONFIG_ADAPTIVE_FEEDBACK_MAX_BT_DEVICES);
            return -EINVAL;
        }
        const int prof = (int)strtol(argv[1], NULL, 10);
        if (prof < 1 || prof > CONFIG_ADAPTIVE_FEEDBACK_MAX_BT_DEVICES) {
            shprint(sh, "ble-profile index must be 1-%d", CONFIG_ADAPTIVE_FEEDBACK_MAX_BT_DEVICES);
            return -EINVAL;
        }
        *event_idx = ZAF_EVTIDX_BLE_PROFILE;
        *sub_idx   = (uint8_t)(prof - 1);
        *consumed  = 2;
    } else if (strcmp(type, zaf_evt_names[ZAF_EVTIDX_ERROR]) == 0) {
        if (argc < 2) {
            shprint(sh, "error requires a slot index");
            return -EINVAL;
        }
        *event_idx = ZAF_EVTIDX_ERROR;
        *sub_idx   = (uint8_t)strtol(argv[1], NULL, 10);
        *consumed  = 2;
    } else {
        static const struct { const char *name; uint8_t idx; } simple_evts[] = {
#define _ZAF_SIMPLE_ROW(tok, str, val) { str, ZAF_EVTIDX_##tok },
            ZAF_SIMPLE_EVT_TABLE(_ZAF_SIMPLE_ROW)
#undef _ZAF_SIMPLE_ROW
        };
        bool found = false;
        for (size_t i = 0; i < ARRAY_SIZE(simple_evts); i++) {
            if (strcmp(type, simple_evts[i].name) == 0) {
                *event_idx = simple_evts[i].idx;
                found = true;
                break;
            }
        }
        if (!found) {
            STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
                if (strcmp(type, cevt->name) == 0) {
                    *custom_out = cevt;
                    return 0;
                }
            }
            shprint(sh, "unknown event type '%s'", type);
            return -EINVAL;
        }
    }

    return 0;
}

static void print_event_info(const struct shell *sh, const struct zaf_event_info *info,
                             const char *label, const bool persistent) {
    if (label != NULL) {
        shprint(sh, "  label:      %s", label);
    }
    shprint(sh, "  anim:       %s",
            info->animation < ARRAY_SIZE(zaf_anim_names) ? zaf_anim_names[info->animation] : "?");
    shprint(sh, "  colors:     %d", info->color_count);
    for (uint8_t i = 0; i < info->color_count; i++) {
        shprint(sh, "    [%d] r=%-3d g=%-3d b=%-3d",
                i, info->colors[i].r, info->colors[i].g, info->colors[i].b);
    }
    shprint(sh, "  blink:      on=%dms off=%dms", info->blink_on_ms, info->blink_off_ms);
    shprint(sh, "  flash-dur:  %dms", info->flash_duration_ms);
    shprint(sh, "  flash-ease-in:  %dms fn=%s", info->flash_ease_in_ms,
            info->flash_ease_in_fn < ARRAY_SIZE(zaf_ease_fn_names)
                ? zaf_ease_fn_names[info->flash_ease_in_fn] : "?");
    shprint(sh, "  flash-ease-out: %dms fn=%s", info->flash_ease_out_ms,
            info->flash_ease_out_fn < ARRAY_SIZE(zaf_ease_fn_names)
                ? zaf_ease_fn_names[info->flash_ease_out_fn] : "?");
    if (info->feedback_pattern_len > 0) {
        char buf[CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN * 8];
        int off = 0;
        for (uint8_t i = 0; i < info->feedback_pattern_len && off < (int)sizeof(buf) - 8; i++) {
            off += snprintf(buf + off, sizeof(buf) - off, "%u", info->feedback_pattern[i]);
            if (i + 1 < info->feedback_pattern_len) {
                buf[off++] = ' ';
            }
        }
        buf[off] = '\0';
        shprint(sh, "  feedback:   [%s]", buf);
    } else {
        shprint(sh, "  feedback:   (none)");
    }
    shprint(sh, "  breathe:    %dms", info->breathe_duration_ms);
    shprint(sh, "  persistent: %s", persistent ? "yes" : "no");
}

static int cmd_evt_parse_color(const struct shell *sh, const int nvals, char **vals,
                                uint8_t *idx_out, struct zaf_rgb *color_out,
                                bool *has_idx) {
    if (nvals < 3) {
        shprint(sh, "usage: color [<idx 0-%d>] <r 0-255> <g 0-255> <b 0-255>",
                CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS - 1);
        return -EINVAL;
    }
    int r, g, b;
    if (nvals >= 4) {
        const int idx = (int)strtol(vals[0], NULL, 10);
        if (idx < 0 || idx >= CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS) {
            shprint(sh, "error: idx must be 0-%d", CONFIG_ZMK_ADAPTIVE_FEEDBACK_MAX_COLORS - 1);
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
            "|flash-ease-in <ms> <fn>|flash-ease-out <ms> <fn>"
            "|feedback <on_ms> [off_ms on_ms ...]>");
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
    int rc = 0;

    /* ---- "show" is handled separately as it reads, not writes ---- */
    if (strcmp(prop, "show") == 0) {
        struct zaf_event_info info;
        if (custom != NULL) {
            rc = zaf_custom_event_get(custom, &info);
            if (rc == 0) {
                print_event_info(sh, &info, zaf_custom_event_get_label(custom),
                                 zaf_custom_event_is_persistent(custom));
            }
        } else {
            rc = zaf_event_get(event_idx, sub_idx, &info);
            if (rc == 0) {
                print_event_info(sh, &info, zaf_event_get_label(event_idx, sub_idx),
                                 zaf_event_is_persistent(event_idx, sub_idx));
            }
        }
        if (rc) { shprint(sh, "error: %d", rc); }
        return rc;
    }

    enum zaf_field field;
    union zaf_field_value val = {0};
    const int val_off = prop_off + 1;
    const int nvals   = (int)argc - val_off;
    char    **vals    = &argv[val_off];

    if (strcmp(prop, "color") == 0) {
        uint8_t idx = 0;
        struct zaf_rgb color;
        bool has_idx;
        rc = cmd_evt_parse_color(sh, nvals, vals, &idx, &color, &has_idx);
        if (rc != 0) { return rc; }
        if (has_idx) {
            field              = ZAF_FIELD_COLOR_AT;
            val.color_at.idx   = idx;
            val.color_at.color = color;
        } else {
            field     = ZAF_FIELD_COLOR;
            val.color = color;
        }
    } else if (strcmp(prop, "anim") == 0) {
        if (nvals < 1) { shprint(sh, "usage: anim <solid|blink|breathe|flash>"); return -EINVAL; }
        if (parse_anim(vals[0], &val.anim) != 0) {
            shprint(sh, "unknown animation '%s'", vals[0]);
            return -EINVAL;
        }
        field = ZAF_FIELD_ANIM;
    } else if (strcmp(prop, "blink") == 0) {
        if (nvals < 2) { shprint(sh, "usage: blink <on_ms> <off_ms>"); return -EINVAL; }
        val.blink.on_ms  = (uint16_t)strtol(vals[0], NULL, 10);
        val.blink.off_ms = (uint16_t)strtol(vals[1], NULL, 10);
        field = ZAF_FIELD_BLINK;
    } else if (strcmp(prop, "flash") == 0) {
        if (nvals < 1) { shprint(sh, "usage: flash <dur_ms>"); return -EINVAL; }
        val.flash_dur_ms = (uint16_t)strtol(vals[0], NULL, 10);
        field = ZAF_FIELD_FLASH;
    } else if (strcmp(prop, "flash-ease-in") == 0) {
        if (nvals < 2) { shprint(sh, "usage: flash-ease-in <ms> <fn>"); return -EINVAL; }
        if (parse_ease_fn(vals[1], &val.ease.fn) != 0) {
            shprint(sh, "unknown easing '%s'", vals[1]);
            return -EINVAL;
        }
        val.ease.ms = (uint16_t)strtol(vals[0], NULL, 10);
        field = ZAF_FIELD_FLASH_EASE_IN;
    } else if (strcmp(prop, "flash-ease-out") == 0) {
        if (nvals < 2) { shprint(sh, "usage: flash-ease-out <ms> <fn>"); return -EINVAL; }
        if (parse_ease_fn(vals[1], &val.ease.fn) != 0) {
            shprint(sh, "unknown easing '%s'", vals[1]);
            return -EINVAL;
        }
        val.ease.ms = (uint16_t)strtol(vals[0], NULL, 10);
        field = ZAF_FIELD_FLASH_EASE_OUT;
    } else if (strcmp(prop, "feedback") == 0) {
        if (nvals < 1) { shprint(sh, "usage: feedback <on_ms> [off_ms on_ms ...]"); return -EINVAL; }
        static uint16_t pat[CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN];
        const uint8_t plen = (uint8_t)MIN(nvals, CONFIG_ZMK_ADAPTIVE_FEEDBACK_FEEDBACK_PATTERN_MAX_LEN);
        for (uint8_t i = 0; i < plen; i++) {
            pat[i] = (uint16_t)strtol(vals[i], NULL, 10);
        }
        val.feedback.pattern = pat;
        val.feedback.len     = plen;
        field = ZAF_FIELD_FEEDBACK;
    } else if (strcmp(prop, "breathe") == 0) {
        if (nvals < 1) { shprint(sh, "usage: breathe <dur_ms>"); return -EINVAL; }
        val.breathe_dur_ms = (uint16_t)strtol(vals[0], NULL, 10);
        field = ZAF_FIELD_BREATHE;
    } else {
        shprint(sh, "unknown property '%s'", prop);
        return -EINVAL;
    }

    rc = (custom != NULL)
        ? zaf_custom_event_set(custom, field, val)
        : zaf_event_set(event_idx, sub_idx, field, val);

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
    if (zaf_event_is_headless(event_idx, sub_idx)) {
        return;
    }
    struct zaf_event_info info;
    if (zaf_event_get(event_idx, sub_idx, &info) != 0) {
        return;
    }
    shprint(sh, "[%s]", name);
    print_event_info(sh, &info, zaf_event_get_label(event_idx, sub_idx),
                     zaf_event_is_persistent(event_idx, sub_idx));
}

static void print_custom_event(const struct shell *sh, const struct zaf_custom_event *evt) {
    if (evt->headless) {
        return;
    }
    struct zaf_event_info info;
    zaf_custom_event_get(evt, &info);
    shprint(sh, "[%s]", evt->name);
    print_event_info(sh, &info, zaf_custom_event_get_label(evt), zaf_custom_event_is_persistent(evt));
}

static int cmd_state(const struct shell *sh, size_t argc, char **argv) {
    shprint(sh, "state: %s", zaf_is_on() ? "on" : "off");
    return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv) {
    shprint(sh, "state: %s", zaf_is_on() ? "on" : "off");

    for (uint8_t i = 0; i < zaf_error_slots_count(); i++) {
        char name[16];
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_ERROR], i);
        print_named_event(sh, name, ZAF_EVTIDX_ERROR, i);
    }

    print_named_event(sh, zaf_evt_names[ZAF_EVTIDX_IDLE],          ZAF_EVTIDX_IDLE,          0);
    print_named_event(sh, zaf_evt_names[ZAF_EVTIDX_USB_CONN],      ZAF_EVTIDX_USB_CONN,      0);
    print_named_event(sh, zaf_evt_names[ZAF_EVTIDX_USB_DISCONN],   ZAF_EVTIDX_USB_DISCONN,   0);
    for (uint8_t prof = 0; prof < CONFIG_ADAPTIVE_FEEDBACK_MAX_BT_DEVICES; prof++) {
        char name[24];
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_BLE_PROFILE], prof + 1);
        print_named_event(sh, name, ZAF_EVTIDX_BLE_PROFILE, prof);
    }
    print_named_event(sh, zaf_evt_names[ZAF_EVTIDX_NO_ENDPOINT],   ZAF_EVTIDX_NO_ENDPOINT,   0);
    print_named_event(sh, zaf_evt_names[ZAF_EVTIDX_STUDIO_UNLOCK], ZAF_EVTIDX_STUDIO_UNLOCK, 0);
    print_named_event(sh, zaf_evt_names[ZAF_EVTIDX_STUDIO_LOCK],   ZAF_EVTIDX_STUDIO_LOCK,   0);

    for (uint8_t lvl = 0; lvl < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; lvl++) {
        char name[16];
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_BATT_WARN], lvl + 1);
        print_named_event(sh, name, ZAF_EVTIDX_BATT_WARN, lvl);
    }
    for (uint8_t lvl = 0; lvl < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; lvl++) {
        char name[16];
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_BATT_CRIT], lvl + 1);
        print_named_event(sh, name, ZAF_EVTIDX_BATT_CRIT, lvl);
    }

    const uint8_t layers = zaf_layer_count();
    for (uint8_t layer = 0; layer < layers; layer++) {
        char name[16];
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_LAYER], layer);
        print_named_event(sh, name, ZAF_EVTIDX_LAYER, layer);
    }

    STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
        print_custom_event(sh, cevt);
    }

    return 0;
}

static int cmd_list(const struct shell *sh, size_t argc, char **argv) {
    bool first = true;
    char name[32];

#define _LIST_EVT(_name_str, _event_idx, _sub_idx)                                    \
    do {                                                                               \
        if (!zaf_event_is_headless((_event_idx), (_sub_idx))) {                        \
            const char *_lbl = zaf_event_get_label((_event_idx), (_sub_idx));         \
            if (!first) { shell_fprintf(sh, SHELL_NORMAL, ", "); }                    \
            if (_lbl && _lbl[0]) {                                                     \
                shell_fprintf(sh, SHELL_NORMAL, "%s (%s)", (_name_str), _lbl);        \
            } else {                                                                   \
                shell_fprintf(sh, SHELL_NORMAL, "%s", (_name_str));                   \
            }                                                                          \
            first = false;                                                             \
        }                                                                              \
    } while (0)

    for (uint8_t i = 0; i < zaf_error_slots_count(); i++) {
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_ERROR], i);
        _LIST_EVT(name, ZAF_EVTIDX_ERROR, i);
    }
    _LIST_EVT(zaf_evt_names[ZAF_EVTIDX_IDLE],          ZAF_EVTIDX_IDLE,          0);
    _LIST_EVT(zaf_evt_names[ZAF_EVTIDX_USB_CONN],      ZAF_EVTIDX_USB_CONN,      0);
    _LIST_EVT(zaf_evt_names[ZAF_EVTIDX_USB_DISCONN],   ZAF_EVTIDX_USB_DISCONN,   0);
    for (uint8_t prof = 0; prof < CONFIG_ADAPTIVE_FEEDBACK_MAX_BT_DEVICES; prof++) {
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_BLE_PROFILE], prof + 1);
        _LIST_EVT(name, ZAF_EVTIDX_BLE_PROFILE, prof);
    }
    _LIST_EVT(zaf_evt_names[ZAF_EVTIDX_NO_ENDPOINT],   ZAF_EVTIDX_NO_ENDPOINT,   0);
    _LIST_EVT(zaf_evt_names[ZAF_EVTIDX_STUDIO_UNLOCK], ZAF_EVTIDX_STUDIO_UNLOCK, 0);
    _LIST_EVT(zaf_evt_names[ZAF_EVTIDX_STUDIO_LOCK],   ZAF_EVTIDX_STUDIO_LOCK,   0);
    for (uint8_t lvl = 0; lvl < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; lvl++) {
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_BATT_WARN], lvl + 1);
        _LIST_EVT(name, ZAF_EVTIDX_BATT_WARN, lvl);
    }
    for (uint8_t lvl = 0; lvl < CONFIG_ZMK_ADAPTIVE_FEEDBACK_BATT_LEVEL_COUNT; lvl++) {
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_BATT_CRIT], lvl + 1);
        _LIST_EVT(name, ZAF_EVTIDX_BATT_CRIT, lvl);
    }
    const uint8_t layers = zaf_layer_count();
    for (uint8_t layer = 0; layer < layers; layer++) {
        snprintf(name, sizeof(name), "%s %d", zaf_evt_names[ZAF_EVTIDX_LAYER], layer);
        _LIST_EVT(name, ZAF_EVTIDX_LAYER, layer);
    }
    STRUCT_SECTION_FOREACH(zaf_custom_event, cevt) {
        if (!cevt->headless) {
            const char *lbl = zaf_custom_event_get_label(cevt);
            if (!first) { shell_fprintf(sh, SHELL_NORMAL, ", "); }
            if (lbl && lbl[0]) {
                shell_fprintf(sh, SHELL_NORMAL, "%s (%s)", cevt->name, lbl);
            } else {
                shell_fprintf(sh, SHELL_NORMAL, "%s", cevt->name);
            }
            first = false;
        }
    }
#undef _LIST_EVT

    shell_fprintf(sh, SHELL_NORMAL, "\n");
    return 0;
}

static int cmd_error(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 3) {
        shprint(sh, "usage: argb error <slot> <trigger|clear|clear-all>");
        return -EINVAL;
    }

    const uint8_t slot = (uint8_t)strtol(argv[1], NULL, 10);
    const char *action = argv[2];
    int rc = 0;

    if (strcmp(action, "trigger") == 0) {
        rc = zaf_error_trigger(slot);
        if (rc == 0) {
            shprint(sh, "error slot %d triggered", slot);
        }
    } else if (strcmp(action, "clear") == 0) {
        rc = zaf_error_clear(slot);
        if (rc == 0) {
            shprint(sh, "error slot %d cleared", slot);
        }
    } else if (strcmp(action, "clear-all") == 0) {
        rc = zaf_error_clear_all();
        shprint(sh, "all error slots cleared");
    } else {
        shprint(sh, "unknown action '%s' (trigger|clear|clear-all)", action);
        return -EINVAL;
    }

    if (rc) {
        shprint(sh, "error: %d", rc);
    }
    return rc;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_argb,
    SHELL_CMD(on,     NULL, "Enable feedback",  cmd_on),
    SHELL_CMD(off,    NULL, "Disable feedback", cmd_off),
    SHELL_CMD(state,  NULL, "Show on/off state", cmd_state),
    SHELL_CMD(status, NULL, "Show state and all event configs", cmd_status),
    SHELL_CMD(list,   NULL, "List all available events", cmd_list),
    SHELL_CMD(clear,  NULL, "Clear all persisted settings", cmd_clear),
    SHELL_CMD_ARG(evt, NULL,
        "Inspect or modify an event's LED config.",
        cmd_evt, 3, 11),
    SHELL_CMD_ARG(error, NULL,
        "Error slot control.\n"
        "Usage: argb error <slot> <trigger|clear|clear-all>",
        cmd_error, 3, 3),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(argb, &sub_argb, "Adaptive Feedback control", NULL);
