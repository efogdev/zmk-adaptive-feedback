#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk_adaptive_feedback/adaptive_feedback.h>
#include "drivers/behavior.h"
#include "dt-bindings/zmk/adaptive_feedback.h"

#define DT_DRV_COMPAT zmk_behavior_adaptive_feedback_toggle

LOG_MODULE_DECLARE(zmk_adaptive_feedback, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_value_metadata mtd_param1_values[] = {
    {
        .display_name = "On",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = AF_ON,
    },
    {
        .display_name = "Off",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = AF_OFF,
    },
    {
        .display_name = "Toggle",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = AF_TOG,
    },
};

static const struct behavior_parameter_metadata_set metadata_sets[] = {
    {
        .param1_values = mtd_param1_values,
        .param1_values_len = ARRAY_SIZE(mtd_param1_values),
    },
};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = ARRAY_SIZE(metadata_sets),
    .sets = metadata_sets,
};
#endif

static int on_af_toggle_binding_pressed(struct zmk_behavior_binding *binding,
                                        const struct zmk_behavior_binding_event event) {
    ARG_UNUSED(event);

    const uint8_t param1 = binding->param1;

    if (param1 == AF_ON) {
        zaf_on();
    } else if (param1 == AF_OFF) {
        zaf_off();
    } else {
        if (zaf_is_on()) {
            zaf_off();
        } else {
            zaf_on();
        }
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_adaptive_feedback_toggle_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static const struct behavior_driver_api behavior_adaptive_feedback_toggle_driver_api = {
    .binding_pressed = on_af_toggle_binding_pressed,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

#define AF_TOGGLE_INST(n)                                                                          \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_adaptive_feedback_toggle_init, NULL, NULL, NULL,          \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                      \
                            &behavior_adaptive_feedback_toggle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AF_TOGGLE_INST)

#endif
