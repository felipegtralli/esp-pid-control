#include "pid_control.h"
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#if CONFIG_PID_CONTROL_LOGGING
    #include "esp_log.h"
    static const char TAG[] = "pid_control";
#endif

/* MISRA C:2012 Dir 1.1 note: Project may build with GNU dialect via ESP-IDF; this module avoids GNU-only constructs. */

struct clamp_range {
    float min;
    float max;
};

struct pid_req {
    size_t size;
    size_t align;
};

struct pid_result {
    float u;
    float err;
};

struct pid_control {
    float kp;
    float ki;
    float kd;
    float kaw;
    float prev_err;
    float prev_err2;
    float last_u;
    float u_min;
    float u_max;
};

_Static_assert(PID_CONTROL_STORAGE_SIZE >= sizeof(struct pid_control), "PID_CONTROL_STORAGE_SIZE is too small");
_Static_assert(PID_CONTROL_STORAGE_ALIGNMENT >= _Alignof(struct pid_control), "PID_CONTROL_STORAGE_ALIGNMENT is too small");
_Static_assert((PID_CONTROL_STORAGE_ALIGNMENT & (PID_CONTROL_STORAGE_ALIGNMENT - 1u)) == 0u, "PID_CONTROL_STORAGE_ALIGNMENT must be power of two");

// global helpers
static inline bool is_finite(float x) {
    const float t = x;
    return (isfinite(t) != 0);
}

// must be power of two
// MISRA C:2012 R11.4 deviation – pointer to integer cast for alignment check (no dereference).
static inline bool is_aligned(const void* ptr, size_t align) {
    const uintptr_t mask = (uintptr_t)align - (uintptr_t)1u;
    return ((uintptr_t)ptr & mask) == (uintptr_t)0;
}

static inline float clampf(float x, struct clamp_range range) {
    const float t = (x < range.min) ? range.min : x;
    return (t > range.max) ? range.max : t;
}

// init helpers
static esp_err_t pid_validate_storage(const void* storage, size_t storage_size, struct pid_req req) {
    if(storage_size < req.size) {
        return ESP_ERR_INVALID_SIZE;
    }
    if(!is_aligned(storage, req.align)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t pid_validate_config(const pid_control_config* config) {
    if(!is_finite(config->kp) || !is_finite(config->ki) || !is_finite(config->kd) || !is_finite(config->kaw) ||
       !is_finite(config->u_min) || !is_finite(config->u_max)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(config->kp < 0.0f || config->ki < 0.0f || config->kd < 0.0f || config->kaw < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if(config->u_min >= config->u_max) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void pid_apply_config(struct pid_control* ctrl, const pid_control_config* config) {
    ctrl->kp = config->kp;
    ctrl->ki = config->ki;
    ctrl->kd = config->kd;
    ctrl->kaw = config->kaw;
    ctrl->prev_err = 0.0f;
    ctrl->prev_err2 = 0.0f;
    const struct clamp_range range = { .min = config->u_min, .max = config->u_max };
    ctrl->last_u = clampf(0.0f, range);
    ctrl->u_min = config->u_min;
    ctrl->u_max = config->u_max;
}

// update helpers
static esp_err_t pid_validate_update_args(float setpoint, float measurement) {
    if(!is_finite(setpoint) || !is_finite(measurement)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

// dU(z) = Kp * (1 - z^-1)E(z) + Ki * E(z) + Kd * (1 - 2z^-1 + z^-2)E(z) + back-calculation
static struct pid_result pid_compute_incremental(const struct pid_control* ctrl, float setpoint, float measurement) {
    static const float pid_second_diff_coeff = 2.0f;
    
    const float err = setpoint - measurement;

    const float du_p = ctrl->kp * (err - ctrl->prev_err);
    const float du_i = ctrl->ki * err;
    const float du_d = ctrl->kd * (err - (pid_second_diff_coeff * ctrl->prev_err) + ctrl->prev_err2);

    const struct clamp_range range = { .min = ctrl->u_min, .max = ctrl->u_max };
    const float u_unsat = ctrl->last_u + du_p + du_i + du_d;
    const float u_sat = clampf(u_unsat, range);

    const float sat_err = u_sat - u_unsat;
    const float du_aw = ctrl->kaw * sat_err;

    const float du = du_p + du_i + du_d + du_aw;
    float u = clampf((ctrl->last_u + du), range);

    struct pid_result result = { .u = u, .err = err };
    return result;
}

static void pid_commit_incremental(struct pid_control* ctrl, struct pid_result res) {
    ctrl->last_u = res.u;
    ctrl->prev_err2 = ctrl->prev_err;
    ctrl->prev_err = res.err;
}

// setters helpers
static esp_err_t pid_validate_gains(float kp, float ki, float kd) {
    if(!is_finite(kp) || !is_finite(ki) || !is_finite(kd)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(kp < 0.0f || ki < 0.0f || kd < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t pid_validate_anti_windup(float kaw) {
    if(!is_finite(kaw)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(kaw < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t pid_validate_output_limits(float u_min, float u_max) {
    if(!is_finite(u_min) || !is_finite(u_max)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(u_min >= u_max) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

// public API
size_t pid_control_storage_size(void) {
    return sizeof(struct pid_control);
}

size_t pid_control_storage_alignment(void) {
    return PID_CONTROL_STORAGE_ALIGNMENT;
}

esp_err_t pid_control_init(void* storage, size_t storage_size, pid_control_handle* handle, const pid_control_config* config) {
    esp_err_t status = ESP_OK;
    if(storage == NULL || handle == NULL || config == NULL) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    } else {
        *handle = NULL;
    }

    if(status == ESP_OK) {
        const struct pid_req req = {
        .size = pid_control_storage_size(),
        .align = pid_control_storage_alignment()
        };
        
        status = pid_validate_storage(storage, storage_size, req);
        if(status != ESP_OK) {
            #if CONFIG_PID_CONTROL_LOGGING
                ESP_LOGE(TAG, "Invalid argument: invalid storage (size or alignment)");
            #endif
        }
    }

    if(status == ESP_OK) {
        status = pid_validate_config(config);
        if(status != ESP_OK) {
            #if CONFIG_PID_CONTROL_LOGGING
                ESP_LOGE(TAG, "Invalid argument: invalid PID configuration");
            #endif
        }
    }

    if(status == ESP_OK) {
        // MISRA C:2012 R11.5 deviation – converting void* storage to object pointer after validation.
        struct pid_control* ctrl = storage;
        pid_apply_config(ctrl, config);

        *handle = ctrl;
    }

    return status;
}

esp_err_t pid_control_update(pid_control_handle handle, float setpoint, float measurement, float* u_out) {
    esp_err_t status = ESP_OK;
    if(handle == NULL || u_out == NULL) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        status = pid_validate_update_args(setpoint, measurement);
        if(status != ESP_OK) {
            #if CONFIG_PID_CONTROL_LOGGING
                ESP_LOGE(TAG, "Invalid argument: invalid setpoint or measurement");
            #endif
        }
    }

    if(status == ESP_OK) {
        struct pid_control* ctrl = handle;

        struct pid_result res = pid_compute_incremental(ctrl, setpoint, measurement);
        pid_commit_incremental(ctrl, res);

        *u_out = res.u;
    }

    return status;
}

esp_err_t pid_control_reset_state(pid_control_handle handle) {
    esp_err_t status = ESP_OK;
    if(handle == NULL) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        struct pid_control* ctrl = handle;
        ctrl->prev_err = 0.0f;
        ctrl->prev_err2 = 0.0f;
        const struct clamp_range range = { .min = ctrl->u_min, .max = ctrl->u_max };
        ctrl->last_u = clampf(0.0f, range);
    }

    return status;
}

esp_err_t pid_control_set_gains(pid_control_handle handle, bool reset_on_change, float kp, float ki, float kd) {
    esp_err_t status = ESP_OK;
    if(handle == NULL) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        status = pid_validate_gains(kp, ki, kd);
        if(status != ESP_OK) {
            #if CONFIG_PID_CONTROL_LOGGING
                ESP_LOGE(TAG, "Invalid argument: invalid PID gains");
            #endif
        }
    }

    if(status == ESP_OK) {
        struct pid_control* ctrl = handle;
        ctrl->kp = kp;
        ctrl->ki = ki;
        ctrl->kd = kd;
    }

    if(status == ESP_OK && reset_on_change) {
        status = pid_control_reset_state(handle);
    }

    return status;
}

esp_err_t pid_control_set_anti_windup(pid_control_handle handle, float kaw) {
    esp_err_t status = ESP_OK;
    if(handle == NULL) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        status = pid_validate_anti_windup(kaw);
        if(status != ESP_OK) {
            #if CONFIG_PID_CONTROL_LOGGING
                ESP_LOGE(TAG, "Invalid argument: invalid anti-windup gain");
            #endif
        }
    }

    if(status == ESP_OK) {
        struct pid_control* ctrl = handle;
        ctrl->kaw = kaw;
    }

    return status;
}

esp_err_t pid_control_set_output_limits(pid_control_handle handle, float u_min, float u_max) {
    esp_err_t status = ESP_OK;
    if(handle == NULL) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        status = pid_validate_output_limits(u_min, u_max);
        if(status != ESP_OK) {
            #if CONFIG_PID_CONTROL_LOGGING
                ESP_LOGE(TAG, "Invalid argument: invalid output limits");
            #endif
        }
    }

    if(status == ESP_OK) {
        struct pid_control* ctrl = handle;

        ctrl->u_min = u_min;
        ctrl->u_max = u_max;

        const struct clamp_range range = { .min = u_min, .max = u_max };
        ctrl->last_u = clampf(ctrl->last_u, range);
    }

    return status;
}
