#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#define PID_CONTROL_STORAGE_SIZE 48u
#define PID_CONTROL_STORAGE_ALIGNMENT 4u

struct pid_control;
typedef struct pid_control* pid_control_handle;

typedef struct {
    float kp;
    float ki;
    float kd;
    float kaw; // anti-windup gain
    float u_min; // minimum output
    float u_max; // maximum output
} pid_control_config;

size_t pid_control_storage_size(void);
size_t pid_control_storage_alignment(void);
esp_err_t pid_control_init(void* storage, size_t storage_size, pid_control_handle* handle, const pid_control_config* config);
esp_err_t pid_control_update(pid_control_handle handle, float setpoint, float measurement, float* u_out);
esp_err_t pid_control_reset_state(pid_control_handle handle);
esp_err_t pid_control_set_gains(pid_control_handle handle, bool reset_on_change, float kp, float ki, float kd);
esp_err_t pid_control_set_anti_windup(pid_control_handle handle, float kaw);
esp_err_t pid_control_set_output_limits(pid_control_handle handle, float u_min, float u_max);

#endif // PID_CONTROL_H
