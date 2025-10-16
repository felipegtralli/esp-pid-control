#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def PID_CONTROL_STORAGE_SIZE
 * @brief Minimum number of bytes required to store a PID controller instance.
 */
#define PID_CONTROL_STORAGE_SIZE 48u

/**
 * @def PID_CONTROL_STORAGE_ALIGNMENT
 * @brief Required alignment (in bytes) for PID controller storage.
 */
#define PID_CONTROL_STORAGE_ALIGNMENT 4u

/**
 * @brief Opaque structure representing a PID controller instance.
 *
 * The internal structure is hidden; use the handle type for all API calls.
 */
struct pid_control;

/**
 * @brief Opaque handle to a PID controller instance.
 */
typedef struct pid_control* pid_control_handle;

/**
 * @brief Configuration structure for initializing a PID controller.
 *
 * Set the anti-windup gain (kaw) to 0 to disable back-calculation.
 * Output limits (u_min, u_max) must satisfy u_min < u_max.
 */
typedef struct {
    float kp;    /**< Proportional gain. */
    float ki;    /**< Integral gain. */
    float kd;    /**< Derivative gain. */
    float kaw;   /**< Anti-windup gain (set to 0 to disable back-calculation). */
    float u_min; /**< Minimum output value. */
    float u_max; /**< Maximum output value. */
} pid_control_config;

/**
 * @brief Get the required storage size (in bytes) for a PID controller instance.
 *
 * @return Number of bytes required for storage.
 */
size_t pid_control_storage_size(void);

/**
 * @brief Get the required storage alignment (in bytes) for a PID controller instance.
 *
 * @return Required alignment in bytes.
 */
size_t pid_control_storage_alignment(void);

/**
 * @brief Initialize a PID controller instance.
 *
 * Validates the provided storage and configuration, then initializes the controller.
 *
 * @param[in]  storage      Pointer to pre-allocated memory for the controller.
 * @param[in]  storage_size Size of the pre-allocated memory (in bytes).
 * @param[out] handle       Pointer to the PID controller handle to be initialized.
 * @param[in]  config       Pointer to the configuration structure.
 *
 * @retval ESP_OK               Success.
 * @retval ESP_ERR_INVALID_ARG  If any argument is NULL, invalid, or storage is not aligned.
 * @retval ESP_ERR_INVALID_SIZE If the provided storage size is insufficient.
 */
esp_err_t pid_control_init(void* storage, size_t storage_size, pid_control_handle* handle, const pid_control_config* config);

/**
 * @brief Compute the next control output using the incremental PID algorithm.
 *
 * Updates the PID controller state with a new setpoint and measurement, and calculates
 * the next control output using the incremental (velocity) PID formula. The implementation
 * includes anti-windup back-calculation, which can be disabled by setting the anti-windup gain
 * (kaw) to 0 in the configuration or via the setter.
 *
 * The output is automatically clamped to the configured limits [u_min, u_max].
 *
 * @param[in]  handle      PID controller handle (must be initialized).
 * @param[in]  setpoint    Desired setpoint value.
 * @param[in]  measurement Current measurement value.
 * @param[out] u_out       Pointer to store the computed control output.
 *
 * @retval ESP_OK              Success.
 * @retval ESP_ERR_INVALID_ARG If any argument is NULL or invalid.
 *
 * @note Uses the incremental PID formula with back-calculation for anti-windup.
 * @note If kaw is set to 0, back-calculation is disabled and the controller behaves as a standard PID.
 * @note This function is not thread-safe; external synchronization is required if used concurrently.
 * @note Argument validation can be disabled at build time by enabling CONFIG_PID_CONTROL_IGNORE_UPDATE_CHECKS
 *       in Kconfig. When disabled, the caller is responsible for ensuring all arguments are valid.
 */
esp_err_t pid_control_update(pid_control_handle handle, float setpoint, float measurement, float* u_out);

/**
 * @brief Reset the internal state of the PID controller.
 *
 * Sets the previous error and output history to zero, without changing gains or limits.
 *
 * @param[in] handle PID controller handle.
 *
 * @retval ESP_OK              Success.
 * @retval ESP_ERR_INVALID_ARG If the handle is NULL.
 *
 * @note This function is not thread-safe; external synchronization is required if used concurrently.
 */
esp_err_t pid_control_reset_state(pid_control_handle handle);

/**
 * @brief Set new PID gains for the controller.
 *
 * Optionally resets the controller state after updating gains.
 *
 * @param[in] handle           PID controller handle.
 * @param[in] reset_on_change  If true, resets the controller state after updating gains.
 * @param[in] kp               New proportional gain.
 * @param[in] ki               New integral gain.
 * @param[in] kd               New derivative gain.
 *
 * @retval ESP_OK              Success.
 * @retval ESP_ERR_INVALID_ARG If the handle is NULL or any gain is invalid.
 *
 * @note This function is not thread-safe; external synchronization is required if used concurrently.
 */
esp_err_t pid_control_set_gains(pid_control_handle handle, bool reset_on_change, float kp, float ki, float kd);

/**
 * @brief Set the anti-windup gain (kaw) for back-calculation.
 *
 * Set kaw to 0 to disable back-calculation.
 *
 * @param[in] handle PID controller handle.
 * @param[in] kaw    New anti-windup gain.
 *
 * @retval ESP_OK              Success.
 * @retval ESP_ERR_INVALID_ARG If the handle is NULL or kaw is invalid.
 *
 * @note This function is not thread-safe; external synchronization is required if used concurrently.
 */
esp_err_t pid_control_set_anti_windup(pid_control_handle handle, float kaw);

/**
 * @brief Set new output limits for the controller.
 *
 * The output will be clamped to [u_min, u_max] after each update.
 *
 * @param[in] handle PID controller handle.
 * @param[in] u_min  New minimum output value.
 * @param[in] u_max  New maximum output value.
 *
 * @retval ESP_OK              Success.
 * @retval ESP_ERR_INVALID_ARG If the handle is NULL or limits are invalid.
 *
 * @note This function is not thread-safe; external synchronization is required if used concurrently.
 */
esp_err_t pid_control_set_output_limits(pid_control_handle handle, float u_min, float u_max);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // PID_CONTROL_H
