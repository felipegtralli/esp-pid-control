#include "pid_control.h"
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

void app_main(void) {
    _Alignas(PID_CONTROL_STORAGE_ALIGNMENT) uint8_t pid_control_storage[PID_CONTROL_STORAGE_SIZE];
    pid_control_config config = {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.01f,
        .kaw = 0.0f,
        .u_max = 100.0f,
        .u_min = -100.0f,
    };

    pid_control_handle pid = NULL;
    esp_err_t err = pid_control_init(pid_control_storage, sizeof(pid_control_storage), &pid, &config);
    if(err != ESP_OK) {
        // Handle error
        return;
    }

    static const float setpoint = 50.0f;
    float measurement = 0.0f;
    float control_output = 0.0f;
    while(1) {
        // Simulate a measurement (replace with actual sensor reading)
        // measurement = read_sensor();

        err = pid_control_update(pid, setpoint, measurement, &control_output);
        if(err != ESP_OK) {
            // Handle error
            break;
        }

        // Use control_output to drive the actuator (e.g., motor, heater)
        // apply_control(control_output);

        vTaskDelay(pdMS_TO_TICKS(100)); // Control loop delay
    }
}
