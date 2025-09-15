# ESP-IDF PID control

MISRA C:2012-compliant PID controller library, designed for use with ESP-IDF.

## Overview
This library implements an incremental (velocity-form) PID controller with optional anti-windup (back-calculation). 

- Written in standard C, with C++ compatibility
- No dynamic memory allocation: user provides statically-allocated, aligned storage
- Configurable gains and output limits at runtime
- Opaque handle API for safety and encapsulation
- All MISRA C:2012 deviations are documented

## Quick Start
```c
#include "pid_control.h"

_Alignas(PID_CONTROL_STORAGE_ALIGNMENT) uint8_t storage[PID_CONTROL_STORAGE_SIZE];
pid_control_config config = {
    .kp = 1.0f, .ki = 0.1f, .kd = 0.01f, .kaw = 0.0f, .u_min = -100.0f, .u_max = 100.0f
};
pid_control_handle pid = NULL;
if (pid_control_init(storage, sizeof(storage), &pid, &config) == ESP_OK) {
    float u = 0.0f, setpoint = 50.f, measurement = 0.0f;
    pid_control_update(pid, setpoint, measument, &u);
    // Use u as your control output
}
```

See [`examples/basic`](examples/basic) for a complete example with a control loop.

## API Reference
All functions and configuration options are documented in [`include/pid_control.h`](include/pid_control.h).

## MISRA Compliance
All deviations from MISRA C:2012 are listed and justified in [`docs/deviations.md`](docs/deviations.md).

## License
MIT License. See [LICENSE](LICENSE) for details.
