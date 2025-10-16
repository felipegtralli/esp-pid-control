
# MISRA C:2012 Deviations Report

## Project: esp-pid-control
## Module: pid_control

This document formally records all known deviations from the MISRA C:2012 standard within the `pid_control` library. Each deviation is identified by rule number, with a summary, precise code location, rationale, and any relevant risk mitigation or safety considerations. This report is intended to support code reviews, audits, and compliance documentation.


### Directive 1.1 — Use of Language Extensions
**Rule Summary:** Only standard language features of ISO C shall be used.

**Location:**
- `src/pid_control.c` (file-level note)

**Rationale:**
The project may be compiled using the GNU dialect via ESP-IDF. However, this module is explicitly designed to avoid GNU-specific language extensions. This is a statement of intent and not a deviation, but is included for completeness and transparency.

**Risk Mitigation:**
No GNU-only constructs are present in this module. Ongoing code reviews ensure continued compliance.


### Rule 11.4 — Pointer to Integer Casts
**Rule Summary:** A cast should not be performed between a pointer to object type and a different pointer to object type (including pointer-to-integer and integer-to-pointer casts).

**Location:**
- `src/pid_control.c`, function: `is_aligned`

**Rationale:**
Pointer-to-integer casting is employed solely for the purpose of alignment checking. No pointer dereferencing occurs as a result of this cast; only address arithmetic is performed. This approach is necessary to ensure correct alignment in a portable and efficient manner.

**Risk Mitigation:**
The cast is safe in this context and does not compromise memory safety. The code is clearly annotated to indicate the deviation and its justification.


### Rule 11.5 — Conversion from void* to Object Pointer
**Rule Summary:** A cast shall not be performed that removes any const or volatile qualification from the type addressed by a pointer. Converting a `void*` to an object pointer type is generally discouraged unless validated.

**Location:**
- `src/pid_control.c`, function: `pid_control_init` (statement: `struct pid_control* ctrl = storage;`)

**Rationale:**
After validating the storage pointer and confirming its alignment and size, the `void*` is cast to the internal `struct pid_control*` type. This is a standard and necessary pattern for implementing opaque handle APIs in C.

**Risk Mitigation:**
The cast is performed only after comprehensive validation of the pointer. The code is explicitly commented to document the deviation and its safety.
