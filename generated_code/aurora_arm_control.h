/*
 * GENERATED FILE. Do not edit by hand.
 * Generator: generated_code/generate_c.py
 * Source: controller/reference.py
 * Requirements: REQ-CTRL-AA-001..004, REQ-SAFE-AA-004
 * Academic student project; not certified safety firmware.
 * Source SHA-256: 247ffb0182de37d025d7e6e0d1ab813d6aa4dfd76d63a4a8b8a71b44d8c28a8b
 */
#ifndef AURORA_ARM_CONTROL_H
#define AURORA_ARM_CONTROL_H

#if defined(_WIN32)
#define AURORA_API __declspec(dllexport)
#else
#define AURORA_API __attribute__((visibility("default")))
#endif

typedef struct {
    float speed_integral_v;
} AuroraControllerState;

AURORA_API void aurora_controller_init(AuroraControllerState *state);
AURORA_API float aurora_controller_step(AuroraControllerState *state,
    float theta_ref_joint, float theta_joint, float omega_motor,
    float current_motor, float torque_load, int collaborative_zone);

#endif
