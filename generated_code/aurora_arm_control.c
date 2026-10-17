/*
 * GENERATED FILE. Do not edit by hand.
 * Generator: generated_code/generate_c.py
 * Source: controller/reference.py
 * Requirements: REQ-CTRL-AA-001..004, REQ-SAFE-AA-004
 * Academic student project; not certified safety firmware.
 * Source SHA-256: 247ffb0182de37d025d7e6e0d1ab813d6aa4dfd76d63a4a8b8a71b44d8c28a8b
 */
#include "aurora_arm_control.h"

#include <math.h>
#define POSITION_KP 32.0f
#define SPEED_KP 0.16f
#define SPEED_KI 3.62f
#define SPEED_I_LIMIT 48.0f
#define N_REDUCTION 100.0f
#define MOTOR_SPEED_LIMIT 65.7142857f
#define R_ARM 0.8f
#define L_ARM 0.0012f
#define KT_MOTOR 0.085f
#define KE_MOTOR 0.085f
#define B_MOTOR 0.00015f
#define TS_CONTROL 0.002f
#define V_LIMIT 48.0f
#define I_LIMIT 7.5f

static float clampf(float x, float low, float high) {
    return x < low ? low : (x > high ? high : x);
}

void aurora_controller_init(AuroraControllerState *state) {
    state->speed_integral_v = 0.0f;
}

float aurora_controller_step(AuroraControllerState *state,
    float theta_ref_joint, float theta_joint, float omega_motor,
    float current_motor, float torque_load, int collaborative_zone) {
    const float position_error = (theta_ref_joint - theta_joint) * N_REDUCTION;
    const float speed_limit = collaborative_zone ? MOTOR_SPEED_LIMIT : 450.0f;
    const float omega_ref = clampf(POSITION_KP * position_error,
        -speed_limit, speed_limit);
    const float speed_error = omega_ref - omega_motor;
    const float feedforward = KE_MOTOR * omega_ref
        + R_ARM * (B_MOTOR * omega_ref + torque_load) / KT_MOTOR;
    const float unsaturated = feedforward + SPEED_KP * speed_error
        + state->speed_integral_v;
    float voltage = clampf(unsaturated, -V_LIMIT, V_LIMIT);
    if (current_motor >= I_LIMIT) {
        const float holding_voltage = R_ARM * current_motor
            + KE_MOTOR * omega_motor;
        if (voltage > holding_voltage) voltage = holding_voltage;
    } else if (current_motor <= -I_LIMIT) {
        const float holding_voltage = R_ARM * current_motor
            + KE_MOTOR * omega_motor;
        if (voltage < holding_voltage) voltage = holding_voltage;
    }
    const float current_decay = expf(-R_ARM * TS_CONTROL / L_ARM);
    const float current_high_voltage = KE_MOTOR * omega_motor + R_ARM
        * (I_LIMIT - current_decay * current_motor) / (1.0f - current_decay);
    const float current_low_voltage = KE_MOTOR * omega_motor + R_ARM
        * (-I_LIMIT - current_decay * current_motor) / (1.0f - current_decay);
    voltage = clampf(voltage, current_low_voltage, current_high_voltage);
    voltage = clampf(voltage, -V_LIMIT, V_LIMIT);
    const float integration = SPEED_KI * speed_error * TS_CONTROL;
    const int voltage_windup = (voltage < unsaturated && integration > 0.0f)
        || (voltage > unsaturated && integration < 0.0f);
    const int current_windup = (current_motor >= I_LIMIT && integration > 0.0f)
        || (current_motor <= -I_LIMIT && integration < 0.0f);
    if (!voltage_windup && !current_windup) {
        state->speed_integral_v = clampf(state->speed_integral_v + integration,
            -SPEED_I_LIMIT, SPEED_I_LIMIT);
    }
    return voltage;
}
