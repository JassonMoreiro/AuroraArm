from pathlib import Path
import hashlib
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.reference import (  # noqa: E402
    POSITION_KP,
    CURRENT_CONTROL_LIMIT_A,
    SPEED_KI,
    SPEED_KP,
    SPEED_INTEGRAL_LIMIT_V,
)
from model.plant import (  # noqa: E402
    B,
    COLLAB_MOTOR_SPEED_COMMAND_MAX_RAD_S,
    KE,
    L,
    KT,
    N,
    R,
    TS,
    VMAX,
)

OUTPUT = Path(__file__).resolve().parent
SOURCE = ROOT / "controller" / "reference.py"
digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()


def c_float(value):
    literal = f"{value:.9g}"
    if "." not in literal and "e" not in literal.lower():
        literal += ".0"
    return literal + "f"


header = f"""/*
 * GENERATED FILE. Do not edit by hand.
 * Generator: generated_code/generate_c.py
 * Source: controller/reference.py
 * Requirements: REQ-CTRL-AA-001..004, REQ-SAFE-AA-004
 * Academic student project; not certified safety firmware.
 * Source SHA-256: {digest}
 */
#ifndef AURORA_ARM_CONTROL_H
#define AURORA_ARM_CONTROL_H

#if defined(_WIN32)
#define AURORA_API __declspec(dllexport)
#else
#define AURORA_API __attribute__((visibility("default")))
#endif

typedef struct {{
    float speed_integral_v;
}} AuroraControllerState;

AURORA_API void aurora_controller_init(AuroraControllerState *state);
AURORA_API float aurora_controller_step(AuroraControllerState *state,
    float theta_ref_joint, float theta_joint, float omega_motor,
    float current_motor, float torque_load, int collaborative_zone);

#endif
"""

source = f"""/*
 * GENERATED FILE. Do not edit by hand.
 * Generator: generated_code/generate_c.py
 * Source: controller/reference.py
 * Requirements: REQ-CTRL-AA-001..004, REQ-SAFE-AA-004
 * Academic student project; not certified safety firmware.
 * Source SHA-256: {digest}
 */
#include "aurora_arm_control.h"

#include <math.h>
#define POSITION_KP {c_float(POSITION_KP)}
#define SPEED_KP {c_float(SPEED_KP)}
#define SPEED_KI {c_float(SPEED_KI)}
#define SPEED_I_LIMIT {c_float(SPEED_INTEGRAL_LIMIT_V)}
#define N_REDUCTION {c_float(N)}
#define MOTOR_SPEED_LIMIT {c_float(COLLAB_MOTOR_SPEED_COMMAND_MAX_RAD_S)}
#define R_ARM {c_float(R)}
#define L_ARM {c_float(L)}
#define KT_MOTOR {c_float(KT)}
#define KE_MOTOR {c_float(KE)}
#define B_MOTOR {c_float(B)}
#define TS_CONTROL {c_float(TS)}
#define V_LIMIT {c_float(VMAX)}
#define I_LIMIT {c_float(CURRENT_CONTROL_LIMIT_A)}

static float clampf(float x, float low, float high) {{
    return x < low ? low : (x > high ? high : x);
}}

void aurora_controller_init(AuroraControllerState *state) {{
    state->speed_integral_v = 0.0f;
}}

float aurora_controller_step(AuroraControllerState *state,
    float theta_ref_joint, float theta_joint, float omega_motor,
    float current_motor, float torque_load, int collaborative_zone) {{
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
    if (current_motor >= I_LIMIT) {{
        const float holding_voltage = R_ARM * current_motor
            + KE_MOTOR * omega_motor;
        if (voltage > holding_voltage) voltage = holding_voltage;
    }} else if (current_motor <= -I_LIMIT) {{
        const float holding_voltage = R_ARM * current_motor
            + KE_MOTOR * omega_motor;
        if (voltage < holding_voltage) voltage = holding_voltage;
    }}
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
    if (!voltage_windup && !current_windup) {{
        state->speed_integral_v = clampf(state->speed_integral_v + integration,
            -SPEED_I_LIMIT, SPEED_I_LIMIT);
    }}
    return voltage;
}}
"""

OUTPUT.mkdir(exist_ok=True)
(OUTPUT / "aurora_arm_control.h").write_text(header, encoding="ascii")
(OUTPUT / "aurora_arm_control.c").write_text(source, encoding="ascii")
print(f"Generated C from {SOURCE.relative_to(ROOT)}")
print(f"Source SHA-256: {digest}")
