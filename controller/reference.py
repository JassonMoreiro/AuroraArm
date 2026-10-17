from dataclasses import dataclass
import math

from model.plant import (
    B,
    COLLAB_MOTOR_SPEED_COMMAND_MAX_RAD_S,
    KE,
    KT,
    L,
    N,
    R,
    TS,
    VMAX,
)

POSITION_KP = 32.0
SPEED_KP = 0.16
SPEED_KI = 3.62
SPEED_INTEGRAL_LIMIT_V = 48.0
CURRENT_CONTROL_LIMIT_A = 7.5


def clamp(value, low, high):
    return min(max(value, low), high)


@dataclass
class ControllerState:
    speed_integral_v: float = 0.0


def step(state, theta_ref_joint, theta_joint, omega_motor, current_motor,
         torque_load, collaborative_zone=True):
    """Return saturated armature voltage; integration uses conditional anti-windup."""
    position_error_motor_rad = (theta_ref_joint - theta_joint) * N
    speed_limit = (
        COLLAB_MOTOR_SPEED_COMMAND_MAX_RAD_S if collaborative_zone else 450.0
    )
    omega_ref = clamp(POSITION_KP * position_error_motor_rad,
                      -speed_limit, speed_limit)
    speed_error = omega_ref - omega_motor
    feedforward = KE * omega_ref + R * (B * omega_ref + torque_load) / KT
    unsaturated = feedforward + SPEED_KP * speed_error + state.speed_integral_v
    voltage = clamp(unsaturated, -VMAX, VMAX)

    # Enforce current limits by preventing voltage from driving current farther
    # beyond the measured limit. Hardware protection remains independently required.
    if current_motor >= CURRENT_CONTROL_LIMIT_A:
        voltage = min(voltage, R * current_motor + KE * omega_motor)
    elif current_motor <= -CURRENT_CONTROL_LIMIT_A:
        voltage = max(voltage, R * current_motor + KE * omega_motor)
    # Enforce the discrete armature-current bound before the next controller
    # update. The motor back-EMF is held at its measured sample value.
    decay = math.exp(-R * TS / L)
    current_high_voltage = KE * omega_motor + R * (
        CURRENT_CONTROL_LIMIT_A - decay * current_motor
    ) / (1.0 - decay)
    current_low_voltage = KE * omega_motor + R * (
        -CURRENT_CONTROL_LIMIT_A - decay * current_motor
    ) / (1.0 - decay)
    voltage = clamp(voltage, current_low_voltage, current_high_voltage)
    voltage = clamp(voltage, -VMAX, VMAX)

    integration = SPEED_KI * speed_error * TS
    drives_saturation_further = (
        voltage < unsaturated and integration > 0
    ) or (voltage > unsaturated and integration < 0)
    drives_current_limit_further = (
        current_motor >= CURRENT_CONTROL_LIMIT_A and integration > 0
    ) or (
        current_motor <= -CURRENT_CONTROL_LIMIT_A and integration < 0
    )
    if not drives_saturation_further and not drives_current_limit_further:
        state.speed_integral_v = clamp(
            state.speed_integral_v + integration,
            -SPEED_INTEGRAL_LIMIT_V,
            SPEED_INTEGRAL_LIMIT_V,
        )
    return voltage
