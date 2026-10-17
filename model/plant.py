from dataclasses import dataclass
import math

R = 0.8
L = 1.2e-3
KT = 0.085
KE = 0.085
J = 4.2e-4
B = 1.5e-4
N = 100.0
VMAX = 48.0
IMAX = 8.0
TS = 0.002
PAYLOAD_RADIUS_M = 0.35
COLLAB_LINEAR_MAX_M_S = 0.250
PROXIMITY_TRIGGER_DISTANCE_M = 0.500
COLLAB_MOTOR_SPEED_MAX_RAD_S = (
    COLLAB_LINEAR_MAX_M_S * N / PAYLOAD_RADIUS_M
)
# 8% speed-reference margin is evaluated against modeled transient overshoot.
COLLAB_MOTOR_SPEED_COMMAND_MAX_RAD_S = (
    0.92 * COLLAB_MOTOR_SPEED_MAX_RAD_S
)
CRITICAL_MOTOR_SPEED_RAD_S = 450.0
CRITICAL_LOAD_N_M = 0.25

A = ((-R / L, -KE / L), (KT / J, -B / J))
INPUT_B = (1.0 / L, 0.0)
OUTPUT_C = (0.0, 1.0)
OUTPUT_D = 0.0


@dataclass
class PlantState:
    current_a: float
    motor_speed_rad_s: float
    motor_angle_rad: float

    @property
    def joint_angle_rad(self) -> float:
        return self.motor_angle_rad / N


def derivative(state: PlantState, voltage_v: float, load_nm: float):
    return (
        (voltage_v - R * state.current_a - KE * state.motor_speed_rad_s) / L,
        (KT * state.current_a - B * state.motor_speed_rad_s - load_nm) / J,
        state.motor_speed_rad_s,
    )


def _add(state, slope, scale):
    return PlantState(
        state.current_a + scale * slope[0],
        state.motor_speed_rad_s + scale * slope[1],
        state.motor_angle_rad + scale * slope[2],
    )


def rk4_step(state: PlantState, voltage_v: float, load_nm: float, dt=TS):
    """Integrate one sample using four deterministic RK stages."""
    k1 = derivative(state, voltage_v, load_nm)
    k2 = derivative(_add(state, k1, dt / 2), voltage_v, load_nm)
    k3 = derivative(_add(state, k2, dt / 2), voltage_v, load_nm)
    k4 = derivative(_add(state, k3, dt), voltage_v, load_nm)
    return PlantState(
        state.current_a + dt * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6,
        state.motor_speed_rad_s
        + dt * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6,
        state.motor_angle_rad
        + dt * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6,
    )


def eigenvalues_2x2(matrix=A):
    """Return real eigenvalues of the coupled two-state plant matrix."""
    a, b = matrix[0]
    c, d = matrix[1]
    discriminant = (a + d) ** 2 - 4 * (a * d - b * c)
    if discriminant < 0:
        raise ValueError("Expected real plant poles for this parameter set")
    root = math.sqrt(discriminant)
    return ((a + d + root) / 2, (a + d - root) / 2)


def controllability_rank():
    """Exact 2x2 rank test for [B, AB]."""
    a, b = A[0]
    c, d = A[1]
    b0, b1 = INPUT_B
    ab0, ab1 = a * b0 + b * b1, c * b0 + d * b1
    determinant = b0 * ab1 - b1 * ab0
    return 2 if abs(determinant) > 1e-12 else (1 if b0 or b1 else 0)


def observability_rank():
    """Exact 2x2 rank test for [C; CA], with speed as measured output."""
    c0, c1 = OUTPUT_C
    ca0 = c0 * A[0][0] + c1 * A[1][0]
    ca1 = c0 * A[0][1] + c1 * A[1][1]
    determinant = c0 * ca1 - c1 * ca0
    return 2 if abs(determinant) > 1e-12 else (1 if c0 or c1 else 0)


def critical_operating_point():
    current = (B * CRITICAL_MOTOR_SPEED_RAD_S + CRITICAL_LOAD_N_M) / KT
    voltage = R * current + KE * CRITICAL_MOTOR_SPEED_RAD_S
    return {
        "current_a": current,
        "voltage_v": voltage,
        "voltage_headroom_v": VMAX - voltage,
        "current_headroom_a": IMAX - current,
    }
