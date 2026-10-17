import unittest

from controller.reference import (
    CURRENT_CONTROL_LIMIT_A,
    ControllerState,
    step,
)
from model.plant import (
    A,
    COLLAB_LINEAR_MAX_M_S,
    COLLAB_MOTOR_SPEED_MAX_RAD_S,
    COLLAB_MOTOR_SPEED_COMMAND_MAX_RAD_S,
    N,
    INPUT_B,
    PAYLOAD_RADIUS_M,
    PROXIMITY_TRIGGER_DISTANCE_M,
    PlantState,
    TS,
    VMAX,
    critical_operating_point,
    controllability_rank,
    eigenvalues_2x2,
    observability_rank,
    rk4_step,
)
from formal_verification.supervisor import enumerate_deadline


class PlantTests(unittest.TestCase):
    def test_full_stop_budget_includes_execution_time(self):
        timing = enumerate_deadline()
        self.assertAlmostEqual(timing["worst_ms"], 24.2)
        self.assertAlmostEqual(timing["actuator_physical_stop_ms_user_supplied"], 72.0)
        self.assertAlmostEqual(timing["worst_total_physical_stop_ms"], 96.2)
        self.assertAlmostEqual(timing["physical_stop_margin_ms"], 3.8)

    def test_state_space_and_ranks(self):
        self.assertAlmostEqual(A[0][0], -666.6666666667)
        self.assertAlmostEqual(INPUT_B[0], 833.3333333333)
        self.assertEqual(controllability_rank(), 2)
        self.assertEqual(observability_rank(), 2)
        self.assertEqual(len(eigenvalues_2x2()), 2)

    def test_critical_operating_point(self):
        operating_point = critical_operating_point()
        self.assertAlmostEqual(operating_point["current_a"], 3.7352941176)
        self.assertAlmostEqual(operating_point["voltage_v"], 41.2382352941)
        self.assertGreater(operating_point["voltage_headroom_v"], 0)
        self.assertGreater(operating_point["current_headroom_a"], 0)
        self.assertAlmostEqual(PROXIMITY_TRIGGER_DISTANCE_M, 0.5)

    def test_rk4_zero_input_decay(self):
        state = PlantState(0.0, 0.0, 0.0)
        updated = rk4_step(state, 0.0, 0.0)
        self.assertEqual(updated.motor_angle_rad, 0.0)
        self.assertEqual(updated.motor_speed_rad_s, 0.0)

    def test_position_step_performance_and_actuator_limits(self):
        load_nm = 0.25
        state = PlantState(load_nm / 0.085, 0.0, 0.0)
        controller = ControllerState()
        target = 0.1
        output = []
        currents = []
        voltages = []
        motor_speeds = []
        for _ in range(1000):
            voltage = step(
                controller, target, state.joint_angle_rad,
                state.motor_speed_rad_s, state.current_a, load_nm, True,
            )
            voltages.append(voltage)
            for _ in range(10):
                state = rk4_step(state, voltage, load_nm, TS / 10)
                motor_speeds.append(state.motor_speed_rad_s)
                currents.append(state.current_a)
            output.append(state.joint_angle_rad)

        overshoot = max(output) - target
        settled_at = next(
            (
                index for index, value in enumerate(output)
                if all(abs(sample - target) <= 0.02 * target for sample in output[index:])
            ),
            None,
        )
        self.assertLessEqual(overshoot / target * 100, 8.0)
        self.assertIsNotNone(settled_at)
        self.assertLessEqual(settled_at * TS, 0.300)
        self.assertLessEqual(abs(output[-1] - target), 1e-5)
        self.assertLessEqual(max(map(abs, voltages)), VMAX)
        self.assertLessEqual(max(map(abs, currents)), CURRENT_CONTROL_LIMIT_A)
        measured_linear_peak = max(map(abs, motor_speeds)) * PAYLOAD_RADIUS_M / N
        self.assertLess(COLLAB_MOTOR_SPEED_COMMAND_MAX_RAD_S,
                        COLLAB_MOTOR_SPEED_MAX_RAD_S)
        self.assertLessEqual(measured_linear_peak, COLLAB_LINEAR_MAX_M_S)


if __name__ == "__main__":
    unittest.main()
