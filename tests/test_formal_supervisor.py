import unittest

from formal_verification.supervisor import (
    Mode,
    Supervisor,
    counterexample_auto_resume,
    enumerate_deadline,
    verify_supervisor,
)
from formal_verification.declared_hardware import (
    BRAKE_ENGAGEMENT_MS,
    BRAKING_TO_STANDSTILL_MS,
    PHYSICAL_STOP_MS,
    RESIDUAL_CURRENT_THRESHOLD_A,
    STO_CURRENT_INTERRUPTION_MS,
)
from model.plant import KT


class SupervisorTests(unittest.TestCase):
    def test_exhaustive_invariants_and_liveness(self):
        result = verify_supervisor()
        self.assertEqual(result["reachable_states"], 6)
        self.assertEqual(result["transitions"], 9)
        self.assertEqual(result["safety_invariant_violations"], 0)
        self.assertEqual(result["liveness_path"][-1], "MOVING")

    def test_no_restart_without_explicit_rearm(self):
        supervisor = Supervisor()
        supervisor.start()
        supervisor.proximity_detected()
        supervisor.safe_stop_complete()
        supervisor.hand_removed()
        self.assertEqual(supervisor.mode, Mode.WAIT_REARM)
        self.assertFalse(supervisor.torque_command)
        supervisor.operator_rearm()
        supervisor.resume_complete()
        self.assertEqual(supervisor.mode, Mode.MOVING)

    def test_preliminary_model_counterexample_is_preserved(self):
        trace = counterexample_auto_resume()
        self.assertIn("automatic restart", trace[-1][1])

    def test_proximity_during_resume_cancels_restart(self):
        supervisor = Supervisor()
        supervisor.start()
        supervisor.proximity_detected()
        supervisor.hand_removed()
        supervisor.safe_stop_complete()
        supervisor.operator_rearm()
        self.assertEqual(supervisor.mode, Mode.RESUMING)
        supervisor.proximity_detected()
        supervisor.resume_complete()
        self.assertEqual(supervisor.mode, Mode.STOP_LATCHED)
        self.assertFalse(supervisor.torque_command)

    def test_all_sensor_jitter_phase_combinations_meet_command_deadline(self):
        result = enumerate_deadline()
        self.assertEqual(result["combinations"], 126)
        self.assertEqual(result["worst_ms"], 24.2)
        self.assertAlmostEqual(result["margin_ms"], 75.8)
        self.assertTrue(result["command_deadline_pass"])
        self.assertEqual(result["actuator_physical_stop_ms_user_supplied"], 72.0)
        self.assertEqual(PHYSICAL_STOP_MS, 72.0)
        self.assertEqual(
            PHYSICAL_STOP_MS,
            STO_CURRENT_INTERRUPTION_MS
            + BRAKE_ENGAGEMENT_MS
            + BRAKING_TO_STANDSTILL_MS,
        )
        self.assertAlmostEqual(result["worst_total_physical_stop_ms"], 96.2)
        self.assertAlmostEqual(result["physical_stop_margin_ms"], 3.8)
        self.assertTrue(result["physical_stop_arithmetic_pass"])
        self.assertFalse(result["actuator_response_independently_verified"])
        self.assertEqual(result["residual_current_threshold_a_user_supplied"], 0.05)
        self.assertAlmostEqual(KT * RESIDUAL_CURRENT_THRESHOLD_A, 0.00425)


if __name__ == "__main__":
    unittest.main()
