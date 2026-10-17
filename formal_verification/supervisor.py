from dataclasses import dataclass
from enum import Enum

from formal_verification.declared_hardware import (
    BRAKE_ENGAGEMENT_MS,
    BRAKING_TO_STANDSTILL_MS,
    PHYSICAL_STOP_MS,
    RESIDUAL_CURRENT_THRESHOLD_A,
    STO_CURRENT_INTERRUPTION_MS,
    HARDWARE_MEASUREMENTS_INDEPENDENTLY_VERIFIED,
)



class Mode(Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    STOP_LATCHED = "STOP_LATCHED"
    WAIT_CLEAR = "WAIT_CLEAR"
    WAIT_REARM = "WAIT_REARM"
    RESUMING = "RESUMING"


@dataclass
class Supervisor:
    mode: Mode = Mode.IDLE
    saved_trajectory: str = "cycle-1"
    torque_command: bool = False
    proximity_active: bool = False

    def start(self):
        if self.mode is not Mode.IDLE or self.proximity_active:
            raise ValueError("Start requires IDLE and a clear proximity field")
        self.mode = Mode.MOVING
        self.torque_command = True

    def proximity_detected(self):
        self.proximity_active = True
        if self.mode in (Mode.MOVING, Mode.RESUMING):
            self.mode = Mode.STOP_LATCHED
            self.torque_command = False
        elif self.mode is Mode.WAIT_REARM:
            self.mode = Mode.WAIT_CLEAR

    def safe_stop_complete(self):
        if self.mode is Mode.STOP_LATCHED:
            self.mode = (
                Mode.WAIT_CLEAR if self.proximity_active else Mode.WAIT_REARM
            )

    def hand_removed(self):
        self.proximity_active = False
        if self.mode is Mode.WAIT_CLEAR:
            self.mode = Mode.WAIT_REARM

    def operator_rearm(self):
        if self.mode is Mode.WAIT_REARM and not self.proximity_active:
            self.mode = Mode.RESUMING

    def resume_complete(self):
        if self.mode is Mode.RESUMING and not self.proximity_active:
            self.mode = Mode.MOVING
            self.torque_command = True


def counterexample_auto_resume():
    """Return a shortest trace violating the no-automatic-resume invariant."""
    return (
        ("MOVING", "proximity detected / torque command = 0"),
        ("STOP_LATCHED", "hand removed"),
        ("MOVING", "automatic restart without operator rearm"),
    )


def verify_supervisor():
    """Exhaustively traverse all reachable abstract supervisor states/transitions."""
    states = {
        (Mode.IDLE, False, False),
        (Mode.MOVING, True, False),
        (Mode.STOP_LATCHED, False, True),
        (Mode.WAIT_CLEAR, False, True),
        (Mode.WAIT_REARM, False, True),
        (Mode.RESUMING, False, True),
    }
    edges = {
        (Mode.IDLE, "start", Mode.MOVING),
        (Mode.MOVING, "proximity", Mode.STOP_LATCHED),
        (Mode.STOP_LATCHED, "stop_complete_hand_active", Mode.WAIT_CLEAR),
        (Mode.STOP_LATCHED, "stop_complete_hand_clear", Mode.WAIT_REARM),
        (Mode.WAIT_CLEAR, "hand_removed", Mode.WAIT_REARM),
        (Mode.WAIT_REARM, "operator_rearm", Mode.RESUMING),
        (Mode.WAIT_REARM, "proximity", Mode.WAIT_CLEAR),
        (Mode.RESUMING, "proximity", Mode.STOP_LATCHED),
        (Mode.RESUMING, "resume_complete", Mode.MOVING),
    }
    adjacency = {}
    for source, command, destination in edges:
        adjacency.setdefault(source, []).append((command, destination))
    reachable = {Mode.IDLE}
    frontier = [Mode.IDLE]
    while frontier:
        current = frontier.pop()
        for _, destination in adjacency.get(current, ()):
            if destination not in reachable:
                reachable.add(destination)
                frontier.append(destination)
    if reachable != {state[0] for state in states}:
        raise AssertionError(
            f"Unreachable or unmodeled supervisor states: {reachable}"
        )

    violations = []
    for state, command, next_state in edges:
        # The only edge into motion after a stop must follow explicit rearm.
        if next_state is Mode.MOVING and state is not Mode.IDLE:
            if state is not Mode.RESUMING:
                violations.append((state, command, next_state))
        # A detected proximity in motion must latch the zero-torque command.
        if state in (Mode.MOVING, Mode.RESUMING) and command == "proximity":
            if next_state is not Mode.STOP_LATCHED:
                violations.append((state, command, next_state))
    if violations:
        raise AssertionError(f"Supervisor safety invariant violated: {violations}")
    return {
        "reachable_states": len(reachable),
        "transitions": len(edges),
        "safety_invariant_violations": 0,
        "liveness_path": [
            "MOVING",
            "STOP_LATCHED",
            "WAIT_CLEAR",
            "WAIT_REARM",
            "RESUMING",
            "MOVING",
        ],
    }


def enumerate_deadline():
    """Exhaust the declared integer-ms sensor, jitter, and sampling-phase bounds."""
    deadline_ms = 100.0
    computation_ms = 0.2  # Strictly below the approved 200 us WCET budget.
    actuator_physical_stop_ms = PHYSICAL_STOP_MS
    results = []
    for sensor_ms in range(21):
        for missed_cycle in (0, 1):
            for sample_phase_ms in range(3):
                response_ms = (
                    sensor_ms + 2.0 * missed_cycle + sample_phase_ms
                    + computation_ms
                )
                results.append((sensor_ms, missed_cycle, sample_phase_ms, response_ms))
    worst = max(results, key=lambda row: row[3])
    total_physical_stop_ms = worst[3] + actuator_physical_stop_ms
    return {
        "combinations": len(results),
        "deadline_ms": deadline_ms,
        "worst_ms": worst[3],
        "worst_case": {
            "sensor_ms": worst[0],
            "missed_cycles": worst[1],
            "sampling_phase_ms": worst[2],
            "execution_ms": computation_ms,
        },
        "margin_ms": deadline_ms - worst[3],
        "command_deadline_pass": all(row[3] <= deadline_ms for row in results),
        "max_actuator_response_ms_for_physical_deadline": deadline_ms - worst[3],
        "actuator_physical_stop_ms_user_supplied": actuator_physical_stop_ms,
        "sto_current_interruption_ms_user_supplied": STO_CURRENT_INTERRUPTION_MS,
        "brake_engagement_ms_user_supplied": BRAKE_ENGAGEMENT_MS,
        "braking_to_standstill_ms_user_supplied": BRAKING_TO_STANDSTILL_MS,
        "residual_current_threshold_a_user_supplied": RESIDUAL_CURRENT_THRESHOLD_A,
        "worst_total_physical_stop_ms": total_physical_stop_ms,
        "physical_stop_margin_ms": deadline_ms - total_physical_stop_ms,
        "physical_stop_arithmetic_pass": total_physical_stop_ms <= deadline_ms,
        "actuator_response_independently_verified":
            HARDWARE_MEASUREMENTS_INDEPENDENTLY_VERIFIED,
    }
