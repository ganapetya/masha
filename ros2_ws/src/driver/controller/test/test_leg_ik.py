# Catch-up beside leg_ik.py. No robot, and no kinematics.so.
# A refused target is a returned mini-step result. The generator is not involved.

import math
import sys

import pytest

from controller.leg_ik import (
    BISECTION_PARTS,
    TICK_S,
    TRACE_COLUMNS,
    TRACE_PATH_DEFAULT,
    TRACE_QUEUE_BOUND,
    USE_HEXAPOD_KINEMATICS_DEFAULT,
    MiniStepResult,
    generators_own_feet_sent,
    last_safe_pose_kept,
    shorter_point_sent,
    HexapodLegIk,
    HoldWatch,
    IkDecision,
    SecondLog,
    TraceWriter,
    VendorLegIk,
    approach_target,
    begin_generator,
    make_leg_ik,
    odometry_increment,
    should_publish,
    startup_line,
    trace_row,
)


def default_feet():
    """build_in_pose.py DEFAULT_POSE, from the same offsets."""
    x1 = 93.60
    y1 = 50.805
    y2 = 73.535
    ix = 70.0
    iy = 110.0
    height = 70.0
    front_y = iy + y1 - 20.0
    return (
        (ix + x1, front_y, -height),
        (0.0, iy + y2, -height),
        (-ix - x1, front_y, -height),
        (-ix - x1, -front_y, -height),
        (0.0, -(iy + y2), -height),
        (ix + x1, -front_y, -height),
    )


# Vendor tape for those feet. Corners and middles differ.
STAND = (
    (0.1206814507, 0.7554684412, -0.6500234354),
    (0.0, 0.7615987475, -0.6870318910),
    (-0.1206814507, 0.7554684412, -0.6500234354),
    (0.1206814507, 0.7554684412, -0.6500234354),
    (0.0, 0.7615987475, -0.6870318910),
    (-0.1206814507, 0.7554684412, -0.6500234354),
)


class Log(object):
    def __init__(self):
        self.warnings = []
        self.errors = []
        self.infos = []

    def warning(self, text):
        self.warnings.append(text)

    def error(self, text):
        self.errors.append(text)

    def info(self, text):
        self.infos.append(text)


class Clock(object):
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class ScriptedIk(object):
    """Allows every pose except the generator's own object, until told."""

    def __init__(self, target_feet_positions):
        self.target_feet_positions = target_feet_positions
        self.last_sent_joint_radians = (0.0,) * 18
        self.allow_target = False
        self.calls = []

    def solve(self, six_feet):
        self.calls.append(six_feet)
        if six_feet is self.target_feet_positions and not self.allow_target:
            return IkDecision(
                False, (1.0,) * 18, "rate_limit", 8, 3, 1.0, 0.0)
        return IkDecision(True, (0.1,) * 18, "", 0, 0, 0.0, 0.0)

    def remember_last_sent_radians(self, angles):
        self.last_sent_joint_radians = tuple(angles)


def _near_straight_feet():
    # 0.02 mm short of straight, level with the femur hinge, leg 6.
    # That close, a 0.5 mm slip moves the knee past the measured
    # joint limit. A foot 0.2 mm short does not.
    coxa = 45.0
    femur = 77.1
    tibia = 115.6
    reach = coxa + femur + tibia - 0.02
    yaw = -math.pi / 4.0
    feet = list(default_feet())
    feet[5] = (
        93.60 + reach * math.cos(yaw),
        -50.805 + reach * math.sin(yaw),
        1.13,
    )
    return tuple(feet)


def _rate_limit_target(ik, anchor):
    """A legal foot move whose full target exceeds one joint limit."""
    foot = anchor[0]
    for millimetres in (5, 10, 15, 20, 30, 40, 60, 80):
        target_feet_positions = list(anchor)
        target_feet_positions[0] = (foot[0] + millimetres, foot[1], foot[2])
        target_feet_positions = tuple(target_feet_positions)
        decision = ik.solve(target_feet_positions)
        if decision.reason == "rate_limit":
            return target_feet_positions
        if decision.reason not in ("",):
            raise AssertionError(decision.reason)
    raise AssertionError("no rate_limit target along +X")


def test_the_flag_defaults_to_the_vendor_backend():
    assert USE_HEXAPOD_KINEMATICS_DEFAULT is False
    assert "kinematics" not in sys.modules
    ik, backend, line = begin_generator()
    assert isinstance(ik, VendorLegIk)
    assert backend == "vendor"
    assert line == "gait 5 backend=vendor"
    assert "kinematics" not in sys.modules
    other, name, hex_line = begin_generator(True)
    assert isinstance(other, HexapodLegIk)
    assert name == "hexapod"
    assert hex_line == "gait 5 backend=hexapod"
    assert isinstance(make_leg_ik(False), VendorLegIk)


def test_startup_line_names_the_measured_peak():
    missing = startup_line(False, TRACE_PATH_DEFAULT, "unavailable")
    assert "measured_peak=unset" in missing
    text = startup_line(False, TRACE_PATH_DEFAULT, 0.06)
    assert "use_hexapod_kinematics=False" in text
    assert TRACE_PATH_DEFAULT in text
    assert "±120°" in text
    assert "max_step_rad=0.06" in text
    # safety.cpp stores 1.5 times the peak, so 0.06 names 0.04.
    assert "measured_peak=0.04" in text


def test_approach_solves_the_generator_feet_and_0996_stays_partial():
    target_feet_positions = tuple(default_feet())
    ik = ScriptedIk(target_feet_positions)
    mini_step_result = approach_target(
        ik, target_feet_positions, target_feet_positions, 0.0)
    assert ik.calls[0] is target_feet_positions
    assert mini_step_result.what_this_wake_published == shorter_point_sent
    assert mini_step_result.fraction_reached_along_the_target == pytest.approx(255.0 / 256.0)
    assert mini_step_result.fraction_reached_along_the_target < 1.0
    assert mini_step_result.fraction_of_target_sent == mini_step_result.fraction_reached_along_the_target
    assert mini_step_result.publish is True
    assert mini_step_result.reason == "rate_limit"
    # A later mini-step on the same target still waits for those feet.
    later = approach_target(
        ik, target_feet_positions, target_feet_positions,
        mini_step_result.fraction_of_target_sent)
    assert later.what_this_wake_published == shorter_point_sent
    assert later.fraction_reached_along_the_target < 1.0
    assert later.fraction_of_target_sent > mini_step_result.fraction_of_target_sent
    ik.allow_target = True
    done = approach_target(
        ik, target_feet_positions, target_feet_positions, later.fraction_of_target_sent)
    assert done.what_this_wake_published == generators_own_feet_sent
    assert done.fraction_reached_along_the_target == 1.0
    assert done.fraction_of_target_sent == 1.0
    assert ik.calls[-1] is target_feet_positions
    assert should_publish(done, dry_run_do_not_publish=True) is False
    assert should_publish(done, dry_run_do_not_publish=False) is True


def test_odometry_leftover_is_not_a_full_mini_step():
    twist = (0.12, -0.05, 0.6)
    full = odometry_increment(*twist, fraction_sent_this_wake=1.0)
    early = odometry_increment(*twist, fraction_sent_this_wake=0.25)
    rest = odometry_increment(*twist, fraction_sent_this_wake=0.75)
    hold = odometry_increment(*twist, fraction_sent_this_wake=0.0)
    assert hold == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    for whole, left, right in zip(full, early, rest):
        assert whole == pytest.approx(left + right)
    # Stored velocity is the command scaled by the fraction.
    assert rest[0] == pytest.approx(twist[0] * 0.75)
    # Integrated body move is that velocity over one mini-step.
    assert rest[3] == pytest.approx(twist[0] * TICK_S * 0.75)
    assert rest[3] != pytest.approx(twist[0] * TICK_S)


def test_a_full_queue_drops_without_blocking_and_warns_once_a_second():
    clock = Clock()
    log = Log()
    writer = TraceWriter(
        "/tmp/hexapod-ik-trace-unused.csv", log, clock=clock, start=False)
    row = ["x"]
    for _ in range(TRACE_QUEUE_BOUND):
        assert writer.put(row) is True
    assert writer.put(row) is False
    assert writer.put(row) is False
    assert len(log.warnings) == 1
    clock.now = 1.0
    assert writer.put(row) is False
    assert len(log.warnings) == 2
    writer.close()


def test_the_writer_appends_a_header_and_a_row(tmp_path):
    path = tmp_path / "hexapod_ik_trace.csv"
    log = Log()
    writer = TraceWriter(str(path), log)
    mini_step_result = MiniStepResult(
        generators_own_feet_sent, 1.0, 1.0, (0.1,) * 18, "", 0, 0, 0.0, 0.0,
        default_feet(), default_feet(), ("",) * 18, True)
    row = trace_row(1.5, "vendor", mini_step_result)
    assert len(row) == len(TRACE_COLUMNS)
    assert writer.put(row) is True
    writer.close()
    assert not writer._thread.is_alive()
    text = path.read_text().splitlines()
    assert text[0].split(",")[0:6] == list(TRACE_COLUMNS[0:6])
    assert text[1].split(",")[1] == "vendor"
    assert text[1].split(",")[2] == "sent"
    # Opening again appends, and does not repeat the header.
    again = TraceWriter(str(path), log)
    again.put(row)
    again.close()
    lines = path.read_text().splitlines()
    assert lines[0].startswith("t_s,")
    assert sum(1 for line in lines if line.startswith("t_s,")) == 1
    assert len(lines) == 3


def test_five_hard_holds_log_one_error_and_a_partial_resets():
    log = Log()
    watch = HoldWatch()
    feet = default_feet()
    held = MiniStepResult(
        last_safe_pose_kept, 0.0, 0.0, (0.0,) * 18, "unreachable", 0, 4,
        0.0, 0.0, feet, feet, ("",) * 18, False)
    for _ in range(4):
        watch.note(log, held)
    assert watch.errors == 0
    assert len(log.warnings) == 4
    watch.note(log, held)
    assert watch.errors == 1
    assert log.errors == [
        "holding last safe pose, generator target frozen"]
    watch.note(log, held)
    assert watch.errors == 1
    partial = MiniStepResult(
        shorter_point_sent, 0.5, 0.5, (0.1,) * 18, "rate_limit", 8, 3,
        1.0, 0.0, feet, feet, (0.01,) * 18, True)
    watch.note(log, partial)
    assert watch.streak == 0
    for _ in range(5):
        watch.note(log, held)
    assert watch.errors == 2
    assert should_publish(held) is False


def test_second_log_prints_the_finished_second():
    log = Log()
    second = SecondLog()
    second.note(log, 0.2, "vendor", generators_own_feet_sent, (0.01, ""))
    second.note(log, 0.8, "vendor", shorter_point_sent, (0.2,))
    assert log.infos == []
    second.note(log, 1.05, "vendor", generators_own_feet_sent, (0.0,))
    assert log.infos == ["backend=vendor decision=partial max_dq=0.2000"]


def test_stand_is_sent_and_matches_the_vendor_tape():
    ik = HexapodLegIk()
    feet = default_feet()
    reference = ik.establish_reference(feet)
    assert reference.gate_allows_these_radians is True
    mini_step_result = approach_target(ik, feet, feet, 0.0)
    assert mini_step_result.what_this_wake_published == generators_own_feet_sent
    assert mini_step_result.fraction_reached_along_the_target == 1.0
    assert mini_step_result.publish is True
    assert mini_step_result.reason == ""
    flat = []
    for triple in STAND:
        flat.extend(triple)
    for got, want in zip(mini_step_result.angles, flat):
        assert abs(got - want) <= 0.005


def test_unreachable_and_near_singular_hold_without_moving_alpha():
    ik = HexapodLegIk()
    feet = default_feet()
    assert ik.establish_reference(feet).gate_allows_these_radians is True
    last_sent_joint_radians = ik.last_sent_joint_radians
    far = list(feet)
    far[3] = (0.0, 0.0, 500.0)
    far = tuple(far)
    held = approach_target(ik, feet, far, 0.25)
    assert held.what_this_wake_published == last_safe_pose_kept
    assert held.reason == "unreachable"
    assert held.leg == 4
    assert held.fraction_reached_along_the_target == 0.0
    assert held.fraction_of_target_sent == 0.25
    assert held.publish is False
    assert ik.last_sent_joint_radians is last_sent_joint_radians
    straight = _near_straight_feet()
    knee = approach_target(ik, feet, straight, 0.25)
    assert knee.what_this_wake_published == last_safe_pose_kept
    assert knee.reason == "near_singular"
    assert knee.leg == 6
    assert knee.fraction_of_target_sent == 0.25
    assert ik.last_sent_joint_radians is last_sent_joint_radians


def test_no_reference_holds_when_the_stand_was_not_solved():
    ik = HexapodLegIk()
    feet = default_feet()
    held = approach_target(ik, feet, feet, 0.0)
    assert held.what_this_wake_published == last_safe_pose_kept
    assert held.reason == "no_reference"
    assert held.publish is False
    assert ik.last_sent_joint_radians is None


def test_a_long_legal_target_sums_its_alphas_to_one():
    ik = HexapodLegIk()
    anchor = default_feet()
    assert ik.establish_reference(anchor).gate_allows_these_radians is True
    target_feet_positions = _rate_limit_target(ik, anchor)
    fraction_of_target_sent = 0.0
    deltas = []
    first = None
    finished = None
    for _ in range(BISECTION_PARTS + 8):
        before = fraction_of_target_sent
        mini_step_result = approach_target(
            ik, anchor, target_feet_positions, fraction_of_target_sent)
        assert mini_step_result.what_this_wake_published != last_safe_pose_kept
        if first is None:
            first = mini_step_result
        deltas.append(mini_step_result.fraction_of_target_sent - before)
        fraction_of_target_sent = mini_step_result.fraction_of_target_sent
        if mini_step_result.what_this_wake_published == generators_own_feet_sent:
            finished = mini_step_result
            break
    assert first.what_this_wake_published == shorter_point_sent
    assert first.fraction_reached_along_the_target < 1.0
    assert first.publish is True
    assert finished is not None
    assert finished.fraction_reached_along_the_target == 1.0
    assert finished.what_this_wake_published == generators_own_feet_sent
    assert sum(deltas) == pytest.approx(1.0)
    assert deltas[-1] == pytest.approx(1.0 - sum(deltas[:-1]))
    twist = (0.12, 0.10, 0.6)
    pieces = [odometry_increment(*twist, fraction_sent_this_wake=d) for d in deltas]
    whole = odometry_increment(*twist, fraction_sent_this_wake=1.0)
    for index in range(6):
        assert sum(piece[index] for piece in pieces) == pytest.approx(
            whole[index])
    last = pieces[-1]
    assert last[3] == pytest.approx(twist[0] * TICK_S * deltas[-1])
    assert last[3] != pytest.approx(twist[0] * TICK_S)


def test_a_same_foot_branch_flip_is_a_hold():
    # The standing branch does not change. A one-radian disagreement
    # on a foot that moves 1 mm is still about a radian at every
    # fraction, so no fraction passes and nothing is published.
    ik = HexapodLegIk()
    feet = default_feet()
    assert ik.establish_reference(feet).gate_allows_these_radians is True
    shifted = list(ik.last_sent_joint_radians)
    shifted[1] = shifted[1] + 1.0
    ik.last_sent_joint_radians = tuple(shifted)
    target_feet_positions = list(feet)
    target_feet_positions[0] = (feet[0][0] + 1.0, feet[0][1], feet[0][2])
    target_feet_positions = tuple(target_feet_positions)
    before = ik.last_sent_joint_radians
    mini_step_result = approach_target(ik, feet, target_feet_positions, 0.0)
    assert mini_step_result.what_this_wake_published == last_safe_pose_kept
    assert mini_step_result.reason == "rate_limit"
    assert mini_step_result.fraction_reached_along_the_target == 0.0
    assert mini_step_result.fraction_of_target_sent == 0.0
    assert mini_step_result.publish is False
    assert ik.last_sent_joint_radians is before
    assert should_publish(mini_step_result) is False
