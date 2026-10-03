# Catch-up beside leg_ik.py. No robot, and no kinematics.so.
# A refused frame is a returned step. The generator is not involved.

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
    FrameStep,
    HexapodLegIk,
    HoldWatch,
    IkDecision,
    SecondLog,
    TraceWriter,
    VendorLegIk,
    approach_frame,
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

    def __init__(self, frame):
        self.frame = frame
        self.previous = (0.0,) * 18
        self.allow_frame = False
        self.calls = []

    def solve(self, pose):
        self.calls.append(pose)
        if pose is self.frame and not self.allow_frame:
            return IkDecision(
                False, (1.0,) * 18, "rate_limit", 8, 3, 1.0, 0.0)
        return IkDecision(True, (0.1,) * 18, "", 0, 0, 0.0, 0.0)

    def remember(self, angles):
        self.previous = tuple(angles)


def _near_straight_feet():
    # 0.2 mm short of straight, level with the femur hinge, leg 6.
    coxa = 45.0
    femur = 77.1
    tibia = 115.6
    reach = coxa + femur + tibia - 0.2
    yaw = -math.pi / 4.0
    feet = list(default_feet())
    feet[5] = (
        93.60 + reach * math.cos(yaw),
        -50.805 + reach * math.sin(yaw),
        1.13,
    )
    return tuple(feet)


def _rate_limit_frame(ik, anchor):
    """A legal foot move whose full frame exceeds one step."""
    foot = anchor[0]
    for millimetres in (5, 10, 15, 20, 30, 40, 60, 80):
        frame = list(anchor)
        frame[0] = (foot[0] + millimetres, foot[1], foot[2])
        frame = tuple(frame)
        decision = ik.solve(frame)
        if decision.reason == "rate_limit":
            return frame
        if decision.reason not in ("",):
            raise AssertionError(decision.reason)
    raise AssertionError("no rate_limit frame along +X")


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


def test_startup_line_names_the_unset_peak():
    text = startup_line(False, TRACE_PATH_DEFAULT, 0.05)
    assert "use_hexapod_kinematics=False" in text
    assert TRACE_PATH_DEFAULT in text
    assert "±120°" in text
    assert "max_step_rad=0.05" in text
    assert "measured_peak=unset" in text


def test_step_one_solves_the_generator_feet_and_0996_stays_partial():
    frame = tuple(default_feet())
    ik = ScriptedIk(frame)
    step = approach_frame(ik, frame, frame, 0.0)
    assert ik.calls[0] is frame
    assert step.decision == "partial"
    assert step.alpha == pytest.approx(255.0 / 256.0)
    assert step.alpha < 1.0
    assert step.alpha_done == step.alpha
    assert step.publish is True
    assert step.reason == "rate_limit"
    # A later tick on the same frame still waits for those exact feet.
    later = approach_frame(ik, frame, frame, step.alpha_done)
    assert later.decision == "partial"
    assert later.alpha < 1.0
    assert later.alpha_done > step.alpha_done
    ik.allow_frame = True
    done = approach_frame(ik, frame, frame, later.alpha_done)
    assert done.decision == "sent"
    assert done.alpha == 1.0
    assert done.alpha_done == 1.0
    assert ik.calls[-1] is frame
    assert should_publish(done, pseudo=True) is False
    assert should_publish(done, pseudo=False) is True


def test_odometry_leftover_is_not_a_full_tick():
    twist = (0.12, -0.05, 0.6)
    full = odometry_increment(*twist, delta_alpha=1.0)
    early = odometry_increment(*twist, delta_alpha=0.25)
    rest = odometry_increment(*twist, delta_alpha=0.75)
    hold = odometry_increment(*twist, delta_alpha=0.0)
    assert hold == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    for whole, left, right in zip(full, early, rest):
        assert whole == pytest.approx(left + right)
    # Stored velocity is the command scaled by the fraction.
    assert rest[0] == pytest.approx(twist[0] * 0.75)
    # Integrated body step is that velocity over one tick.
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
    step = FrameStep(
        "sent", 1.0, 1.0, (0.1,) * 18, "", 0, 0, 0.0, 0.0,
        default_feet(), default_feet(), ("",) * 18, True)
    row = trace_row(1.5, "vendor", step)
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
    held = FrameStep(
        "held", 0.0, 0.0, (0.0,) * 18, "unreachable", 0, 4,
        0.0, 0.0, feet, feet, ("",) * 18, False)
    for _ in range(4):
        watch.note(log, held)
    assert watch.errors == 0
    assert len(log.warnings) == 4
    watch.note(log, held)
    assert watch.errors == 1
    assert log.errors == [
        "holding last safe pose, generator frame frozen"]
    watch.note(log, held)
    assert watch.errors == 1
    partial = FrameStep(
        "partial", 0.5, 0.5, (0.1,) * 18, "rate_limit", 8, 3,
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
    second.note(log, 0.2, "vendor", "sent", (0.01, ""))
    second.note(log, 0.8, "vendor", "partial", (0.2,))
    assert log.infos == []
    second.note(log, 1.05, "vendor", "sent", (0.0,))
    assert log.infos == ["backend=vendor decision=partial max_dq=0.2000"]


def test_stand_is_sent_and_matches_the_vendor_tape():
    ik = HexapodLegIk()
    feet = default_feet()
    reference = ik.establish_reference(feet)
    assert reference.allow is True
    step = approach_frame(ik, feet, feet, 0.0)
    assert step.decision == "sent"
    assert step.alpha == 1.0
    assert step.publish is True
    assert step.reason == ""
    flat = []
    for triple in STAND:
        flat.extend(triple)
    for got, want in zip(step.angles, flat):
        assert abs(got - want) <= 0.005


def test_unreachable_and_near_singular_hold_without_moving_alpha():
    ik = HexapodLegIk()
    feet = default_feet()
    assert ik.establish_reference(feet).allow is True
    previous = ik.previous
    far = list(feet)
    far[3] = (0.0, 0.0, 500.0)
    far = tuple(far)
    held = approach_frame(ik, feet, far, 0.25)
    assert held.decision == "held"
    assert held.reason == "unreachable"
    assert held.leg == 4
    assert held.alpha == 0.0
    assert held.alpha_done == 0.25
    assert held.publish is False
    assert ik.previous is previous
    straight = _near_straight_feet()
    knee = approach_frame(ik, feet, straight, 0.25)
    assert knee.decision == "held"
    assert knee.reason == "near_singular"
    assert knee.leg == 6
    assert knee.alpha_done == 0.25
    assert ik.previous is previous


def test_no_reference_holds_when_the_stand_was_not_solved():
    ik = HexapodLegIk()
    feet = default_feet()
    held = approach_frame(ik, feet, feet, 0.0)
    assert held.decision == "held"
    assert held.reason == "no_reference"
    assert held.publish is False
    assert ik.previous is None


def test_a_long_legal_frame_sums_its_alphas_to_one():
    ik = HexapodLegIk()
    anchor = default_feet()
    assert ik.establish_reference(anchor).allow is True
    frame = _rate_limit_frame(ik, anchor)
    alpha_done = 0.0
    deltas = []
    first = None
    finished = None
    for _ in range(BISECTION_PARTS + 8):
        before = alpha_done
        step = approach_frame(ik, anchor, frame, alpha_done)
        assert step.decision != "held"
        if first is None:
            first = step
        deltas.append(step.alpha_done - before)
        alpha_done = step.alpha_done
        if step.decision == "sent":
            finished = step
            break
    assert first.decision == "partial"
    assert first.alpha < 1.0
    assert first.publish is True
    assert finished is not None
    assert finished.alpha == 1.0
    assert finished.decision == "sent"
    assert sum(deltas) == pytest.approx(1.0)
    assert deltas[-1] == pytest.approx(1.0 - sum(deltas[:-1]))
    twist = (0.12, 0.10, 0.6)
    pieces = [odometry_increment(*twist, delta_alpha=d) for d in deltas]
    whole = odometry_increment(*twist, delta_alpha=1.0)
    for index in range(6):
        assert sum(piece[index] for piece in pieces) == pytest.approx(
            whole[index])
    last = pieces[-1]
    assert last[3] == pytest.approx(twist[0] * TICK_S * deltas[-1])
    assert last[3] != pytest.approx(twist[0] * TICK_S)


def test_a_same_foot_branch_flip_is_a_hold():
    # The standing branch does not change. A one-radian disagreement
    # on a foot that moves 1 mm is still about a radian at every
    # fraction, so no alpha passes and nothing is published.
    ik = HexapodLegIk()
    feet = default_feet()
    assert ik.establish_reference(feet).allow is True
    shifted = list(ik.previous)
    shifted[1] = shifted[1] + 1.0
    ik.previous = tuple(shifted)
    frame = list(feet)
    frame[0] = (feet[0][0] + 1.0, feet[0][1], feet[0][2])
    frame = tuple(frame)
    before = ik.previous
    step = approach_frame(ik, feet, frame, 0.0)
    assert step.decision == "held"
    assert step.reason == "rate_limit"
    assert step.alpha == 0.0
    assert step.alpha_done == 0.0
    assert step.publish is False
    assert ik.previous is before
    assert should_publish(step) is False
