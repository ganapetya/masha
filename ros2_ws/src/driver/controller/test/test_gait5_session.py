# The unfinished gait-5 frame. No robot, and no 20 ms thread.
# The loop's job is to call these in order and not to call send()
# while waiting() is true. That order is what these tests lock.

import os
import sys

import pytest

from controller.leg_ik import (
    USE_HEXAPOD_KINEMATICS_DEFAULT,
    Gait5Session,
    HexapodLegIk,
    IkDecision,
    VendorLegIk,
    odometry_increment,
    read_max_step_rad,
    start_gait5,
)
from test_leg_ik import _rate_limit_frame, default_feet


class AllowIk(object):
    """Every foot is legal. One tick finishes the frame."""

    def __init__(self):
        self.previous = None
        self.solved = []

    def solve(self, pose):
        self.solved.append(pose)
        return IkDecision(True, (0.1,) * 18, "", 0, 0, 0.0, 0.0)

    def remember(self, angles):
        self.previous = tuple(angles)

    def establish_reference(self, pose):
        decision = self.solve(pose)
        self.remember(decision.angles)
        return decision


class HoldIk(object):
    """The generator's own feet are unreachable. Nothing is published."""

    def __init__(self, frame):
        self.frame = frame
        self.previous = (0.0,) * 18

    def solve(self, pose):
        if pose is self.frame:
            return IkDecision(
                False, (0.0,) * 18, "unreachable", 0, 4, 0.0, 0.0)
        return IkDecision(True, (0.1,) * 18, "", 0, 0, 0.0, 0.0)

    def remember(self, angles):
        self.previous = tuple(angles)

    def establish_reference(self, pose):
        decision = self.solve(pose)
        if decision.allow:
            self.remember(decision.angles)
        return decision


def test_start_uses_the_flag_from_that_moment_and_defaults_false():
    assert USE_HEXAPOD_KINEMATICS_DEFAULT is False
    loaded = "kinematics" in sys.modules
    session, line = start_gait5(object(), False)
    assert isinstance(session.ik, VendorLegIk)
    assert session.backend == "vendor"
    assert line == "gait 5 backend=vendor"
    assert session.waiting() is False
    if not loaded:
        assert "kinematics" not in sys.modules
    other, hex_line = start_gait5(object(), True)
    assert isinstance(other.ik, HexapodLegIk)
    assert other.backend == "hexapod"
    assert hex_line == "gait 5 backend=hexapod"


def test_an_allowed_frame_finishes_in_one_tick_and_releases_the_boundary():
    ik = AllowIk()
    stance = default_feet()
    frame = tuple(
        (foot[0] + 1.0, foot[1], foot[2]) for foot in stance)
    session = Gait5Session("generator", ik, "vendor")
    assert session.attach(stance).allow is True
    assert session.anchor is stance
    session.open_frame(frame, "params", "cmd_false", last_part=True)
    assert session.waiting() is True
    step, delta, boundary = session.step()
    assert step.decision == "sent"
    assert step.alpha == 1.0
    assert delta == 1.0
    assert boundary is True
    assert session.waiting() is False
    assert session.anchor is frame
    # The next film frame is a new open. The boundary does not stick.
    session.open_frame(stance, "params", "cmd_false", last_part=False)
    step, delta, boundary = session.step()
    assert step.decision == "sent"
    assert delta == 1.0
    assert boundary is False
    whole = odometry_increment(0.12, 0.10, 0.6, delta_alpha=1.0)
    once = odometry_increment(0.12, 0.10, 0.6, delta_alpha=delta)
    assert once == pytest.approx(whole)


def test_a_partial_frame_stays_open_and_the_boundary_waits():
    ik = HexapodLegIk()
    anchor = default_feet()
    session = Gait5Session("generator", ik, "hexapod")
    assert session.attach(anchor).allow is True
    frame = _rate_limit_frame(ik, anchor)
    session.open_frame(frame, "params", "cmd_true", last_part=True)
    deltas = []
    boundary = False
    step = None
    for _ in range(300):
        assert session.waiting()
        held_feet, params, slow = session.held()
        assert held_feet is frame
        assert params == "params"
        assert slow == "cmd_true"
        step, delta, boundary = session.step()
        deltas.append(delta)
        if step.decision == "sent":
            break
        assert boundary is False
        assert session.anchor is anchor
    assert step is not None
    assert step.decision == "sent"
    assert step.alpha == 1.0
    assert boundary is True
    assert session.waiting() is False
    assert deltas[0] < 1.0
    assert sum(deltas) == pytest.approx(1.0)
    twist = (0.12, -0.05, 0.6)
    pieces = [
        odometry_increment(*twist, delta_alpha=delta) for delta in deltas]
    whole = odometry_increment(*twist, delta_alpha=1.0)
    for index in range(6):
        assert sum(piece[index] for piece in pieces) == pytest.approx(
            whole[index])
    # The finishing tick is only the leftover fraction.
    assert pieces[-1][3] == pytest.approx(twist[0] * 0.02 * deltas[-1])
    assert pieces[-1][3] != pytest.approx(twist[0] * 0.02)


def test_a_hold_does_not_move_alpha_or_release_the_boundary():
    stance = default_feet()
    frame = tuple(stance)
    ik = HoldIk(frame)
    session = Gait5Session("generator", ik, "hexapod")
    session.attach(stance)
    session.open_frame(frame, "params", "cmd_false", last_part=True)
    step, delta, boundary = session.step()
    assert step.decision == "held"
    assert step.reason == "unreachable"
    assert step.publish is False
    assert delta == 0.0
    assert boundary is False
    assert session.waiting() is True
    assert session.held()[0] is frame
    again, delta2, boundary2 = session.step()
    assert again.decision == "held"
    assert again.alpha_done == step.alpha_done
    assert delta2 == 0.0
    assert boundary2 is False
    assert odometry_increment(0.12, 0.1, 0.6, delta_alpha=delta2) == (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_the_step_limit_is_still_the_stand_in():
    assert read_max_step_rad() == pytest.approx(0.05)


def test_launch_argument_defaults_to_false():
    launch = os.path.join(
        os.path.dirname(__file__), os.pardir, "launch",
        "move_controller.launch.py")
    text = open(launch, encoding="utf-8").read()
    assert "DeclareLaunchArgument(" in text
    assert "'use_hexapod_kinematics'" in text
    assert "default_value='false'" in text
    assert "value_type=bool" in text
