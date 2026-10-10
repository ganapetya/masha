# The unfinished gait-5 target. No robot, and no 20 ms thread.
# The loop's job is to call these in order and not to call
# gen_next_mini_step_yield while target_still_open() is true.
# That order is what these tests lock.

import os
import sys

import pytest

from controller.leg_ik import (
    USE_HEXAPOD_KINEMATICS_DEFAULT,
    Gait5Session,
    HexapodLegIk,
    IkDecision,
    VendorLegIk,
    generators_own_feet_sent,
    last_safe_pose_kept,
    odometry_increment,
    read_max_step_rad,
    start_gait5,
)
from test_leg_ik import _rate_limit_target, default_feet


class AllowIk(object):
    """Every foot is legal. One mini-step finishes the target."""

    def __init__(self):
        self.last_sent_joint_radians = None
        self.solved = []

    def solve(self, six_feet):
        self.solved.append(six_feet)
        return IkDecision(True, (0.1,) * 18, "", 0, 0, 0.0, 0.0)

    def remember_last_sent_radians(self, angles):
        self.last_sent_joint_radians = tuple(angles)

    def establish_reference(self, six_feet):
        decision = self.solve(six_feet)
        self.remember_last_sent_radians(decision.angles)
        return decision


class HoldIk(object):
    """The generator's own feet are unreachable. Nothing is published."""

    def __init__(self, target_feet_positions):
        self.target_feet_positions = target_feet_positions
        self.last_sent_joint_radians = (0.0,) * 18

    def solve(self, six_feet):
        if six_feet is self.target_feet_positions:
            return IkDecision(
                False, (0.0,) * 18, "unreachable", 0, 4, 0.0, 0.0)
        return IkDecision(True, (0.1,) * 18, "", 0, 0, 0.0, 0.0)

    def remember_last_sent_radians(self, angles):
        self.last_sent_joint_radians = tuple(angles)

    def establish_reference(self, six_feet):
        decision = self.solve(six_feet)
        if decision.gate_allows_these_radians:
            self.remember_last_sent_radians(decision.angles)
        return decision


def test_start_uses_the_flag_from_that_moment_and_defaults_false():
    assert USE_HEXAPOD_KINEMATICS_DEFAULT is False
    loaded = "kinematics" in sys.modules
    session, line = start_gait5(object(), False)
    assert isinstance(session.ik, VendorLegIk)
    assert session.backend == "vendor"
    assert line == "gait 5 backend=vendor"
    assert session.target_still_open() is False
    if not loaded:
        assert "kinematics" not in sys.modules
    other, hex_line = start_gait5(object(), True)
    assert isinstance(other.ik, HexapodLegIk)
    assert other.backend == "hexapod"
    assert hex_line == "gait 5 backend=hexapod"


def test_an_allowed_target_finishes_in_one_mini_step_and_accepts_the_gait_step():
    ik = AllowIk()
    stance = default_feet()
    target_feet_positions = tuple(
        (foot[0] + 1.0, foot[1], foot[2]) for foot in stance)
    session = Gait5Session("generator", ik, "vendor")
    assert session.solve_stance_already_stood_on(stance).gate_allows_these_radians is True
    assert session.feet_where_this_target_started is stance
    session.begin_target(
        target_feet_positions, "params", False, last_part=True)
    assert session.target_still_open() is True
    mini_step_result, fraction_sent_this_wake, gait_step_accepted = session.mini_step()
    assert mini_step_result.what_this_wake_published == generators_own_feet_sent
    assert mini_step_result.fraction_reached_along_the_target == 1.0
    assert fraction_sent_this_wake == 1.0
    assert gait_step_accepted is True
    assert session.target_still_open() is False
    assert session.feet_where_this_target_started is target_feet_positions
    # The next target is a new begin. Accepting the gait step on
    # the target that just finished does not make the next target
    # accepted as well.
    session.begin_target(stance, "params", False, last_part=False)
    mini_step_result, fraction_sent_this_wake, gait_step_accepted = session.mini_step()
    assert mini_step_result.what_this_wake_published == generators_own_feet_sent
    assert fraction_sent_this_wake == 1.0
    assert gait_step_accepted is False
    whole = odometry_increment(0.12, 0.10, 0.6, fraction_sent_this_wake=1.0)
    once = odometry_increment(0.12, 0.10, 0.6, fraction_sent_this_wake=fraction_sent_this_wake)
    assert once == pytest.approx(whole)


def test_a_partial_target_stays_open_until_the_gait_step_is_accepted():
    ik = HexapodLegIk()
    anchor = default_feet()
    session = Gait5Session("generator", ik, "hexapod")
    assert session.solve_stance_already_stood_on(anchor).gate_allows_these_radians is True
    target_feet_positions = _rate_limit_target(ik, anchor)
    session.begin_target(
        target_feet_positions, "params", True, last_part=True)
    deltas = []
    gait_step_accepted = False
    mini_step_result = None
    for _ in range(300):
        assert session.target_still_open()
        six_feet, command, ask_for_50_ms_blend = session.open_target()
        assert six_feet is target_feet_positions
        assert command == "params"
        assert ask_for_50_ms_blend is True
        mini_step_result, fraction_sent_this_wake, gait_step_accepted = session.mini_step()
        deltas.append(fraction_sent_this_wake)
        if mini_step_result.what_this_wake_published == generators_own_feet_sent:
            break
        assert gait_step_accepted is False
        assert session.feet_where_this_target_started is anchor
    assert mini_step_result is not None
    assert mini_step_result.what_this_wake_published == generators_own_feet_sent
    assert mini_step_result.fraction_reached_along_the_target == 1.0
    assert gait_step_accepted is True
    assert session.target_still_open() is False
    assert deltas[0] < 1.0
    assert sum(deltas) == pytest.approx(1.0)
    twist = (0.12, -0.05, 0.6)
    pieces = [
        odometry_increment(*twist, fraction_sent_this_wake=fraction_sent_this_wake) for fraction_sent_this_wake in deltas]
    whole = odometry_increment(*twist, fraction_sent_this_wake=1.0)
    for index in range(6):
        assert sum(piece[index] for piece in pieces) == pytest.approx(
            whole[index])
    # The finishing mini-step is only the leftover fraction.
    assert pieces[-1][3] == pytest.approx(twist[0] * 0.02 * deltas[-1])
    assert pieces[-1][3] != pytest.approx(twist[0] * 0.02)


def test_a_hold_does_not_move_alpha_or_accept_the_gait_step():
    stance = default_feet()
    target_feet_positions = tuple(stance)
    ik = HoldIk(target_feet_positions)
    session = Gait5Session("generator", ik, "hexapod")
    session.solve_stance_already_stood_on(stance)
    session.begin_target(
        target_feet_positions, "params", False, last_part=True)
    mini_step_result, fraction_sent_this_wake, gait_step_accepted = session.mini_step()
    assert mini_step_result.what_this_wake_published == last_safe_pose_kept
    assert mini_step_result.reason == "unreachable"
    assert mini_step_result.publish is False
    assert fraction_sent_this_wake == 0.0
    assert gait_step_accepted is False
    assert session.target_still_open() is True
    assert session.open_target()[0] is target_feet_positions
    again, delta2, gait_step_accepted2 = session.mini_step()
    assert again.what_this_wake_published == last_safe_pose_kept
    assert again.fraction_of_target_sent == mini_step_result.fraction_of_target_sent
    assert delta2 == 0.0
    assert gait_step_accepted2 is False
    assert odometry_increment(0.12, 0.1, 0.6, fraction_sent_this_wake=delta2) == (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_max_step_rad_is_the_measured_peak_times_the_margin():
    # Vendor pseudo cycle, Twist +0.12 m/s, +0.10 m/s, +0.6 rad/s,
    # period 0.60 s, lift 35 mm. safety.cpp names that command.
    assert read_max_step_rad() == pytest.approx(0.17622116866774196 * 1.5)


def test_launch_argument_defaults_to_false():
    launch = os.path.join(
        os.path.dirname(__file__), os.pardir, "launch",
        "move_controller.launch.py")
    text = open(launch, encoding="utf-8").read()
    assert "DeclareLaunchArgument(" in text
    assert "'use_hexapod_kinematics'" in text
    assert "default_value='false'" in text
    assert "value_type=bool" in text
