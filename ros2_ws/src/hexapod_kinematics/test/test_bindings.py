# The Python door. Same decisions as the C++ tests, crossed through
# the binding. A refused foot returns a result. It does not raise.

import pytest

import hexapod_kinematics as hk


def default_feet():
    # build_in_pose.py DEFAULT_POSE. X1 93.60, Y1 50.805, Y2 73.535.
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


# set_leg_position on those six feet. Corner coxa ±0.1206814507,
# femur 0.7554684412, tibia -0.6500234354. Middles differ.
STAND = (
    (0.1206814507, 0.7554684412, -0.6500234354),
    (0.0, 0.7615987475, -0.6870318910),
    (-0.1206814507, 0.7554684412, -0.6500234354),
    (0.1206814507, 0.7554684412, -0.6500234354),
    (0.0, 0.7615987475, -0.6870318910),
    (-0.1206814507, 0.7554684412, -0.6500234354),
)


def flat(angles):
    out = []
    for coxa, femur, tibia in angles:
        out.extend((coxa, femur, tibia))
    return out


def test_stand_matches_the_vendor_tape():
    pose = hk.solve_pose(default_feet())
    assert pose.ok is True
    assert pose.reason == ""
    assert len(pose.angles) == 6
    for got, want in zip(pose.angles, STAND):
        for a, b in zip(got, want):
            assert abs(a - b) <= 0.005


def test_a_far_foot_returns_unreachable_and_does_not_raise():
    feet = list(default_feet())
    feet[3] = (0.0, 0.0, 500.0)
    feet[5] = (0.0, 0.0, 500.0)
    pose = hk.solve_pose(feet)
    assert pose.ok is False
    assert pose.reason == "unreachable"
    assert pose.leg == 4
    assert pose.angles[3] == (0.0, 0.0, 0.0)
    assert pose.angles[5] == (0.0, 0.0, 0.0)


def test_the_wrong_number_of_feet_raises():
    with pytest.raises(ValueError):
        hk.solve_pose([(0.0, 0.0, -70.0)])


def test_gate_allows_a_repeated_stand_from_triples_or_eighteen():
    pose = hk.solve_pose(default_feet())
    by_triples = hk.gate(pose, pose.angles)
    by_flat = hk.gate(pose, flat(pose.angles))
    assert by_triples.allow is True
    assert by_triples.reason == ""
    assert by_triples.joint_id == 0
    assert by_flat.allow is True
    assert by_flat.joint_id == 0


def test_no_previous_pose_is_no_reference():
    pose = hk.solve_pose(default_feet())
    decision = hk.gate(pose)
    assert decision.allow is False
    assert decision.reason == "no_reference"
    assert decision.joint_id == 0


def test_a_one_radian_jump_is_rate_limit():
    pose = hk.solve_pose(default_feet())
    previous = flat(pose.angles)
    previous[7] = previous[7] + 1.0  # joint 8, left-rear femur
    decision = hk.gate(pose, previous)
    assert decision.allow is False
    assert decision.reason == "rate_limit"
    assert decision.joint_id == 8


def test_a_joint_past_the_window_is_joint_limit():
    command = hk.command_from_angles([2.2 if i == 7 else 0.0 for i in range(18)])
    decision = hk.gate(command, [0.0] * 18)
    assert command.ok is True
    assert decision.allow is False
    assert decision.reason == "joint_limit"
    assert decision.joint_id == 8
    assert decision.q_now_rad == pytest.approx(2.2)


def test_near_singular_reason_is_kept():
    # 0.02 mm short of straight, level with the femur hinge, leg 6.
    # A 0.5 mm slip there moves the knee past the measured limit.
    # A foot 0.2 mm short is allowed at that limit. Lengths are the
    # URDF guesses in geometry.cpp.
    import math

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
    pose = hk.solve_pose(feet)
    assert pose.ok is False
    assert pose.reason == "near_singular"
    assert pose.leg == 6
    stand = hk.solve_pose(default_feet())
    decision = hk.gate(pose, stand.angles)
    assert decision.allow is False
    assert decision.reason == "near_singular"
    assert decision.joint_id == 0
