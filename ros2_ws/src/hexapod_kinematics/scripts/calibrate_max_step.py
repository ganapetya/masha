#!/usr/bin/env python3
# The calibration print for the gait-5 step limit.
#
# This file is a measurement, not a library. The controller does not
# import it. It asks the closed solver (kinematics.set_leg_position)
# for angles and does not construct a node, a publisher, or
# JointControl, so no servo pulse leaves this process. That is what
# the plan calls pseudo.
#
# Two questions:
#   1. On the stand, and on a few nearby poses, how far are our
#      angles from the vendor angles? The stand contract is 0.005 rad
#      on every joint. A miss there is a geometry problem. This
#      script does not edit a length to hide it.
#   2. Over one FollowGaitGenerator cycle, what is the largest |Δq|
#      from one 20 ms frame to the next, on the vendor angles? The
#      number that belongs in safety.cpp is that peak times 1.5.

import math
import sys

# Hunter lift. masha_hunter.yaml cmd_height, and the same 35 mm in
# follow_gait.hpp. CmdVelParams would otherwise default to 10 mm,
# which is not the walk a hunt plays.
HUNT_LIFT_MM = 35.0

# One cycle. move_controller clamps a Twist to these, in m/s and
# rad/s, before StepController scales the linear part to mm/s.
VX_CLAMP_M_S = 0.12
VY_CLAMP_M_S = 0.10
WZ_CLAMP_RAD_S = 0.6
PERIOD_S = 0.60

# The stand contract. Same bound test_leg_ik locks against the tape.
STAND_RAD = 0.005

# Margin written next to the peak. A normal tick stays under the
# product. A knee flip, about a radian, does not.
MARGIN = 1.5

JOINTS = (
    "coxa_LF", "femur_LF", "tibia_LF",
    "coxa_LM", "femur_LM", "tibia_LM",
    "coxa_LR", "femur_LR", "tibia_LR",
    "coxa_RR", "femur_RR", "tibia_RR",
    "coxa_RM", "femur_RM", "tibia_RM",
    "coxa_RF", "femur_RF", "tibia_RF",
)


def vendor_angles(feet):
    """Eighteen radians from kinematics.set_leg_position.

    One leg at a time, ids 1..6, the same call set_pose_base makes.
    The .so solves. It does not publish. The return is coxa, femur,
    tibia, in the set_leg_position sense, before JointControl's
    direction.
    """
    from kinematics import kinematics

    angles = []
    for leg_id, foot in enumerate(feet, start=1):
        tip = (float(foot[0]), float(foot[1]), float(foot[2]))
        joints = kinematics.set_leg_position(leg_id, tip)
        angles.extend(float(joint) for joint in joints)
    return angles


def our_angles(feet):
    """Eighteen radians from hexapod_kinematics.solve_pose.

    Returns (angles, None) when the pose is accepted, and
    (None, reason) when the solver refuses. A refusal is not a
    number we can subtract from the vendor tape.
    """
    import hexapod_kinematics as hk

    pose = hk.solve_pose(feet)
    if not pose.ok:
        return None, pose.reason
    angles = []
    for triple in pose.angles:
        angles.extend(float(joint) for joint in triple)
    return angles, None


def shift(pose, dx, dy, dz):
    """The same six feet, each moved by the same body-frame offset."""
    return tuple(
        (float(foot[0]) + dx, float(foot[1]) + dy, float(foot[2]) + dz)
        for foot in pose)


def worst_gap(ours, vendor):
    """Largest |ours − vendor|, and the joint id (1..18) that holds it."""
    gaps = [abs(a - b) for a, b in zip(ours, vendor)]
    joint = max(range(len(gaps)), key=lambda i: gaps[i]) + 1
    return gaps[joint - 1], joint, gaps


def print_stand(name, feet):
    """Print one pose beside the vendor angles. Return the worst |Δ|."""
    ours, reason = our_angles(feet)
    vendor = vendor_angles(feet)
    print(name)
    if ours is None:
        print("  our solver refused: %s" % reason)
        return None
    worst, joint, gaps = worst_gap(ours, vendor)
    for index, (a, b, gap) in enumerate(zip(ours, vendor, gaps), start=1):
        print(
            "  %2d %-9s  ours=% .8f  vendor=% .8f  abs=% .8f"
            % (index, JOINTS[index - 1], a, b, gap))
    print(
        "  worst joint %d %s  abs=%.8f rad (%.4f deg)"
        % (joint, JOINTS[joint - 1], worst, math.degrees(worst)))
    return worst


def cycle_frames(vx_mm_s, vy_mm_s, wz_rad_s, stance):
    """One steady gait-5 cycle, in the order the 20 ms loop plays it.

    FollowGaitGenerator is primed with send(None), the way
    StepController starts it. Status 'first' plays the intro. Only
    its first frame is returned, so the stand-to-splice step can be
    printed beside the cycle. Status 'running' then yields the baked
    table from frame 0 through the last frame. That table is one
    period. The caller adds the seam from the last frame back to the
    first, which is the tick the next lap actually takes.

    finish_ps is cleared first. A leftover hand-off pose would splice
    this cycle against some earlier run.
    """
    import controller.move as move

    move.finish_ps = []
    move.stop = False
    move.slow = "cmd_false"
    params = move.CmdVelParams(
        gait=5,
        velocity_x=vx_mm_s,
        velocity_y=vy_mm_s,
        angular_z=wz_rad_s,
        height=HUNT_LIFT_MM,
        relative_h=False,
        period=PERIOD_S,
        linear_factor=1.0,
        rotate_factor=1.0,
    )
    generator = move.FollowGaitGenerator(params, log=None)
    generator.send(None)
    status = "first"
    running = []
    first_intro = None
    while True:
        # The stance object stays the same one. A new tuple would
        # make the generator throw the table away and bake again.
        pose, last_part, _params, _slow = generator.send((stance, status))
        if status == "first":
            if first_intro is None:
                first_intro = pose
            if last_part:
                status = "running"
            continue
        running.append(pose)
        if last_part:
            return running, first_intro


def peak_step(frames):
    """Largest |q_now − q_previous| on the vendor angles.

    Consecutive frames, then the seam from the last frame to the
    first. Returns the peak in radians, the joint id, and a short
    label for the tick ('seam' or the destination frame index).
    """
    series = [vendor_angles(frame) for frame in frames]
    peak = 0.0
    joint = 0
    tick = "none"

    def consider(now, previous, label):
        nonlocal peak, joint, tick
        for index, (a, b) in enumerate(zip(now, previous), start=1):
            step = abs(a - b)
            if step > peak:
                peak = step
                joint = index
                tick = label

    for index in range(1, len(series)):
        consider(series[index], series[index - 1], str(index))
    consider(series[0], series[-1], "seam")
    return peak, joint, tick


def command_label(vx_m_s, vy_m_s, wz):
    return "vx=%+.2f m/s vy=%+.2f m/s wz=%+.1f rad/s" % (vx_m_s, vy_m_s, wz)


def opening_step(first_feet, stance):
    """|Δq| from the stand into the first intro frame.

    That tick is not a step of the steady cycle. The generator
    splices into the table, and the previous angles are the stand.
    Catch-up is what absorbs it when the flag is on.
    """
    now = vendor_angles(first_feet)
    previous = vendor_angles(stance)
    peak = 0.0
    joint = 0
    for index, (a, b) in enumerate(zip(now, previous), start=1):
        step = abs(a - b)
        if step > peak:
            peak = step
            joint = index
    return peak, joint


def measure_command(vx_m_s, vy_m_s, wz, stance):
    """Peak |Δq| for one Twist inside the clamps. Vendor angles only."""
    frames, first = cycle_frames(vx_m_s * 1000.0, vy_m_s * 1000.0, wz, stance)
    peak, joint, tick = peak_step(frames)
    depart, depart_joint = opening_step(first, stance)
    print(
        "  %s  frames=%d  peak=%.8f rad  joint %d %s  tick %s"
        % (command_label(vx_m_s, vy_m_s, wz), len(frames), peak,
           joint, JOINTS[joint - 1], tick))
    return peak, joint, tick, len(frames), depart, depart_joint


def main():
    from controller.build_in_pose import DEFAULT_POSE

    stance = tuple(
        (float(foot[0]), float(foot[1]), float(foot[2])) for foot in DEFAULT_POSE)
    print("stand versus kinematics.set_leg_position  (contract %.3f rad)" % STAND_RAD)
    stand_worst = print_stand("DEFAULT_POSE", stance)
    print("offsets (printed, not the contract)")
    for name, dx, dy, dz in (
            ("+20 mm x", 20.0, 0.0, 0.0),
            ("+10 mm y", 0.0, 10.0, 0.0),
            ("+15 mm z", 0.0, 0.0, 15.0),
            ("-10 mm x", -10.0, 0.0, 0.0)):
        print_stand(name, shift(stance, dx, dy, dz))

    if stand_worst is None or stand_worst > STAND_RAD:
        print("STAND_FAIL worst=%s" % stand_worst)
        return 2

    print("STAND_OK worst=%.8f rad" % stand_worst)
    print(
        "one vendor gait-5 cycle, no servos, period %.2f s, lift %.0f mm"
        % (PERIOD_S, HUNT_LIFT_MM))

    # The clamp box. The plan names the positive corner. The other
    # signs, and each axis alone, are the same clamps: a hunt can
    # send any one of them, and the limit has to cover the largest.
    commands = [(VX_CLAMP_M_S, VY_CLAMP_M_S, WZ_CLAMP_RAD_S)]
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            for sz in (1.0, -1.0):
                corner = (sx * VX_CLAMP_M_S, sy * VY_CLAMP_M_S, sz * WZ_CLAMP_RAD_S)
                if corner not in commands:
                    commands.append(corner)
    commands.extend((
        (VX_CLAMP_M_S, 0.0, 0.0),
        (-VX_CLAMP_M_S, 0.0, 0.0),
        (0.0, VY_CLAMP_M_S, 0.0),
        (0.0, -VY_CLAMP_M_S, 0.0),
        (0.0, 0.0, WZ_CLAMP_RAD_S),
        (0.0, 0.0, -WZ_CLAMP_RAD_S),
    ))

    # A later sign combination can win by one unit in the last place.
    # That is the same peak. Keep the earlier command, which is the
    # positive corner the plan names.
    best = None
    for vx, vy, wz in commands:
        measured = measure_command(vx, vy, wz, stance)
        if best is None or measured[0] > best[0] + 1e-12:
            best = measured + (vx, vy, wz)

    peak, joint, tick, count, depart, depart_joint, vx, vy, wz = best
    limit = MARGIN * peak
    print(
        "PEAK %.17g rad  joint %d %s  tick %s  frames %d"
        % (peak, joint, JOINTS[joint - 1], tick, count))
    print("COMMAND %s  period %.2f s  lift %.0f mm  vendor  pseudo" % (
        command_label(vx, vy, wz), PERIOD_S, HUNT_LIFT_MM))
    print(
        "STAND_TO_FIRST %.17g rad  joint %d %s"
        % (depart, depart_joint, JOINTS[depart_joint - 1]))
    print("LIMIT %.17g" % limit)
    return 0


if __name__ == "__main__":
    sys.exit(main())
