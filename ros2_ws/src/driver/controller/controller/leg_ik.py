# The door the 20 ms loop will use for hunter gait 5.
#
# Two formulas sit behind one call. VendorLegIk asks the closed
# kinematics.so, one leg at a time, which is the path every walk
# uses today. HexapodLegIk asks hexapod_kinematics.solve_pose and
# then the safety gate. make_leg_ik picks one of them. The default
# is the vendor. This file does not read a ROS parameter and it
# does not publish a servo pulse. StepController reads the
# parameter when a gait-5 generator starts, and Gait5Session is
# the unfinished target that generator is still walking.
#
# A refused pose comes back as an IkDecision. An exception here
# would fall into the loop's moving-pose handler and drop the
# generator. The wrong shape of feet is the exception that still
# raises: that is a caller bug, and the binding reports ValueError.
#
# Pulses stay out of this file. Both backends return radians in the
# set_leg_position sense. JointControl is what multiplies by
# SERVOS['direction'] and turns the radian into a pulse. The safety
# gate already used that same direction when it checked the ±120°
# window. Applying it again here would move the foot.

import csv
import math
import os
import queue
import threading
import time

# Read once, when a gait-5 generator starts. False keeps today's .so.
USE_HEXAPOD_KINEMATICS_DEFAULT = False

# Append-only. The loop does not open this file.
TRACE_PATH_DEFAULT = "/home/ubuntu/ros2_ws/log/hexapod_ik_trace.csv"

# About four seconds of 20 ms ticks. A full queue drops a row.
TRACE_QUEUE_BOUND = 200

# The writer flushes on whichever comes first.
TRACE_FLUSH_ROWS = 25
TRACE_FLUSH_S = 0.5

# One WARN per second while rows are being dropped, not one per row.
TRACE_DROP_WARN_S = 1.0

# Shutdown may flush. It may not hold up a halt.
TRACE_CLOSE_S = 0.2

# Eight bisections. The smallest accepted advance is 1/256 of what
# is left of this target. A smaller candidate is no progress.
BISECTION_PARTS = 256

# One ordinary mini-step, seconds. Catch-up scales this by
# fraction_sent_this_wake, the slice of the target this wake sent.
TICK_S = 0.02

# What one 20 ms wake published. Compared beside the field, so the
# line reads as a sentence. The CSV column `decision` and the line
# logged once a second still use the short file words sent, partial,
# and held. Learning notes filter the file on decision == partial.
# file_decision_word is the only translation.
generators_own_feet_sent = "generators_own_feet_sent"
shorter_point_sent = "shorter_point_sent"
last_safe_pose_kept = "last_safe_pose_kept"

_FILE_DECISION = {
    generators_own_feet_sent: "sent",
    shorter_point_sent: "partial",
    last_safe_pose_kept: "held",
}

# The file header. Column names stay short on purpose. Python fields
# are longer; trace_row copies them into these columns.
TRACE_COLUMNS = (
    "t_s", "backend", "decision", "alpha", "reason", "joint",
    "x1", "y1", "z1", "x2", "y2", "z2", "x3", "y3", "z3",
    "x4", "y4", "z4", "x5", "y5", "z5", "x6", "y6", "z6",
    "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9",
    "q10", "q11", "q12", "q13", "q14", "q15", "q16", "q17", "q18",
    "dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq7", "dq8", "dq9",
    "dq10", "dq11", "dq12", "dq13", "dq14", "dq15", "dq16", "dq17",
    "dq18",
)


def file_decision_word(what_this_wake_published):
    """The short word the CSV column and the once-a-second line use.

    Python compares the long name. The file keeps sent, partial,
    and held, so a filter written against the trace still matches.
    A word that is already one of those three is returned as itself.
    """
    return _FILE_DECISION.get(
        what_this_wake_published, what_this_wake_published)


class IkDecision(object):
    """What one solve said.

    gate_allows_these_radians is whether these 18 radians may be
    published. The C++ gate still calls that answer allow.
    _from_binding copies the C++ field onto this longer name.
    reason is empty when the gate allows the radians. The reason
    words match the C++ gate: unreachable, near_singular,
    joint_limit, rate_limit, non_finite, no_reference.
    joint_id is 1..18 when one joint caused the refusal, else 0.
    leg is 1..6 when the solver named a leg, else 0.
    angles is always 18 radians. On a solver refusal the failed leg
    and the legs after it are zeros, which is solve_pose's own fill.
    refused_joint_radians and last_sent_radians_of_that_joint are
    filled only when one joint caused the refusal. When the gate
    allows the radians they stay 0. The C++ struct keeps the names
    q_now_rad and q_previous_rad.
    """

    def __init__(self, gate_allows_these_radians, angles, reason,
                 joint_id, leg, refused_joint_radians,
                 last_sent_radians_of_that_joint):
        self.gate_allows_these_radians = gate_allows_these_radians
        self.angles = tuple(angles)
        self.reason = reason
        self.joint_id = joint_id
        self.leg = leg
        self.refused_joint_radians = refused_joint_radians
        self.last_sent_radians_of_that_joint = (
            last_sent_radians_of_that_joint)


class MiniStepResult(object):
    """What one mini-step did with one target.

    A mini-step is one 20 ms wake. A target is the six toe positions
    the generator yielded for the current part of the gait step.
    what_this_wake_published is generators_own_feet_sent,
    shorter_point_sent, or last_safe_pose_kept.
    fraction_reached_along_the_target is where this wake landed on
    the line from the feet where the target started to the
    generator's own six feet. It is 1 when those own feet were
    sent, the landed fraction when a shorter point was sent, and
    0 when the last safe pose was kept. A hold writes 0 here even
    though the cumulative fraction does not move.
    fraction_of_target_sent is how far along the target has been
    sent, including this mini-step. A hold leaves it where it was.
    On a shorter point the two fractions are the same number. On a
    hold they are not.
    feet are the six tips that were solved this mini-step,
    millimetres. source_feet are target_feet_positions, the six
    tips the generator yielded. A shorter point solves a point
    between the feet where this target started and those tips.
    The WARN names the generator's foot, because that is the foot
    that was refused.
    joint_change_from_last_sent is the 18 joint deltas from the
    last radians that were actually sent. Empty strings when no
    pose has been sent yet. The controller's joints_state starts
    at 0 and is not that memory.
    publish is false when the last safe pose was kept.
    dry_run_do_not_publish is applied later, by should_publish.
    A dry run still carries generators_own_feet_sent or
    shorter_point_sent here.
    The CSV column is still named alpha, and the joint-delta
    columns are still dq1 through dq18. trace_row writes those
    columns from these longer fields, and it writes the short
    file word into the decision column.
    """

    def __init__(self, what_this_wake_published,
                 fraction_reached_along_the_target,
                 fraction_of_target_sent, angles, reason,
                 joint_id, leg, refused_joint_radians,
                 last_sent_radians_of_that_joint, feet,
                 source_feet, joint_change_from_last_sent, publish):
        self.what_this_wake_published = what_this_wake_published
        self.fraction_reached_along_the_target = (
            fraction_reached_along_the_target)
        self.fraction_of_target_sent = fraction_of_target_sent
        self.angles = tuple(angles)
        self.reason = reason
        self.joint_id = joint_id
        self.leg = leg
        self.refused_joint_radians = refused_joint_radians
        self.last_sent_radians_of_that_joint = (
            last_sent_radians_of_that_joint)
        self.feet = feet
        self.source_feet = source_feet
        self.joint_change_from_last_sent = tuple(
            joint_change_from_last_sent)
        self.publish = publish


class LegIk(object):
    """One formula from six feet to eighteen radians.

    last_sent_joint_radians is the last 18 radians this object was
    told had been sent, or the stand solved by
    establish_reference and never published. It starts empty.
    The controller's joints_state is not a substitute.
    """

    def __init__(self):
        self.last_sent_joint_radians = None

    def solve(self, six_feet):
        """Return an IkDecision for these six feet. Do not publish."""
        raise NotImplementedError

    def remember_last_sent_radians(self, angles):
        """Store angles as the last radians that were actually sent.

        Call this only after a publish, or from establish_reference
        for the stance the body is already standing on.
        """
        self.last_sent_joint_radians = tuple(angles)

    def establish_reference(self, six_feet):
        """Solve the stance the body is already standing on.

        On success those angles become last_sent_joint_radians.
        Nothing is published: the feet are already there. The first
        target is compared with this solve. A stance the backend
        cannot accept leaves last_sent_joint_radians empty, and the
        next target keeps the last safe pose with that reason.
        """
        raise NotImplementedError


class VendorLegIk(LegIk):
    """kinematics.set_leg_position, one leg at a time.

    This backend does not run the new rate gate. Its angles are the
    reference that gate will be measured against, and JointControl
    still enforces the ±120° window on the way to the servo, as it
    does today. The import of kinematics.so happens on the first
    solve, so constructing this object does not load the binary.
    """

    def solve(self, six_feet):
        from kinematics import kinematics
        angles = []
        for leg_id, foot in enumerate(six_feet, start=1):
            joints = kinematics.set_leg_position(leg_id, foot)
            angles.extend(joints)
        return IkDecision(
            True, angles, "", 0, 0, 0.0, 0.0)

    def establish_reference(self, six_feet):
        decision = self.solve(six_feet)
        if decision.gate_allows_these_radians:
            self.remember_last_sent_radians(decision.angles)
        return decision


class HexapodLegIk(LegIk):
    """hexapod_kinematics.solve_pose, then the safety gate, once.

    This does not subdivide a target. approach_target is the only
    place that turns one rate_limit into a shorter fraction. The
    import happens on the first solve, so a vendor-only process
    that never builds this object does not load the module.
    """

    def solve(self, six_feet):
        import hexapod_kinematics as hk
        result = hk.solve_pose(six_feet)
        # `decision` here is the C++ SafetyDecision. allow and the
        # q_* fields stay on that struct. _from_binding copies them
        # onto the longer Python names.
        decision = hk.gate(result, self.last_sent_joint_radians)
        return _from_binding(result, decision)

    def establish_reference(self, six_feet):
        # The gate answers no_reference when last_sent_joint_radians
        # is empty, even for a legal stand. Compare the stand with
        # itself so the window is checked and the joint change is
        # zero. A solver refusal is returned as itself and the
        # memory stays empty.
        import hexapod_kinematics as hk
        result = hk.solve_pose(six_feet)
        angles = _flatten(result.angles)
        if not result.ok:
            blank = hk.gate(result, None)
            return _from_binding(result, blank)
        decision = hk.gate(result, angles)
        made = _from_binding(result, decision)
        if made.gate_allows_these_radians:
            self.remember_last_sent_radians(made.angles)
        return made


def make_leg_ik(use_hexapod=USE_HEXAPOD_KINEMATICS_DEFAULT):
    """Pick the formula for one gait-5 generator.

    Call this when that generator starts, and keep the object until
    the generator is replaced. A param change during a gait step must
    not build a second object for the same feet: the formula would
    change under a moving foot. The next generator, at the next Twist
    after a gait-step boundary or at the next hunt bout, is the next
    call, and that call sees the new flag.

    The default is false. Joystick gaits, voice gait 2, body lean,
    and action groups do not call this. They stay on set_pose_base.
    """
    if use_hexapod:
        return HexapodLegIk()
    return VendorLegIk()


def begin_generator(use_hexapod=USE_HEXAPOD_KINEMATICS_DEFAULT):
    """One gait-5 generator's backend, and the line that names it.

    The line is `gait 5 backend=hexapod` or `gait 5 backend=vendor`.
    Log it once, here, not on every mini-step.
    """
    ik = make_leg_ik(use_hexapod)
    backend = "hexapod" if use_hexapod else "vendor"
    return ik, backend, "gait 5 backend=%s" % backend


def start_gait5(generator, use_hexapod=USE_HEXAPOD_KINEMATICS_DEFAULT):
    """Build the session for one gait-5 generator.

    Call this when that generator starts, with the flag value from
    that moment. Keep the session until the generator object is
    replaced. A later param change does not come back here: a second
    session would swap the formula under a moving foot. The next
    generator, at the next Twist after a gait-step boundary or at the
    next hunt bout, is the next call.

    The returned line is logged once by the caller.
    solve_stance_already_stood_on is separate so constructing the
    vendor backend does not load kinematics.so until the stance is
    actually solved.
    """
    ik, backend, line = begin_generator(use_hexapod)
    return Gait5Session(generator, ik, backend), line


def approach_target(ik, feet_where_this_target_started,
                    target_feet_positions, fraction_of_target_sent):
    """Walk one target as far as the gate allows.

    feet_where_this_target_started is where this target started:
    the last feet that were sent when the fraction reached 1, or
    the feet of the body pose at the start of a bout.
    target_feet_positions are the six tips the generator yielded.
    fraction_of_target_sent is how far along that segment has
    already been sent, in [0, 1].

    The order is fixed.

    1. Solve the generator's own feet. When the gate allows them,
       fraction_of_target_sent becomes 1 by assignment and the next
       mini-step may ask the generator for a new target.
    2. When the only failure is rate_limit, the feet are legal and
       the joint move is too big. Eight bisections search the open
       interval past fraction_of_target_sent. One fraction is shared
       by all six legs:

           feet(fraction) = start + fraction * (target - start)

       start is feet_where_this_target_started. The largest fraction
       that passes the whole gate is a shorter point. The generator
       stays on this target.
    3. Any other reason keeps the last safe pose. Nothing is
       published. fraction_of_target_sent and the last sent radians
       stay where they are.
    4. A rate_limit whose search finds nothing is the same hold.
       A branch flip is this case: the foot barely moves and every
       fraction still jumps by about a radian, because the standing
       branch does not change along the segment. Catch-up must not
       publish that jump.

    fraction_of_target_sent becomes 1 only when point 1 passes.
    Eight bisections land on a 1/256 grid of the remaining segment,
    so a value such as 0.996 is a shorter point. Rounding that up
    to 1 would start the next target while these feet were still
    short, and it would do it without the gate having allowed this
    target. A candidate less than one part in 256 past
    fraction_of_target_sent is no progress, and that mini-step
    keeps the last safe pose.

    Point 1 solves the generator's own tuple. It does not solve the
    reconstruction at fraction 1. That sum can miss the original
    millimetres by a fraction of a unit in the last place.

    This pause is for hunter gait 5, a tripod. Three feet stay on
    the ground while one target is stretched over several mini-steps,
    so the body is not depending on a swing finishing on time. A gait
    that stays upright by momentum would fall if a swing were slowed
    this way. That gait is outside this module.

    The generator object is not dropped here. A hold returns. The
    caller keeps the same target. A halt (gait -2) is the existing
    stand path, and that is what clears a hold.
    """
    if fraction_of_target_sent < 0.0:
        fraction_of_target_sent = 0.0
    if fraction_of_target_sent > 1.0:
        fraction_of_target_sent = 1.0
    last_sent_joint_radians = ik.last_sent_joint_radians
    solve_of_the_generators_own_feet = ik.solve(target_feet_positions)
    if solve_of_the_generators_own_feet.gate_allows_these_radians:
        joint_change_from_last_sent = _steps(
            last_sent_joint_radians,
            solve_of_the_generators_own_feet.angles)
        ik.remember_last_sent_radians(
            solve_of_the_generators_own_feet.angles)
        return _mini_step_result(
            generators_own_feet_sent, 1.0, 1.0,
            solve_of_the_generators_own_feet, "", 0,
            target_feet_positions, target_feet_positions,
            joint_change_from_last_sent, True)
    if (solve_of_the_generators_own_feet.reason != "rate_limit"
            or fraction_of_target_sent >= 1.0):
        return _hold(
            solve_of_the_generators_own_feet, fraction_of_target_sent,
            target_feet_positions, last_sent_joint_radians)
    found = _bisect(
        ik, feet_where_this_target_started, target_feet_positions,
        fraction_of_target_sent)
    if found is None:
        return _hold(
            solve_of_the_generators_own_feet, fraction_of_target_sent,
            target_feet_positions, last_sent_joint_radians)
    fraction_reached_along_the_target, trial, feet = found
    joint_change_from_last_sent = _steps(
        last_sent_joint_radians, trial.angles)
    ik.remember_last_sent_radians(trial.angles)
    # The shorter point was allowed. The row still says rate_limit,
    # because the generator's own feet were the pose that was too
    # big. The joint named is that refusal, so the WARN matches the
    # target the generator asked for.
    return _mini_step_result(
        shorter_point_sent,
        fraction_reached_along_the_target,
        fraction_reached_along_the_target,
        trial, "rate_limit",
        solve_of_the_generators_own_feet.joint_id,
        feet, target_feet_positions, joint_change_from_last_sent, True,
        refused_joint_radians=(
            solve_of_the_generators_own_feet.refused_joint_radians),
        last_sent_radians_of_that_joint=(
            solve_of_the_generators_own_feet
            .last_sent_radians_of_that_joint),
        leg=solve_of_the_generators_own_feet.leg)


def should_publish(mini_step_result, dry_run_do_not_publish=False):
    """False when the last safe pose was kept, and false on a dry run.

    A dry run still solved, still passed through the gate, and still
    belongs in the trace with whatever this wake published. It does
    not publish. The vendor keyword on set_pose_base is still named
    pseudo. This flag is only the gait-5 publish decision.
    """
    if dry_run_do_not_publish:
        return False
    return mini_step_result.publish


def odometry_increment(linear_x, linear_y, angular_z,
                       fraction_sent_this_wake, dt=TICK_S):
    """The body twist for the fraction of this target sent this wake.

    Returns scaled linear_x, linear_y, angular_z, then the body-frame
    dx, dy and the yaw change. The scaled velocities are the numbers
    to store on the node. Integrating those over dt is the same as
    integrating the command over dt * fraction_sent_this_wake.

    A hold has fraction_sent_this_wake 0 and adds nothing. The
    mini-step that reaches fraction 1 adds only the leftover
    fraction. The seconds already covered by earlier shorter points
    stay covered. The next target, after the generator advances, is
    a fresh dt of the full command.

    The yaw rotation that turns dx, dy into the world frame stays in
    the 20 ms loop. This function does not know the current yaw.
    """
    scale = dt * fraction_sent_this_wake
    vx = linear_x * fraction_sent_this_wake
    vy = linear_y * fraction_sent_this_wake
    wz = angular_z * fraction_sent_this_wake
    return (vx, vy, wz, linear_x * scale, linear_y * scale,
            angular_z * scale)


class Gait5Session(object):
    """One gait-5 generator, and the target it has not finished.

    gen_next_mini_step_yield yields six feet and then moves the
    cursor. While target_still_open() is true the loop keeps those
    feet and does not call it. A new Twist sitting in the queue
    does not take over either: takeover waits for last_part, and
    last_part is honored only when target_feet_positions are
    accepted with the whole target sent.

    fraction_of_target_sent is how far along the open target has
    been sent. It becomes 1 only when the generator's own feet
    pass the gate. A bisected fraction stays below 1, and the
    cursor stays put.
    """

    def __init__(self, generator, ik, backend):
        self.generator = generator
        self.ik = ik
        self.backend = backend
        # 1 and no target: the loop may call gen_next_mini_step_yield.
        # A fresh session has not begun a target yet.
        self.fraction_of_target_sent = 1.0
        self.feet_where_this_target_started = None
        self.target_feet_positions = None
        self.params = None
        self.ask_for_50_ms_blend = None
        self.last_part = False

    def solve_stance_already_stood_on(self, six_feet):
        """Solve the stance the body is already standing on.

        Nothing is published. Those angles become the last sent
        radians, so the first target is measured from the stand
        and not from zeros. feet_where_this_target_started is this
        stance: the start of the bout. A stance the backend refuses
        leaves that memory empty, and the next target keeps the
        last safe pose with that reason.
        """
        self.feet_where_this_target_started = six_feet
        return self.ik.establish_reference(six_feet)

    def target_still_open(self):
        """True while this target has not been sent in full."""
        return (
            self.target_feet_positions is not None
            and self.fraction_of_target_sent < 1.0)

    def open_target(self):
        """Feet, params, and the 50 ms blend flag of the open target.

        The loop reuses these instead of asking the generator for
        a new yield. A new yield would move the cursor and drop
        the feet this target is still walking toward.
        """
        return (
            self.target_feet_positions,
            self.params,
            self.ask_for_50_ms_blend)

    def begin_target(self, feet, params, ask_for_50_ms_blend, last_part):
        """gen_next_mini_step_yield just yielded these feet.

        They are the open target. last_part is stored and not acted
        on here. The loop acts on it only when mini_step() reports
        that the gait step was accepted, which means these feet
        were sent and they close the period. Calling this again on
        a target that is still open would throw away the fraction
        already sent. The loop calls this only after that yield.
        """
        if self.feet_where_this_target_started is None:
            self.feet_where_this_target_started = feet
        self.target_feet_positions = feet
        self.params = params
        self.ask_for_50_ms_blend = ask_for_50_ms_blend
        self.last_part = bool(last_part)
        self.fraction_of_target_sent = 0.0

    def mini_step(self):
        """Walk the open target as far as the gate allows.

        Returns the MiniStepResult, fraction_sent_this_wake, and
        gait_step_accepted. fraction_sent_this_wake is how much of
        the target this wake added. gait_step_accepted is true only
        when this wake sent the generator's own six feet and those
        feet are the ones that close the period. The controller
        then updates the walk state. A hold returns a fraction of
        0 and leaves the gait step unaccepted, so the same target
        stays open. A shorter point also leaves it unaccepted and
        keeps the same target. When the generator's own feet are
        sent, feet_where_this_target_started moves to those feet
        and the hold is cleared, so the next mini-step may call
        gen_next_mini_step_yield(). last_part is cleared on that
        send, so the same target cannot accept the gait step a
        second time.
        """
        before = self.fraction_of_target_sent
        taken = approach_target(
            self.ik, self.feet_where_this_target_started,
            self.target_feet_positions, self.fraction_of_target_sent)
        self.fraction_of_target_sent = taken.fraction_of_target_sent
        own_feet_were_sent = (
            taken.what_this_wake_published == generators_own_feet_sent)
        # True only for a fully sent target that also closed the
        # period. The controller reads this and, when it is true,
        # updates the walk state. A hold and a shorter point stay
        # false, which keeps the generator on the feet it already
        # yielded.
        gait_step_accepted = own_feet_were_sent and self.last_part
        if own_feet_were_sent:
            self.feet_where_this_target_started = (
                self.target_feet_positions)
            self.target_feet_positions = None
            self.last_part = False
        fraction_sent_this_wake = (
            self.fraction_of_target_sent - before)
        return taken, fraction_sent_this_wake, gait_step_accepted


def read_max_step_rad():
    """The max_step_rad limit safety.cpp returns, or None if the import fails.

    The startup line prints this number once, and prints the peak
    as that number divided by 1.5, which is how safety.cpp defines
    it. HexapodLegIk still imports the module on its own first
    solve. None means the controller can still walk with the closed
    solver: joystick gaits do not need this module.
    """
    try:
        import hexapod_kinematics as hk
    except Exception:
        return None
    return hk.max_step_rad()


def _measured_peak(max_step_rad):
    """The gait-5 peak implied by max_step_rad, or 'unset'.

    safety.cpp stores 1.5 times the measured peak. A startup value
    that is not a number means the import failed, and there is no
    peak to print. A bool is an int in Python, and it is not a limit.
    """
    if isinstance(max_step_rad, bool) or not isinstance(
            max_step_rad, (int, float)):
        return "unset"
    return "%.8g" % (float(max_step_rad) / 1.5)


def startup_line(use_hexapod, trace_path, max_step_rad):
    """The one line at node start.

    The window is the driver's ±120°. max_step_rad is whatever
    safety.cpp currently returns. The peak beside it is that number
    divided by 1.5. The flag passed in is the caller's flag.
    """
    return (
        "use_hexapod_kinematics=%s trace=%s window=±120° "
        "max_step_rad=%s measured_peak=%s"
        % (True if use_hexapod else False, trace_path, max_step_rad,
           _measured_peak(max_step_rad)))


class HoldWatch(object):
    """Hard-hold streak for the step_controller logger.

    A shorter point is not a hold, and it resets the streak. Five
    holds in a row is 100 ms. That logs one ERROR. Further holds
    keep the WARN and do not repeat the ERROR. A shorter point or
    the generator's own feet clears the streak, so a later run of
    five holds logs again.

    The generator is not dropped. The ERROR is the sentence a person
    standing next to the robot looks for. A stand clears the
    generator the way a pose-set already does.
    """

    def __init__(self):
        self.streak = 0
        self.errors = 0

    def note(self, logger, mini_step_result):
        published = mini_step_result.what_this_wake_published
        if published == last_safe_pose_kept:
            _warn_refusal(logger, mini_step_result, "holding")
            self.streak += 1
            if self.streak == 5:
                self.errors += 1
                logger.error(
                    "holding last safe pose, generator target frozen")
            return
        if published == shorter_point_sent:
            _warn_refusal(logger, mini_step_result, "partial")
        self.streak = 0


class SecondLog(object):
    """One INFO per second: backend, last file word, largest |dq|.

    The 50 Hz detail stays in the CSV. The first call of a new
    second prints the second that just finished. The decision
    word in that line is the short file word, sent, partial, or
    held, even though the caller passes the long Python name.
    """

    def __init__(self):
        self._second = None
        self._max_dq = 0.0
        self._decision = ""
        self._backend = ""

    def note(self, logger, t_s, backend, what_this_wake_published,
             joint_change_from_last_sent):
        second = int(math.floor(t_s))
        if self._second is None:
            self._second = second
        elif second != self._second:
            logger.info(
                "backend=%s decision=%s max_dq=%.4f"
                % (self._backend, self._decision, self._max_dq))
            self._second = second
            self._max_dq = 0.0
        self._backend = backend
        self._decision = file_decision_word(what_this_wake_published)
        for value in joint_change_from_last_sent:
            if value == "":
                continue
            magnitude = abs(float(value))
            if magnitude > self._max_dq:
                self._max_dq = magnitude


class TraceWriter(object):
    """The only thread that writes the CSV.

    The 20 ms loop builds a row and calls put. put uses block=False.
    A full queue raises queue.Full; that row is dropped and the
    mini-step continues. A blocking put would stop the mini-step
    when the queue hits 200, which is the stall this thread exists
    to avoid. A drop logs one WARN per second, not one line per
    dropped row.

    The file is opened on the first row, appended, and given a header
    when it is created. The thread flushes about twice a second, or
    every 25 rows. close() joins for a short time and then returns,
    so a halt is not waiting on the disk.
    """

    def __init__(self, path, logger, clock=None, start=True):
        self.path = path
        self.logger = logger
        self.clock = time.monotonic if clock is None else clock
        self.rows = queue.Queue(maxsize=TRACE_QUEUE_BOUND)
        # None so the first dropped row warns. A later drop inside
        # the same second does not.
        self._last_drop_warn = None
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="hexapod_ik_trace", daemon=True)
        # start is false only for the queue-full test, which fills the
        # queue by hand. A live controller starts the thread.
        if start:
            self._thread.start()

    def put(self, row):
        """Hand one row to the writer. Never blocks. False if dropped."""
        try:
            self.rows.put(row, block=False)
            return True
        except queue.Full:
            now = self.clock()
            quiet = self._last_drop_warn is not None
            quiet = quiet and now - self._last_drop_warn < TRACE_DROP_WARN_S
            if not quiet:
                self.logger.warning(
                    "hexapod ik trace queue full; dropping rows")
                self._last_drop_warn = now
            return False

    def close(self, timeout=TRACE_CLOSE_S):
        """Ask the writer to flush, then wait at most timeout seconds."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout)

    def _run(self):
        pending = 0
        last_flush = self.clock()
        handle = None
        try:
            while True:
                try:
                    # Short wait so close() can drain within TRACE_CLOSE_S.
                    # The flush itself is still twice a second, below.
                    row = self.rows.get(timeout=0.05)
                except queue.Empty:
                    row = None
                if row is not None:
                    if handle is None:
                        handle = _TraceFile(self.path)
                    handle.writerow(row)
                    pending += 1
                now = self.clock()
                due = pending >= TRACE_FLUSH_ROWS
                due = due or (pending and now - last_flush >= TRACE_FLUSH_S)
                if due and handle is not None:
                    handle.flush()
                    pending = 0
                    last_flush = now
                if self._stop.is_set() and self.rows.empty():
                    break
        finally:
            if handle is not None:
                handle.flush()
                handle.close()


def trace_row(t_s, backend, mini_step_result):
    """One CSV row. Strings, already formatted, header order.

    A hold stores the angles that were refused, so a later plot
    shows the command that was blocked. A shorter point and the
    generator's own feet store the angles that were sent. The
    foot columns are the feet solved this mini-step. The decision
    column is the short file word. The alpha column is
    fraction_reached_along_the_target, which is 0 on a hold.
    The dq columns are joint_change_from_last_sent.
    """
    joint = ""
    if mini_step_result.joint_id:
        joint = str(mini_step_result.joint_id)
    fields = [
        _fmt(t_s, 6),
        backend,
        file_decision_word(mini_step_result.what_this_wake_published),
        _fmt(mini_step_result.fraction_reached_along_the_target, 8),
        mini_step_result.reason,
        joint,
    ]
    for foot in mini_step_result.feet:
        for coord in foot:
            fields.append(_fmt(coord, 4))
    for angle in mini_step_result.angles:
        fields.append(_fmt(angle, 8))
    for value in mini_step_result.joint_change_from_last_sent:
        if value == "":
            fields.append("")
        else:
            fields.append(_fmt(value, 8))
    return fields


def _from_binding(result, decision):
    # decision is the C++ SafetyDecision. These three attributes
    # stay the binding names. The IkDecision on the way out uses
    # the longer Python names.
    return IkDecision(
        bool(decision.allow),
        _flatten(result.angles),
        decision.reason,
        int(decision.joint_id),
        int(result.leg),
        float(decision.q_now_rad),
        float(decision.q_previous_rad))


def _flatten(angles):
    flat = []
    for leg in angles:
        flat.extend(leg)
    return tuple(flat)


def _steps(last_sent_joint_radians, angles):
    if last_sent_joint_radians is None:
        return ("",) * 18
    return tuple(
        a - b for a, b in zip(angles, last_sent_joint_radians))


def _mini_step_result(
        what_this_wake_published, fraction_reached_along_the_target,
        fraction_of_target_sent, source, reason, joint_id,
        feet, source_feet, joint_change_from_last_sent, publish,
        refused_joint_radians=None,
        last_sent_radians_of_that_joint=None, leg=None):
    if refused_joint_radians is None:
        refused_joint_radians = source.refused_joint_radians
    if last_sent_radians_of_that_joint is None:
        last_sent_radians_of_that_joint = (
            source.last_sent_radians_of_that_joint)
    return MiniStepResult(
        what_this_wake_published,
        fraction_reached_along_the_target,
        fraction_of_target_sent,
        source.angles, reason, joint_id,
        source.leg if leg is None else leg,
        refused_joint_radians,
        last_sent_radians_of_that_joint,
        feet, source_feet, joint_change_from_last_sent, publish)


def _hold(solve_of_the_generators_own_feet, fraction_of_target_sent,
          target_feet_positions, last_sent_joint_radians):
    # The refused-joint columns are the angles that were blocked.
    # last_sent_joint_radians is not updated.
    return _mini_step_result(
        last_safe_pose_kept, 0.0, fraction_of_target_sent,
        solve_of_the_generators_own_feet,
        solve_of_the_generators_own_feet.reason,
        solve_of_the_generators_own_feet.joint_id,
        target_feet_positions, target_feet_positions,
        _steps(
            last_sent_joint_radians,
            solve_of_the_generators_own_feet.angles),
        False)


def _bisect(ik, feet_where_this_target_started, target_feet_positions,
            fraction_of_target_sent):
    span = 1.0 - fraction_of_target_sent
    grain = span / float(BISECTION_PARTS)
    lo = fraction_of_target_sent
    hi = 1.0
    best = None
    best_decision = None
    best_feet = None
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        if mid - fraction_of_target_sent < grain:
            break
        feet = _interpolate(
            feet_where_this_target_started, target_feet_positions, mid)
        trial = ik.solve(feet)
        if trial.gate_allows_these_radians:
            best = mid
            best_decision = trial
            best_feet = feet
            lo = mid
        else:
            hi = mid
    if best is None or best - fraction_of_target_sent < grain:
        return None
    return best, best_decision, best_feet


def _interpolate(feet_where_this_target_started, target_feet_positions,
                 fraction_along_the_target):
    # One fraction for every leg. A per-leg fraction would twist the body.
    feet = []
    for starting_foot, target_position in zip(
            feet_where_this_target_started, target_feet_positions):
        feet.append(tuple(
            starting_foot[axis] + fraction_along_the_target * (
                target_position[axis] - starting_foot[axis])
            for axis in range(3)))
    return tuple(feet)


def _warn_refusal(logger, mini_step_result, how):
    foot = _foot_of(mini_step_result)
    joint = mini_step_result.joint_id if mini_step_result.joint_id else ""
    logger.warning(
        "%s joint=%s angle=%.4f previous=%.4f foot=%s %s"
        % (mini_step_result.reason, joint,
           mini_step_result.refused_joint_radians,
           mini_step_result.last_sent_radians_of_that_joint,
           foot, how))


def _foot_of(mini_step_result):
    # The generator's foot, not the shorter point. joint 1 is leg 1.
    index = None
    if mini_step_result.joint_id >= 1:
        index = (mini_step_result.joint_id - 1) // 3
    elif mini_step_result.leg >= 1:
        index = mini_step_result.leg - 1
    if index is None:
        return None
    if index >= len(mini_step_result.source_feet):
        return None
    return mini_step_result.source_feet[index]


def _fmt(value, digits):
    return format(float(value), ".%df" % digits)


class _TraceFile(object):
    """Append handle. The header is written when the file is created."""

    def __init__(self, path):
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        self._handle = open(path, "a", newline="")
        self._writer = csv.writer(self._handle)
        if new_file:
            self._writer.writerow(TRACE_COLUMNS)
            self._handle.flush()

    def writerow(self, row):
        self._writer.writerow(row)

    def flush(self):
        self._handle.flush()

    def close(self):
        self._handle.flush()
        self._handle.close()
