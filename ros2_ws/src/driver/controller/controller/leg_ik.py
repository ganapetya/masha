# The door the 20 ms loop will use for hunter gait 5.
#
# Two formulas sit behind one call. VendorLegIk asks the closed
# kinematics.so, one leg at a time, which is the path every walk
# uses today. HexapodLegIk asks hexapod_kinematics.solve_pose and
# then the safety gate. make_leg_ik picks one of them. The default
# is the vendor. This file does not read a ROS parameter and it
# does not publish a servo pulse. The loop, the launch argument,
# and the unfinished-frame wait are the next step.
#
# A refused pose comes back as a decision object. An exception here
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

# Eight bisections. The smallest accepted step is 1/256 of what is
# left of this frame. A smaller candidate is no progress.
BISECTION_PARTS = 256

# One ordinary gait tick, seconds. Catch-up scales this by delta_alpha.
TICK_S = 0.02

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


class IkDecision(object):
    """What one solve said.

    allow is whether these 18 radians may be published.
    reason is empty when allow is true. The names match the C++
    gate: unreachable, near_singular, joint_limit, rate_limit,
    non_finite, no_reference.
    joint_id is 1..18 when one joint caused the refusal, else 0.
    leg is 1..6 when the solver named a leg, else 0.
    angles is always 18 radians. On a solver refusal the failed leg
    and the legs after it are zeros, which is solve_pose's own fill.
    """

    def __init__(self, allow, angles, reason, joint_id, leg,
                 q_now_rad, q_previous_rad):
        self.allow = allow
        self.angles = tuple(angles)
        self.reason = reason
        self.joint_id = joint_id
        self.leg = leg
        self.q_now_rad = q_now_rad
        self.q_previous_rad = q_previous_rad


class FrameStep(object):
    """What one tick did with one generator frame.

    decision is sent, partial, or held.
    alpha is the fraction of the frame solved this tick. It is 1
    when the generator's own feet were sent, between 0 and 1 for a
    partial, and 0 when nothing was sent.
    alpha_done is how far along the frame has been sent, including
    this tick. A hold leaves it where it was.
    feet are the six tips that were solved this tick, millimetres.
    source_feet are the six tips the generator yielded. A partial
    solves a point between the anchor and those tips. The WARN names
    the generator's foot, because that is the foot that was refused.
    dq is the 18 steps from the previous sent pose. Empty strings
    when no pose has been sent yet. The controller's joints_state
    starts at 0 and is not that previous pose.
    publish is false on a hold. pseudo is applied later, by
    should_publish: a dry run still carries sent or partial here.
    """

    def __init__(self, decision, alpha, alpha_done, angles, reason,
                 joint_id, leg, q_now_rad, q_previous_rad, feet,
                 source_feet, dq, publish):
        self.decision = decision
        self.alpha = alpha
        self.alpha_done = alpha_done
        self.angles = tuple(angles)
        self.reason = reason
        self.joint_id = joint_id
        self.leg = leg
        self.q_now_rad = q_now_rad
        self.q_previous_rad = q_previous_rad
        self.feet = feet
        self.source_feet = source_feet
        self.dq = tuple(dq)
        self.publish = publish


class LegIk(object):
    """One formula from six feet to eighteen radians.

    previous is the last pose this object was told had been sent.
    It starts empty. joints_state is not a substitute.
    """

    def __init__(self):
        self.previous = None

    def solve(self, pose):
        """Return an IkDecision for these six feet. Do not publish."""
        raise NotImplementedError

    def remember(self, angles):
        """Store angles as the last pose that was actually sent.

        Call this only after a publish, or from establish_reference
        for the pose the body is already standing on.
        """
        self.previous = tuple(angles)

    def establish_reference(self, pose):
        """Solve the pose the body is already standing on.

        On success those angles become previous. Nothing is published:
        the feet are already there. The first moving frame is compared
        with this solve. A pose the backend cannot accept leaves
        previous empty, and the next frame holds with that reason.
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

    def solve(self, pose):
        from kinematics import kinematics
        angles = []
        for leg_id, foot in enumerate(pose, start=1):
            joints = kinematics.set_leg_position(leg_id, foot)
            angles.extend(joints)
        return IkDecision(
            True, angles, "", 0, 0, 0.0, 0.0)

    def establish_reference(self, pose):
        decision = self.solve(pose)
        if decision.allow:
            self.remember(decision.angles)
        return decision


class HexapodLegIk(LegIk):
    """hexapod_kinematics.solve_pose, then the safety gate, once.

    This does not subdivide a frame. approach_frame is the only
    place that turns one rate_limit into a shorter step. The import
    happens on the first solve, so a vendor-only process that never
    builds this object does not load the module.
    """

    def solve(self, pose):
        import hexapod_kinematics as hk
        result = hk.solve_pose(pose)
        decision = hk.gate(result, self.previous)
        return _from_binding(result, decision)

    def establish_reference(self, pose):
        # The gate answers no_reference when previous is empty, even
        # for a legal stand. Compare the stand with itself so the
        # window is checked and the step is zero. A solver refusal
        # is returned as itself and previous stays empty.
        import hexapod_kinematics as hk
        result = hk.solve_pose(pose)
        angles = _flatten(result.angles)
        if not result.ok:
            blank = hk.gate(result, None)
            return _from_binding(result, blank)
        decision = hk.gate(result, angles)
        made = _from_binding(result, decision)
        if made.allow:
            self.remember(made.angles)
        return made


def make_leg_ik(use_hexapod=USE_HEXAPOD_KINEMATICS_DEFAULT):
    """Pick the formula for one gait-5 generator.

    Call this when that generator starts, and keep the object until
    the generator is replaced. A param change mid-step must not build
    a second object for the same feet: the formula would change under
    a moving foot. The next generator, at the next Twist after a step
    boundary or at the next hunt bout, is the next call, and that
    call sees the new flag.

    The default is false. Joystick gaits, voice gait 2, body lean,
    and action groups do not call this. They stay on set_pose_base.
    """
    if use_hexapod:
        return HexapodLegIk()
    return VendorLegIk()


def begin_generator(use_hexapod=USE_HEXAPOD_KINEMATICS_DEFAULT):
    """One gait-5 generator's backend, and the line that names it.

    The line is `gait 5 backend=hexapod` or `gait 5 backend=vendor`.
    Log it once, here, not on every tick.
    """
    ik = make_leg_ik(use_hexapod)
    backend = "hexapod" if use_hexapod else "vendor"
    return ik, backend, "gait 5 backend=%s" % backend


def approach_frame(ik, feet_anchor, feet_frame, alpha_done):
    """Walk one generator frame as far as the gate allows.

    feet_anchor is where this frame started: the last feet that were
    sent at alpha 1, or the feet of the body pose at the start of a
    bout. feet_frame is the six tips the generator yielded for this
    tick. alpha_done is how far along that segment has already been
    sent, in [0, 1].

    The order is fixed.

    1. Solve the generator's own feet. When the gate allows them,
       alpha_done becomes 1 by assignment and the next tick may ask
       the generator for a new frame.
    2. When the only failure is rate_limit, the feet are legal and
       the step is too big. Eight bisections search the open interval
       past alpha_done. One fraction is shared by all six legs:

           feet(alpha) = anchor + alpha * (frame - anchor)

       The largest alpha that passes the whole gate is a partial.
       The generator stays on this frame.
    3. Any other reason is a hard hold. Nothing is published.
       alpha_done and the previous sent pose stay where they are.
    4. A rate_limit whose search finds nothing is the same hold.
       A branch flip is this case: the foot barely moves and every
       fraction still jumps by about a radian, because the standing
       branch does not change along the segment. Catch-up must not
       publish that jump.

    alpha_done becomes 1 only when step 1 passes. Eight bisections
    land on a 1/256 grid of the remaining segment, so a value such
    as 0.996 is a partial step. Rounding that up to 1 would start
    the next frame while these feet were still short, and it would
    do it without the gate having allowed this frame. A candidate
    less than one part in 256 past alpha_done is no progress, and
    that tick is a hold.

    Step 1 solves the generator's own tuple. It does not solve the
    reconstruction at alpha 1. That sum can miss the original
    millimetres by a fraction of a unit in the last place.

    This pause is for hunter gait 5, a tripod. Three feet stay on
    the ground while one frame is stretched over several ticks, so
    the body is not depending on a swing finishing on time. A gait
    that stays upright by momentum would fall if a swing were slowed
    this way. That gait is outside this module.

    The generator object is not dropped here. A hold returns. The
    caller keeps the same frame. A halt (gait -2) is the existing
    stand path, and that is what clears a hold.
    """
    if alpha_done < 0.0:
        alpha_done = 0.0
    if alpha_done > 1.0:
        alpha_done = 1.0
    previous = ik.previous
    full = ik.solve(feet_frame)
    if full.allow:
        dq = _steps(previous, full.angles)
        ik.remember(full.angles)
        return _step(
            "sent", 1.0, 1.0, full, "", 0, feet_frame, feet_frame,
            dq, True)
    if full.reason != "rate_limit" or alpha_done >= 1.0:
        return _hold(full, alpha_done, feet_frame, previous)
    found = _bisect(ik, feet_anchor, feet_frame, alpha_done)
    if found is None:
        return _hold(full, alpha_done, feet_frame, previous)
    alpha, trial, feet = found
    dq = _steps(previous, trial.angles)
    ik.remember(trial.angles)
    # The partial pose was allowed. The row still says rate_limit,
    # because the generator's own feet were the pose that was too big.
    # The joint named is that refusal, so the WARN matches the frame.
    return _step(
        "partial", alpha, alpha, trial, "rate_limit", full.joint_id,
        feet, feet_frame, dq, True,
        q_now_rad=full.q_now_rad, q_previous_rad=full.q_previous_rad,
        leg=full.leg)


def should_publish(step, pseudo=False):
    """False on a hard hold, and false on a dry run.

    pseudo still solved, still gated, and still belongs in the trace
    with whatever decision the gate gave. It does not publish.
    """
    if pseudo:
        return False
    return step.publish


def odometry_increment(linear_x, linear_y, angular_z, delta_alpha,
                       dt=TICK_S):
    """The body twist for the fraction of this frame sent this tick.

    Returns scaled linear_x, linear_y, angular_z, then the body-frame
    dx, dy and the yaw step. The scaled velocities are the numbers to
    store on the node. Integrating those over dt is the same as
    integrating the command over dt * delta_alpha.

    A hold has delta_alpha 0 and adds nothing. The tick that reaches
    alpha 1 adds only the leftover fraction. The seconds already
    covered by earlier partials stay covered. The next frame, after
    the generator advances, is a fresh dt of the full command.

    The yaw rotation that turns dx, dy into the world frame stays in
    the 20 ms loop. This function does not know the current yaw.
    """
    scale = dt * delta_alpha
    vx = linear_x * delta_alpha
    vy = linear_y * delta_alpha
    wz = angular_z * delta_alpha
    return (vx, vy, wz, linear_x * scale, linear_y * scale,
            angular_z * scale)


def startup_line(use_hexapod, trace_path, max_step_rad):
    """The one line at node start.

    The window is the driver's ±120°. max_step_rad is whatever
    safety.cpp currently returns. The measured peak is not written
    yet, so the line says it is unset. While it is unset the flag
    that is passed in stays false.
    """
    return (
        "use_hexapod_kinematics=%s trace=%s window=±120° "
        "max_step_rad=%s measured_peak=unset"
        % (True if use_hexapod else False, trace_path, max_step_rad))


class HoldWatch(object):
    """Hard-hold streak for the step_controller logger.

    A partial send is not a hold, and it resets the streak. Five
    holds in a row is 100 ms. That logs one ERROR. Further holds
    keep the WARN and do not repeat the ERROR. A partial or a sent
    pose clears the streak, so a later run of five holds logs again.

    The generator is not dropped. The ERROR is the sentence a person
    standing next to the robot looks for. A stand clears the
    generator the way a pose-set already does.
    """

    def __init__(self):
        self.streak = 0
        self.errors = 0

    def note(self, logger, step):
        if step.decision == "held":
            _warn_refusal(logger, step, "holding")
            self.streak += 1
            if self.streak == 5:
                self.errors += 1
                logger.error(
                    "holding last safe pose, generator frame frozen")
            return
        if step.decision == "partial":
            _warn_refusal(logger, step, "partial")
        self.streak = 0


class SecondLog(object):
    """One INFO per second: backend, last decision, largest |dq|.

    The 50 Hz detail stays in the CSV. The first call of a new second
    prints the second that just finished.
    """

    def __init__(self):
        self._second = None
        self._max_dq = 0.0
        self._decision = ""
        self._backend = ""

    def note(self, logger, t_s, backend, decision, dq):
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
        self._decision = decision
        for value in dq:
            if value == "":
                continue
            magnitude = abs(float(value))
            if magnitude > self._max_dq:
                self._max_dq = magnitude


class TraceWriter(object):
    """The only thread that writes the CSV.

    The 20 ms loop builds a row and calls put. put uses block=False.
    A full queue raises queue.Full; that row is dropped and the tick
    continues. A blocking put would stop the tick at the moment the
    queue hits 200, which is the stall this thread exists to avoid.
    A drop logs one WARN per second, not one line per dropped row.

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


def trace_row(t_s, backend, step):
    """One CSV row. Strings, already formatted, header order.

    held stores the angles that were refused, so a later plot shows
    the command that was blocked. partial and sent store the angles
    that were sent. The foot columns are the feet solved this tick.
    """
    joint = ""
    if step.joint_id:
        joint = str(step.joint_id)
    fields = [
        _fmt(t_s, 6),
        backend,
        step.decision,
        _fmt(step.alpha, 8),
        step.reason,
        joint,
    ]
    for foot in step.feet:
        for coord in foot:
            fields.append(_fmt(coord, 4))
    for angle in step.angles:
        fields.append(_fmt(angle, 8))
    for value in step.dq:
        if value == "":
            fields.append("")
        else:
            fields.append(_fmt(value, 8))
    return fields


def _from_binding(result, decision):
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


def _steps(previous, angles):
    if previous is None:
        return ("",) * 18
    return tuple(a - b for a, b in zip(angles, previous))


def _step(decision, alpha, alpha_done, source, reason, joint_id,
          feet, source_feet, dq, publish,
          q_now_rad=None, q_previous_rad=None, leg=None):
    return FrameStep(
        decision, alpha, alpha_done, source.angles, reason, joint_id,
        source.leg if leg is None else leg,
        source.q_now_rad if q_now_rad is None else q_now_rad,
        source.q_previous_rad if q_previous_rad is None else q_previous_rad,
        feet, source_feet, dq, publish)


def _hold(full, alpha_done, feet_frame, previous):
    # q columns are the angles that were refused. previous is not updated.
    return _step(
        "held", 0.0, alpha_done, full, full.reason, full.joint_id,
        feet_frame, feet_frame, _steps(previous, full.angles), False)


def _bisect(ik, feet_anchor, feet_frame, alpha_done):
    span = 1.0 - alpha_done
    grain = span / float(BISECTION_PARTS)
    lo = alpha_done
    hi = 1.0
    best = None
    best_decision = None
    best_feet = None
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        if mid - alpha_done < grain:
            break
        feet = _interpolate(feet_anchor, feet_frame, mid)
        trial = ik.solve(feet)
        if trial.allow:
            best = mid
            best_decision = trial
            best_feet = feet
            lo = mid
        else:
            hi = mid
    if best is None or best - alpha_done < grain:
        return None
    return best, best_decision, best_feet


def _interpolate(feet_anchor, feet_frame, alpha):
    # One fraction for every leg. A per-leg fraction would twist the body.
    feet = []
    for anchor, frame in zip(feet_anchor, feet_frame):
        feet.append(tuple(
            anchor[axis] + alpha * (frame[axis] - anchor[axis])
            for axis in range(3)))
    return tuple(feet)


def _warn_refusal(logger, step, how):
    foot = _foot_of(step)
    joint = step.joint_id if step.joint_id else ""
    logger.warning(
        "%s joint=%s angle=%.4f previous=%.4f foot=%s %s"
        % (step.reason, joint, step.q_now_rad, step.q_previous_rad,
           foot, how))


def _foot_of(step):
    # The generator's foot, not the partial point. joint 1 is leg 1.
    index = None
    if step.joint_id >= 1:
        index = (step.joint_id - 1) // 3
    elif step.leg >= 1:
        index = step.leg - 1
    if index is None:
        return None
    if index >= len(step.source_feet):
        return None
    return step.source_feet[index]


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
