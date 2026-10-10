import sys
import math
import time
import numpy as np
from kinematics import kinematics
import copy

# Gait generators for the hexapod.
# MovingGenerator, CmdVelGenerator, and FollowGaitGenerator are Python
# generators. Calling one only builds the object. StepController resumes
# it through _GeneratorCall, and those methods are what call send():
#   1) gen_advance_to_first_yield once, up to the first yield
#   2) then gen_next_mini_step_yield(pose, generator_slice), which returns
#      (out_pose, last_part, params, slow)
# generator_slice is one of the three texts defined just below.
# The generator uses it to choose which part of its prepared foot
# list to yield. last_part is True when a gait step just wrapped,
# so the outer loop in step_controller can swap or stop generators
# on that accepted gait step.
# slow is a bool. True asks for a 50 ms servo blend on this one pose.
# False keeps the usual 20 ms command. A counted walk always yields
# False. The boundary tells a counted walk from a velocity walk by the
# params object (MovingParams versus CmdVelParams), not by this flag.

# The three texts _WalkState.generator_slice can hold. They are
# defined here because this file is where the generator compares
# them, and step_controller already imports this module. The
# controller passes one text on every resume.
#
# generator_intro is the opening section, from the splice point
# to the end of the prepared list. A cold start uses it, and so
# does the first period after a new generator is installed.
# generator_running is the whole prepared gait step, wrapping at
# the end. generator_outro is the ending section, from the start
# of the list up to the splice. While that ending plays, each
# pose is saved in finish_ps so the next generator can begin
# near those feet.
generator_intro = 'generator_intro'
generator_running = 'generator_running'
generator_outro = 'generator_outro'


class MovingParams:
    def __init__(self, **kwargs):
        self.forever = False # 永远运行(run forever, ignore repeat)
        self.repeat = 1 # 重复次数(how many full steps to take)
        self.interrupt = True # 是否中断当前移动(whether a new command may interrupt the current one)

        self.gait = 1 # 步态选择 1-波纹步态 2-三角步态(gait: 1 = ripple, 2 = tripod)
        self.stride = 0 # 步态步幅(commanded stride length, mm)
        self.height = 0 # 步态抬腿高度(foot lift height; mm, or percent if relative_h)
        self.direction = 0 # 移动方向(travel heading in radians, body frame)
        self.rotation = 0 # 每步自转角度(yaw change per step, radians)
        self.relative_h = False # 步高使用相对高度(True: height is percent of current stance z)
        self.linear_factor = 1.0 # 直线行走时的步幅系数， 即走一步给定的步幅和实际的步幅(scale from commanded stride to the value sent to kinematics)
        self.rotate_factor = 1.0 # 转向时的转向系数， 即给定的梅(scale from commanded yaw-per-step to the kinematics value; Chinese comment is truncated)
        self.velocity_x = 0 
        self.velocity_y = 0 

        self.period = 1 # 每步间隔(duration of one full step, seconds)
        self.__dict__.update(kwargs) # 用具名参数更新实例(overwrite defaults with the named kwargs)
    
    def __str__(self) -> str:
        return str(self.__dict__)

class CmdVelParams:
    """Parameters for velocity-mode walking (cmd_vel). Units: mm/s, rad/s, mm, seconds."""
    def __init__(self, **kwargs):
        self.gait = 1
        self.velocity_x = 0 
        self.velocity_y = 0 
        self.angular_z = 0
        self.height = 10.0
        self.relative_h = False # 步高使用相对高度(True: height is percent of current stance z)

        self.period = 1 # 每步间隔(duration of one gait cycle, seconds)

        self.linear_factor = 1.0 # 直线行走时的步幅系数， 即走一步给定的步幅和实际的步幅(scale from commanded stride to actual stride)
        self.rotate_factor = 1.0 # 转向时的转向系数， 即给定的值(scale from commanded yaw rate to actual yaw rate)

        self.__dict__.update(kwargs) # 用具名参数更新实例(overwrite defaults with the named kwargs)

    def __str__(self) -> str:
        return str(self.__dict__)
    


def MovingGenerator(params, log=None):
    """Yield counted gait poses for Traveling / set_step_mode.

    One gait step is 6 kinematic parts (i = 0..5). Each part is further
    split into sub_action_num mini-steps so the gait step lasts
    params.period at 50 Hz.
    """

    # 根据是相对高度还是绝对高度计算实际的高度(actual lift height is computed later from relative_h vs absolute)
    org_pose = None
    part_index = 0
    sub_index = 0

    # 50 Hz loop (0.02 s). One gait step = 6 parts, so
    # mini-steps per part = period / 6 / 0.02
    sub_action_num = params.period / 6.0 / 0.02 # 一步会分为六部分，每部分多少个小步(mini-steps per part)
    sub_action_num = math.ceil(round(max(sub_action_num, 1), 3)) # 每一分步最少要有一个子动作(each part must have at least one sub-action)
    height = params.height
    real_stride = params.stride * params.linear_factor
    real_rotate = params.rotation * params.rotate_factor

    # Per mini-step increments. kinematics.set_step_mode still gets the full
    # gait-step real_stride / real_rotate; these locals are unused leftovers.
    sub_stride = params.stride / (sub_action_num * 6.0)
    sub_rotate = params.rotation / (sub_action_num * 6.0)

    # gen_advance_to_first_yield parks here. The next resume fills
    # cur_pose from the stance, and generator_slice from the walk
    # state. A counted walk receives that text and does not branch
    # on it. When the period ends, the controller replaces this
    # whole generator instead of asking it to play an ending.
    cur_pose, generator_slice = yield None
    while params.repeat > 0 or params.forever:
        org_pose = cur_pose
        if params.relative_h:
            # percent of current foot z (stance height), not an absolute mm lift
            height = abs(org_pose[0][2]) * (params.height / 100.0) 
        poses = [] # 计算到的步态的所有姿态(all gait poses for this one gait step, 6 parts)
        # 波纹步态时有根据不同的方向会有条腿刚好在下落过程，
        # 如果在立正状态下下落的话会将机体撑起来，所以需要先将这条腿抬起
        # (In ripple gait, depending on heading, one leg is already in its down-stroke.
        #  From a stand pose that would push the body up, so pre-lift that leg before computing the six parts.)
        if params.gait == 1:  # 波纹步态(ripple gait)
            # heading >= pi: pre-lift leg 0; otherwise pre-lift leg 3
            if params.direction >= math.pi:
                start_leg = list(org_pose[0])
                start_leg[2] += height
                start_pose = list(org_pose)
                start_pose[0] = start_leg
            else:
                start_leg = list(org_pose[3])
                start_leg[2] += height
                start_pose = list(org_pose)
                start_pose[3] = start_leg
        else: # 非波纹步态(not ripple: usually tripod — start from the current pose as-is)
            start_pose = org_pose
        
        # 计算整个过程的所有步态过程(precompute all 6 parts of this gait step)
        for i in range(6):
            ps = kinematics.set_step_mode(sub_action_num, 
                                        i, 
                                        start_pose, 
                                        params.gait, 
                                        real_stride,
                                        height, 
                                        params.direction, 
                                        real_rotate)            

            ps = np.array(ps)
            # 出来的数据是每条腿的多个姿态，转成每个小步的六腿快照(per-leg sequences become one six-leg snapshot per mini-step)
            ps = ps.reshape((6, -1, 3))  # 转换为[六条腿[子动作[坐标]]](shape: [6 legs][sub-actions][x,y,z])
            ps = np.transpose(ps, (1, 0, 2))  # 转换为[子动作[六条腿[坐标]]](shape: [sub-actions][6 legs][x,y,z])
            start_pose = ps[-1]  # next part continues from this part's last mini-step
            poses.append(ps)
        # log.info(f'{generator_slice}, {poses}')    
        while params.repeat > 0 or params.forever:
            out_pose = poses[part_index][sub_index]
            # log.info('forvoer'+str(params.forever)) 

            # 各个计数进行调整(advance the two nested counters)
            sub_index = (sub_index + 1) % sub_action_num # 子动作计数 +1(next sub-action in this part)
            if sub_index == 0:
                part_index = (part_index + 1) % 6 # 分步计数 +1(next of the 6 parts)
                if part_index == 0:
                    # wrapped one gait step; forever keeps repeat at 0 so the outer while stays true
                    params.repeat = max(params.repeat - 1, 0)
            # log.info(f'{part_index} {sub_index}')
            # last_part is True only when both counters have just
            # wrapped, which is the end of one gait step. The fourth
            # value is False because a counted walk never asks for
            # the 50 ms blend. The controller sees MovingParams on
            # that True and swaps generators inside the same call.
            # generator_slice comes back on the resume and is not
            # consulted; the swap replaces this generator outright.
            cur_pose, generator_slice = yield (
                out_pose,
                part_index == 0 and sub_index == 0,
                params,
                False)

            # 两次初始姿态变了我们需要重算(stance pose object changed: break inner loop and recompute the 6 parts)
            # Identity check (is not), not a value compare: set_pose_base assigns a new tuple.
            if cur_pose is not org_pose:
                break


def total_dist_sq(a_pts, b_pts):
    """
    计算两个 6×(x,y,z) 列表之间的“距离平方和”：
      sum_{j=0..5} [ (ax-bx)^2 + (ay-by)^2 + (az-bz)^2 ]
    注意：这里不做最后的 sqrt，这样可以少调用 n 次 sqrt。
    (Sum of squared 3D distances between two 6-foot poses.
     Skip the final sqrt so we only need this as a nearest-neighbor score.)
    """
    s = 0.0
    for (x1,y1,z1), (x2,y2,z2) in zip(a_pts, b_pts):
        dx = x1 - x2
        dy = y1 - y2
        dz = z1 - z2
        s += dx*dx + dy*dy + dz*dz
    return s

# Module-level handoff shared by CmdVelGenerator and FollowGaitGenerator.
# While generator_slice is generator_outro, the live generator stores
# the pose it is about to yield into finish_ps, on every outro wake,
# so the last one is the pose the feet were given when that section
# wrapped. The next generator searches its new gait step for the
# target nearest that pose and starts there, which keeps the feet
# from jumping. slow is the blend flag yielded with each pose. True
# asks the servo loop for a 50 ms blend. False keeps the usual 20 ms
# command. stop means we refused to splice at target 0, because that
# target would make the outro an empty list. finish_index is the
# cursor after an outro yield. No other function reads it.
finish_ps = []
slow = False
stop = False   
finish_index = None

def CmdVelGenerator(params, log=None):
    """Yield continuous gait poses for a velocity command.

    Builds one gait-step table, from phase 0 around to just before
    a full turn, then plays it. The controller's walk state chooses
    the section by the text it sends on each resume.
    generator_intro reads from the splice point to the end of the
    table, which is how a walk starts and how a new generator
    starts after a swap. generator_outro reads from the start of
    the table up to that splice, so the feet can stop on a pose
    the next generator knows how to meet. generator_running ignores
    the cut and reads the whole table, wrapping at the end.
    """
    global finish_ps, slow,  stop, finish_index
    height = params.height
    # kinematics.cmd_vel_new_point uses the opposite numbering from MovingParams:
    # here 1 = tripod, 2 = ripple. Incoming params.gait is still 1=ripple, 2=tripod.
    gait = 2 if params.gait == 1 else 1

    org_pose = None
    phase_index = 0 # 当前相位游标(index into the current phase slice being played)
    # One target every 20 ms. round then ceil so 1.0 s → 50 targets.
    targets_per_gait_step = math.ceil(
        round((params.period * 1000.0 / 20.0), 1))
    phase_list = [
        (i / targets_per_gait_step) * 2.0 * math.pi
        for i in range(targets_per_gait_step)]

    # AEP/PEP offsets from body velocity. AEP = anterior extreme position
    # (where a swinging foot will land), PEP = posterior extreme position
    # (where a stance foot will lift). frac_theta_2_* is the half-yaw
    # rotation of the body during one cycle, as sin/cos.
    aep_offset_x, aep_offset_y, frac_theta_2_sin, frac_theta_2_cos = kinematics.cmd_vel_basic_data(
        params.velocity_x, #* 1.0, #0.633 ,
        params.velocity_y,
        params.angular_z, #*0.720,
        params.period
    )

    # gen_advance_to_first_yield parks here.
    cur_pose, generator_slice = yield None
    while True:
        # phase_index = 0
        org_pose = cur_pose
        if params.relative_h:
            height = abs(org_pose[0][2]) * (params.height / 100.0) 
        aep, pep = kinematics.cmd_vel_aep_pep(
            cur_pose,
            aep_offset_x,
            aep_offset_y,
            frac_theta_2_sin,
            frac_theta_2_cos,
        )

        # Precompute one full cycle of 6-leg poses, one pose per phase.
        steps = []
        for phase in phase_list:
            ps = kinematics.cmd_vel_new_point(
                gait,
                height,
                phase,
                aep,
                pep
            )
            steps.append(ps)

        # Pick a splice index `idx` so we do not start at a pose far from
        # the feet's current position (finish_ps from the previous generator).
        if finish_ps:
            dists = [ total_dist_sq(finish_ps, bi) for bi in steps ]
            idx = min(range(len(dists)), key=lambda i: dists[i])

            if idx == 0:
                # phase 0 is the gait-step wrap; skip it so the first yield is a real target
                idx = 1

                stop = True    
            else:
                stop = False                     
        else:
            # No previous pose: start near the mid-cycle sample whose leg-1 x is closest to 0
            idx = min(range(len(steps)//2), key=lambda i: abs(steps[:len(steps) // 2][i][1][0]))
            if idx == 0:
                idx = 1

                stop = True      
            else:
                stop = False     
 
        # The table is cut at the splice. steps1 is the opening
        # section, played while the walk state says generator_intro.
        # steps2 is the ending section, played while it says
        # generator_outro. generator_running does not use either
        # cut. It reads steps from beginning to end.
        steps1 = steps[idx:]
        steps2 = steps[:idx]

        while True:
            if generator_slice == generator_intro:
                # Opening section: from the splice toward the end of
                # the table. The first pose of a new generator is
                # compared with the pose the previous generator
                # saved. A jump larger than 7 mm, or a splice that
                # had to skip the seam, asks the controller for a
                # 50 ms blend on this one wake. The saved pose is
                # then cleared, so later wakes of this same opening
                # do not ask again.
                ps = steps1[phase_index]
                if finish_ps:
                    distances = max([math.dist(p1, p2) for p1, p2 in zip(ps, finish_ps)])
                    if distances > 7 or stop:
                        slow = True
                    finish_ps = []
                else:
                    slow = False
                phase_index = (phase_index + 1) % (len(steps) - idx)
            elif generator_slice == generator_outro:
                # Ending section: from the start of the table up to
                # the splice, so a stop can park on a pose the next
                # generator can meet. Every pose of this section is
                # written into finish_ps. The write that matters is
                # the last one, because that is the pose the feet
                # actually hold when the section wraps.
                ps = steps2[phase_index]
                phase_index = (phase_index + 1) % (idx)
                finish_ps = ps
                finish_index = phase_index
            else:
                # generator_running, and any text that is neither
                # the opening nor the ending. The whole gait step,
                # seam included, one target after another.
                ps = steps[phase_index]
                phase_index = (phase_index + 1) % targets_per_gait_step
            if phase_index == 0:
                # The cursor wrapped, so this pose is the last
                # target of whichever section we are in. last_part
                # is True and the controller may order an outro or
                # start a generator it stored earlier. The text that
                # comes back on the resume is the walk state's new
                # generator_slice, which is how the section changes
                # without this function writing the walk state.
                cur_pose, generator_slice = yield ps, True, params, slow
            else:
                cur_pose, generator_slice = yield ps, False, params, slow
            # 两次初始姿态变了我们需要重算(stance pose object changed: recompute AEP/PEP and the cycle table)
            if cur_pose is not org_pose:
                break


# ---------------------------------------------------------------------------
# Follow gait (gait=5). The foot curves match
# proud_up/include/proud_up/follow_gait.hpp, retyped in sample_follow_gait
# below. This file does not include that header and does not call it.
#
# FollowGaitGenerator does not call kinematics.set_step_mode or
# cmd_vel_new_point. StepController still owns the 20 ms loop and turns
# the tips into angles. With use_hexapod_kinematics on,
# hexapod_kinematics does that job, and the safety gate can refuse
# the result. With the flag off, the closed set_leg_position still does.
#
# Same three slice texts as CmdVelGenerator, so a halt can finish
# through the ending section and a new Twist can start at the
# splice on last_part without the feet jumping.
# ---------------------------------------------------------------------------

_PI = math.pi
_TWO_PI = 2.0 * math.pi
_W_EPS = 1.0e-6
_GROUP_A = (0, 2, 4)


def _sigma(tau):
    """Swing clock only. σ-dot = 0 at lift and land (kiss the floor)."""
    t = 0.0 if tau < 0.0 else (1.0 if tau > 1.0 else tau)
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


def _se2_exp(vx, vy, wz, t):
    """SE(2) exponential of a constant body twist. vx, vy mm/s, t seconds."""
    theta = wz * t
    if abs(wz) < _W_EPS:
        return vx * t, vy * t, theta
    s = math.sin(theta)
    omc = 1.0 - math.cos(theta)
    return (vx * s - vy * omc) / wz, (vy * s + vx * omc) / wz, theta


def _rot2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c


def _body_of_world_fixed(p0, vx, vy, wz, t):
    tx, ty, th = _se2_exp(vx, vy, wz, t)
    x, y = _rot2(p0[0] - tx, p0[1] - ty, -th)
    return x, y, p0[2]


def _clamp_stride(end_xy, p0, stride_max):
    dx = end_xy[0] - p0[0]
    dy = end_xy[1] - p0[1]
    d = math.hypot(dx, dy)
    if d <= stride_max or d < 1e-9:
        return end_xy
    s = stride_max / d
    return (p0[0] + dx * s, p0[1] + dy * s)


def _aep_pep(p0, vx, vy, wz, ts, stride_max, linear_factor=1.0):
    vx *= linear_factor
    vy *= linear_factor
    half = 0.5 * ts
    aep = _body_of_world_fixed(p0, vx, vy, wz, -half)
    pep = _body_of_world_fixed(p0, vx, vy, wz, +half)
    ax, ay = _clamp_stride((aep[0], aep[1]), p0, stride_max)
    px, py = _clamp_stride((pep[0], pep[1]), p0, stride_max)
    return (ax, ay, p0[2]), (px, py, p0[2])


def sample_follow_gait(phi, vx, vy, wz, lift, period, nominal, stride_max=55.0, linear_factor=1.0):
    """One 20 ms bead. vx,vy mm/s; lift mm; period s; nominal six (x,y,z) mm.

    Cycle clock keeps rolling (no rest at wrap). Swing uses sigma(τ).
    """
    phi = phi % _TWO_PI
    if phi < 0.0:
        phi += _TWO_PI
    T = period if period > 1e-3 else 0.70
    ts = 0.5 * T
    a_swing = phi < _PI
    tau_lin = (phi / _PI) if a_swing else ((phi - _PI) / _PI)
    sig = _sigma(tau_lin)
    feet = []
    for leg in range(6):
        p0 = nominal[leg]
        aep, pep = _aep_pep(p0, vx, vy, wz, ts, stride_max, linear_factor)
        swinging = a_swing if (leg in _GROUP_A) else (not a_swing)
        if swinging:
            o = 1.0 - sig
            x = o * pep[0] + sig * aep[0]
            y = o * pep[1] + sig * aep[1]
            z = p0[2] + 4.0 * sig * (1.0 - sig) * lift
        else:
            o = 1.0 - tau_lin
            x = o * aep[0] + tau_lin * pep[0]
            y = o * aep[1] + tau_lin * pep[1]
            z = p0[2]
        feet.append((x, y, z))
    return feet


def FollowGaitGenerator(params, log=None):
    """Yield omnidirectional-tripod poses for gait=5 / cmd_gait=5.

    Never calls kinematics.set_step_mode. StepController turns the tips
    into angles. With use_hexapod_kinematics on, hexapod_kinematics does
    that job, and the safety gate can refuse the result.

    Python generator: `yield` freezes this function and hands one target to
    StepController. The controller's 20 ms loop plays one mini-step.
    This function is the table of targets. Calling FollowGaitGenerator(...)
    only builds the generator object. The body runs when _GeneratorCall
    resumes it, and that method is what calls Python's send().

    The walk state uses the same three texts as CmdVelGenerator.
    gen_advance_to_first_yield runs this function up to the first
    yield, and no feet exist yet. generator_intro then reads the
    opening section, from the splice to the end of the table, which
    is a cold start and also the first period after a swap.
    generator_running reads the whole table. generator_outro reads
    the ending section and saves each pose into finish_ps. The bool
    beside each pose is last_part. True means this section just
    wrapped, so the controller may order an outro or start a
    generator it stored earlier. A new Twist does not edit this
    table. It builds a new generator and waits for last_part.
    """
    # `global` is a write permit, not an import. These four names already
    # exist at the top of this file and are shared with CmdVelGenerator.
    # Without `global`, `finish_ps = []` below would create a local and
    # the next generator would never see the hand-off pose.
    # finish_ps is that hand-off: the previous animation's last feet, used
    # once so two generators can be glued without a servo snap.
    global finish_ps, slow, stop, finish_index

    # Requested peak toe clearance, millimetres. relative_h may replace it
    # with a percent of the current stance height on each bake.
    height = params.height
    # Stance object this bake was computed from. The inner loop compares
    # it with `is` (same Python object). It is StepController.pose, the
    # stored foot targets, not a reading from the servos.
    org_pose = None
    # Cursor into whichever slice we are playing. Survives a rebake;
    # the outer loop does not put it back to 0.
    phase_index = 0

    # How many targets fit in one gait step. One target is the nominal
    # 20 ms sample. round-to-1-decimal, then ceil, so binary float noise
    # (30.0000001) does not become an extra target. 1.00 s → 50 targets.
    # The hunter's 0.60 s → 30 targets.
    targets_per_gait_step = math.ceil(
        round((params.period * 1000.0 / 20.0), 1))
    # Two targets minimum. A one-target gait step wraps on every
    # mini-step (`% 1` is always 0), and the controller would treat
    # every mini-step as a gait-step boundary.
    targets_per_gait_step = max(int(targets_per_gait_step), 2)

    # The gait clock is an angle. A for-loop cannot walk "0 to 2π", so
    # each target gets one sample: target i is at fraction
    # i/targets_per_gait_step of the circle. The list starts at 0 and
    # stops just before 2π. The next lap is target 0 again. That seam
    # is why index 0 is special below.
    phase_list = [
        (i / targets_per_gait_step) * 2.0 * math.pi
        for i in range(targets_per_gait_step)]

    # Offset cap (|AEP − home| and |PEP − home|), not the full PEP→AEP
    # travel. 55 mm → about 75 mm of toe travel at the hunter's
    # vx 0.25 m/s and period 0.60 s.
    # 40 mm was the short shuffle. sample_follow_gait clamps each foot so
    # a large command cannot park a toe past this radius from its home.
    stride_max = 55.0
    # Multiplies vx and vy inside _aep_pep. wz is left alone.
    # CmdVelParams always has this attribute; getattr keeps a stray params
    # object from crashing the bake. Missing → 1.0, meaning "use vx, vy as given".
    linear_factor = getattr(params, 'linear_factor', 1.0)

    # Once, before any foot pose. Later mini-steps do not log. The numbers are
    # the command this generator was built with; a new Twist is a new object.
    if log is not None:
        log.info(
            'FollowGaitGenerator start (gait=5, never set_step_mode). '
            f'vx={params.velocity_x:.1f} mm/s vy={params.velocity_y:.1f} '
            f'wz={params.angular_z:.3f} T={params.period:.2f}s h={height:.1f} mm '
            f'stride_cap={stride_max:.0f} mm'
        )

    # Prime. `cur_pose, generator_slice = yield None` gives None to the caller
    # and, on the next resume, receives whatever they pass.
    # gen_advance_to_first_yield only gets us here and discards the None.
    # The first gen_next_mini_step_yield(stance, generator_intro)
    # fills cur_pose and generator_slice. The controller starts
    # every new generator on the opening section.
    cur_pose, generator_slice = yield None

    # Outer loop, the animator. Bakes one full cycle into `steps`, then
    # the inner loop plays it. We come back here only when the controller
    # replaces self.pose with a different object (a stand, or a body move).
    # A normal mini-step does not. A new velocity does not either: that
    # waits for last_part and then runs a new generator from the top.
    while True:
        org_pose = cur_pose
        # Six homes as plain (x, y, z) floats, millimetres, body frame.
        # Each foot's curve is drawn around its own home.
        nominal = [(float(p[0]), float(p[1]), float(p[2])) for p in cur_pose]
        if params.relative_h:
            # Stance z is negative: the feet are below the body origin
            # (DEFAULT_POSE uses −INITIAL_HEIGHT). abs() is the ruler.
            # params.height is then a percent of leg 0's stance height.
            # That number is the peak of the swing arch in sample_follow_gait:
            # z = home_z + 4·σ·(1−σ)·height, highest when σ = 0.5.
            # Twist walking passes relative_h False, so this stays off;
            # set_step_mode gait 5 can turn it on.
            height = abs(nominal[0][2]) * (params.height / 100.0)

        # Bake every target now. This for-loop does not yield. After it,
        # `steps[i]` is the six toes at phase_list[i].
        steps = []
        for phase in phase_list:
            # phase: radians along the circle. vx, vy: mm/s. angular_z:
            # rad/s. height: peak lift, mm. period: seconds. nominal: the
            # six homes. stride_max / linear_factor: resolved above.
            steps.append(sample_follow_gait(
                phase,
                params.velocity_x,
                params.velocity_y,
                params.angular_z,
                height,
                params.period,
                nominal,
                stride_max,
                linear_factor,
            ))

        # Splice, once per bake. Target 0 puts one tripod at PEP and the
        # other at AEP, the two extremes. Playback has to be allowed to
        # start at a later target that already looks like the current feet.
        if finish_ps:
            # Sum of squared foot-to-foot distances. Skipping the square
            # root is enough: we only need which target is nearest.
            dists = [total_dist_sq(finish_ps, bi) for bi in steps]
            idx = min(range(len(dists)), key=lambda i: dists[i])
            if idx == 0:
                # Target 0 would make the outro steps[:0] empty, and
                # the first generator_outro wake would index that
                # empty list. Start one target later. stop remembers
                # that we refused the seam. The generator_intro branch
                # uses that memory, while finish_ps is still set, to
                # ask for the slow blend.
                idx = 1
                stop = True
            else:
                stop = False
        else:
            # No hand-off pose. That is a cold start, and also a rebake.
            # The opening section already cleared finish_ps, so a later
            # stance change comes through here too. Leg 1 (the second
            # tuple) stands at x = 0 in DEFAULT_POSE, so "leg 1's x
            # closest to 0" means that foot is near its forward home.
            # Search only the first half of the cycle. The second half
            # is the other tripod.
            idx = min(range(len(steps) // 2), key=lambda i: abs(steps[i][1][0]))
            if idx == 0:
                # Same empty-outro guard as the hand-off path. This
                # branch does not arm the slow blend. finish_ps is
                # empty, so the opening section below takes the other
                # arm and writes slow = False.
                idx = 1
                stop = True
            else:
                stop = False

        # Cut the table at the splice. Target 0 is the seam, where
        # the two tripods are at opposite extremes, so playback has
        # to be allowed to start mid-table. steps1 is the opening
        # section, played when generator_slice is generator_intro.
        # steps2 is the ending section, played when it is
        # generator_outro. generator_running ignores both cuts and
        # reads the whole table.
        steps1 = steps[idx:]
        steps2 = steps[:idx]

        # Inner loop. One yield per resume. At each yield we freeze.
        # About 20 ms later the controller resumes us with the stance
        # it still holds, plus the next generator_slice. That text is
        # how the walk state changes which section we read, without
        # this function writing the walk state itself.
        while True:
            if generator_slice == generator_intro:
                # Opening section: from the splice toward the end of
                # the math cycle. Only the first wake of this section
                # still has the saved pose from the previous
                # generator. The largest single-toe jump to this
                # target, in millimetres, is compared with 7. Over
                # 7 mm, or a splice that skipped the seam, asks the
                # controller for a 50 ms command on this one wake.
                # The thread still sleeps 20 ms, and the next wake
                # replaces that pulse. The saved pose is cleared
                # here, or every later wake of the opening would
                # run the test again.
                ps = steps1[phase_index]
                if finish_ps:
                    distances = max([math.dist(p1, p2) for p1, p2 in zip(ps, finish_ps)])
                    if distances > 7 or stop:
                        slow = True
                    finish_ps = []
                else:
                    # Later wakes of the opening, and a cold start
                    # that had nothing saved: ordinary 20 ms timing.
                    slow = False
                # Advance inside the opening. Its length is
                # targets_per_gait_step minus idx, which is
                # len(steps1). Wrapping this modulo is what makes
                # phase_index come back to 0 at the end of the
                # opening, and that 0 is what sets last_part.
                phase_index = (phase_index + 1) % (len(steps) - idx)
            elif generator_slice == generator_outro:
                # Ending section: from the math seam back toward the
                # splice, so a stop can park on a pose the next
                # generator knows how to meet. Every pose is written
                # into finish_ps. The write that matters is the one
                # on the wrapping yield, because that pose is what
                # the next generator splices against.
                ps = steps2[phase_index]
                # max(idx, 1) keeps this modulo off zero if idx were
                # ever left at 0. The splice above already forces
                # idx to 1.
                phase_index = (phase_index + 1) % max(idx, 1)
                finish_ps = ps
                # Cursor after the increment. Stored, never read.
                finish_index = phase_index
            else:
                # generator_running, and any text that is neither the
                # opening nor the ending. The whole gait step, seam
                # included. When targets_per_gait_step is 30, the
                # step from 29 back to 0 is the wrap. The target we
                # just took was the last one of this gait step.
                ps = steps[phase_index]
                phase_index = (phase_index + 1) % targets_per_gait_step

            # The target was chosen first, then the cursor moved.
            # Cursor 0 means the move wrapped, so this pose is the
            # last target of whichever section we are in. True lets
            # the controller order an outro or start a generator it
            # stored earlier. False means we are still in the middle
            # of the section.
            if phase_index == 0:
                cur_pose, generator_slice = yield ps, True, params, slow
            else:
                cur_pose, generator_slice = yield ps, False, params, slow

            # Wake-up, about 20 ms later. `is` checks the object, not the
            # millimetres. Gait ticks call set_pose_base(..., update_pose=
            # False), so self.pose stays this same object and `steps` keeps
            # playing. A stand or a pose transform stores a new tuple; that
            # object arrives here and we break out to bake again.
            # phase_index is still wherever the old table had reached.
            if cur_pose is not org_pose:
                break

