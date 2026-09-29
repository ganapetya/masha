import sys
import math
import time
import numpy as np
from kinematics import kinematics
import copy

# Gait generators for the hexapod.
# Both MovingGenerator and CmdVelGenerator are Python generators:
#   1) caller does gen.send(None) once to prime them
#   2) then repeatedly send((cur_pose, status)) and receive
#      (out_pose, last_part, params, slow)
# last_part is True when a full step/cycle just wrapped, so the outer
# loop in step_controller can swap or stop generators on a step boundary.
# slow tells the servo loop which timing to use ('move' / 'cmd_true' / 'cmd_false').


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
    """Yield discrete-step gait poses for Traveling / set_step_mode.

    One step is 6 kinematic parts (i = 0..5). Each part is further split into
    sub_action_num servo frames so the whole step lasts params.period at 50 Hz.
    """

    # 根据是相对高度还是绝对高度计算实际的高度(actual lift height is computed later from relative_h vs absolute)
    slow = 'move'  # tag so step_controller treats this as a discrete-step generator, not cmd_vel
    org_pose = None
    part_index = 0
    sub_index = 0

    # 50 Hz loop (0.02 s). One step = 6 parts, so frames-per-part = period / 6 / 0.02
    sub_action_num = params.period / 6.0 / 0.02 # 一步会分为六步幅， 计算每一部分有多少个子动作(one step is six parts; how many sub-actions each part has)
    sub_action_num = math.ceil(round(max(sub_action_num, 1), 3)) # 每一分步最少要有一个子动作(each part must have at least one sub-action)
    height = params.height
    real_stride = params.stride * params.linear_factor
    real_rotate = params.rotation * params.rotate_factor

    # Per-frame increments. kinematics.set_step_mode still gets the full-step
    # real_stride / real_rotate; these locals are unused leftovers.
    sub_stride = params.stride / (sub_action_num * 6.0)
    sub_rotate = params.rotation / (sub_action_num * 6.0)

    cur_pose, status = yield None  # prime: wait for the first (pose, status) from the loop thread
    while params.repeat > 0 or params.forever:
        org_pose = cur_pose
        if params.relative_h:
            # percent of current foot z (stance height), not an absolute mm lift
            height = abs(org_pose[0][2]) * (params.height / 100.0) 
        poses = [] # 计算到的步态的所有姿态(all gait poses for this one step, 6 parts)
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
        
        # 计算整个过程的所有步态过程(precompute all 6 parts of this step)
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
            # 出来的数据是每条对的多个姿态，转换为六条腿的子动作的姿态(kinematics returns per-leg sequences; reshape into per-frame snapshots of all six legs)
            ps = ps.reshape((6, -1, 3))  # 转换为[六条腿[子动作[坐标]]](shape: [6 legs][sub-actions][x,y,z])
            ps = np.transpose(ps, (1, 0, 2))  # 转换为[子动作[六条腿[坐标]]](shape: [sub-actions][6 legs][x,y,z])
            start_pose = ps[-1]  # next part continues from this part's last frame
            poses.append(ps)
        # log.info(f'{status}, {poses}')    
        while params.repeat > 0 or params.forever:
            out_pose = poses[part_index][sub_index]
            # log.info('forvoer'+str(params.forever)) 

            # 各个计数进行调整(advance the two nested counters)
            sub_index = (sub_index + 1) % sub_action_num # 子动作计数 +1(next sub-action in this part)
            if sub_index == 0:
                part_index = (part_index + 1) % 6 # 分步计数 +1(next of the 6 parts)
                if part_index == 0:
                    # wrapped a full step; forever keeps repeat at 0 so the outer while stays true
                    params.repeat = max(params.repeat - 1, 0)
            # log.info(f'{part_index} {sub_index}')
            # last_part is True only at the wrap (part 0, sub 0): a complete-step boundary
            cur_pose, status = yield out_pose, part_index == 0 and sub_index == 0, params, slow

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
# On status=='finish' the live generator stores the pose it is about to
# yield into finish_ps, every finish tick, so the last one is the pose the
# feet were given when that slice wrapped. The next generator searches its
# new cycle for the frame nearest that pose and starts there (avoids a jump).
# slow is the servo-timing tag returned with each pose ('cmd_true' asks for
# a 50 ms blend, 'cmd_false' keeps the usual 20 ms command).
# stop means "we refused to splice at frame 0". finish_index is the cursor
# after a finish yield; no other function reads it.
finish_ps = []
slow = 'cmd_false'
stop = False   
finish_index = None

def CmdVelGenerator(params, log=None):
    """Yield continuous gait poses for Twist / cmd_vel.

    Builds one gait-cycle table (phase 0 .. 2pi), then plays it. The outer
    loop in step_controller drives a small state machine via `status`:
      'first'  — resume from a splice index (after start or after a switch)
      'finish' — play the leftover prefix so the cycle can stop on a stable pose
      other    — normal wrap around the full cycle
    """
    global finish_ps, slow,  stop, finish_index
    height = params.height
    # kinematics.cmd_vel_new_point uses the opposite numbering from MovingParams:
    # here 1 = tripod, 2 = ripple. Incoming params.gait is still 1=ripple, 2=tripod.
    gait = 2 if params.gait == 1 else 1

    org_pose = None
    phase_index = 0 # 当前相位游标(index into the current phase slice being played)
    # one frame every 20 ms; round then ceil so 1.0 s → 50 phases
    phase_num = math.ceil(round((params.period * 1000.0 / 20.0), 1)) # 细分的相位个数(how many discrete phases in one cycle)
    phase_list = [(i / phase_num) * 2.0 * math.pi for i in range(phase_num)] # 细分相位列表(phase samples covering 0 .. 2pi)

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

    cur_pose, status = yield None  # prime
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
                # phase 0 is the cycle wrap; skip it so the first yield is a real step
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
 
        # Split the cycle at idx:
        #   steps1 = idx .. end  — played while status == 'first'  (resume / start)
        #   steps2 = 0 .. idx    — played while status == 'finish' (wind down)
        steps1 = steps[idx:]
        steps2 = steps[:idx]   
                    
        
        while True:
            """
            ps = kinematics.cmd_vel_new_point(
                params.gait,
                height,
                phase_list[phase_index],
                aep,
                pep
            )
            """
            if status == 'first':
                ps = steps1[phase_index]
                if finish_ps:
                    # largest per-foot jump from the previous generator's last pose
                    distances = max([math.dist(p1, p2) for p1, p2 in zip(ps, finish_ps)])
                    if distances > 7 or stop:
                        slow = 'cmd_true'  # ask the servo loop for a slower 50 ms blend
                    finish_ps = []
                else:
                    slow = 'cmd_false'    
                phase_index = (phase_index + 1) % (len(steps) - idx)
            elif status == 'finish':
                # play the leftover prefix so the gait can halt near a stable pose
                ps = steps2[phase_index]
                phase_index = (phase_index + 1) % (idx)
                finish_ps = ps  # hand this pose to the next CmdVelGenerator
                finish_index = phase_index
            else:
                # normal running: wrap the full cycle
                ps = steps[phase_index]
                phase_index = (phase_index + 1) % phase_num
            if phase_index == 0:
                # wrapped this slice — last_part True so the outer loop may switch generators
                cur_pose, status = yield ps, True, params, slow
            else:
                cur_pose, status = yield ps, False, params, slow
            # 两次初始姿态变了我们需要重算(stance pose object changed: recompute AEP/PEP and the cycle table)
            if cur_pose is not org_pose:
                break


# ---------------------------------------------------------------------------
# Follow gait (gait=5). The foot curves match
# proud_up/include/proud_up/follow_gait.hpp, retyped in sample_follow_gait
# below. This file does not include that header and does not call it.
#
# FollowGaitGenerator does not call kinematics.set_step_mode or
# cmd_vel_new_point. StepController still owns the 20 ms loop and the IK
# (set_leg_position).
#
# Same handshake as CmdVelGenerator so a halt can finish a cycle and a new
# Twist can splice on last_part without teleporting the feet.
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

    Never calls kinematics.set_step_mode. IK stays in StepController.

    Python generator: `yield` freezes this function and hands one value to
    StepController. The controller's 20 ms loop is the projector. This
    function is the film. Calling FollowGaitGenerator(...) only builds the
    generator object; the body runs on send().

    Handshake, same words as CmdVelGenerator:
      send(None) once     — prime, runs up to the first yield
      status 'first'      — play the intro slice (steps1), startup or swap
      status 'running'    — play the full baked cycle
      status 'finish'     — play the outro slice (steps2), stash finish_ps
    The bool beside each pose is last_part: True means this slice just
    wrapped, so the controller may wind down or swap generators.
    A new Twist does not edit this table. It builds a new generator and
    waits for last_part.
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

    # How many 20 ms frames fit in one period. round-to-1-decimal, then
    # ceil, so binary float noise (30.0000001) does not become an extra
    # frame. 1.00 s → 50 frames. The hunter's 0.60 s → 30 frames.
    phase_num = math.ceil(round((params.period * 1000.0 / 20.0), 1))
    # Two frames minimum. A 1-frame cycle wraps on every tick (`% 1`
    # is always 0), and the controller would treat every tick as a boundary.
    phase_num = max(int(phase_num), 2)

    # The gait clock is an angle. A for-loop cannot walk "0 to 2π", so
    # each frame gets one sample: frame i is at fraction i/phase_num of
    # the circle. The list starts at 0 and stops just before 2π. The next
    # lap is frame 0 again. That seam is why index 0 is special below.
    phase_list = [(i / phase_num) * 2.0 * math.pi for i in range(phase_num)]

    # Offset cap (|AEP − home| and |PEP − home|), not the full PEP→AEP
    # travel. 55 mm → ~75 mm step at the hunter's vx 0.25 m/s and T 0.60 s.
    # 40 mm was the short shuffle. sample_follow_gait clamps each foot so
    # a large command cannot park a toe past this radius from its home.
    stride_max = 55.0
    # Multiplies vx and vy inside _aep_pep. wz is left alone.
    # CmdVelParams always has this attribute; getattr keeps a stray params
    # object from crashing the bake. Missing → 1.0, meaning "use vx, vy as given".
    linear_factor = getattr(params, 'linear_factor', 1.0)

    # Once, before any foot pose. Later ticks do not log. The numbers are
    # the command this generator was built with; a new Twist is a new object.
    if log is not None:
        log.info(
            'FollowGaitGenerator start (gait=5, never set_step_mode). '
            f'vx={params.velocity_x:.1f} mm/s vy={params.velocity_y:.1f} '
            f'wz={params.angular_z:.3f} T={params.period:.2f}s h={height:.1f} mm '
            f'stride_cap={stride_max:.0f} mm'
        )

    # Prime. `cur_pose, status = yield None` both gives None to the caller
    # and, on the next send, receives whatever they pass. cmd_vel's first
    # send(None) only gets us here and discards the None. The loop thread's
    # first send((stance, 'first')) is what fills cur_pose and status.
    cur_pose, status = yield None

    # Outer loop, the animator. Bakes one full cycle into `steps`, then
    # the inner loop plays it. We come back here only when the controller
    # replaces self.pose with a different object (a stand, or a body move).
    # A normal 20 ms tick does not. A new velocity does not either: that
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

        # Bake every frame now. This for-loop does not yield. After it,
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

        # Splice, once per bake. Frame 0 puts one tripod at PEP and the
        # other at AEP, the two extremes. Playback has to be allowed to
        # start at a later frame that already looks like the current feet.
        if finish_ps:
            # Sum of squared foot-to-foot distances. Skipping the square
            # root is enough: we only need which frame is nearest.
            dists = [total_dist_sq(finish_ps, bi) for bi in steps]
            idx = min(range(len(dists)), key=lambda i: dists[i])
            if idx == 0:
                # Frame 0 would make the outro `steps[:0]` empty, and the
                # first 'finish' tick would index that empty list. Start
                # one frame later. stop remembers that we refused the seam;
                # the 'first' branch ORs it into the slow-blend test while
                # finish_ps is still set.
                idx = 1
                stop = True
            else:
                stop = False
        else:
            # No hand-off pose. That is a cold start, and also a rebake:
            # the 'first' tick already cleared finish_ps, so a later stance
            # change comes through here too. Leg 1 (the second tuple) stands
            # at x = 0 in DEFAULT_POSE, so "leg 1's x closest to 0" means
            # that foot is near its forward home. Search only the first half
            # of the cycle; the second half is the other tripod.
            idx = min(range(len(steps) // 2), key=lambda i: abs(steps[i][1][0]))
            if idx == 0:
                # Same empty-outro guard as the hand-off path. This branch
                # does not arm the slow blend: finish_ps is empty, so the
                # 'first' tick below takes the other arm and writes
                # slow = 'cmd_false'.
                idx = 1
                stop = True
            else:
                stop = False

        # Cut the film at the splice. Math frame 0 is the seam, not "where
        # the feet are now", so playback must be allowed to start mid-table.
        # steps1, the intro: splice → end of the table. status 'first'.
        # steps2, the outro: start of the table → splice. status 'finish'.
        # status 'running' ignores both and plays `steps` whole.
        steps1 = steps[idx:]
        steps2 = steps[:idx]

        # Inner loop, the projector. One yield per wake-up. At each yield
        # we freeze; ~20 ms later the controller send()s the stance it
        # still holds, plus the next status word.
        while True:
            if status == 'first':
                # Intro: from the splice toward the end of the math cycle.
                ps = steps1[phase_index]
                if finish_ps:
                    # Only the first 'first' tick still has the stash. The
                    # largest single-toe jump to this frame, in millimetres.
                    # Over 7 mm, or we skipped the seam (stop), asks
                    # StepController for a 50 ms command on this one frame
                    # (slow == 'cmd_true'). The thread still sleeps 20 ms;
                    # the next tick replaces that pulse.
                    distances = max([math.dist(p1, p2) for p1, p2 in zip(ps, finish_ps)])
                    if distances > 7 or stop:
                        slow = 'cmd_true'
                    # Drop the stash now, or every later 'first' tick would
                    # run this test again.
                    finish_ps = []
                else:
                    # Later 'first' ticks, and a cold start: ordinary timing.
                    slow = 'cmd_false'
                # Advance inside the intro. Its length is phase_num − idx,
                # which is len(steps1). Wrapping this modulo is what makes
                # phase_index == 0 at the end of the intro.
                phase_index = (phase_index + 1) % (len(steps) - idx)
            elif status == 'finish':
                # Outro: from the math seam back toward the splice, so a
                # stop can park on the way to a pose the next generator
                # knows how to meet.
                ps = steps2[phase_index]
                # max(idx, 1) keeps this modulo off zero if idx were ever
                # left at 0. The splice above already forces idx to 1.
                phase_index = (phase_index + 1) % max(idx, 1)
                # Overwrite the hand-off every finish tick. The write that
                # matters is the one on the wrapping yield: that pose is
                # what the next generator splices against.
                finish_ps = ps
                # Cursor after the increment. Stored, never read.
                finish_index = phase_index
            else:
                # 'running', and any word that is not 'first' or 'finish'.
                # Full cycle, seam included.
                ps = steps[phase_index]
                # 29 + 1 wraps to 0 when phase_num is 30. That 0 is the
                # signal below; the frame we just took was the last one.
                phase_index = (phase_index + 1) % phase_num

            # The frame was chosen first, then the cursor moved. Cursor 0
            # means the move wrapped, so `ps` is the last frame of this
            # slice. True: controller may enter 'finish' or swap in the
            # queued generator. False: mid-slice, keep playing.
            if phase_index == 0:
                cur_pose, status = yield ps, True, params, slow
            else:
                cur_pose, status = yield ps, False, params, slow

            # Wake-up, about 20 ms later. `is` checks the object, not the
            # millimetres. Gait ticks call set_pose_base(..., update_pose=
            # False), so self.pose stays this same object and `steps` keeps
            # playing. A stand or a pose transform stores a new tuple; that
            # object arrives here and we break out to bake again.
            # phase_index is still wherever the old film had reached.
            if cur_pose is not org_pose:
                break

