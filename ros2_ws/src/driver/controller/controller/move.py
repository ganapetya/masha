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

# Module-level handoff between consecutive CmdVelGenerator instances.
# When a generator is told status=='finish', it stores the last pose it
# yielded in finish_ps. The next generator uses that pose to pick a start
# phase close to where the previous cycle left the feet (avoids a jump).
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
# Follow gait (gait=5). Formulas: proud_up/include/proud_up/follow_gait.hpp
#
# This generator is a *client* of that map. It does not call
# kinematics.set_step_mode or cmd_vel_new_point. StepController still
# owns the 20 ms loop and the IK (set_leg_position).
#
# Same handshake as CmdVelGenerator so halt can finish a cycle and a new
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


def sample_follow_gait(phi, vx, vy, wz, lift, period, nominal, stride_max=40.0, linear_factor=1.0):
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
    """
    global finish_ps, slow, stop, finish_index
    height = params.height
    org_pose = None
    phase_index = 0
    phase_num = math.ceil(round((params.period * 1000.0 / 20.0), 1))
    phase_num = max(int(phase_num), 2)
    phase_list = [(i / phase_num) * 2.0 * math.pi for i in range(phase_num)]
    stride_max = 40.0
    linear_factor = getattr(params, 'linear_factor', 1.0)

    if log is not None:
        log.info(
            'FollowGaitGenerator start (gait=5, never set_step_mode). '
            f'vx={params.velocity_x:.1f} mm/s vy={params.velocity_y:.1f} '
            f'wz={params.angular_z:.3f} T={params.period:.2f}s h={height:.1f} mm'
        )

    cur_pose, status = yield None
    while True:
        org_pose = cur_pose
        nominal = [(float(p[0]), float(p[1]), float(p[2])) for p in cur_pose]
        if params.relative_h:
            height = abs(nominal[0][2]) * (params.height / 100.0)
        steps = []
        for phase in phase_list:
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

        if finish_ps:
            dists = [total_dist_sq(finish_ps, bi) for bi in steps]
            idx = min(range(len(dists)), key=lambda i: dists[i])
            if idx == 0:
                idx = 1
                stop = True
            else:
                stop = False
        else:
            idx = min(range(len(steps) // 2), key=lambda i: abs(steps[i][1][0]))
            if idx == 0:
                idx = 1
                stop = True
            else:
                stop = False

        steps1 = steps[idx:]
        steps2 = steps[:idx]

        while True:
            if status == 'first':
                ps = steps1[phase_index]
                if finish_ps:
                    distances = max([math.dist(p1, p2) for p1, p2 in zip(ps, finish_ps)])
                    if distances > 7 or stop:
                        slow = 'cmd_true'
                    finish_ps = []
                else:
                    slow = 'cmd_false'
                phase_index = (phase_index + 1) % (len(steps) - idx)
            elif status == 'finish':
                ps = steps2[phase_index]
                phase_index = (phase_index + 1) % max(idx, 1)
                finish_ps = ps
                finish_index = phase_index
            else:
                ps = steps[phase_index]
                phase_index = (phase_index + 1) % phase_num
            if phase_index == 0:
                cur_pose, status = yield ps, True, params, slow
            else:
                cur_pose, status = yield ps, False, params, slow
            if cur_pose is not org_pose:
                break

