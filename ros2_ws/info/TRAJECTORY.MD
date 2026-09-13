# Trajectory generation on Masha — Chapter 9 vs this robot

Reading note. Maps Lynch & Park, *Modern Robotics*, Chapter 9 (Trajectory Generation) onto **this** codebase: Hiwonder ROSpider / Jetson Orin NX, ROS 2 Humble, workspace `~/ros2_ws`.

Gait encoding, leg numbers, halt codes, and unit traps live in [`MASHA_GAITS.md`](MASHA_GAITS.md). This file is the Chapter 9 layer: path vs time scaling vs trajectory, joint-space vs task-space, and what polynomial (cubic / quintic / S-curve) is **not** here.

Nothing here is an implementation task.

Sampled on this Jetson against `kinematics.so`. Formulas below are measured, not guessed from comments.

---

## The difference, in prose

The book and Masha are answering different questions.

The book asks: how do you go from here to there so the metal barely notices the start and the stop? It splits the problem in two. First you draw a curve — only *where*. Then you invent a clock that walks along that curve — *when*. Almost all of Chapter 9’s cleverness is in the clock. Leave slowly. Arrive slowly. Do not crack a gearbox with a shove.

Masha mostly does not own that clock.

When she **walks**, she does have a “where.” Each foot draws an arch in the air, plants, then drags back under the body. That arch is a picture in millimetres, redrawn many times a second, then turned into joint angles. That part would make the book nod. The clock that walks along the arch, though, is a metronome. Every twenty milliseconds she takes the next equally spaced bead on the string. She does not ease in or ease out. The foot speeds up and slows down because of the *shape* of the arch, not because someone wrote a gentle law for time. At the moment the foot should kiss the floor, it still has speed straight down. It arrives like a small stomp, not like a cat.

When she is given a **speed command**, the picture changes: the foot flies a straight line between two extreme points and pops up on a hump, like a thrown stone. Still a picture in space. Still a metronome.

When she **dances**, there is no picture in space at all. Someone once posed her with sliders and saved a row of servo ticks plus “take this many milliseconds.” Playback is a cassette: dump the next pose at the board, wait, dump the next. The board crawls each servo from here to there at constant tick speed. Place is continuous. Speed is not. Every keyframe is a shove. The book invented cubics and quintics exactly so you would not do this. Masha does this because that is how the action editor works, and because the servo protocol is “be at this tick in this much time.”

**Leaning** the body is a third thing. All six feet stay down and the torso slides and tilts over them. The code eases that by adding a little to the three translations and a little to roll, pitch, and yaw on each tick — six knobs, turned in lockstep. The book would rather spin the whole pose around one screw in space, a true rigid twist. Masha turns the knobs. For small leans it looks fine. It is not the same idea.

So the real difference is not that she “lacks a formula.” The book is one pipeline for an arm. Masha is three appliances that share the last wire to the motors.

- Walk thinks in feet.
- Lean thinks in a tilted body over planted feet.
- Dance thinks in recorded servo ticks.

All three finish as “go to these pulses in this much time.” None of them owns a clock that starts and stops softly. The ROS action that *looks* like a textbook trajectory (`FollowJointTrajectory`) throws away the velocity and acceleration the book would have sent. It is the costume of Chapter 9 on the body of the same duration field.

If you later write C++ from the book, the missing piece is that clock — a gentle time law inside the twenty-millisecond loop — not a new kind of foot shape, and not a rewrite of the dance files. Those files will never be Chapter 9 trajectories until duration stops being the whole timing law.

---

Notation in this file is plain Markdown (backticks + `text` fences), not LaTeX. Book names:

| Here | Book | Meaning |
|---|---|---|
| `theta(s)` | joint-space path | Geometry of joint angles vs a parameter `s` in `[0, 1]`. No clock. |
| `X(s)` | task-space path | Geometry of a pose / foot tip vs `s`. No clock. |
| `s(t)` | time scaling | The clock: time `t` in `[0, T]` maps onto `s`. Cubic / quintic live here. |
| `theta(s(t))` | trajectory | Path composed with a clock. |
| `s-dot`, `s-dot-dot` | ds/dt, d²s/dt² | First / second time derivatives of the scaling. |

---

## 0. Chapter 9 vocabulary (keep these three words apart)

| Term | Book | On Masha |
|---|---|---|
| **Path** `theta(s)` or `X(s)` | Pure geometry. `s` in `[0, 1]`. No clock. | Exists only in the gait / body-pose stack: six foot tips in millimetres, body frame. |
| **Time scaling** `s(t)` | Monotonic map from time `t` in `[0, T]` onto `s` in `[0, 1]`. This is where cubic / quintic / S-curve live. | **Linear** in the 20 ms tick. No `s-dot(0) = s-dot(T) = 0`. No quintic. |
| **Trajectory** `theta(s(t))` | Path composed with a clock. | IK of that path every tick, then pulse targets to the bus. |

Spatial path types in the book:

| Book path | On Masha |
|---|---|
| **Joint-space straight line** | Action groups, and the servo `duration` field. Pulse `0..1000`, not radians. |
| **Task-space straight line / screw `X(s)` in `SE(3)`** | Body pose uses **linear Euler + translation**, then IK of planted feet. Not matrix exponential. |

Time-scaling profiles in the book — **none of them run on the walk or the dance**:

| Profile | Book intent | On Masha |
|---|---|---|
| Linear `s = t / T` | Constant parameter speed; acceleration steps | **This is what she does** |
| Cubic (3rd) | `s-dot = 0` at start and goal; jerk spikes | Not present |
| Quintic (5th) | Also `s-dot-dot = 0` at ends; finite jerk | Not present |
| Septic / S-curve | Also zero jerk at ends; less gear chatter | Not present |
| Via-point polynomials | Smooth across keyframes | Action groups are via-points **without** the polynomial |

---

## 1. Three motion machines (same split as `MASHA_GAITS.md`)

Masha has **three** motion machines. Only the first two have a Chapter 9 *path*. The third already glues geometry to time in each file row.

| Machine | Chapter 9 object | Typical trigger | Feet |
|---|---|---|---|
| **Gait** | Task-space foot path `X(s)` + linear `s(t)` + IK | `/controller/traveling`, `/controller/cmd_vel` | Some swing, some stance |
| **Body pose** | Task-space body motion; Euler-linear, not a screw | `/controller/pose_transform_euler`, `/controller/set_pose_euler`, built-in poses | All six planted |
| **Action group** | Joint-space via-points in pulse units | `/controller/run_actionset` or `ActionGroupController.run_action` | Whatever the `.d6a` recorded |

Voice “go forward” is a **gait**. Voice “dance” is **`twist.d6a`**. They never share a generator. Both end on the same topic `/servo_controller`, so a gait that is still ticking will fight a dance. Halt is `traveling(gait=-2)` **before** `run_action`. See `MASHA_GAITS.md` §1 and §10.

---

## 2. Action sets (keyframe dance)

**Not interpolation in Python. Not IK. Not a gait.**

A `.d6a` is SQLite. Path: `/home/ubuntu/software/actionset_editor/ActionGroups/<name>.d6a`.

Table `ActionGroup`:

```text
Index INTEGER PK, Time INT (milliseconds), Servo1 … Servo24 INT (pulse 0–1000)
```

Example: `init.d6a` is one row, `Time=500`. `twist.d6a` is 25 rows; times in that file are 100 / 500 / 700 / 1000 ms.

Player: `src/driver/servo_controller/servo_controller/action_group_controller.py`.

```text
for each row:
    publish ServosPosition { position_unit='pulse', duration=Time/1000, position=Servo1…Servo24 }
    sleep(Time/1000)
```

That is Chapter 9 **via-point joint-space**: send target pulses plus a time, sleep that time, send the next row.

### What `duration` actually does

The Jetson does **not** spline the keyframes. It publishes `duration` on `/servo_controller`.

1. `ServoManager.set_position` (`servo_controller/servo_controller.py`) clamps duration to `[0.02, 30]` s and pulse to `[0, 1000]`.
2. `ros_robot_controller` packs STM32 bus-servo command **`0x01`** (Hiwonder / LewanSoul *move-time write*): duration in milliseconds + `(id, pulse)` pairs. See `ros_robot_controller_sdk.py` `bus_servo_set_position`.
3. The servo (or the STM32 forwarding that command) interpolates **pulse vs time** from the current tick to the new target. Protocol is the linear move-time command, not a cubic.

So the path is a polyline in 24-D pulse space (18 leg servos + 6 arm). Time scaling on each segment is **linear** `s(t) = t / T_i`. At a keyframe, velocity jumps (infinite jerk). That is the “simple, mechanical shock” case in the book, not quintic.

Boot stand is the same machine: bringup plays action group `init` (`controller/launch/init_pose.launch.py`, `action_name:=init`).

### Two different “dances”

| Name | Machine |
|---|---|
| Voice “dance”, most `.d6a` files (`twist`, `wave`, `clap`, pick groups, …) | Action group player above |
| `/controller/run_actionset` with `dance_1` / `dance_2` / `dance_3` | Forwarded to `/perform_actions/actions`. **Not** a `.d6a`. Scripted mix of body-pose IK, traveling gaits, and arm IK in `app/perform_actions.py` |

`perform_actions` can construct its **own** `StepController` if that node is launched. That is a classroom / demo process, not bringup.

---

## 3. Gait (locomotion engine)

**Task-space 3D foot curves, IK every 20 ms. Not a joint-space table.**

Python never stores coxa / femur / tibia trajectories. It stores six Cartesian foot tips, asks Rust for the next tips, then:

```text
kinematics.set_leg_position(leg_id, (x, y, z))  →  3 joint radians
JointControl  radians → pulses   (center 500, 1000 ticks over 240°)
publish ServosPosition  duration=0.02
```

Closed-source core: `src/driver/kinematics/kinematics/kinematics.so` (Rust `src/lib.rs`, PyO3). Python API:

| Function | Role |
|---|---|
| `set_leg_position(leg, p)` | One-leg IK. `p` in mm, body frame |
| `set_step_mode(subdivision, step, leg_positions, gait, amplitude, height, direction, rotation)` | Traveling gait samples |
| `cmd_vel_basic_data` / `cmd_vel_aep_pep` / `cmd_vel_new_point` | `cmd_vel` AEP/PEP + phase |
| `transform_pose(leg, current_pos, translate, quaternions)` | Rigid body motion of one planted foot |

There is **no** precomputed joint-space gait table on the Jetson.

### 3.1 `MovingGenerator` — `/controller/traveling`

File: `src/driver/controller/controller/move.py`.

One step (`Traveling.time` seconds, called `period` in the generator) is split into **six equal parts**, each into

```text
sub_action_num = ceil(period / 6 / 0.02)
```

slices so the 50 Hz loop has a pose every tick. Each sixth calls:

```text
ps = kinematics.set_step_mode(
        sub_action_num, i, start_pose,
        params.gait,          # 1 ripple / 2 tripod  (NOT inverted here)
        real_stride, height, params.direction, real_rotate)
```

The `.so` returns samples per leg; Python reshapes to `[slice][leg][xyz]`.

**Measured swing** (RMSE 0 against this formula, including cases with `height != stride/2`):

```text
x(s) = x_mid  - (stride / 2) * cos(pi * s)
z(s) = z_gnd  +  height      * sin(pi * s)
s in [0, 1]
```

That is a **half-ellipse in the sagittal plane**, uniform in the angle parameter.

| It is not | Why we know |
|---|---|
| Bézier | Closed form matches ellipse with RMSE 0 |
| Cycloid | Cycloid has `z ~ 1 - cos(2 * pi * s)` and `z-dot = 0` at the ends. This sine-`z` does **not** |
| Quintic `s(t)` | `s = k / N`, equal time per sample |

Consequences vs the book:

- Vertical speed is **max** at lift-off / touchdown (`z-dot` goes with `cos(pi * s)`, nonzero at `s = 0` and `s = 1`).
- Horizontal swing speed **is** zero at the ends (`x-dot` goes with `sin(pi * s)`).
- Stance is retraction at `z = z_gnd`. Approximately linear in `x`, not a perfect line (small residual).

`gait=1` ripple, `gait=2` tripod. `3` / `4` exist in the `.so` (“quadruped, front feet up” / “middle feet up”) but ROS traveling does not use them.

Ripple-only preamble (`gait==1`): if `direction >= pi`, lift leg 1 (LF) by `height` before the cycle; otherwise lift leg 4 (RR). Comment: a ripple foot may be mid-descend; starting from a full stand would jack the body up. Tripod skips this.

Time scaling: **linear** `s = k / N`. No trapezoid, no quintic.

### 3.2 `CmdVelGenerator` — `/controller/cmd_vel`

Same file, different algorithm: AEP / PEP (anterior / posterior extreme positions).

1. `kinematics.cmd_vel_basic_data(vx_mm, vy_mm, wz, period)` → offsets and `sin` / `cos` of half the yaw increment.
2. `kinematics.cmd_vel_aep_pep(cur_pose, …)` → per-leg AEP and PEP.
3. For phase `phi` in `[0, 2*pi)`, sampled at `period / 0.02` points: `kinematics.cmd_vel_new_point(gait, height, phase, aep, pep)`.

**Measured swing:**

- **XY: exactly linear** PEP → AEP (RMSE 0).
- **Z: parabola** `z = z_gnd + 4 * s * (1 - s) * h` (RMSE ~0.02 mm on a 20 mm lift).

Stance is linear AEP → PEP at ground.

Duty from a 50-sample sweep of the `.so` (lift threshold `z > -69 mm`):

| `.so` `gait` | Swing fraction | Meaning |
|---|---|---|
| 1 | ~0.48 | Tripod (≈ 50 %) |
| 2 | ~0.32 | Ripple |
| 3 | ~0.15 | Shorter swing |
| 4 | panics | `index out of bounds` in `src/lib.rs` — do not call |

Python **inverts** the number before the `.so` call:

```python
gait = 2 if params.gait == 1 else 1
```

So Traveling `gait=2` is tripod, but `cmd_vel` with `cmd_gait=2` is **ripple** inside Rust. Joystick analog vs D-pad hits this trap. Details: `MASHA_GAITS.md` §8.

Time scaling: uniform phase, i.e. linear `s(t)` on the gait oscillator. After a new Twist the generator picks a phase close to the current feet (`total_dist_sq`) so a foot does not teleport. Catch-up can stretch one slice to 50 ms (`slow == 'cmd_true'`).

Example: `cmd_vel_basic_data(80 mm/s, 0, 0, period=1)` returns AEP offset 40 mm, so AEP − PEP = 80 mm = `vx * T`. That is the book’s “path length from a speed and a period,” not a polynomial `s(t)`.

### 3.3 The 20 ms loop is the clock

`StepController.loop()` (`step_controller.py`) is a daemon thread, niceness −19, `sleep(0.02)` → 50 Hz. Cooperative Python generators.

Each tick, in this order:

1. If `new_pose_setter`: snap / one-shot built-in pose; clear all generators.
2. Else if `pose_transformer`: one slice of body roll / pitch / translate (feet stay conceptually planted).
3. Else if `moving_generator`: one slice of a gait (foot tips move).
4. `set_pose_base(pose, duration=0.02)` → IK → `/servo_controller`.
5. If this was a `cmd_vel` slice, integrate a crude odom (`position`, `pose_yaw`).

A started step is finished before the generator is swapped (`last_part`). Halt can take up to one step period. That is why a dance must wait after `gait=-2`.

When the loop sends `duration=0.02` every 20 ms, the servo barely interpolates: it is almost a new target every tick. Path smoothness is entirely from the 50 Hz IK samples, not from cubic `s(t)`.

---

## 4. Body pose (planted-feet rigid motion)

Not a gait. All six feet stay down; the body moves over them.

`PoseTransformer` (`controller/pose_transformer.py`) slices `duration` into `ceil(T / 0.02)` ticks and **adds** `dx, dy, dz` and `d(RPY)` linearly. The comment in code is explicit: translation and Euler are linear. Then:

```text
kinematics_calculate.transform_euler(org_pose, (nx,ny,nz), 'xyz', (nu,nv,nw))
    → kinematics.transform_pose(leg, foot, translate, quat)   for leg 1..6
```

That is task-space body motion, **not** a screw interpolation on `SE(3)`. Linear Euler is not a geodesic on `SO(3)`. Small angles, so it is approximately OK; it is not Chapter 9’s matrix exponential.

Built-in snapshots (`build_in_pose.py`: `DEFAULT_POSE`, `SLAM_POSE`, …) are six foot-tip coordinates in millimetres. Halt `gait=-2` goes to `DEFAULT_POSE` as a pose-setter with the Traveling `time` as servo duration (one shot, linear pulse interpolation), not as a gait.

Classroom `body_wave.py` / `posture_adjustment.py` use this machine. `body_ik.py` moves **one** foot in task space (`set_leg_absolute`).

---

## 5. Control cascade (exact order)

Bringup (`bringup.launch.py` → `controller.launch.py`) starts **one process, two nodes**:

```text
process  move_controller
  ├─ node /controller         MoveController     public API
  └─ node /step_controller    StepController     20 ms loop + IK
```

`MoveController.__init__` constructs a `StepController` and adds both to a `MultiThreadedExecutor`.

### 5.1 Walk / body pose (IK path)

```text
app / voice / joystick / Nav2
    → /controller/traveling              kinematics_msgs/Traveling
    → /controller/cmd_vel                geometry_msgs/Twist
    → /controller/pose_transform_euler   or  set_pose_euler
    → /controller/set_leg_absolute | set_leg_relatively
        → node /controller                 MoveController
            → node /step_controller        StepController  (20 ms)
                → MovingGenerator | CmdVelGenerator | PoseTransformer
                    → kinematics.so
                       set_step_mode / cmd_vel_new_point / transform_pose
                       → six foot tips (mm)
                    → set_pose_base
                       6 × set_leg_position  → 18 joint radians
                    → JointControl  radians → pulses
                        → /servo_controller     ServosPosition  unit=pulse
                            → /controller_manager
                                → /ros_robot_controller/bus_servo/set_position
                                    → STM32 /dev/ttyACM0   cmd 0x01
                                        → bus servos 1–18
```

TF (`end_LF`, …) is a **side effect** of `/controller_manager` publishing `/joint_states` into `robot_state_publisher`. The gait loop does not read TF. It keeps `StepController.pose` in millimetres.

### 5.2 Action group (skips IK)

```text
/controller/run_actionset
    → ActionGroupController.run_action(<name>.d6a)
        → /servo_controller                 (same topic as the gait)
            → controller_manager → STM32 → servos 1–24 (legs + arm)
```

`run_actionset` special cases: `stop` halts the group then plays `init_pose`; `dance_1/2/3` go to `perform_actions` if that node is up; anything else starts a thread on the basename if the previous group finished (`/action_complete`).

### 5.3 Side doors (not the walk cascade)

| Path | Role vs Chapter 9 |
|---|---|
| `/leg_controller/follow_joint_trajectory` (also arm, gripper) | MoveIt-style action. Sends each waypoint’s **positions** with segment duration. **Ignores** velocities and accelerations in the `JointTrajectory` message. Same linear pulse move as a `.d6a`. Hosted by `controller_manager.py`. |
| `/arm_kinematics/set_pose_target` | 5-DOF arm IK, separate package. Pick/place uses this, not hexapod `kinematics.so`. Some demos `np.linspace` a Cartesian path and send pulses with equal `step_duration` — linear `s(t)` in task space for the arm. |
| `perform_actions` | Scripted mix of pose-transform, traveling, and arm IK. |

Halt codes (not a time-scaling): `gait=0` finish current step then freeze; `gait=-1` `SLAM_POSE`; `gait=-2` `DEFAULT_POSE` (what voice stop uses). A zero Twist is **not** that halt — it starts a `CmdVelGenerator` with `v = 0`.

---

## 6. File map (trajectory-specific)

| Path | Role |
|---|---|
| `driver/controller/controller/move_controller.py` | ROS façade: Traveling / cmd_vel / pose / action set |
| `driver/controller/controller/step_controller.py` | 20 ms loop, generators, IK dispatch |
| `driver/controller/controller/move.py` | `MovingGenerator`, `CmdVelGenerator` |
| `driver/controller/controller/pose_transformer.py` | Linear Euler + translation slices |
| `driver/controller/controller/build_in_pose.py` | Named six-foot snapshots (mm) |
| `driver/kinematics/kinematics/kinematics.so` | Rust IK + ellipse/parabola samples + AEP/PEP |
| `driver/kinematics/kinematics/kinematics_calculate.py` | `transform_euler` / `transform_quat` wrappers |
| `driver/kinematics/kinematics/x_joint_control.py` | Radians → pulses, publish `/servo_controller` |
| `driver/servo_controller/…/action_group_controller.py` | `.d6a` player |
| `driver/servo_controller/…/joint_trajectory_action_controller.py` | FollowJointTrajectory; positions only |
| `driver/ros_robot_controller/…/ros_robot_controller_sdk.py` | STM32 `0x01` move-time write |
| `app/app/perform_actions.py` | `dance_1/2/3` choreography (not `.d6a`) |
| `software/actionset_editor/ActionGroups/*.d6a` | Keyframe files |

Related: [`MASHA_GAITS.md`](MASHA_GAITS.md) (who walks how), [`nodes-topics.md`](nodes-topics.md) (live graph).

---

## 7. How to read the book against this robot

1. **Path vs trajectory.** A `.d6a` is already a trajectory in joint pulse space (geometry and timing glued in each row). A gait **does** split them: Cartesian foot path in the `.so`, time = uniform 20 ms samples.
2. **Joint-space vs task-space.** Dance = joint-space pulses. Walk = task-space feet + IK. Body tilt = task-space body, Euler-linear, planted feet.
3. **Why cubics exist in the book.** Masha’s action-group and servo `duration` are the thing those polynomials are meant to replace: constant-velocity segments, acceleration steps, jerk spikes. The gait ellipse / parabola is a **spatial** smoother, not a **time-scaling** smoother. Touchdown `z-dot != 0` on both generators.
4. **50 Hz is the clock.** A quintic `s(t)` would live *inside* that loop as a scalar on `s`, then the same IK. It is not there today.
5. **Do not mix machines.** Putting a quintic on `cmd_vel` does not smooth a `.d6a`. Playing a dance does not give you a support polygon.

---

## 8. Direct answers (the three questions)

**Action sets.** Pre-programmed keyframes are `.d6a` rows of 24 pulses plus a millisecond duration. Python does not interpolate (linear, cubic, or quintic) between poses. It sends raw target pulses to the servo board with that duration and sleeps. The board/servo linearly interpolates pulse vs time (command `0x01`). Boot `init` is the same player.

**Gait.** The leg controller generates **task-space** 3D foot curves and solves IK at every 20 ms tick. Not a joint-space table. Traveling swing is a **half-ellipse** (uniform angle). `cmd_vel` swing is **linear XY + parabolic Z**. Time scaling in the 50 Hz loop is **linear** (equal `ds` or equal `d(phi)`), not trapezoidal, not quintic.

**Cascade.** High-level command → `MoveController` → `StepController` 20 ms loop → (optional) body-pose generator or foot-trajectory generator → `kinematics.so` IK → pulse conversion → `/servo_controller` → `controller_manager` → STM32 bus → servos. Action groups skip everything above `/servo_controller`.
