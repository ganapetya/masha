# How Masha walks — gaits as they are programmed

This is a reading note on **this robot**, not a textbook comparison of “tripod vs ripple.” Gemini’s Sprint 3 Day 1 table (legs 1,3,5 vs 2,4,6) is the right *idea*. Below is how that idea is actually encoded, published, and executed on Masha (Jetson Orin NX, ROS 2 Humble, package `controller`).

Read this, then ask questions. Nothing here is an implementation task.

Copies: `~/steps/MASHA_GAITS.md` (Sprint 3 reading) and `~/ros2_ws/info/MASHA_GAITS.md` (same text, next to `voice.md`).

---

## 1. Three different “motions” (do not mix them)

Masha has **three** motion machines. Only the first two are *gaits*.

| Machine | What it is | Typical trigger | Feet during the motion |
|---|---|---|---|
| **Gait** | Cyclic stepping. Inverse kinematics of six foot tips, 50 Hz. | `/controller/traveling` or `/controller/cmd_vel` | Some feet swing, some stance |
| **Body pose** | All six feet stay planted; the body translates/tilts over them | `/controller/pose_transform_euler`, `/controller/set_pose_euler`, built-in poses | All six on the ground |
| **Action group** | Open-loop pulse sequence from a `.d6a` SQLite file | `/controller/run_actionset` or `ActionGroupController.run_action` | Whatever the file recorded (dance, init, pick) |

Sprint 3 Step 1’s *dance* is an **action group**, not a gait. Sprint 3’s *safety halt* must stop the **gait** (`gait=-2`) before the dance starts. If a gait generator is still ticking, it will fight the `.d6a` for `/servo_controller`.

Sprint 3 Step 2 (after the dance finishes) **is** a gait: a new generator (`gait=5`) that we write, using existing IK. It must wait for `/action_complete` and must **not** call `kinematics.set_step_mode(..., 5)` — the `.so` does not know that number. See `~/steps/quarter-01-sprint-03-plan.md` Step 2.

Voice “go forward” uses a **gait**. Voice “dance” uses **`twist.d6a`**. They never share a generator.

---

## 2. Who is running

Boot (`bringup` → `controller.launch.py`) starts one process that is **two ROS nodes**:

```
process  move_controller
  ├─ node /controller         MoveController     public API
  └─ node /step_controller    StepController     20 ms loop + IK
```

`MoveController.__init__` constructs a `StepController` and adds **both** to a `MultiThreadedExecutor` (`move_controller.py` `main()`).

Downstream:

```
/step_controller
    JointControl.set_multi_joints()
        → topic  servo_controller   (servo_controller_msgs/ServosPosition, unit pulse)
            → node /controller_manager   (package servo_controller)
                → /ros_robot_controller/bus_servo/set_position
                    → STM32 on /dev/ttyACM0
                        → bus servos 1–18 (legs)
```

TF for the legs is a *side effect* of `/controller_manager` publishing `/joint_states` into `robot_state_publisher`. The gait loop itself does **not** look at TF. It keeps an in-memory pose: six foot-tip coordinates in millimetres.

---

## 3. Body frame and leg numbers

Kinematics (`kinematics.so`, Rust) documents:

> leg 1–6: head facing forward, **leg 1 is the upper-left foot**, then **counter-clockwise**.

Mapped onto names and the URDF (`coxa_*` / `femur_*` / `tibla_*` / `end_*`):

| `leg_id` (IK / `pose[]` index+1) | Name | SERVOS kinematic ids | Physical bus ids (coxa, femur, tibia) | DEFAULT_POSE (mm) ≈ |
|---|---|---|---|---|
| 1 | **LF** left front | 1, 2, 3 | 5, 3, 1 | (+164, +141, −70) |
| 2 | **LM** left middle | 4, 5, 6 | 11, 9, 7 | (0, +184, −70) |
| 3 | **LR** left rear | 7, 8, 9 | 17, 15, 13 | (−164, +141, −70) |
| 4 | **RR** right rear | 10, 11, 12 | 18, 16, 14 | (−164, −141, −70) |
| 5 | **RM** right middle | 13, 14, 15 | 12, 10, 8 | (0, −184, −70) |
| 6 | **RF** right front | 16, 17, 18 | 6, 4, 2 | (+164, −141, −70) |

Body frame used by that pose: **+X toward the head, +Y to the robot’s left, +Z up**. Foot \(z\) is **negative** (below the body). Height 70 mm means the body CoM is about that far above the feet; `DEFAULT_POSE_TRANSFORM` stores a related body offset `(0, 0, 130) mm`.

`config.py` has a comment diagram for `LEG_LIST` that **does not match** this table. Trust `SERVOS`, `set_pose_base`, and the `.so` docstring, not that comment.

Gemini’s tripod groups, with **these** numbers:

- Swing group A: **1, 3, 5 = LF, LR, RM**
- Swing group B: **2, 4, 6 = LM, RR, RF**

That is the classic hexapod tripod: each group is a triangle that still supports the body while the other three swing.

---

## 4. What a “pose” is

Everywhere in the gait stack, `pose` is:

```text
(
  (x1, y1, z1),  # leg 1 LF
  (x2, y2, z2),  # leg 2 LM
  ...
  (x6, y6, z6),  # leg 6 RF
)
```

Units: **millimetres**, body frame above. `build_in_pose.py` lists named snapshots:

| Name | Typical use |
|---|---|
| `DEFAULT_POSE` | Stand, height 70 mm. Halt `gait=-2` goes here |
| `DEFAULT_POSE_M` | Taller stand (95 mm). Some demos / SLAM joystick |
| `SLAM_POSE` | Even taller (100 mm). Halt `gait=-1` |
| `NARROW_POSE` | Feet pulled in |
| `HOLE_POSE` / `SHRINK_POSE` / `SIDE_SHIFT_POSE` | Special stances |
| `RELAX_POSE` | Feet at z = 0 (unloaded) |

IK: `kinematics.set_leg_position(leg_id, (x,y,z))` → three joint angles (radians). `StepController.set_pose_base` does that for all six legs, then `JointControl` converts radians → **pulses** (center 500, 1000 ticks over 240°) using `config.SERVOS` (direction, offset, physical id).

---

## 5. The 20 ms loop (the real executor)

`StepController.loop()` is a **daemon thread**, nice’d to −19, sleeping **0.02 s** (50 Hz). It is a cooperative state machine of **Python generators**:

```
each 20 ms:
  1. If new_pose_setter: snap/interpolate to a built-in pose; clear all generators
  2. Else if pose_transformer: one slice of body roll/pitch/translate (feet stay conceptually planted)
  3. Else if moving_generator: one slice of a gait (foot tips move)
  4. set_pose_base(that pose, duration=0.02) → IK → /servo_controller
  5. If this was a cmd_vel slice, integrate a crude odom (position, pose_yaw)
```

**Rule the comments insist on:** a gait step that has started is finished before a new generator is swapped in (`last_part` flag). That is why a halt can take up to one step period, and why Sprint 3 should wait a fraction of a second after `gait=-2` before playing a dance.

Generators are created on the ROS callback thread, primed with `.send(None)`, stored as `new_moving_generator` under `self.lock`. The loop thread consumes them. Same pattern for pose transformers.

---

## 6. Public API on `/controller`

`MoveController` (node name `controller`) remaps nothing: topics are `~/…` → `/controller/…`.

### 6.1 `kinematics_msgs/Traveling` on `/controller/traveling`

```
int8    gait
float32 stride          # mm
float32 height          # mm, toe lift
float32 direction       # see units trap below
float32 rotation
float32 time            # seconds per step  (called period in the generator)
uint32  steps           # 0 = forever
bool    relative_height
bool    interrupt
```

How `gait` is interpreted (`set_traveling_callback` + `set_step_mode`):

| `gait` | Meaning |
|---|---|
| **1** | Ripple. `MovingGenerator` → `kinematics.set_step_mode(..., gait=1, ...)` |
| **2** | Tripod. Same path, `gait=2` |
| **3, 4** | Exist in the `.so` (“quadruped, front feet up” / “middle feet up”). Not used by ROS traveling |
| **0** | Stop the generator, then `set_pose` with current feet (soft stop) |
| **−1** | `set_build_in_pose('SLAM_POSE')` |
| **−2** | `set_build_in_pose('DEFAULT_POSE')` and zero the pose-transform accumulators. **This is the halt voice uses.** |
| **11, 12, 13** | **Not a walk.** `cmd_gait = gait-10`, also stores `cmd_height` and `cmd_period` for the *next* `cmd_vel`. Joystick does `traveling(10 + self.gait, …)` then `cmd_vel`. |

`ControllerClient.traveling()` is the usual publisher. Examples (`tripod_gait.py`, `ripple_gait.py`, `left_and_right.py`, …) are thin wrappers around that client. They wait for `/controller_manager/init_finish` then send one Traveling and sleep.

### 6.2 `geometry_msgs/Twist` on `/controller/cmd_vel`

Clamped in `cmd_vel_callback`:

- `linear.x` ∈ [−0.12, +0.12] m/s
- `linear.y` ∈ [−0.10, +0.10] m/s
- `angular.z` ∈ [−0.6, +0.6] rad/s

Then `StepController.cmd_vel` converts linear speeds to **mm/s** and starts a `CmdVelGenerator`.

**Joystick analog stick and D-pad walk this path**, not Traveling 1/2. Voice currently publishes **both**: a Traveling *and* a Twist. The Twist starts a second generator that can replace the Traveling one after the current step. That is messy; treat “voice walk” as “Traveling gait=2, 6 steps” plus an extra cmd_vel that the 5 s watchdog later kills with `gait=-2`.

### 6.3 Other `/controller` topics (not gaits)

| Topic | Role |
|---|---|
| `pose_transform_euler` | Relative body RPY + translation, feet IK’d as a rigid stance |
| `set_pose_euler` | Absolute body pose |
| `set_leg_absolute` / `set_leg_relatively` | One foot Cartesian (`leg_id` 1–6, mm) |
| `run_actionset` | `.d6a` by basename |
| service `set_pose_1` | Built-in pose by name string |

`/step_controller/cmd_param` (`interfaces/CmdParam`: pose name, gait, height, period) is how the joystick sets stand height / which gait `cmd_vel` will use.

---

## 7. How a Traveling step is computed (`MovingGenerator`)

File: `controller/move.py`.

One “step” (`params.period` seconds, Traveling field `time`) is split into **six equal parts**, each part into `sub_action_num = ceil(period / 6 / 0.02)` slices so the 20 ms loop has something to send every tick.

For `i in 0..5`:

```text
ps = kinematics.set_step_mode(
        sub_action_num,   # how many IK samples in this sixth
        i,                # which sixth (0..5)
        start_pose,       # six foot tips
        params.gait,      # 1 ripple / 2 tripod  (NOT inverted here)
        real_stride,      # mm
        height,           # mm
        params.direction,
        real_rotate)
```

The `.so` returns samples per leg; Python reshapes to `[slice][leg][xyz]`.

Ripple-only preamble (`gait == 1`): if `direction >= π`, lift **leg 1 (LF)** by `height` before the cycle; otherwise lift **leg 4 (RR)**. Comment: in ripple a foot may be mid-descend; starting from a full stand would jack the body up. Tripod skips this.

`repeat` comes from Traveling `steps`. `steps==0` sets `forever=True`.

The closed-source core is **Rust** (`kinematics.so` was built from `src/lib.rs`). Python never sees which legs are in the air. You only see the resulting foot coordinates.

---

## 8. How `cmd_vel` is computed (`CmdVelGenerator`)

Same file. This is a **different** algorithm: AEP / PEP (anterior / posterior extreme positions), standard hexapod literature.

1. `kinematics.cmd_vel_basic_data(vx_mm, vy_mm, wz, period)` → offsets and `sin/cos` of half the yaw increment.
2. `kinematics.cmd_vel_aep_pep(cur_pose, …)` → per-leg AEP and PEP.
3. For each phase in `[0, 2π)` sampled at `period / 0.02` points: `kinematics.cmd_vel_new_point(gait, height, phase, aep, pep)`.

**Gait number is inverted here:**

```python
gait = 2 if params.gait == 1 else 1
```

`StepController.cmd_gait` defaults to **2** (named tripod). After this line the `.so` is called with **1** (ripple). Joystick D-pad sets `self.gait = 1` then `traveling(11)` so `cmd_gait=1`, which this line turns into **tripod** for cmd_vel. Analog stick sets `self.gait = 2` → cmd_vel uses **ripple**.

So: **Traveling `gait=2` is tripod. cmd_vel’s `cmd_gait=2` is *not*.** Always check which generator is running.

The generator also tries to pick a phase close to the current feet (`total_dist_sq`) so a new Twist does not teleport a foot. `slow == 'cmd_true'` uses a 50 ms slice instead of 20 ms for that catch-up.

---

## 9. Who uses which gait

| Caller | File | What it sends |
|---|---|---|
| Voice walk | `xf_mic_asr_offline/scripts/voice_control_move.py` | `traveling(gait=2, stride=40, height=20, time=0.7, steps=6)` plus a Twist; stop is `gait=-2` + zero Twist |
| Voice dance | same | `run_action('twist')` — **not a gait** |
| Joystick analog | `peripherals/joystick_control.py` | `traveling(12, height, period)` then `cmd_vel` (intended tripod config, see inversion) |
| Joystick D-pad | same | `traveling(11, …)` then `cmd_vel` (intended ripple config) |
| Joystick Start | same | `gait=-2` (or −1 in SLAM) |
| Classroom tripod | `example/.../tripod_gait.py` | `traveling(gait=2)` then 5 s then `gait=0` |
| Classroom ripple | `ripple_gait.py` | `traveling(gait=1)` |
| Strafe demo | `left_and_right.py` | `gait=1`, `direction=radians(90)` / `270` |
| Diagonal demo | `diagonally.py` | `gait=1`, `direction=radians(param)` |
| Speed demo | `speed_control.py` | `gait=1`, tunable stride / period |
| Body wave | `body_wave.py` | **not a gait** — `transform_euler` on planted feet |
| Body IK demo | `body_ik.py` | one foot (`leg_id=2`) absolute |
| Posture demo | `posture_adjustment.py` | `transform_pose_euler` |
| Nav2 / apps | various | `/controller/cmd_vel` (clamped 0.12 m/s) |

---

## 10. Action groups vs gaits (Sprint 3)

`.d6a` files are SQLite. `ActionGroupController` reads rows `(duration_ms, pulse1, pulse2, …)` and publishes `ServosPosition` with `position_unit='pulse'`, then `sleep(duration)`. No IK, no support polygon, no ZMP.

Path: `/home/ubuntu/software/actionset_editor/ActionGroups/<name>.d6a`.

`/controller/run_actionset` (`interfaces/msg/RunActionSet`: `action_path`, `interrupt`):

- `stop` → halt the group, then `init_pose`
- `dance_1/2/3` → forwarded to `/perform_actions/actions` if that node is up
- anything else → `start_action_thread(basename)` if the previous group finished (`action_complete`)

Gaits and action groups both publish to **the same** `servo_controller` topic. That is why halt-then-dance is mandatory.

---

## 11. Units traps (read before you send a Traveling)

**Direction**

- Joystick and `example/body_control` use **radians** (`math.radians(90)` = left). `MovingGenerator` compares `direction >= math.pi`, which only makes sense in radians.
- Voice uses **0, 90, 180, 270** as if they were degrees. Those values are all `≥ π` except `0`, so the ripple preamble always takes the “backward” branch, and the `.so` is given ~90 rad if you ever send voice’s 90 on a ripple Traveling.

**Rotation**

- `.so` docstring says rotation ∈ [−1, 1].
- Joystick convert degrees to radians (`SMALL_ROTATE = radians(15)`) then, for cmd_vel, `rotate = SMALL_ROTATE / period / 2` as **angular velocity**.
- Voice Traveling uses `rotation=±18` (looks like degrees per step). Different path, different unit.

**Stride / height**

- Traveling: millimetres (stride ~40, height ~15–20).
- Joystick `SMALL_STEP = 0.08`, `BIG_STEP = 0.15` are **metres of “stick stride”**, turned into m/s via `/ period / 2`, then cmd_vel scales ×1000 to mm/s.

**cmd_vel vs traveling gait id:** inverted in `CmdVelGenerator` only (section 8).

---

## 12. How to watch a gait live

```bash
source ~/.zshrc
ros2 topic echo /controller/traveling --once
ros2 topic echo /controller/cmd_vel
ros2 topic hz /servo_controller          # ~50 Hz while walking
ros2 topic echo /controller_manager/joint_states --once
ros2 run tf2_ros tf2_echo base_link end_LF
```

Classroom (these **replace** other users of `/controller`; do not run on top of Nav2 or a dance):

```bash
ros2 launch example tripod_gait.launch.py    # gait=2 for 5 s, then 0
ros2 launch example ripple_gait.launch.py    # gait=1 for 5 s, then 0
```

Halt from a shell:

```bash
ros2 topic pub --once /controller/traveling kinematics_msgs/msg/Traveling \
  "{gait: -2, time: 1.0, steps: 0, interrupt: true}"
```

---

## 13. File map

| Path | Role |
|---|---|
| `driver/controller/controller/move_controller.py` | ROS façade: Traveling / cmd_vel / pose / action set |
| `driver/controller/controller/step_controller.py` | 20 ms loop, generators, IK dispatch, halt poses |
| `driver/controller/controller/move.py` | `MovingGenerator`, `CmdVelGenerator` |
| `driver/controller/controller/build_in_pose.py` | Named six-foot snapshots (mm) |
| `driver/controller/controller/pose_transformer.py` | Body RPY/translate generator |
| `driver/controller/controller/controller_client.py` | What apps call |
| `driver/kinematics/kinematics/kinematics.so` | Rust IK + gait samples + AEP/PEP |
| `driver/kinematics/kinematics/config.py` | Servo id map, directions, 240° pulses |
| `driver/kinematics/kinematics/kinematics_calculate.py` | `transform_euler` / `transform_quat` wrappers |
| `driver/kinematics_msgs/msg/Traveling.msg` | Walk packet |
| `driver/servo_controller/…/action_group_controller.py` | `.d6a` player |
| `example/example/body_control/include/tripod_gait.py` | Minimal Traveling client |
| `example/example/body_control/include/ripple_gait.py` | Same, `gait=1` |
| `peripherals/peripherals/joystick_control.py` | cmd_vel + `gait=10+n` config |
| `xf_mic_asr_offline/scripts/voice_control_move.py` | Voice → Traveling / twist.d6a |

---

## 14. What this means for Sprint 3 (ZMP + dance)

- A **gait** already implements support-polygon thinking inside `kinematics.so` (tripod always leaves a triangle down). You cannot read that polygon from a ROS topic today; you reconstruct it from TF `end_*` or from `step_controller.pose` if you add a publisher.
- A **dance `.d6a`** has no gait engine. Feet can all leave a stable hull. That is why Sprint 3 computes ZMP **during the action group**, not by setting `gait=2`.
- Halt is **`Traveling.gait = -2`**, not a zero Twist alone. Zero Twist only starts a cmd_vel generator with vx=vy=wz=0; it does not snap to `DEFAULT_POSE`.
- Do not start a second `StepController` (the body_ik demo does; it is a classroom process, not bringup).

When you have questions, quote a section number or a file path. Useful ones: “why cmd_vel flips 1↔2”, “why voice direction is not radians”, “where to read foot tips for ZMP”, “what halt waits for”.
