# Masha follows Saveli — corrected plan

**Status:** plan only. Do **not** implement until Peter asks.  
**Source:** host clipboard paste (2026-09-16) titled “Step 2 Execution Plan: Masha Following Savelij (Mecanum Car)”.  
**This memo:** feasibility against Masha as she actually runs (Jetson Orin NX, ROS 2 Humble, `ROS_DOMAIN_ID=27`, `proud_up`, Aurora 930, LD19, bus servos).  
**Name:** the other robot is **Saveli** in the trio docs; the clipboard used **Savelij**. Same machine. This file uses **Saveli**.

Related reading (do not re-derive):

- `~/steps/MASHA_GAITS.md` — 50 Hz loop, Traveling vs `cmd_vel`, halt codes, AEP/PEP.
- `~/ros2_ws/info/TRAJECTORY.MD` — what “string of beads” actually means here.
- `~/steps/quarter-01-sprint-02-masha-plan.md` — camera topics, QoS, arm rest pose, `/controller/cmd_vel`.
- `~/steps/quarter-01-sprint-03-plan.md` — `gait=5` FollowGaitGenerator (person follow, **not** this project).

---

## Verdict

**Following Saveli is feasible.** Rigid body + a rear AprilTag is a better first target than a person.

**The clipboard pipeline is not feasible as written.** It invents a second 50 Hz gait node, the wrong halt, the wrong `cmd_vel` topic, optical-frame range math that is not `base_link`, and a 50 cm / 60 cm band that a hexapod cannot hold.

| Clipboard idea | Feasible? | Why |
|---|---|---|
| Track Saveli with an AprilTag on the rear | **Yes** | Family `tag36h11` already used on this Jetson. `apriltag_ros` is installed. OpenCV ArUco has `DICT_APRILTAG_36h11`. |
| Use Aurora RGB + LD19 `/scan` | **Yes** | Both already up in slim bringup. |
| Map range/heading error → body velocity | **Yes** | Same idea as vendor `apriltag_track` and `lidar_controller` follow mode, but those demos halt wrong and are not metric. |
| Hard LiDAR stop in front of Saveli | **Yes** | `/scan` exists; range filter already drops hits `< 0.2 m`. Must halt with Traveling, not a zero Twist. |
| Drive existing legs from `/controller/cmd_vel` | **Yes** | `CmdVelGenerator` is already AEP/PEP + parabolic lift at 50 Hz. |
| New `custom_gait_node` publishing `/servo_controller` at 50 Hz | **No** | Fights `StepController`. Two writers on the servo topic. |
| Replace the gait engine before following | **No, not this project** | The parametric engine already exists. “Beads” are the linear clock on that curve, not a keyframe table. |
| `/cmd_vel` | **No** | Walk topic is `/controller/cmd_vel`. Bare `/cmd_vel` is empty on slim bringup. |
| Stop by `vx = 0, ωz = 0` | **No** | Zero Twist **starts a gait in place**. Halt is Traveling `gait=0` then `gait=-2`. |
| `r = √(x² + z²)`, `θ = atan2(x, z)` as body heading | **Only in optical frame** | Control must be in `base_link`: `r = √(x² + y²)`, `θ = atan2(y, x)`. Camera sits on the arm. |
| `r_target = 0.60 m`, hard stop `0.50 m` | **Too tight** | One unfinished step (~0.7 s) at 0.05–0.12 m/s overshoots 3–8 cm. LiDAR is 10 cm forward of `base_link`. Use ~0.80 m standoff. |
| Three nodes in series: tracker → lidar_safety → pid → gait | **Wrong shape** | Safety is an **override**, not a pipe stage. One C++ node in `proud_up` (plus stock `apriltag_ros`) is the first build. |
| Match Saveli’s Mecanum strafe with `vy` | **Not first** | Masha *can* `linear.y` (±0.10 m/s). First tests: `vx` + `ωz` only. Strafe-out-of-FOV is a lost-target halt, not a crab match. |

Do **not** mix this with Sprint 2 `follow_the_cat_node`, Sprint 3 `masha_interaction_node` FOLLOW, vendor `lidar_app`, Nav2, or joystick walk. Two publishers on `/controller/cmd_vel` fight.

---

## Learner comments (standing rule)

Same rule as Sprints 2 and 3. Headers, `.cpp`, tests, launch, yaml: comments a Java/Python learner can learn from.

- **Why, not restating the line.** Why zero Twist walks in place, why the arm is locked, why LiDAR metres and camera metres are not the same origin.
- **Formulas next to code.** Name frames (`rgb_camera_link` / optical, `base_link`, `lidar_frame`) and units (m, rad, pulse).
- **Masha specifics.** Real topics, `ROS_DOMAIN_ID=27`, clamps, QoS.
- **Concurrency.** Image callback stores pointers; control timer owns PID + publish; never OpenCV on the DDS callback.
- **Safety.** Halt with Traveling. Lost tag → halt. LiDAR floor → halt. Watchdog if commands go quiet.
- **Tests.** Each gtest names the invariant (optical vs body heading, clamp, hysteresis, “zero Twist is not a halt”).

Do not spam `// increment i`. Match existing `proud_up` tone.

---

## What Masha already has (do not rebuild)

### Gait (clipboard Phase 1 — already done)

Boot starts one process, two ROS nodes:

```
process  move_controller
  ├─ /controller         MoveController     public API
  └─ /step_controller    StepController     20 ms loop + IK
```

`StepController.loop()` is a 50 Hz daemon. A gait is a Python generator of six foot tips (mm, body frame: **+X head, +Y left, +Z up**, feet at negative z). Each tick: IK `kinematics.set_leg_position` → `/servo_controller`.

`CmdVelGenerator` (`driver/controller/controller/move.py`) is **already** the clipboard’s “parametric engine”:

1. `kinematics.cmd_vel_basic_data(vx_mm, vy_mm, wz, period)` → AEP/PEP offsets.
2. For `φ ∈ [0, 2π)` sampled every 20 ms: `kinematics.cmd_vel_new_point(...)`.
3. XY: linear PEP → AEP. Z: parabolic lift.

The TRAJECTORY.MD “string of beads” is the **clock**: equal time samples, no quintic. It is **not** a `.d6a` keyframe walk. Action groups (`twist.d6a`, dances) are the keyframe tables. Mixing those two machines is how you break the robot.

Clipboard Phase 1 (“write `custom_gait_node`”) would duplicate this and steal the servos. **Out of scope.** If a *distinct* follow walk is wanted later, that is Sprint 3’s `gait=5` `FollowGaitGenerator` hook — still inside `move.py`, still using vendor IK, **never** a second servo publisher. Not a gate for Saveli following.

### Walk command

| Want | Topic | Type | Notes |
|---|---|---|---|
| Body velocity | `/controller/cmd_vel` | `geometry_msgs/Twist` | Clamped `vx ∈ [-0.12, 0.12]`, `vy ∈ [-0.10, 0.10]`, `ωz ∈ [-0.6, 0.6]`. |
| Configure which gait `cmd_vel` uses | `/controller/traveling` | `kinematics_msgs/Traveling` | `gait: 12` stores tripod-named defaults for the next Twist (same as joystick analog). |
| **Halt / stand** | `/controller/traveling` | `gait: 0` then `gait: -2` | `follow_the_cat_node::halt_legs()`. **Not** `Twist{}`. |

A new Twist replaces the generator on a step boundary (`last_part`). A halt can take up to one period (~0.7 s). Budget that in the stop distance.

Before the first Twist, publish Traveling `gait: 12` once (height ~20 mm, `time` ~0.7 s) so `cmd_gait` / `cmd_height` / `cmd_period` are defined. Default `cmd_gait` is 2 if nothing was sent.

### Perception

| Sensor | Topics | QoS / rate |
|---|---|---|
| Aurora 930 RGB | `/depth_cam/rgb/image_raw`, `/depth_cam/rgb/camera_info` | **best-effort**, ~15 fps. Reliable subscriptions often see zero frames. |
| Aurora depth | `/depth_cam/depth/image_raw` | Filtered ~0.15–4.0 m (`minimum_filter_depth_value=150` mm). Optional range cross-check. |
| LD19 | `/scan` (`sensor_msgs/LaserScan`) | best-effort. `laser_filters` already replaces ranges `< 0.2 m` with `-inf`. Frame `lidar_frame`. |
| Camera TF | `depth_cam_link` → `rgb_camera_link` (static, optical) | Camera parent is **arm `link4`**, not the body. |

Vendor AprilTag demo (`example` `apriltag_recognition`): family `tag36h11`, publishes `interfaces/ApriltagsInfo`. **Do not use `d` for metres.** `ApriltagInfo.d` is `int32` from `solvePnP` on a unit-size board (`OBJP` is ±1, not the real tag edge). `apriltag_track.py` then PIDs that integer into `/controller/cmd_vel` and stops with `Twist{}`. Prototype only.

Better detector, already on the Jetson:

- System / third-party: `apriltag_ros` (`AprilTagNode`), messages `apriltag_msgs/AprilTagDetectionArray`, yaml tag size in **metres**.
- Or OpenCV ArUco `DICT_APRILTAG_36h11` inside `proud_up` (OpenCV 4.11 on this board; `cv2.aruco` present). Needs the **measured** tag edge length.

Prefer **stock `apriltag_ros`** for pose (it already does PnP with `size`) plus a new C++ follower. Do not launch the vendor `apriltag_recognition` at the same time.

### LiDAR geometry

`lidar_link` is on `base_link` at about **+0.102 m X, +0.035 m Z** (URDF). Front feet in `DEFAULT_POSE` are at **x ≈ +0.164 m**. Own legs are inside the 0.2 m range filter. Saveli at “50 cm from Masha’s body” is ~40 cm from the lidar origin. **Pick one frame for every threshold: `base_link`.** Convert lidar `d_min` into that frame (approximately `d_min + 0.10 m` along X if the hit is dead ahead — do not hard-code that as the only case; transform the scan point).

Vendor `lidar_controller` follow mode uses a 90° front sector, PID, threshold default 0.6 m. It also publishes `Twist{}` to idle. Reuse the **sector + min range** idea, not the node.

---

## Corrections vs the clipboard plan

| Clipboard | Masha | What we do |
|---|---|---|
| `/cmd_vel` | `/controller/cmd_vel` | Publish Twist there. |
| Zero Twist = stop | Zero Twist = walk in place | `halt_legs()`: Traveling `gait=0` then `gait=-2`. Same helper as `follow_the_cat_node`. |
| `custom_gait_node` @ 50 Hz → `/servo_controller` | `StepController` already owns that wire | **No new servo publisher.** |
| Replace “string of beads” gait | Beads = linear clock on an existing AEP/PEP curve | Skip. Optional later: Sprint 3 `gait=5` quintic **swing** clock. |
| `camera_link` | `depth_cam_link` / `rgb_camera_link`, parent `link4` | Lock arm at camera-forward rest pose. TF detections into `base_link`. |
| `r = √(x²+z²)`, `θ = atan2(x,z)` | Optical: X right, Z forward. Body: X forward, Y left | Optical only inside the detector. Control: `r = hypot(x,y)`, `θ = atan2(y,x)` in `base_link`. |
| `r_target = 0.60`, stop `0.50`, go `0.55` | Halt delay + lidar offset + body ~0.4 m wide | Start: `r_target = 0.80 m`, `d_stop = 0.55 m`, `d_go = 0.70 m` (all `base_link`). First `vx` cap **0.05 m/s**. |
| P-only `vx = Kp er`, `ωz = Kp eθ` | Hexapod lags a full step | PD + deadband. No I on first bring-up (windup during the unfinished step). |
| Three nodes in series | Safety must override | One node: `savelij_follow_node` in `proud_up`. Tracker, PID, lidar, halt, watchdog. |
| 60° LiDAR sector | Vendor follow uses 90° | Parameter, default **90°**. Ignore non-finite / `< 0.2 m`. |
| Test 3: match Mecanum strafe | Camera FOV + hexapod `vy` is a later skill | First: turn then chase (`vx`,`ωz`). Lost tag → halt. |

---

## Architecture (corrected)

```
Saveli (Mecanum car)
  rear: AprilTag 36h11, measured edge length in yaml
           │
           ▼
Aurora RGB  /depth_cam/rgb/image_raw     LD19 /scan
  + camera_info (best-effort)
           │
           ▼
apriltag_ros  AprilTagNode
  → detections (pose in camera / optical frame)
           │
           ▼
savelij_follow_node   (NEW, package proud_up)
  1. lock arm rest pose (ids 19–24), never pan while following
  2. TF tag pose → base_link   (x forward, y left)
  3. r = hypot(x,y),  θ = atan2(y,x)
  4. optional: depth at tag pixel as range sanity check
  5. frontal /scan → d_min in base_link
  6. if lost / lidar stop / watchdog: halt_legs()
     else: PD(e_r, e_θ) → Twist on /controller/cmd_vel
           │
           ▼
existing  /controller  →  CmdVelGenerator @ 50 Hz  →  IK  →  /servo_controller
```

Saveli is a **separate** computer. Do not assume a shared ROS domain. Masha only *sees* the tag.

Do **not** start:

- `follow_the_cat_node`
- `masha_interaction_node` in FOLLOW
- `lidar_app` (`app/lidar_controller.py`)
- Nav2
- `apriltag_recognition` (vendor example)
- joystick analog walk (unplug the pad or leave sticks centered)

Voice stack may stay up. “Stop” via existing voice halt is a plus if it does not also emit a competing Twist; first tests: no voice walk commands.

---

## Hardware (Peter, before any node)

1. Print **tag36h11 id 0** (or another id, then set yaml). Measure the **black-square edge** in metres. Clipboard-era yaml in `apriltag_ros` defaults to **0.08 m** — do not trust that; measure.
2. Mount it **vertical, facing backward**, on Saveli’s rear, as high as the Aurora sees at 0.6–1.5 m with Masha’s arm in the rest pose. Not on the floor. Not at an angle that foreshortens to a line.
3. Lighting: AprilTag wants contrast. Avoid pointing the tag at a window behind Masha.
4. Keep Saveli’s rear clear of cables that cover the tag when it turns.

Arm rest pose (camera-forward, same numbers as proud-up / follow-the-cat):

```
19=500, 20=810, 21=180, 22=150, 23=500, 24=500
```

Hold 23 and 24 at 500. **Do not** command servo 23 as tilt (it is wrist yaw). This project **does not** gaze-track: the camera stays bolted forward so `base_link` ↔ camera TF is stable.

---

## Frames and math

Optical (ROS camera, `rgb_camera_link`): **X right, Y down, Z out of the lens.**  
Clipboard `r = √(x_s² + z_s²)`, `θ = atan2(x_s, z_s)` is **this** frame. Fine as a detector local, wrong as a walk command.

Body (`base_link`): **X forward (head), Y left, Z up.** Same as the gait pose.

After `tf2` of the tag origin into `base_link`:

```text
r     = hypot(x, y)                 # metres, ignore z (tag height)
θ     = atan2(y, x)                 # rad, +θ = tag to Masha’s left
e_r   = r - r_target
e_θ   = θ                           # want Saveli on the nose
```

PD (first bring-up; I-term off):

```text
vx = clamp( Kp_r * e_r  + Kd_r * ė_r,  -vx_max, +vx_max )
ωz = clamp( Kp_θ * e_θ  + Kd_θ * ė_θ,  -wz_max, +wz_max )
```

Deadband so she does not hunt:

- if `|e_r| < 0.05 m` → `vx = 0` (and if also `|e_θ|` small, **halt**, do not stream zeros).
- if `|e_θ| < 5°` → `ωz = 0`.

First live caps: `vx_max = 0.05`, `wz_max = 0.3`. Software clamp is extra; `move_controller` also clamps 0.12 / 0.6.

When `vx` and `ωz` are both commanded zero for more than one control tick **and** we are at the setpoint: `halt_legs()`. Publishing `Twist{}` every 10 Hz is how the vendor demos leave her marching.

LiDAR:

```text
d_min = min finite range in the front sector, transformed toward base_link X
if d_min < d_stop:          halt_legs();  latched_stop = true
if latched_stop and d_min > d_go:         latched_stop = false   # hysteresis
```

Start values (all discussed as **base_link** range to the obstacle, not “lidar metres”):

| Name | Start | Role |
|---|---|---|
| `r_target` | 0.80 m | follow standoff |
| `d_stop` | 0.55 m | hard stop |
| `d_go` | 0.70 m | re-enable |
| `lost_timeout` | 0.5 s | no tag → halt |
| `walk_watchdog` | 0.3 s | no control tick → halt |

`r_target` must sit **above** `d_go`, otherwise the PID drives her into the latch. Clipboard 0.60 / 0.50 / 0.55 violates that once halt delay is included.

Lost tag: halt, stay in LOST. Optional later: slow search `ωz` in place — **not** first tests.

Depth sanity (optional, Phase 2+): sample `/depth_cam/depth/image_raw` at the tag centre. If `|z_depth - z_pnp|` is large, trust lidar for range and tag for bearing, or halt.

---

## Node design

**Package:** `proud_up` (no new ament package).  
**Executable:** `savelij_follow_node`.  
**Launch:** `ros2 launch proud_up savelij_follow.launch.py` after slim bringup — do **not** run `~/.stop_ros.sh` to start it.

Split like follow-the-cat:

- `include/proud_up/savelij_follow.hpp` + `src/savelij_follow.cpp` — no rclcpp: PD, deadband, hysteresis, optical→body heading tests with fake poses.
- `src/savelij_follow_node.cpp` — subscriptions, TF, timer, publishers, services.
- `test/test_savelij_follow.cpp` — offline gtest.
- `launch/savelij_follow.launch.py` + `config/savelij_follow.yaml`.
- Launch also starts `apriltag_ros` remapped to `/depth_cam/rgb/image_raw` and `/depth_cam/rgb/camera_info`, yaml with **measured** `size` and `tag_ids`.

**QoS:** image, camera_info, scan: `KeepLast(5)` + `best_effort()`, copy `follow_the_cat_node` / `lidar_controller`.

**Phases:** `IDLE → LOCKED → FOLLOW → STOPPED / LOST`.

- `IDLE`: rest pose, `halt_legs()`, wait `~/start`.
- `LOCKED`: tag seen for `lock_seconds` (~0.5–1 s) and `r` in `[d_go, 1.5]`.
- `FOLLOW`: PD Twist. LiDAR latch → `STOPPED`. Miss `lost_timeout` → `LOST` + halt.
- `STOPPED`: halt; wait `d_min > d_go` and tag still locked → `FOLLOW`.
- `LOST`: halt; tag returns → `LOCKED`.

Services (Masha app pattern): `~/start`, `~/stop` (`std_srvs/Trigger`). Parameter `enable_walk` **default false**: first live bring-up is **detect-and-print** (`r`, `θ`, `d_min`) with legs halted. Turn walk on after the numbers look right on the real floor.

Debug: optional overlay image `~/image_result` (tag box, `r`, `θ`, `d_min`, phase). Do not `cv2.imshow` on the Jetson (vendor trackers do; we do not).

---

## Phases (execution order)

Clipboard Phase 1 (new gait engine) is **removed**. Optional gait cosmetics are last.

### Phase A — Tag visible (no walk)

1. Mount tag. Measure edge. Put metres in yaml.
2. Slim bringup. Arm rest pose (node or a one-shot `pose_commander`).
3. Launch `apriltag_ros` on Aurora RGB. `ros2 topic echo` detections. Walk Saveli by hand through 0.6–1.5 m, ±30° yaw.
4. Confirm TF `base_link` → tag. Print `r`, `θ`. If `r` is nonsense, the yaml size is wrong.

DoD: stable `r` within ~5 cm of a tape measure at 1.0 m; `θ` sign matches “tag on Masha’s left → +θ”.

### Phase B — Controller math, legs off

gtest: body heading, deadband, lidar hysteresis, “zero Twist is not halt” as a documented invariant (the halt helper publishes Traveling, tested with a fake publisher or a pure function `should_halt(...)`).

`enable_walk:=false`. Start the node. Drive Saveli. Foxglove / logs show commanded `vx`,`ωz` but **no** Twist on the walk topic (or Twist not published). Legs stay in `DEFAULT_POSE`.

### Phase C — Follow, slow

`enable_walk:=true`. Floor clear. First `vx_max = 0.05`.

1. Traveling `gait: 12` once.
2. Tests below.
3. Raise `vx_max` only after Test 2 is boring.

### Phase D — LiDAR latch on the real car

Same node, no extra process. Confirm Test 4. Confirm she does **not** chatter at the boundary (hysteresis). Confirm halt is a stand, not a march.

### Phase E — optional, later (not a gate)

Sprint 3 `FollowGaitGenerator` (`gait=5`) if Peter wants the follow *walk* to look different (bounce, quintic swing). Still no second servo node. Do **not** pass `5` into `kinematics.set_step_mode`.

---

## Tests (clipboard Phase 5, corrected)

Joystick on Saveli. Masha’s pad unplugged. `enable_walk` only after Phase B.

**Test 1 — Stationary heading.** Saveli still, offset ~20–30° at ~1 m. Masha turns in place (`ωz` only) until `|θ| < 5°`, then **halt** (stand). She must not step in place after alignment.

**Test 2 — Straight follow.** Drive Saveli forward slowly. Masha holds ~0.80 m. No contact. Lost tag (you cover it) → stand within one step.

**Test 3 — Gentle curve.** Saveli yaws. Masha updates `ωz` and `vx`. If Saveli **strafes** out of the camera, she **stops** (pass). Matching `vy` is a later test, not a fail of Test 3.

**Test 4 — LiDAR cutoff.** From a follow, drive Saveli **backward** toward Masha. She stands at `d_stop` and stays stood until Saveli opens past `d_go`. Repeat three times: no chatter, no servo fight, no march-in-place.

**Test 5 — Watchdog.** Kill `apriltag_ros` while following. Masha stands. Same if the follow node is `Ctrl-C`’d (`on_shutdown` → `halt_legs()`).

Tape-measure the real standoff once. If 0.80 m feels far on this floor, lower `r_target` toward 0.70 **after** Test 4, never below `d_go + 0.10`.

---

## Files to add (when implementing)

**Add in `proud_up`:**

- `include/proud_up/savelij_follow.hpp`
- `src/savelij_follow.cpp`
- `src/savelij_follow_node.cpp`
- `test/test_savelij_follow.cpp`
- `launch/savelij_follow.launch.py`
- `config/savelij_follow.yaml` (topics, tag id, **measured** size, `r_target`, `d_stop`, `d_go`, gains, `enable_walk: false`)
- `config/apriltag_36h11_savelij.yaml` (copy of `apriltag_ros` 36h11 template with our size / id / image remaps)

**Edit:** `proud_up/CMakeLists.txt`, `proud_up/package.xml` (`tf2_ros`, `tf2_geometry_msgs`, `apriltag_msgs`, `kinematics_msgs` already used by follow-the-cat).

Do **not** edit vendor `app/lidar_controller.py`, `example/.../apriltag_*.py`, `controller/move.py`, or `kinematics.so` for this project.

C++ rebuild: `colcon build --packages-select proud_up` then `source ~/ros2_ws/install/local_setup.zsh`. `need_compile=False` does not help C++.

---

## Why the clipboard gait rewrite can wait

Clipboard Phase 1 is a real research topic (quintic swing clock, support-polygon guarantee). It is **Sprint 3 Step 2**, aimed at following a **person** after a dance, and it is not required to chase a tagged car.

What following Saveli actually needs from the legs:

- omnidirectional-enough `vx` and `ωz` (already),
- a halt that stands,
- a speed cap the unfinished step cannot overshoot through the LiDAR wall.

`CmdVelGenerator` already supplies the first. `halt_legs()` supplies the second. Parameters supply the third.

---

## Do not implement on this turn

This file is the corrected plan. No node, no launch, no yaml in `proud_up` until Peter says to build it.
