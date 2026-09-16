# Masha follows Saveli — corrected plan

**Status:** plan only. Do **not** implement until Peter asks.  
**Source:** host clipboard paste (2026-09-16) titled “Step 2 Execution Plan: Masha Following Savelij (Mecanum Car)”.  
**This memo:** feasibility against Masha as she actually runs (Jetson Orin NX, ROS 2 Humble, `ROS_DOMAIN_ID=27`, `proud_up`, Aurora 930, LD19, bus servos).  
**Name:** the other robot is **Saveli** in the trio docs; the clipboard used **Savelij**. Same machine. This file uses **Saveli**.  
**Revision (2026-09-16, later same day):** custom gait is **in scope**. The first draft dropped it because a vendor `CmdVelGenerator` walk would already chase a tagged car. That was the *efficient* answer. Peter’s goal is **learning**, so we write the walk ourselves. How the walk *looks* (bounce, wave, high-step, …) is a later decision; the injection point is not.

Related reading (do not re-derive):

- `~/steps/MASHA_GAITS.md` — 50 Hz loop, Traveling vs `cmd_vel`, halt codes, AEP/PEP, gait numbers.
- `~/ros2_ws/info/TRAJECTORY.MD` — what “string of beads” actually means here (linear clock, not a keyframe table).
- `~/steps/quarter-01-sprint-02-masha-plan.md` — camera topics, QoS, arm rest pose, `/controller/cmd_vel`.
- `~/steps/quarter-01-sprint-03-plan.md` — `gait=5` `FollowGaitGenerator`, swing-only quintic, support-polygon contract. **Same gait engine.** Saveli-follow is a second *client* of that generator, not a second generator.

---

## Two products (both in scope)

| Product | What we learn | What we do **not** do |
|---|---|---|
| **1. Custom gait** | Foot-tip map, two clocks (cycle vs swing), support triangle, hook into the 20 ms loop | A second 50 Hz node on `/servo_controller`. A new mode inside closed-source `kinematics.so`. Replacing IK. |
| **2. Saveli follower** | AprilTag → `base_link` \((r,\theta)\), PD, LiDAR latch, halt that *stands* | Vendor `apriltag_track` / `lidar_app`. Bare `/cmd_vel`. Zero Twist as “stop”. |

Product 1 is the walk. Product 2 is who she walks toward. The clipboard glued them as `custom_gait_node` publishing servos. We keep them glued as **intent**, and split them at the **wire**: follower publishes velocity; our generator owns the feet; `StepController` still owns the servos.

---

## Verdict

**Following Saveli is feasible.** Rigid body + a rear AprilTag is a better first target than a person.

**Writing our own gait is feasible, and it is a goal of this project.** The 20 ms loop does not require the vendor curve. It only requires a tuple of six foot positions, then IK, then pulses.

**The clipboard pipeline is not feasible as written.** It invents a second 50 Hz gait *node* on the servo topic, the wrong halt, the wrong `cmd_vel` topic, optical-frame range math that is not `base_link`, and a 50 cm / 60 cm band that a hexapod cannot hold.

| Clipboard idea | Feasible? | Why |
|---|---|---|
| Track Saveli with an AprilTag on the rear | **Yes** | Family `tag36h11` already used on this Jetson. `apriltag_ros` is installed. OpenCV ArUco has `DICT_APRILTAG_36h11`. |
| Use Aurora RGB + LD19 `/scan` | **Yes** | Both already up in slim bringup. |
| Map range/heading error → body velocity | **Yes** | Same idea as vendor `apriltag_track` and `lidar_controller` follow mode, but those demos halt wrong and are not metric. |
| Hard LiDAR stop in front of Saveli | **Yes** | `/scan` exists; range filter already drops hits `< 0.2 m`. Must halt with Traveling, not a zero Twist. |
| **Write a custom gait** (AEP/PEP, lift curve, 50 Hz phase clock, 3-DOF IK) | **Yes — this project** | As a **generator** inside `move.py` / `StepController`. C++ math in `proud_up`. Vendor IK kept. |
| New `custom_gait_node` publishing `/servo_controller` at 50 Hz | **No** | Fights `StepController`. Two writers on the servo topic. Custom gait ≠ custom servo publisher. |
| Drive legs from `/controller/cmd_vel` | **Yes** | After Traveling selects `gait=5`, Twist feeds **our** generator, not vendor `CmdVelGenerator`. |
| `/cmd_vel` | **No** | Walk topic is `/controller/cmd_vel`. Bare `/cmd_vel` is empty on slim bringup. |
| Stop by `vx = 0, ωz = 0` | **No** | Zero Twist **starts a gait in place**. Halt is Traveling `gait=0` then `gait=-2`. |
| `r = √(x² + z²)`, `θ = atan2(x, z)` as body heading | **Only in optical frame** | Control must be in `base_link`: `r = √(x² + y²)`, `θ = atan2(y, x)`. Camera sits on the arm. |
| `r_target = 0.60 m`, hard stop `0.50 m` | **Too tight** | One unfinished step (~0.7 s) at 0.05–0.12 m/s overshoots 3–8 cm. LiDAR is 10 cm forward of `base_link`. Use ~0.80 m standoff. |
| Three nodes in series: tracker → lidar_safety → pid → gait | **Wrong shape** | Safety is an **override**, not a pipe stage. One C++ follower node + stock `apriltag_ros` + the gait **hook** in `controller`. |
| Match Saveli’s Mecanum strafe with `vy` | **Not first** | Masha *can* `linear.y` (±0.10 m/s). First tests: `vx` + `ωz` only. Strafe-out-of-FOV is a lost-target halt, not a crab match. Later, if the custom gait’s map includes `vy`, we can add it as a character choice. |

Do **not** mix this with Sprint 2 `follow_the_cat_node`, Sprint 3 `masha_interaction_node` FOLLOW, vendor `lidar_app`, Nav2, or joystick walk. Two publishers on `/controller/cmd_vel` fight. The **gait library** (`follow_gait.hpp` + `FollowGaitGenerator`) is shared with Sprint 3 Step 2; the **follow node** is not.

---

## Learner comments (standing rule)

Same rule as Sprints 2 and 3. Headers, `.cpp`, tests, launch, yaml, and the Python generator: comments a Java/Python learner can learn from.

- **Why, not restating the line.** Why zero Twist walks in place, why the arm is locked, why LiDAR metres and camera metres are not the same origin, why `gait=5` must never enter `kinematics.set_step_mode`.
- **Formulas next to code.** Name frames (`rgb_camera_link` / optical, `base_link`, `lidar_frame`) and units (m, rad, mm, pulse). Quintic `sigma(tau)` next to the swing sampler. Support triangle next to the phase split.
- **Masha specifics.** Real topics, `ROS_DOMAIN_ID=27`, clamps, QoS, gait numbers (`1` ripple, `2` tripod, `5` ours, `12`/`15` as `cmd_vel` selectors, `-2` halt).
- **Concurrency.** Image callback stores pointers; control timer owns PID + publish; never OpenCV on the DDS callback. The 20 ms gait thread already exists — we do not add another.
- **Safety.** Halt with Traveling. Lost tag → halt. LiDAR floor → halt. Watchdog if commands go quiet. Stance triangle contains CoM projection at every sample.
- **Tests.** Each gtest names the invariant (optical vs body heading, clamp, hysteresis, “zero Twist is not a halt”, swing `z-dot≈0` at lift/land, cycle clock does **not** rest at wrap).

Do not spam `// increment i`. Match existing `proud_up` tone.

---

## What Masha already has (reuse, do not duplicate)

### The 50 Hz executor (keep)

Boot starts one process, two ROS nodes:

```
process  move_controller
  ├─ /controller         MoveController     public API
  └─ /step_controller    StepController     20 ms loop + IK
```

`StepController.loop()` is a 50 Hz daemon. A gait is a Python generator of six foot tips (mm, body frame: **+X head, +Y left, +Z up**, feet at negative z). Each tick: IK `kinematics.set_leg_position` → `/servo_controller`.

That loop is the **only** legal writer of leg pulses during a walk. Our custom gait is a new generator it consumes. It is not a new ROS node.

### Vendor gait sampler (skip for follow)

`CmdVelGenerator` (`driver/controller/controller/move.py`) is the clipboard’s “parametric engine”, already:

1. `kinematics.cmd_vel_basic_data(vx_mm, vy_mm, wz, period)` → AEP/PEP offsets.
2. For `φ ∈ [0, 2π)` sampled every 20 ms: `kinematics.cmd_vel_new_point(...)`.
3. XY: linear PEP → AEP. Z: parabolic lift.

The TRAJECTORY.MD “string of beads” is the **clock**: equal time samples, no quintic. It is **not** a `.d6a` keyframe walk. Action groups (`twist.d6a`, dances) are the keyframe tables. Mixing those two machines is how you break the robot.

We **skip this sampler for Saveli follow.** We still use:

| Call | What it is | This project |
|---|---|---|
| `set_step_mode` / `cmd_vel_new_point` / AEP-PEP | Vendor **gait sampler** (ellipse / parabola, linear clock) | **Skip.** `gait=5` must never enter these. Closed-source; cannot add a mode. |
| `set_leg_position(leg, xyz)` | Vendor **one-leg IK** (foot in mm → 3 joint radians) | **Keep.** Our generator emits six foot tips; `set_pose_base` already calls this every tick. |
| `transform_pose` | Planted-foot body lean | Not the follow gait. Leave it alone. |

Clipboard Phase 1 (“write `custom_gait_node`”) would duplicate the **executor** and steal the servos. That part stays **out**. Writing the **map** that the executor samples is **in**.

### Walk command

| Want | Topic | Type | Notes |
|---|---|---|---|
| Body velocity | `/controller/cmd_vel` | `geometry_msgs/Twist` | Clamped `vx ∈ [-0.12, 0.12]`, `vy ∈ [-0.10, 0.10]`, `ωz ∈ [-0.6, 0.6]`. |
| Configure which gait `cmd_vel` uses | `/controller/traveling` | `kinematics_msgs/Traveling` | `gait: 12` → vendor tripod (`cmd_gait=2`). **`gait: 15` → our follow gait (`cmd_gait=5`).** Same 10+N selector the joystick already uses for 1/2/3. |
| Discrete / in-place custom walk | `/controller/traveling` | `gait: 5` | Starts `FollowGaitGenerator` directly (dry-run, stride 0). **Never** forwarded to `kinematics.set_step_mode`. |
| **Halt / stand** | `/controller/traveling` | `gait: 0` then `gait: -2` | `follow_the_cat_node::halt_legs()`. **Not** `Twist{}`. |

A new Twist replaces the generator on a step boundary (`last_part`). A halt can take up to one period (~0.7 s). Budget that in the stop distance.

Before the first follow Twist, publish Traveling `gait: 15` once (height ~25–30 mm, `time` ~0.6–0.8 s) so `cmd_gait=5` / `cmd_height` / `cmd_period` are defined. Default `cmd_gait` is 2 if nothing was sent — that would silently use the vendor tripod. The follower must not forget this select.

Hook (when implementing), in `step_controller.py`:

- `set_step_mode`: treat `gait == 15` like 11/12/13 (`cmd_gait = gait - 10` → 5). Do **not** start a walk on 15.
- `set_step_mode_base`: `gait == 5` → `FollowGaitGenerator`, **never** `MovingGenerator` → `set_step_mode(..., 5, ...)`.
- `cmd_vel`: `cmd_gait == 5` → `FollowGaitGenerator` (velocity in, feet out), **never** `CmdVelGenerator`.

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

## Custom gait (the learning work)

### What “develop a gait” means on this robot

A gait is a **periodic map** \(\varphi \in [0, 2\pi) \mapsto\) six foot tips in millimetres (body frame, \(+X\) head, \(+Y\) left, \(+Z\) up, feet at negative \(z\)). Every 20 ms `StepController` takes one sample, runs `kinematics.set_leg_position` (the **open** IK), and sends pulses.

We write that map. We do **not** call `kinematics.set_step_mode(..., gait=5, ...)` — the `.so` does not know 5.

**Injection point:** a new Python generator next to `MovingGenerator` / `CmdVelGenerator` in `controller/move.py`, selected when `Traveling.gait == 5`. `cmd_vel` with `cmd_gait == 5` uses the same generator so follow can be closed-loop (velocity in, feet out). Do **not** stream six `/controller/set_leg_absolute` from C++ at 50 Hz: those updates are not atomic across legs, `update_pose` defaults false, and the 20 ms loop will not treat them as a step.

C++ math lives in `proud_up` like ZMP: `include/proud_up/follow_gait.hpp` + `src/follow_gait.cpp` (no rclcpp) + `test/test_follow_gait.cpp`. Python generator copies the same formulas; comments in `move.py` point at the header. Do not invent a second IK.

This is the same engine Sprint 3 Step 2 named `FollowGaitGenerator`. Write it **once**. Saveli-follow and (later) person-follow are two publishers of Twist into it. Do not create `gait=6` unless Peter later wants a *second* character that must run at the same time as this one (it cannot: one generator slot).

### Support constraint (the actual “special” contract)

At every \(\varphi\), at least three feet are down and their triangle contains the CoM projection (static). That is how a gait stays a gait instead of becoming another dance. Optional later: keep Sprint 3’s ZMP estimator running during follow; not a gate for Saveli.

Gemini’s tripod groups, with **this** robot’s numbers (`MASHA_GAITS.md` §3):

- Swing group A: **1, 3, 5 = LF, LR, RM**
- Swing group B: **2, 4, 6 = LM, RR, RF**

### Two clocks (Chapter 9, already decided)

Rest-to-rest quintic on the **whole** step (`s-dot = 0` at both ends of the cycle) is the right clock for “go there and stop.” It is the **wrong** clock for walking. Repeating it is slow–fast–slow–**stop**, then again: a lurch, not a gait. The body never rests between steps in a natural walk; only the **foot in the air** should leave and land gently.

| Clock | What it times | Law | Why |
|---|---|---|---|
| **Cycle** (the walk) | Phase around the whole period | Keep rolling. Nearly constant phase rate. Does **not** go to zero at wrap. | Body and stance feet keep sliding. Cadence can be short; motion stays continuous. |
| **Swing** (one foot’s air time) | Progress along that foot’s arch, `sigma` in `[0, 1]` | Quintic: leave and land with zero speed and zero acceleration of `sigma`. | Kiss the floor. No stomp. No yank when the foot becomes stance. |

```text
sigma(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5     # tau = t_swing / T_swing
# only while that foot is in the air
# stance feet: body-frame retraction at roughly steady speed (no rest)
```

Beads analogy: the **necklace keeps turning** at a steady rate. We only re-space beads **on the flying stretch** of each foot — packed at lift-off and touchdown, spread at the top of the arch. We do **not** pack beads at the join between two walking cycles; that join must not be a stop.

Chassis jerk is fixed by that landing (and by matching swing speed into stance speed), not by pausing the body. Do **not** apply this quintic to `.d6a` playback. Do **not** ease by sleeping extra in the 20 ms loop. Do **not** zero the cycle clock at wrap.

The vendor walk *does* stomp: linear `s = t/T` on a parabola, so `z-dot ≠ 0` at touchdown. That difference is visible. It is also the Chapter 9 lesson TRAJECTORY.MD says is missing.

### Character — decide later (not a gate)

Peter: follow Saveli “in a bit customized way (we decide later how).” Until that pick, implement **one** map that is already a gait (support triangle + swing quintic + rolling cycle) and expose the look as **parameters**, not as a second generator.

| Knob | Start (until we pick) | What it changes |
|---|---|---|
| Grouping | Tripod A/B above | Wave/ripple later = one swing foot at a time; more stable, slower |
| Lift `height` | 25–30 mm | Higher than voice walk (~20 mm) so landings are obvious |
| Period | 0.6–0.8 s | Stalking = longer; scurrying = shorter |
| Stance compression | 0 mm (off) | Non-zero → body bob (“follow bounce”) |
| `vx` cap | 0.05 m/s | Safety vs unfinished-step overshoot |
| `vy` | 0 (off) | Crab match to Mecanum strafe — later skill |
| Duty (fraction of cycle a foot is up) | 0.5 (tripod) | Wave uses ~1/6 |

Menu for the later decision (change the map / knobs, not the ROS wiring):

1. **Quintic-swing tripod** — default. Looks different from vendor stomp; same grouping as `gait=2`.
2. **Follow bounce** — extra stance compression, visible body bob.
3. **Wave** — one leg at a time; slower, always five feet down; good for close standoff.
4. **High-step** — more clearance, same tripod.
5. **Stalking** — long period, small stride, still quintic landings.

Do not invent a sixth option in chat and then a seventh generator. Pick from this list when we have seen Test G2 on the floor.

---

## Corrections vs the clipboard plan

| Clipboard | Masha | What we do |
|---|---|---|
| `/cmd_vel` | `/controller/cmd_vel` | Publish Twist there. |
| Zero Twist = stop | Zero Twist = walk in place | `halt_legs()`: Traveling `gait=0` then `gait=-2`. Same helper as `follow_the_cat_node`. |
| `custom_gait_node` @ 50 Hz → `/servo_controller` | `StepController` already owns that wire | **Write the gait. Do not write a servo publisher.** |
| Replace “string of beads” gait | Beads = linear clock on the vendor AEP/PEP curve | **Replace the clock and the map** inside a new generator. Keep the 20 ms sampler. |
| `camera_link` | `depth_cam_link` / `rgb_camera_link`, parent `link4` | Lock arm at camera-forward rest pose. TF detections into `base_link`. |
| `r = √(x²+z²)`, `θ = atan2(x,z)` | Optical: X right, Z forward. Body: X forward, Y left | Optical only inside the detector. Control: `r = hypot(x,y)`, `θ = atan2(y,x)` in `base_link`. |
| `r_target = 0.60`, stop `0.50`, go `0.55` | Halt delay + lidar offset + body ~0.4 m wide | Start: `r_target = 0.80 m`, `d_stop = 0.55 m`, `d_go = 0.70 m` (all `base_link`). First `vx` cap **0.05 m/s**. |
| P-only `vx = Kp er`, `ωz = Kp eθ` | Hexapod lags a full step | PD + deadband. No I on first bring-up (windup during the unfinished step). |
| Three nodes in series | Safety must override | One follower node: tracker, PID, lidar, halt, watchdog. Gait lives in `controller`, selected by `gait=5` / `15`. |
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
  6. once: Traveling gait=15  (select our generator for cmd_vel)
  7. if lost / lidar stop / watchdog: halt_legs()
     else: PD(e_r, e_θ) → Twist on /controller/cmd_vel
           │
           ▼
/controller
  cmd_gait==5  →  FollowGaitGenerator @ 50 Hz   (OUR map + swing quintic)
           │
           ▼
StepController  →  kinematics.set_leg_position (IK only)  →  /servo_controller
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

## Hardware (Peter, before any follow node)

Gait dry-run (Phases G0–G2) does **not** need Saveli or a tag. Follow phases do.

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

## Frames and math (follower)

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

Depth sanity (optional, after Phase C): sample `/depth_cam/depth/image_raw` at the tag centre. If `|z_depth - z_pnp|` is large, trust lidar for range and tag for bearing, or halt.

---

## Node design (follower)

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
- `FOLLOW`: PD Twist on our gait. LiDAR latch → `STOPPED`. Miss `lost_timeout` → `LOST` + halt.
- `STOPPED`: halt; wait `d_min > d_go` and tag still locked → `FOLLOW`.
- `LOST`: halt; tag returns → `LOCKED`.

On entering `FOLLOW` the first time (and after a halt that cleared `cmd_gait`): publish Traveling `gait: 15` so the next Twist is ours.

Services (Masha app pattern): `~/start`, `~/stop` (`std_srvs/Trigger`). Parameter `enable_walk` **default false**: first live bring-up of the *follower* is **detect-and-print** (`r`, `θ`, `d_min`) with legs halted. Turn walk on after the numbers look right on the real floor. Gait dry-run does not use this node.

Debug: optional overlay image `~/image_result` (tag box, `r`, `θ`, `d_min`, phase). Do not `cv2.imshow` on the Jetson (vendor trackers do; we do not).

---

## Phases (execution order)

**Gait first, then tag, then glue.** That is the learning order. Clipboard Phase 1 (new gait *node*) stays rejected. Clipboard Phase 1 (new gait *map*) is Phases G0–G2.

### Phase G0 — gait map on paper + gtest (no robot)

Write `follow_gait.hpp`: (1) spatial map: phase, stride, height, vx, wz → six feet; (2) quintic `sigma(t)` on **swing only**; cycle phase keeps rolling. Invariants in `test_follow_gait.cpp`:

- At \(\varphi = 0\) and \(\varphi = \pi\), the stance triangle is the tripod group that is supposed to be down.
- CoM projection (equal masses ok in the unit test) is inside that triangle.
- `vx > 0` moves swing feet toward \(+X\); `wz > 0` rotates the AEP/PEP the correct way.
- No foot \(z\) above the contact band while marked stance.
- Swing quintic: at lift-off and touchdown, that foot’s `z-dot` and `sigma-dot` are ~0. Beads on the arch cluster at the ground ends, not at cycle wrap.
- Two cycles concatenated: **body / stance speed at the join is not ~0** (no rest-to-rest walk). A test that wants zero cycle-end `s-dot` is the wrong test.

No robot motion yet. Reading: `ros2_ws/info/TRAJECTORY.MD`.

### Phase G1 — generator hook, in place

`FollowGaitGenerator` in `move.py`. Branches in `step_controller.py` as in “Walk command” above. Precompute one cycle’s beads: even in **phase**, quintic along each **swing**. Yield one bead per 20 ms.

Dry-run: `ros2 topic pub` Traveling `gait: 5` in place (`stride: 0`) — legs cycle, no translation, landings look soft, the body does **not** pause at wrap — then halt `-2`.

If this dry-run falls through to `set_step_mode(..., 5)` the `.so` will do something undefined or nothing useful. Watch the logs; that is the first hardware invariant.

### Phase G2 — our gait under `cmd_vel`, no Saveli

Traveling `gait: 15` once, then small Twists by hand (`vx = 0.03`, then `ωz` only). Confirm: she translates, she turns, halt `-2` stands. Floor clear. This is also when Peter can look at the walk and pick a character from the menu (or keep the default).

DoD: visibly not vendor tripod (softer landings / different lift), still a tripod (or whichever grouping we chose), halt stands.

### Phase A — Tag visible (no walk)

1. Mount tag. Measure edge. Put metres in yaml.
2. Slim bringup. Arm rest pose (node or a one-shot `pose_commander`).
3. Launch `apriltag_ros` on Aurora RGB. `ros2 topic echo` detections. Walk Saveli by hand through 0.6–1.5 m, ±30° yaw.
4. Confirm TF `base_link` → tag. Print `r`, `θ`. If `r` is nonsense, the yaml size is wrong.

DoD: stable `r` within ~5 cm of a tape measure at 1.0 m; `θ` sign matches “tag on Masha’s left → +θ”.

G0–G2 and A can overlap in calendar (gait on the bench vs tag on the car). Do **not** start Phase C until both G2 and A are green.

### Phase B — Controller math, legs off

gtest: body heading, deadband, lidar hysteresis, “zero Twist is not halt” as a documented invariant (the halt helper publishes Traveling, tested with a fake publisher or a pure function `should_halt(...)`).

`enable_walk:=false`. Start the node. Drive Saveli. Foxglove / logs show commanded `vx`,`ωz` but **no** Twist on the walk topic (or Twist not published). Legs stay in `DEFAULT_POSE`.

### Phase C — Follow, slow, **our** gait

`enable_walk:=true`. Floor clear. First `vx_max = 0.05`.

1. Node publishes Traveling `gait: 15` once.
2. Tests 1–3 below.
3. Raise `vx_max` only after Test 2 is boring.

### Phase D — LiDAR latch on the real car

Same node, no extra process. Confirm Test 4. Confirm she does **not** chatter at the boundary (hysteresis). Confirm halt is a stand, not a march.

### Phase E — character retune (after C/D, not a gate)

Pick from the character menu. Change parameters or the map. Do **not** add a second servo node. Do **not** pass `5` into `kinematics.set_step_mode`.

---

## Tests

### Gait (Phases G0–G2)

Joystick / pad on Masha unplugged. No Saveli required.

**Test G0 — gtest.** All invariants in Phase G0. Fail the build if swing `z-dot` at touchdown is not ~0, or if cycle-end stance speed *is* ~0.

**Test G1 — in-place cycle.** Traveling `gait: 5`, `stride: 0`. Tripod (or chosen grouping) visible. Soft landings. No body pause at wrap. `gait: -2` → `DEFAULT_POSE`. Repeat. No servo fight (one publisher).

**Test G2 — slow `cmd_vel`.** `gait: 15` then `vx = 0.03` for ~3 s, halt. Then `ωz` only, halt. She must translate / yaw and then **stand**, not march.

### Follow (clipboard Phase 5, corrected)

Joystick on Saveli. Masha’s pad unplugged. `enable_walk` only after Phase B. Walk is **gait=5**.

**Test 1 — Stationary heading.** Saveli still, offset ~20–30° at ~1 m. Masha turns in place (`ωz` only) until `|θ| < 5°`, then **halt** (stand). She must not step in place after alignment.

**Test 2 — Straight follow.** Drive Saveli forward slowly. Masha holds ~0.80 m. No contact. Lost tag (you cover it) → stand within one step.

**Test 3 — Gentle curve.** Saveli yaws. Masha updates `ωz` and `vx`. If Saveli **strafes** out of the camera, she **stops** (pass). Matching `vy` is a later test, not a fail of Test 3.

**Test 4 — LiDAR cutoff.** From a follow, drive Saveli **backward** toward Masha. She stands at `d_stop` and stays stood until Saveli opens past `d_go`. Repeat three times: no chatter, no servo fight, no march-in-place.

**Test 5 — Watchdog.** Kill `apriltag_ros` while following. Masha stands. Same if the follow node is `Ctrl-C`’d (`on_shutdown` → `halt_legs()`).

Tape-measure the real standoff once. If 0.80 m feels far on this floor, lower `r_target` toward 0.70 **after** Test 4, never below `d_go + 0.10`.

---

## Files to add (when implementing)

**Add in `proud_up` (gait math):**

- `include/proud_up/follow_gait.hpp`
- `src/follow_gait.cpp`
- `test/test_follow_gait.cpp`

**Add in `proud_up` (follower):**

- `include/proud_up/savelij_follow.hpp`
- `src/savelij_follow.cpp`
- `src/savelij_follow_node.cpp`
- `test/test_savelij_follow.cpp`
- `launch/savelij_follow.launch.py`
- `config/savelij_follow.yaml` (topics, tag id, **measured** size, `r_target`, `d_stop`, `d_go`, gains, `enable_walk: false`, `follow_gait: 5` / select `15`)
- `config/apriltag_36h11_savelij.yaml` (copy of `apriltag_ros` 36h11 template with our size / id / image remaps)

**Edit:** `proud_up/CMakeLists.txt`, `proud_up/package.xml` (`tf2_ros`, `tf2_geometry_msgs`, `apriltag_msgs`, `kinematics_msgs` already used by follow-the-cat).

**Gait hook (this project, same files Sprint 3 Step 2 would touch):**

- `driver/controller/controller/move.py` — `FollowGaitGenerator`
- `driver/controller/controller/step_controller.py` — branch `gait == 5`, `gait == 15`, `cmd_gait == 5`

Do **not** rebuild `kinematics.so`. Do **not** pass `gait=5` into `set_step_mode`. Do **not** edit vendor `app/lidar_controller.py` or `example/.../apriltag_*.py`.

C++ rebuild: `colcon build --packages-select proud_up` then `source ~/ros2_ws/install/local_setup.zsh`. Python controller is usually symlink-installed; still restart `move_controller` after editing `move.py`. `need_compile=False` does not help C++.

If Sprint 3 Step 2 lands first, **reuse** `follow_gait.*` and the generator hook; this project then only adds the follower node. If this project lands first, Sprint 3 Step 2 reuses the gait and only adds ASK / YOLO.

---

## Why not `custom_gait_node`

The clipboard’s 50 Hz node is the right *ambition* (we own the foot curve) aimed at the wrong *socket* (the servo topic).

What following Saveli needs from the legs, and where it comes from once we write the gait:

- omnidirectional-enough `vx` and `ωz` — **our** generator, driven by Twist,
- a halt that stands — `halt_legs()`,
- a speed cap the unfinished step cannot overshoot through the LiDAR wall — parameters,
- a walk that is *ours* to change — the map + swing clock, not a second publisher.

Two writers on `/servo_controller` is how a gait fights a dance, and how a “custom gait node” would fight `StepController`. We do not learn that lesson by repeating it.

---

## Do not implement on this turn

This file is the corrected plan, with custom gait restored as a learning product. No node, no generator, no launch, no yaml in `proud_up` until Peter says to build it. Character of the walk stays a later pick from the menu above.
