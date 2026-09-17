# Masha hunter — pluggable target (Saveli first)

**Status:** plan only. Do **not** implement until Peter asks.  
**Source:** host clipboard paste (2026-09-16) titled “Step 2 Execution Plan: Masha Following Savelij (Mecanum Car)”.  
**This memo:** feasibility against Masha as she actually runs (Jetson Orin NX, ROS 2 Humble, `ROS_DOMAIN_ID=27`, `proud_up`, Aurora 930, LD19, bus servos).  
**Name:** the other robot is **Saveli** in the trio docs; the clipboard used **Savelij**. Same machine. This file uses **Saveli**.  
**Revision (2026-09-16, later same day):** custom gait is **in scope**. Map pick: **variant 3 — omnidirectional tripod.**  
**Revision (2026-09-16, hunter):** Saveli is the **first plug**, not the product. End goal is a spider **hunter**: deep stand, slow head scan, recognize, **call the target by name**, follow ≤ **60 s** or until lost, then scan again. Node: **`masha_hunter_node`**. Next plug: **cat**. Filename kept for the GitHub URL.  
**Revision (2026-09-16, wander):** No SLAM / Nav2. Optional HUNT rule: go into open space, halt at an obstacle, rotate, look again. Default **off**.  
**Revision (2026-09-17, start):** Hunter is live in `proud_up`. Operator recipe is [Start Instructions](#start-instructions). Launch **must** pass `enable_walk:=true` (yaml/launch default is **false**). Crab-follow run: `enable_crab:=true` `vx_max:=0.12` `vy_max:=0.08`.

Related reading (do not re-derive):

- `~/steps/MASHA_GAITS.md` — 50 Hz loop, Traveling vs `cmd_vel`, halt codes, AEP/PEP, gait numbers.
- `~/ros2_ws/info/TRAJECTORY.MD` — what “string of beads” actually means here (linear clock, not a keyframe table).
- `~/steps/quarter-01-sprint-02-masha-plan.md` — camera topics, QoS, arm pan/tilt (ids 19 / 22), `/controller/cmd_vel`.
- `~/steps/quarter-01-sprint-03-plan.md` — `gait=5` `FollowGaitGenerator`, swing-only quintic, `aplay` worker. **Same gait engine.** Hunter is a *client* of that generator, not a second generator.

---

## Start Instructions

Checked on Masha (2026-09-17). These files exist:

| File | Role |
|---|---|
| `/opt/ros/humble/setup.bash` | ROS 2 Humble (`setup.zsh` also exists) |
| `~/ros2_ws/install/setup.bash` | overlay with `proud_up` (`setup.zsh` also exists) |
| `~/ros2_ws/.typerc` | `ROS_DOMAIN_ID=27` |
| `~/ros2_ws/src/proud_up/launch/masha_hunter.launch.py` | launch args below |
| `~/ros2_ws/src/proud_up/config/masha_hunter.yaml` | node params (overridden by launch args) |

A normal Masha zsh already sources `.typerc` + Humble + the overlay via `.robotrc`. Re-source anyway so a bare terminal still sees domain **27** and `masha_hunter_node`. Do **not** run `~/.stop_ros.sh`. Slim bringup is assumed already up. Do **not** start `follow_the_cat` or `masha_interaction` with the hunter. Unplug Masha’s joystick (shared 2.4 GHz pad with Saveli). Place Savelij ~**0.8–1.0 m** in front, rear **36h11 id 0** facing the camera.

Launch args do **not** hot-reload. Ctrl-C the old hunter, then relaunch.

### Terminal 1 — launch

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/.typerc
ros2 launch proud_up masha_hunter.launch.py \
  enabled_targets:=saveli \
  enable_walk:=true \
  enable_crab:=true \
  vx_max:=0.12 \
  vy_max:=0.08 \
  dry_run:=false
```

First log line must show `enable_walk=true enable_crab=true`. `enable_wander` stays **false** unless you pass it in yaml. First lock is safer with `enable_walk:=false` (Twist is logged only; legs stay in hunter pose). After a clean lock, relaunch with walk on.

### Launch arguments (all of them)

| Argument | This run | Launch default | What it does |
|---|---|---|---|
| `enabled_targets` | `saveli` | `saveli` | Which plugs run. `apriltag_ros` starts only if `saveli` is listed. |
| `enable_walk` | **`true`** | **`false`** | If false: HUNT/NAME only, log Twist, do not publish `/controller/cmd_vel`. |
| `enable_crab` | **`true`** | `true` | Side offset uses `vy` when `\|bearing\| < 15°` and `\|y\| > 6 cm`. |
| `vx_max` | `0.12` | `0.12` | Forward cap m/s (voice “go forward” is 0.12). |
| `vy_max` | `0.08` | `0.08` | Crab cap m/s (`move_controller` also clamps 0.10). |
| `dry_run` | `false` | `false` | Log servo / Traveling / Twist and skip publishes. |
| `params_file` | (omit) | `proud_up/config/masha_hunter.yaml` | Node YAML. |
| `apriltag_params` | (omit) | `proud_up/config/apriltag_36h11_savelij.yaml` | Tag size in metres (not the 0.08 stock default). |

### Terminal 2 — start / watch / stop

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/.typerc
ros2 service call /masha_hunter_node/start std_srvs/srv/Trigger
```

Watch crab (same terminal after start, or a third one):

```bash
ros2 topic echo /controller/cmd_vel
```

- **Sideways = crab gated on:** `linear.y` nonzero (and hunter log `crab=true`).
- **Pure turn-in-place again:** crab **not** gated. Saveli is too far around (`\|bearing\| ≥ 15°`); policy yaws first (`angular.z`, `linear.y = 0`).
- If start hangs on “waiting for service”, launch + AprilTag can still be up while `masha_hunter_node` has died. Confirm the node is in `ros2 node list`. Watch `/apriltag/apriltag_detections` and `/masha_hunter_node/image_result`. Non-empty detections = tag seen.

Stop:

```bash
ros2 service call /masha_hunter_node/stop std_srvs/srv/Trigger
```

Stop centres the head, halt-stands (`gait=-2`), returns IDLE. Halt is Traveling `gait=-2`, **not** a zero Twist (zero Twist marches in place).

### Crab gate and yaml (not launch args)

C++ defaults (not overlaid by the launch line above):

| Name | Value | Role |
|---|---|---|
| `θ_crab` | 15° | already roughly facing him |
| `y_on` / `y_off` | 0.06 / 0.03 m | start / stop crab (hysteresis) |
| `wz_max` | 0.30 rad/s | yaw cap; while crabbing, `ωz` is scaled ×0.3 |
| `r_target` | 0.80 m | follow standoff |
| `d_stop` / `d_go` | 0.55 / 0.70 m | LiDAR latch in `base_link` |
| `lost_timeout` | 1.5 s | FOLLOW → HUNT after no fresh TF |
| `pose_max_age` | 0.40 s | older TF is not a live hit (coast) |
| `walk_watchdog` | 0.30 s | no `cmd_vel` → halt |
| `follow_max_s` | 60 | then back to HUNT |
| `enable_wander` | false | HUNT stay/pan, do not walk into open space |
| `follow_gait` / select | 5 / 15 | our generator, not vendor tripod |
| `cmd_period` / `cmd_height` | 0.70 s / 25 mm | Traveling `gait: 15` |

---

## Three products (all in scope)

| Product | What we learn | What we do **not** do |
|---|---|---|
| **1. Custom gait** | Foot-tip map, two clocks (cycle vs swing), support triangle, hook into the 20 ms loop | A second 50 Hz node on `/servo_controller`. A new mode inside closed-source `kinematics.so`. Replacing IK. |
| **2. Hunter node** | `HUNT → NAME → FOLLOW (≤60 s) → HUNT`. Pluggable detect, speak name, PD + crab + LiDAR, halt that *stands*, slow pan “head” | Vendor `apriltag_track` / `lidar_app`. Bare `/cmd_vel`. Zero Twist as “stop”. A looping `.d6a` “spider dance.” |
| **3. Target plugs** | Saveli = AprilTag (first, no train). Cat = COCO YOLO class 15 on existing `yolov8n.onnx` (v2 train only if that misses) | A second DNN node. Training on the Jetson as a gate. |

Product 1 is the walk. Product 2 is the spider that uses the walk. Product 3 is **who** she hunts. The clipboard glued walk + target as `custom_gait_node` publishing servos. Split at the **wire**: hunter publishes velocity; our generator owns the feet; `StepController` still owns the servos; each target is a `TargetSource`.

---

## Verdict

**Hunter is feasible.** Saveli (rigid body + rear AprilTag, joystick in Peter’s hands) is the right **first** plug. Cat is the next plug, not a second node. A person is still a worse first target than either.

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
| Three nodes in series: tracker → lidar_safety → pid → gait | **Wrong shape** | Safety is an **override**, not a pipe stage. One C++ **hunter** node + plugs (`apriltag_ros` and/or in-process YOLO) + the gait **hook** in `controller`. |
| Match Saveli’s Mecanum strafe with `vy` | **Not first** | Masha *can* `linear.y` (±0.10 m/s). First tests: `vx` + `ωz` only. Strafe-out-of-FOV is a lost-target halt, not a crab match. Later, if the custom gait’s map includes `vy`, we can add it as a character choice. |

Do **not** mix this with Sprint 2 `follow_the_cat_node`, Sprint 3 `masha_interaction_node` FOLLOW, vendor `lidar_app`, Nav2, or joystick walk. Two publishers on `/controller/cmd_vel` **and** two writers on arm servos 19/22 fight. Reuse Sprint 2 **gaze math** and the YOLO ONNX loader inside the hunter; do not launch that node. The **gait library** (`follow_gait.hpp` + `FollowGaitGenerator`) is shared with Sprint 3 Step 2; the hunter is the walk client.

---

## Learner comments (standing rule)

Same rule as Sprints 2 and 3. Headers, `.cpp`, tests, launch, yaml, and the Python generator: comments a Java/Python learner can learn from.

- **Why, not restating the line.** Why zero Twist walks in place, why HUNT pans and FOLLOW only *nudges* gaze, why LiDAR metres and camera metres are not the same origin, why `gait=5` must never enter `kinematics.set_step_mode`.
- **Formulas next to code.** Name frames (`rgb_camera_link` / optical, `base_link`, `lidar_frame`) and units (m, rad, mm, pulse). Quintic `sigma(tau)` next to the swing sampler. Support triangle next to the phase split.
- **Masha specifics.** Real topics, `ROS_DOMAIN_ID=27`, clamps, QoS, gait numbers (`1` ripple, `2` tripod, `5` ours, `12`/`15` as `cmd_vel` selectors, `-2` halt).
- **Concurrency.** Image callback stores pointers; control timer owns PID + publish; never OpenCV on the DDS callback. The 20 ms gait thread already exists — we do not add another.
- **Safety.** Halt with Traveling. Lost target → halt + HUNT sweep (not a march). LiDAR floor → halt. 60 s follow cap. Watchdog if commands go quiet. Stance triangle contains CoM projection at every sample. Never OpenCV or `aplay` on the DDS callback.
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

We **skip this sampler for hunter follow.** We still use:

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

Before the first follow Twist, publish Traveling `gait: 15` once (height ~25–30 mm, `time` ~0.6–0.8 s) so `cmd_gait=5` / `cmd_height` / `cmd_period` are defined. Default `cmd_gait` is 2 if nothing was sent — that would silently use the vendor tripod. The hunter must not forget this select.

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

**Saveli plug:** prefer **stock `apriltag_ros`** for pose (PnP with `size` in metres). Do not launch vendor `apriltag_recognition` at the same time.

**Cat plug:** same `proud_up/models/yolov8n.onnx` Sprint 2 already serves for COCO **person** (class 0). Filter class **`cat` (15)**. Do not launch vendor `yolo_detect`. See [Target plugs](#target-plugs).

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

This is the same engine Sprint 3 Step 2 named `FollowGaitGenerator`. Write it **once**. Hunter (Saveli plug, later cat plug) and Sprint 3 person-follow are *clients* of that generator. Do not create `gait=6` unless Peter later wants a *second* character that must run at the same time as this one (it cannot: one generator slot).

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

### Three map variants (Peter picks one)

Same injection point for all three: `FollowGaitGenerator`, `gait=5` / select `15`, vendor IK, swing quintic, rolling cycle, halt `-2`. The choice is the **map** (which feet, what body motion), not a second servo node. Implement **one**. Do not add `gait=6`.

Shared, not optional:

- Cycle clock keeps rolling (no rest-to-rest on the whole step).
- Swing only: `sigma(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5`. Because `sigma-dot = 0` at lift and plant, `x-dot` and `z-dot` are ~0 there even on a spatial parabola. Vendor `s = t/T` stomps.
- Support: at every 20 ms sample the grounded feet contain the CoM projection.
- First live `vx` cap 0.05 m/s. Lift start 25–30 mm.

| | **1. Quintic-swing tripod** | **2. Wave (one leg up)** | **3. Omnidirectional tripod** |
|---|---|---|---|
| What you see | Same 3+3 as vendor, but she *kisses* the floor | Insect: one foot in the air, five down | Same 3+3, but she can **crab** |
| Grouping | A = LF, LR, RM; B = LM, RR, RF | Legs 1→6 in 60° steps | Same as 1 |
| Duty (time in air) | ~50% | ~15–25% | ~50% |
| Command | `vx`, `ωz` | `vx`, `ωz` | `vx`, `vy`, `ωz` |
| Book lesson | Ch. 9 **clock** vs vendor metronome | Duty factor + large support polygon | Planar **twist** → six AEP/PEP |
| Follow-Saveli fit | Polite copy of today’s walk | Best for 0.80 m standoff + reverse (Test 4) | Best match to a Mecanum car |
| Risk | Low | Low–medium (phase table) | Medium (no-slip stance math) |
| “That’s ours” from across the room | Weak unless you watch the feet | Strong | Strong when Saveli strafes |

**Pick (locked):** **3 — omnidirectional tripod.** Peter, 2026-09-16, [this heading](#variant-3--omnidirectional-tripod). Variants 1 and 2 stay as rejected maps, not extra generators. Full math, crab policy, and tests: [Variant 3 design](#variant-3-design-locked).

#### Variant 1 — Quintic-swing tripod

Vendor skeleton, our clock.

- `φ ∈ [0, π)`: group A swings, B stance. Then they swap.
- Swing foot: linear PEP→AEP in `xy`, lift in `z`, progress `sigma(tau)` quintic.
- Stance: retract at constant `z` (ground). Cycle does **not** pause at wrap.

Why it is interesting: cleanest A/B against Traveling `gait=2`. Same triangle, different time law. That is the TRAJECTORY.MD gap.

Why it may bore: from two metres away she still “looks like Masha walking,” only quieter. Following Saveli she does not gain a new skill (no crab, no extra stability).

Period start ~0.6–0.8 s.

#### Variant 2 — Wave (one leg up)

Different **who moves**, same clock.

- Six slices of 60°. One leg swings; five stay down. Support polygon is almost the whole body.
- Swing still quintic. Period longer (~1.0–1.2 s). Top speed lower for the same stride — fine at 0.05 m/s.
- **Not** vendor ripple. Ripple in the `.so` is ~32% swing (overlapping pairs). This is pentapedal wave.

Why it is interesting: you can count the walk. Best when the car may reverse into the LiDAR wall (Test 4): always a huge polygon, small unfinished-step overshoot. Closest to “stalking follow.”

Cost: six phase windows, not two. Turns in place are slower. If Saveli drives off at normal speed she will lag more than a tripod.

#### Variant 3 — Omnidirectional tripod

**This is the map we implement.** Different **body math**, same 3+3 grouping.

Clipboard Phase 1 was “live `(vx, vy, ωz)` → feet.” Vendor `CmdVelGenerator` already does that inside the `.so`. We write it in the open:

- Body planar twist over stance time `Ts = T/2`: exact `SE(2)` exponential (linearized check in gtest).
- Stance foot is fixed on the floor, so in `base_link` it traces the **inverse** rigid motion. AEP/PEP are the ends of that track (no-slip).
- Swing is the quintic bridge PEP→AEP, including the **lateral** gap, not only `x`.

Crab is **not** a second gait. The generator always accepts `vy`. The hunter **turns `vy` on** when the target is already ahead and slides left/right, so Masha sidesteps and stays in one line instead of yawing after it. See [Crab policy](#crab-policy-hunter-not-gait). First live crab is the **Saveli** plug (Mecanum). Cat plug keeps `enable_crab` false until that is boring.

Cost: wrong AEP/PEP signs scuff or trip. First live gait tests keep `vy = 0` until `vx + ωz` is boring (Phase G2), then a hand `vy` Twist, then the hunter crab gate.

#### Not a variant (knobs on the chosen map)

Do not spend a fourth generator on these:

- **Bounce / body bob** — stance-`z` amplitude on variant 1 or 3.
- **High-step / stalking period** — `height` and `T`, same map.
- **Rest-to-rest quintic on the whole cycle** — point-to-point, not a walk (lurch–pause). Correct for a single creep step; wrong as a cyclic gait.

---

## Variant 3 design (locked)

Status: design only. Do **not** implement until Peter says to build. Package split stays: C++ math in `proud_up/follow_gait.hpp` (no rclcpp) + gtest; Python `FollowGaitGenerator` copies the same formulas; `StepController` consumes the generator; hunter publishes Twist including `vy`.

### What “omnidirectional” means here

The body command is a **planar twist** in `base_link`:

```text
V = (vx, vy, ωz)     # m/s, m/s, rad/s   (generator uses mm/s internally)
```

- `vx > 0` — walk toward the head
- `vy > 0` — crab toward Masha’s **left** (same sign as `base_link` +Y)
- `ωz > 0` — yaw CCW (left turn)

A tripod gait with only `vx` and `ωz` is still a tripod. What we add is the **no-slip map** from this twist onto six AEP/PEP points, including the lateral term. Without that map, `linear.y` on `/controller/cmd_vel` would either be ignored or fall through to vendor `CmdVelGenerator`.

The gait does **not** decide when to crab. It only knows the twist it was given this cycle. When to set `vy ≠ 0` is the hunter policy below.

### Frames, units, nominal feet

Gait frame = `base_link`: **+X head, +Y left, +Z up**, feet at **negative** z. Millimetres inside the generator (same as `move.py`). Twist from ROS is m/s; `StepController.cmd_vel` already multiplies linear speeds by 1000.

Nominal footholds = `DEFAULT_POSE` (`build_in_pose.py`), which is also halt `gait=-2`. Recompute AEP/PEP from the **current** six-foot pose when a cycle table is rebuilt (new Twist, or first start), so a taller stand still works.

| `leg_id` | Name | `DEFAULT_POSE` (mm) | Tripod group |
|---|---|---|---|
| 1 | LF | (+163.6, +140.8, −70) | **A** |
| 2 | LM | (0.0, +183.5, −70) | **B** |
| 3 | LR | (−163.6, +140.8, −70) | **A** |
| 4 | RR | (−163.6, −140.8, −70) | **B** |
| 5 | RM | (0.0, −183.5, −70) | **A** |
| 6 | RF | (+163.6, −140.8, −70) | **B** |

- Swing **A** = LF, LR, RM (1, 3, 5)
- Swing **B** = LM, RR, RF (2, 4, 6)

Each group is a triangle that still contains the body origin in `xy` (static CoM projection with equal masses). That is the support contract.

### Phase clock and duty

Period `T` (Traveling `time` / `cmd_period`), start **0.70 s**. 50 Hz → `N = round(T / 0.02)` beads, `φ_k = 2π k / N`.

Duty **0.5** (classic tripod). Stance time `Ts = T / 2`. No extra all-six dwell at wrap (that would pause the cycle clock).

```text
φ ∈ [0, π)     group A SWING,  group B STANCE
φ ∈ [π, 2π)    group A STANCE, group B SWING
```

At the two switch samples (`φ = 0` and `φ = π`) all six feet are at `z = z_gnd` (AEP of the landing group, PEP of the lifting group). Discrete 20 ms may hold that for one tick. That is support, not a rest-to-rest walk.

Cycle phase **does not** go to zero speed at wrap. `last_part` is true when `k` wraps `N → 0`, so `StepController` may splice a new Twist.

### No-slip AEP / PEP (the actual map)

A stance foot is **fixed in the world**. In `base_link` it must therefore move as the inverse of the body’s `SE(2)` motion.

Integrate a **constant body-frame twist** `(vx, vy, ωz)` for time `t`, starting at the identity at **mid-stance** (`t = 0`). Lynch & Park: this is the exponential of `se(2)`.

```text
θ(t) = ωz * t

if |ωz| < ω_eps:                    # ω_eps start 1e-6 rad/s
    trans(t) = (vx * t,  vy * t)
else:
    trans_x = ( vx * sin(θ) - vy * (1 - cos(θ)) ) / ωz
    trans_y = ( vy * sin(θ) + vx * (1 - cos(θ)) ) / ωz
```

A world-fixed foot that sits at body point `p0 = (x0, y0)` at mid-stance:

```text
p_body(t) = R(-θ(t)) * (p0 - trans(t))
# R(α) is 2D yaw:  (x,y) → (x cosα - y sinα,  x sinα + y cosα)
```

Name the ends of stance (same for every foot, using that foot’s own `p0`):

```text
AEP = p_body(-Ts / 2)     # just landed; start of stance
PEP = p_body(+Ts / 2)     # about to lift; end of stance
z_AEP = z_PEP = z_gnd     # p0.z of that foot (nominal ground)
```

**Sign check (gtest, not comments-only):**

| Command | AEP of LF relative to `p0` |
|---|---|
| `vx > 0`, `vy = ωz = 0` | **+X** (ahead of the hip) |
| `vy > 0`, `vx = ωz = 0` | **+Y** (to the left) |
| `ωz > 0`, `vx = vy = 0` | right-side feet (`y0 < 0`) get extra **+X** (longer step on the outside of a left turn) |

Linearized small-angle check (`|ωz| Ts ≪ 1`), used only as a gtest against the exponential:

```text
v_hip = ( vx - ωz * y0,  vy + ωz * x0 )     # velocity of the hip in the world, body coords
AEP ≈ p0 + 0.5 * v_hip * Ts
PEP ≈ p0 - 0.5 * v_hip * Ts
```

Stride in body frame during stance is `AEP − PEP ≈ v_hip * Ts`, **not** `v * T`. Vendor `cmd_vel_basic_data(80 mm/s, 0, 0, T=1)` reports an AEP **offset** of 40 mm so `AEP − PEP = 80 mm = vx T`. That is a factor-of-two vs no-slip with `Ts = T/2`. We do **not** copy that factor. If the real robot undershoots tape-measure odom, add a measured `linear_factor` later (default 1.0), same name as `CmdVelParams`.

**Workspace clamp:** if `|AEP − p0|` or `|PEP − p0|` exceeds `stride_max` (start **40 mm**; vendor traveling examples use ~40–65 mm), scale that foot’s offset down, preserving direction. At follow caps (`vx=0.05`, `vy=0.04`, `T=0.7`, `Ts=0.35`) half-stride is ~9–18 mm — well inside.

### Stance path (on the ground)

Group in stance, `τ_s ∈ [0, 1]` linear in cycle time (not quintic — the body must keep moving):

```text
# group B stance while φ ∈ [0, π):   τ_s = φ / π
# group A stance while φ ∈ [π, 2π):  τ_s = (φ - π) / π

xy(τ_s) = (1 - τ_s) * AEP_xy + τ_s * PEP_xy
z       = z_gnd
```

`d(xy)/dt` at mid-stance is about `−v_hip`. Cycle wrap must **not** zero this.

### Swing path (in the air) — quintic clock

Group in swing, `τ ∈ [0, 1]` linear in that half-period, then Chapter 9 time scaling **on the swing only**:

```text
sigma(τ) = 10 τ^3 - 15 τ^4 + 6 τ^5     # σ(0)=0, σ(1)=1, σ-dot=σ-ddot=0 at 0 and 1

xy = (1 - sigma) * PEP_xy + sigma * AEP_xy     # includes lateral gap when vy ≠ 0
z  = z_gnd + 4 * sigma * (1 - sigma) * h       # h = lift height, start 25 mm
```

Because `σ-dot(0) = σ-dot(1) = 0`, both `x-dot` and `z-dot` are ~0 at lift-off and touchdown even though the spatial parabola `4s(1-s)` has a nonzero `dz/ds` at the ends. That is the whole point vs vendor linear `s = t/T`.

`h` is **lift above ground**, not stance compression. Bounce stays a later knob (`z_gnd` wobble), off for G0–G2.

### One sample

`sample(φ, V, h, T, nominal_pose) →` six `(x,y,z)` plus six stance flags.

1. `Ts = T/2`. For each leg, AEP/PEP from `p0` and `V`.
2. Classify swing/stance from `φ` and group.
3. Fill `xy,z` from the two paths above.
4. Invariant: every stance foot `z` within contact band of min stance `z` (start 3 mm). CoM projection (equal mass at six hips, or body origin as the cheap proxy) inside the stance triangle.

C++ owns this function. Python generator calls the same math (port, with a comment pointing at the header) and yields one bead per 20 ms. Do not call `kinematics.cmd_vel_new_point` / `set_step_mode`.

### Generator hook (unchanged socket)

`FollowGaitGenerator` in `move.py`, same handshake as `CmdVelGenerator` (`status` `first` / `finish` / run, `last_part` at wrap, splice by `total_dist_sq` so feet do not teleport when Twist changes).

`step_controller.py`:

- `gait == 15` → store `cmd_gait = 5`, `cmd_height`, `cmd_period` (no walk)
- `gait == 5` → this generator, **never** `MovingGenerator` / `set_step_mode(..., 5)`
- `cmd_vel` when `cmd_gait == 5` → this generator with `velocity_x, velocity_y, angular_z`

IK remains `kinematics.set_leg_position` inside `set_pose_base`.

### Crab policy (hunter, not gait)

Peter’s assumption is right, with a gate so crab does not fight a large heading error.

**“In one line with Saveli”** means: Masha stays **directly behind him**, on his rear centerline, heading roughly parallel — not “turn until the tag is on my nose and walk at him.”

AprilTag pose in `base_link` after TF (tag origin; ignore z for the walk):

```text
p = (x, y)              # m, +x ahead of Masha, +y to her left
θ = atan2(y, x)         # bearing to the tag
ψ = yaw of Saveli in base_link   # from tag orientation; 0 = he faces the same way we do
```

Tag is on the **rear**, facing Masha. Saveli’s forward is opposite the tag face. Measure once on the car and put the yaw offset in yaml (`tag_yaw_to_savelij_heading`). If `ψ` is garbage on day one, treat `ψ = 0` and use Masha-frame `y` only.

Errors when we already share a heading (`ψ ≈ 0`, the strafe case):

```text
e_x = x - r_target      # too far / too close
e_y = y                 # he is left (+) or right (−) of our nose
e_θ = θ                 # bearing; small if we are already lined up
```

When Saveli **strafes** (Mecanum left/right, little yaw): `x` stays ~`r_target`, `y` grows, `θ` grows only as `atan2(y, x)`. The correction that keeps the original line is **`vy`, not `ωz`**. If we only yaw, we face him and then walk, and we are no longer behind him on his centerline.

**Gate** (start values):

```text
θ_crab    = 15°         # already roughly facing him
y_on      = 0.06 m      # start crabbing
y_off     = 0.03 m      # hysteresis, stop crabbing
vy_max    = 0.04 m/s    # first live; move_controller also clamps 0.10
```

```text
# always (FOLLOW, after lidar / lost checks)
vx = clamp( Kp_x * e_x + Kd_x * ė_x,  -vx_max, +vx_max )
ωz = clamp( Kp_θ * e_θ + Kd_θ * ė_θ,  -wz_max, +wz_max )

if |θ| < θ_crab and |y| > y_on:          # he slid left/right while we still see him ahead
    crab = true
elif crab and (|θ| > θ_crab or |y| < y_off):
    crab = false

if crab:
    vy = clamp( Kp_y * e_y + Kd_y * ė_y,  -vy_max, +vy_max )
    # optional: shrink ωz while crabbing so we do not turn off the line
    ωz *= 0.3
else:
    vy = 0
```

So crab is **activated when Saveli moves left or right and we are already looking at him**. It is not activated when he is 40° off to the side (that is Test 1: turn in place). Lost tag / LiDAR latch: `vy = 0` and `halt_legs()`, then HUNT sweep (lost) or STOPPED (lidar).

Do **not** command `vy` from `θ` itself (`vy = Kp θ`). `θ` is a heading; `y` is the line offset. At 0.80 m, `θ = 5°` is only 7 cm of `y`.

Deadband (hunt):

- `|e_x| < 0.05 m` → `vx = 0`
- `|e_y| < 0.03 m` → `vy = 0`
- `|e_θ| < 5°` → `ωz = 0`
- all three quiet → `halt_legs()`, do **not** stream `Twist{}`

Later (not G0): rotate `(e_x, e_y)` by `ψ` so the “line” is Saveli’s heading even if we are slightly yawed. First live uses Masha-frame `y` under the `|θ| < θ_crab` gate — that **is** his line when we are already facing him.

### Parameters (start)

| Name | Start | Where |
|---|---|---|
| `T` / `cmd_period` | 0.70 s | Traveling `gait: 15` |
| `h` / `cmd_height` | 25 mm | same |
| `Ts` | `T/2` | gait math |
| `stride_max` | 40 mm | gait clamp |
| `vx_max` | 0.05 m/s | hunter |
| `vy_max` | 0.04 m/s | hunter; 0 until crab gate |
| `wz_max` | 0.3 rad/s | hunter |
| `r_target` | 0.80 m | hunter |
| `θ_crab` | 15° | crab gate |
| `y_on` / `y_off` | 0.06 / 0.03 m | crab hysteresis |
| `enable_crab` | **false** until Test G2 `vy` is boring | yaml; Saveli plug first |
| `follow_max_s` | 60 | hunter; then back to HUNT |
| `search_timeout_s` | 20 | sticky id while scanning |

### Bring-up order for this map

1. **G0** — `follow_gait.hpp` + gtest (no robot). Includes `vy > 0` → AEP `+Y`, `ωz > 0` → outside foot longer, swing `z-dot ≈ 0`, stance speed at wrap ≠ 0, CoM in triangle at `φ = 0, π`.
2. **G1** — hook, Traveling `gait: 5`, `stride 0` / `V = 0`: tripod in place, soft landings, halt `-2`.
3. **G2a** — `gait: 15`, `vx = 0.03`, halt; then `ωz` only, halt.
4. **G2b** — `vy = 0.03` only, halt. She must **translate left**, not yaw. This is the gait proof of crab. Tape a line on the floor.
5. **H2** — `enable_crab:=false`. Tests 1–2 (`vx`, `ωz`). 60 s cap. Lost → HUNT sweep.
6. **Test 3b** — `enable_crab:=true` (Saveli plug). Drive Saveli **sideways**. Masha sidesteps, stays behind him. Cover the tag → HUNT sweep, not a march.

### Failure modes (this map)

| Symptom | Likely cause |
|---|---|
| Crab is a spin | `vy` sign flipped, or hunter is writing `ωz` from `y` instead of `vy` |
| Scuff / trip on turn | AEP/PEP rotation term sign (`ωz × r`) flipped |
| March in place after alignment | streaming `Twist{}`; need `halt_legs()` |
| Looks like vendor tripod, stomps | fell through to `CmdVelGenerator` (`cmd_gait` still 2; forgot `gait: 15`) or linear `s` on swing |
| `.so` panic / nonsense | `gait=5` passed into `set_step_mode` |
| Two servo writers | a “custom gait node” on `/servo_controller` |

### C++ surface (when implementing)

```text
# include/proud_up/follow_gait.hpp  (no rclcpp)

sigma(tau) -> double                         # quintic, tau in [0,1]

se2_exp(vx, vy, wz, t) -> (tx, ty, theta)    # mm, rad; vx,vy in mm/s

aep_pep(p0, vx, vy, wz, Ts) -> (AEP, PEP)    # Foot mm

sample(phi, cmd, nominal_pose) ->
    feet[6], stance[6]
```

gtest names the invariant in the test title: `VyPositiveMovesAepLeft`, `WzPositiveLengthensRightStride`, `SwingZDotZeroAtTouchdown`, `StanceSpeedNonzeroAtWrap`, `TripodBDownAtPhiZero`.

---

## Corrections vs the clipboard plan

| Clipboard | Masha | What we do |
|---|---|---|
| `/cmd_vel` | `/controller/cmd_vel` | Publish Twist there. |
| Zero Twist = stop | Zero Twist = walk in place | `halt_legs()`: Traveling `gait=0` then `gait=-2`. Same helper as `follow_the_cat_node`. |
| `custom_gait_node` @ 50 Hz → `/servo_controller` | `StepController` already owns that wire | **Write the gait. Do not write a servo publisher.** |
| Replace “string of beads” gait | Beads = linear clock on the vendor AEP/PEP curve | **Replace the clock and the map** inside a new generator. Keep the 20 ms sampler. |
| `camera_link` | `depth_cam_link` / `rgb_camera_link`, parent `link4` | **HUNT** pans id 19 (there is no neck). FOLLOW nudges gaze. TF detections into `base_link` (joint states update). |
| `r = √(x²+z²)`, `θ = atan2(x,z)` | Optical: X right, Z forward. Body: X forward, Y left | Optical only inside the detector. Control: `r = hypot(x,y)`, `θ = atan2(y,x)` in `base_link`. |
| `r_target = 0.60`, stop `0.50`, go `0.55` | Halt delay + lidar offset + body ~0.4 m wide | Start: `r_target = 0.80 m`, `d_stop = 0.55 m`, `d_go = 0.70 m` (all `base_link`). First `vx` cap **0.05 m/s**. |
| P-only `vx = Kp er`, `ωz = Kp eθ` | Hexapod lags a full step | PD + deadband. No I on first bring-up (windup during the unfinished step). |
| Three nodes in series | Safety must override | One hunter node: plugs, PID, lidar, halt, watchdog, 60 s cap. Gait lives in `controller`, selected by `gait=5` / `15`. |
| 60° LiDAR sector | Vendor follow uses 90° | Parameter, default **90°**. Ignore non-finite / `< 0.2 m`. |
| Test 3: match Mecanum strafe | Variant 3 gait accepts `vy`; hunter **gates** it | G2b: hand `vy` Twist. H2: `vx`,`ωz`. Test 3b: crab when `|θ|<15°` and `|y|` grows. Lost → HUNT sweep. |

---

## Architecture (corrected)

```
enabled_targets yaml:  [saveli]           later: [saveli, cat]

  Saveli plug                         Cat plug
  apriltag_ros 36h11                  yolov8n.onnx COCO class 15
  pose in camera → TF                 bbox + depth/heuristic → TF
           \                               /
            \                             /
             ▼                           ▼
         TargetHit { id, spoken_wav, pose_in_base, pixel }
                           │
                           ▼
masha_hunter_node   (NEW, package proud_up)
  IDLE:   halt, hunter_pose, wait ~/start
  HUNT:   halt + slow pan id 19; optional wander (open → walk, wall → halt+turn)
          no map / no Nav2; any enabled plug can fire
  NAME:   halt, hold gaze, aplay spoken_wav once (worker thread)
  FOLLOW: gait 15 + PD Twist, gentle gaze, sticky id, ≤ 60 s
          LiDAR latch / lost / timeout → halt → HUNT (sticky search)
                           │
                           ▼
/controller
  cmd_gait==5  →  FollowGaitGenerator @ 50 Hz   (SE(2) AEP/PEP + swing quintic)
                           │
                           ▼
StepController  →  kinematics.set_leg_position (IK only)  →  /servo_controller
```

Saveli is a **separate** computer. Do not assume a shared ROS domain. Masha only *sees* the tag (or the cat).

Do **not** start:

- `follow_the_cat_node`
- `masha_interaction_node` in FOLLOW
- `lidar_app` (`app/lidar_controller.py`)
- Nav2
- `apriltag_recognition` (vendor example)
- joystick analog walk (unplug the pad or leave sticks centered)

Voice stack may stay up. “Stop” via existing voice halt is a plus if it does not also emit a competing Twist; first tests: no voice walk commands.

---

## Hardware (Peter)

Gait dry-run (G0–G2) needs no target. Hunter HUNT sweep needs no target. Saveli plug needs the tag. Cat plug needs a cat (or a printed photo on the bench).

**Saveli (first plug):**

1. Print **tag36h11 id 0** (or another id, then set yaml). Measure the **black-square edge** in metres. Clipboard-era yaml in `apriltag_ros` defaults to **0.08 m** — do not trust that; measure.
2. Mount it **vertical, facing backward**, on Saveli’s rear, as high as the Aurora sees at 0.6–1.5 m during the pan sweep (not only at rest 19=500). Not on the floor. Not at an angle that foreshortens to a line.
3. Lighting: AprilTag wants contrast. Avoid pointing the tag at a window behind Masha.
4. Keep Saveli’s rear clear of cables that cover the tag when it turns.

**Arm (there is no neck).** Camera is on `link4`. “Rotating head” = **id 19 pan**. Optional small **id 22 tilt**. Hold 23 and 24 at 500. **Do not** command servo 23 as tilt (wrist yaw). Sprint 2 rest snapshot (centre of the sweep):

```
19=500, 20=810, 21=180, 22=150, 23=500, 24=500
```

Pan window start ~200–800 (same clamps as `follow_the_cat`). Sweep period ~8–12 s. TF `base_link` → camera **is valid while panning** because `robot_state_publisher` follows `/joint_states`. The old “bolt the camera forward for the whole follow” rule is **dropped**. HUNT must pan. FOLLOW only *nudges* gaze to keep the hit centered (`integrate_gaze`).

**Deep hunter pose:** halt `gait=-2` → `DEFAULT_POSE` (body ~70 mm). That is already a low hexapod stand. Yaml `hunter_pose` (built-in name) if Peter later wants a deeper crouch. **Not** a looping `.d6a` (action groups fight the gait).

---

## Frames and math (hunter walk)

Optical (ROS camera, `rgb_camera_link`): **X right, Y down, Z out of the lens.**  
Clipboard `r = √(x_s² + z_s²)`, `θ = atan2(x_s, z_s)` is **this** frame. Fine as a detector local, wrong as a walk command.

Body (`base_link`): **X forward (head), Y left, Z up.** Same as the gait pose.

After `tf2` of the tag origin into `base_link`:

```text
x, y              # metres, ignore z (tag height)
r     = hypot(x, y)
θ     = atan2(y, x)                 # rad, +θ = tag to Masha’s left
e_x   = x - r_target                # along our nose (≈ range when |θ| small)
e_y   = y                           # left/right of our nose — crab error
e_θ   = θ
```

PD (first bring-up; I-term off). Crab gate is in [Variant 3 design](#crab-policy-hunter-not-gait); `enable_crab` default **false**.

```text
vx = clamp( Kp_x * e_x  + Kd_x * ė_x,  -vx_max, +vx_max )
ωz = clamp( Kp_θ * e_θ  + Kd_θ * ė_θ,  -wz_max, +wz_max )
vy = 0
# if enable_crab and |θ| < θ_crab and |y| > y_on:
#     vy = clamp( Kp_y * e_y + Kd_y * ė_y,  -vy_max, +vy_max )
```

Deadband so she does not hunt:

- if `|e_x| < 0.05 m` → `vx = 0`
- if `|e_y| < 0.03 m` → `vy = 0`
- if `|e_θ| < 5°` → `ωz = 0`
- if all three quiet, **halt**, do not stream zeros.

First live caps: `vx_max = 0.05`, `vy_max = 0.04`, `wz_max = 0.3`. Software clamp is extra; `move_controller` also clamps 0.12 / 0.10 / 0.6.

When `vx`, `vy`, and `ωz` are all commanded zero for more than one control tick **and** we are at the setpoint: `halt_legs()`. Publishing `Twist{}` every 10 Hz is how the vendor demos leave her marching.

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
| `lost_timeout` | 0.5 s | no hit → halt + HUNT |
| `walk_watchdog` | 0.3 s | no control tick → halt |
| `follow_max_s` | 60 s | even if still tracking → HUNT |
| `search_timeout_s` | 20 s | sticky id while scanning |

`r_target` must sit **above** `d_go`, otherwise the PID drives her into the latch. Clipboard 0.60 / 0.50 / 0.55 violates that once halt delay is included.

Lost target: `halt_legs()`, enter **HUNT**, sticky to the last id until `search_timeout_s`. Default HUNT is **stand + pan** (head sweep, not a walking spin). Optional **wander** (below) may then walk into open space; still no SLAM.

Depth sanity (optional, after H2): sample `/depth_cam/depth/image_raw` at the tag/box centre. If `|z_depth - z_pnp|` is large, trust lidar for range and the plug for bearing, or halt.

---

## Node design (hunter)

**Package:** `proud_up` (no new ament package).  
**Executable:** `masha_hunter_node`.  
**Launch:** `ros2 launch proud_up masha_hunter.launch.py` after slim bringup — do **not** run `~/.stop_ros.sh` to start it.

Split:

- `include/proud_up/hunter.hpp` + `src/hunter.cpp` — no rclcpp: PD, deadband, lidar hysteresis, crab gate, 60 s cap, wander (open / stop / rotate), phase helpers. Fake poses and fake scans in gtest.
- `include/proud_up/target_source.hpp` — `TargetHit` + virtual `TargetSource`.
- `src/target_saveli.cpp` — AprilTag array + TF → `TargetHit` (`id = saveli`).
- `src/target_cat.cpp` — YOLO class 15 + pixel ray / depth → `TargetHit` (`id = cat`). Reuse Sprint 2 ONNX load path; do not copy-paste a second net if one net can filter two classes.
- `src/masha_hunter_node.cpp` — subscriptions, TF, timer, arm sweep, `aplay` worker, publishers, services.
- `test/test_hunter.cpp` — offline gtest (including “NAME once”, “60 s → HUNT”, crab, halt ≠ zero Twist).
- `launch/masha_hunter.launch.py` + `config/masha_hunter.yaml`.
- Launch starts `apriltag_ros` **only if** `saveli` is in `enabled_targets`.

**QoS:** image, camera_info, depth, scan: `KeepLast(5)` + `best_effort()`, copy `follow_the_cat_node`.

**Phases:** `IDLE → HUNT → NAME → FOLLOW → STOPPED` (and HUNT on lost / timeout).

| Phase | Legs | Head (arm) | What happens |
|---|---|---|---|
| **IDLE** | `halt_legs()` | `hunter_pose` centre (19=500, …) | Wait `~/start`. No sweep. |
| **HUNT** | Default: halt. If `enable_wander`: walk into open, halt at `d_stop`, then rotate (see wander rule). Deep stand `DEFAULT_POSE` unless yaml `hunter_pose`. | Slow pan id 19 first, optional tilt 22, ~8–12 s period, ~200–800. Chassis yaw only after pan window or a wall. | Any **enabled** plug can fire. No map. |
| **NAME** | Halt. | Hold the pose that sees the hit. | `aplay` `spoken_wav` **once**, worker thread, Sprint 3 pattern. Skip re-name if re-lock within `search_timeout_s`. |
| **FOLLOW** | Traveling `gait: 15` then Twist. Sticky `id`. | Gentle `integrate_gaze` (no wide scan). | ≤ **`follow_max_s = 60`**. LiDAR latch → `STOPPED`. Lost / timeout → halt → **HUNT**. |
| **STOPPED** | Halt. | Hold gaze. | `d_min > d_go` and time left → FOLLOW; else HUNT. |

On entering `FOLLOW` the first time (and after a halt that cleared `cmd_gait`): publish Traveling `gait: 15`.

Services: `~/start`, `~/stop` (`std_srvs/Trigger`). `enable_walk` **default false**. `enable_wander` **default false**. `enabled_targets: [saveli]` first. Gait dry-run does not use this node.

Debug: overlay `~/image_result` (box, id, `r`, `θ`, `d_min`, phase, follow time left, wander action). No `cv2.imshow`.

---

## Optional wander rule (HUNT, no SLAM)

Peter: no mapped room required. A simpler search is enough: **go into the open, stop before an obstacle, rotate, look again.**

This is **not** Nav2, AMCL, or an occupancy grid. The plan already forbids launching Nav2 (it would fight `/controller/cmd_vel`). LD19 `/scan` in `base_link` is the whole world model.

**Default `enable_wander: false`.** H0–H2 stay stationary HUNT (halt + pan). Turn wander on only after Test H0 (pan) and Test G2 (our gait under `cmd_vel`) are boring.

### What it does

Only while phase **HUNT** (and after `~/start`). FOLLOW still uses the target PD + LiDAR latch. IDLE / NAME / STOPPED do not wander.

```text
# each hunter timer tick, HUNT, enable_wander, enable_walk
# d_min = min finite range in the front sector, in base_link (same as the follow latch)

1. Arm pan always runs (cheap look). Plug hit → NAME immediately (halt).

2. If d_min < d_stop:
      halt_legs()                         # not Twist{}
      if arm pan has not finished this sweep: keep panning, vx = ωz = 0
      else: ωz = ±wander_wz               # turn toward the more-open half of /scan
      # after heading changes enough that d_min > d_go: go to 3

3. If d_min > d_go:
      ωz = 0
      vx = wander_vx                      # walk into the open, cap start 0.03 m/s
      # still panning

4. Else (between d_stop and d_go): hold last action (hysteresis, no chatter)
```

**Pan first, body yaw second.** Id 19 already covers a wide cone. Do not start a walking spin while the camera can still look. Chassis `ωz` is for “wall ahead and the arm has finished a sweep” or “target was last seen off to the side outside the pan window.”

**Halt, then turn.** Streaming `Twist{}` marches in place. Same `halt_legs()` as everywhere else, then a non-zero `ωz` on gait 15 if we actually need to yaw the body.

**Open heading:** on a wall hit, pick the scan half (left vs right of +X) with the larger median finite range; `ωz` sign toward that half. No map of “I have been here.” She can loop a table. Fine for a living-room demo.

### What it does not do

| Tempting extra | Why not |
|---|---|
| SLAM / Nav2 / “go to kitchen” | Different project. Two `cmd_vel` sources. |
| Walk in a circle to search | Head pan already does that cheaper; walking spin is a gait and a trip hazard. |
| Zero Twist as “pause at the wall” | Walks in place. |
| Wander during FOLLOW | FOLLOW is sticky tracking; LiDAR there is a **stop**, not a detour. |

### Parameters (start)

| Name | Start | Role |
|---|---|---|
| `enable_wander` | **false** | yaml; HUNT only |
| `wander_vx` | 0.03 m/s | into open space (`d_min > d_go`) |
| `wander_wz` | 0.25 rad/s | after halt at `d_stop`, toward open half |
| `wander_sector` | 90° | same front sector as follow LiDAR unless yaml splits them |

gtest (no robot): fake scan all-clear → `vx`; fake wall ahead → halt then `ωz` toward the open side; `enable_wander false` → never `vx` in HUNT; never emit `Twist{}` as the wall action.

---

## Target plugs

```text
TargetHit:
  id            # "saveli" | "cat"
  display_name  # "Saveli" | "cat"
  spoken_wav    # basename under feedback_voice/english/
  stamp
  pose_in_base  # x, y, z, yaw if known; walk ignores z
  pixel         # gaze + overlay
  range_ok      # metric (tag) vs estimated (bbox+depth)
  confidence
```

```text
TargetSource (virtual, no rclcpp in the header):
  id()
  enabled()
  detect(...) -> optional<TargetHit>   # timer thread; never OpenCV on DDS callback
```

Yaml:

```text
enabled_targets: [saveli]     # first bring-up; later [saveli, cat]
follow_max_s: 60.0
search_timeout_s: 20.0
enable_wander: false          # optional HUNT: open → walk, wall → halt+turn
wander_vx: 0.03
wander_wz: 0.25
```

Two plugs in one tick: prefer **sticky** id, else yaml order.

**Saveli (first, no training).** Stock `apriltag_ros`, tag36h11, measured size, TF → `base_link`. Crab / `r_target` / LiDAR as in variant 3. Spoken wav: `saveli.wav` (“Saveli”).

**Cat (next).** Same `yolov8n.onnx` already in `proud_up/models/` (COCO 80). Filter class **`cat` = 15**. Range: finite depth at box centre, else bbox-height heuristic. Bearing: pixel ray → TF → `base_link`. `enable_crab` default **false** for this plug until Saveli Test 3b is boring. Spoken wav: `cat.wav` (“cat”).

Do **not** launch vendor `yolo_detect` or a second DNN process.

---

## Training / serving

| Target | Train? | Serve |
|---|---|---|
| **Saveli** | **No.** Fiducial. | `apriltag_ros` + TF |
| **Cat v1** | **No.** COCO `cat` on existing `yolov8n.onnx`. | OpenCV DNN ONNX, CUDA if Sprint 2 path works, else CPU. `imgsz` start 320. |
| **Cat v2** | Only if v1 misses the house cat. **Off-board** fine-tune (not a Jetson training job). Export ONNX (optional TensorRT `.engine` later). Yaml `cat_onnx`. Same `TargetSource`. | Same hunter node, swap the file. |

Do **not** gate hunter on a training pipeline. During Saveli work, grab a few Aurora frames of the cat; if COCO recall is poor, then v2.

Wavs next to Sprint 3 clips: `~/ros2_ws/src/xf_mic_asr_offline/feedback_voice/english/saveli.wav`, `cat.wav`. `aplay -D pulse` on a **worker thread**, timeout, never `system()` on the timer.

---

## Phases (execution order)

**Gait first, then hunter sweep, then Saveli plug, then cat plug.** Clipboard Phase 1 (new gait *node*) stays rejected. Clipboard Phase 1 (new gait *map*) is G0–G2.

### Phase G0 — gait map on paper + gtest (no robot)

Pick is **variant 3**. Write `follow_gait.hpp` for the [locked map](#variant-3-design-locked): (1) `SE(2)` AEP/PEP from `(vx, vy, ωz)`; (2) quintic `sigma(t)` on **swing only**; cycle phase keeps rolling. Invariants in `test_follow_gait.cpp`:

- Grounded feet at each sample contain the CoM projection (equal masses ok in the unit test). Variant 1/3: at \(\varphi = 0\) and \(\varphi = \pi\) the stance triangle is tripod group B then A. Variant 2: five feet down in every 60° slice; the one marked swing is the only one up.
- `vx > 0` moves swing feet toward \(+X\); `wz > 0` rotates the AEP/PEP the correct way. Variant 3: `vy > 0` moves swing feet toward \(+Y\) (body left).
- No foot \(z\) above the contact band while marked stance.
- Swing quintic: at lift-off and touchdown, that foot’s `z-dot` and `sigma-dot` are ~0. Beads on the arch cluster at the ground ends, not at cycle wrap.
- Two cycles concatenated: **body / stance speed at the join is not ~0** (no rest-to-rest walk). A test that wants zero cycle-end `s-dot` is the wrong test.

No robot motion yet. Reading: `ros2_ws/info/TRAJECTORY.MD`.

### Phase G1 — generator hook, in place

`FollowGaitGenerator` in `move.py`. Branches in `step_controller.py` as in “Walk command” above. Precompute one cycle’s beads: even in **phase**, quintic along each **swing**. Yield one bead per 20 ms.

Dry-run: `ros2 topic pub` Traveling `gait: 5` in place (`stride: 0`) — legs cycle, no translation, landings look soft, the body does **not** pause at wrap — then halt `-2`.

If this dry-run falls through to `set_step_mode(..., 5)` the `.so` will do something undefined or nothing useful. Watch the logs; that is the first hardware invariant.

### Phase G2 — our gait under `cmd_vel`, no Saveli

Traveling `gait: 15` once, then small Twists by hand (`vx = 0.03`, then `ωz` only). Confirm: she translates, she turns, halt `-2` stands. Floor clear. Variant 3: after that, `vy` only.

DoD: visibly not vendor walk (softer landings; wave if pick is 2). Halt stands.

### Phase H0 — hunter sweep, no plug

`enable_walk:=false`. `~/start`. Legs `DEFAULT_POSE`. Id 19 pans slowly. No Twist. `~/stop` centres the arm and stops the sweep.

### Phase H1 / old A–B — Saveli plug, legs off

1. Mount tag. Measure edge. Put metres in yaml.
2. Slim bringup. `enabled_targets: [saveli]`.
3. `apriltag_ros` on Aurora RGB. Walk Saveli by hand 0.6–1.5 m, ±30° yaw, **and** through the pan sweep.
4. Confirm TF `base_link` → tag. Print `x,y,θ`. If `r` is nonsense, yaml size is wrong.
5. Lock → **NAME** once (`saveli.wav`). Re-cover within 20 s → no second wav.

DoD: `r` within ~5 cm of tape at 1.0 m; `θ` sign matches “tag on Masha’s left → +θ”; one spoken name.

G0–G2 and H0/H1 can overlap in calendar. Do **not** start H2 until G2 and H1 are green.

gtest: body heading, deadband, lidar hysteresis, crab gate, “zero Twist is not halt”, “NAME once”, “60 s → HUNT”.

`enable_walk:=false`. Logs show commanded `vx`,`ωz` but **no** Twist on the walk topic. Legs stay in `DEFAULT_POSE`.

### Phase H2 / old C–D — follow Saveli, our gait

`enable_walk:=true`. Floor clear. First `vx_max = 0.05`. `follow_max_s = 60`.

1. Node publishes Traveling `gait: 15` once.
2. Tests 1–5 below (lost → **HUNT sweep**, not stand-and-wait only).
3. Raise `vx_max` only after Test 2 is boring.
4. Test 4 LiDAR latch. Halt is a stand, then sweep if still in HUNT.
5. After Test 2: Test 3b crab. After 60 s still tracking → halt + HUNT.

### Phase E — knobs on the chosen variant (after H2, not a gate)

Height, period, optional bounce. Do **not** switch to a second map. Do **not** add a second servo node. Do **not** pass `5` into `kinematics.set_step_mode`.

### Phase H3 — optional wander (after H0 + G2, not a gate)

`enable_wander:=true`, `enable_walk:=true`, no target required. Floor with one clear path and one wall/chair.

1. Open ahead → slow `vx`, arm still panning. No map.
2. Obstacle at `d_stop` → **stand**, then rotate toward the more-open side. Not a march.
3. Plug fires mid-wander → halt, NAME, FOLLOW. Wander does not resume until HUNT.

Skip H3 until stationary HUNT and gait `cmd_vel` are trusted. Still **no Nav2**.

### Phase C0 — cat plug, no walk

`enabled_targets: [cat]` (or `[saveli, cat]` only after sticky-id gtest). `enable_walk:=false`. COCO class 15. Overlay + `cat.wav`. Bench stand-in: a printed cat photo is allowed for C0.

### Phase C1 — cat follow

Same 60 s / lost / LiDAR as H2. `enable_crab` stays false until Saveli 3b is boring. Do not debug both plugs walking in the same run until sticky-id is proven.

---

## Tests

### Gait (Phases G0–G2)

Joystick / pad on Masha unplugged. No Saveli required.

**Test G0 — gtest.** All invariants in Phase G0. Fail the build if swing `z-dot` at touchdown is not ~0, or if cycle-end stance speed *is* ~0.

**Test G1 — in-place cycle.** Traveling `gait: 5`, `stride: 0`. Chosen grouping visible (tripod A/B, or one-leg wave). Soft landings. No body pause at wrap. `gait: -2` → `DEFAULT_POSE`. Repeat. No servo fight (one publisher).

**Test G2 — slow `cmd_vel`.** `gait: 15` then `vx = 0.03` for ~3 s, halt. Then `ωz` only, halt. She must translate / yaw and then **stand**, not march. Variant 3 only, after that is boring: `vy` only, then halt.

### Hunter (no walk)

**Test H0 — sweep.** `~/start`. `enable_wander:=false`. Id 19 moves slowly. Legs `DEFAULT_POSE`. No Twist. `~/stop` centres pan.

**Test NAME.** One `saveli.wav` per lock. Cover and uncover within 20 s → no second wav.

**Test H3 — wander (optional).** `enable_wander:=true`. Open floor → she walks slowly. Chair in front → stand, then turn, not march-in-place. Tag appears → NAME, wander stops. No Nav2, no map file.

### Follow (clipboard Phase 5, corrected)

Joystick on Saveli. Masha’s pad unplugged. `enable_walk` only after H1. Walk is **gait=5**. `enabled_targets: [saveli]`.

**Test 1 — Stationary heading.** Saveli still, offset ~20–30° at ~1 m. Masha turns in place (`ωz` only) until `|θ| < 5°`, then **halt**. She must not step in place after alignment.

**Test 2 — Straight follow.** Drive Saveli forward slowly. Masha holds ~0.80 m. No contact. Cover tag → halt within one step, then **HUNT sweep**.

**Test 3 — Gentle curve.** Saveli yaws. Masha updates `ωz` and `vx`. `enable_crab:=false`: if Saveli **strafes** out of the camera, she **stops** and hunts (pass).

**Test 3b — Crab / one line.** `enable_crab:=true`. Drive Saveli sideways (Mecanum). Masha crabs (`vy`) and stays behind him; she does **not** spin in place to face the tag. If `|θ| > θ_crab` she yaws first. Cover tag → HUNT.

**Test 4 — LiDAR cutoff.** From a follow, drive Saveli **backward** toward Masha. She stands at `d_stop` and stays stood until Saveli opens past `d_go` (if follow time remains). Repeat three times: no chatter, no servo fight, no march-in-place.

**Test 5 — Watchdog / 60 s.** Kill `apriltag_ros` while following → halt + HUNT. Same if the hunter is `Ctrl-C`’d (`on_shutdown` → `halt_legs()`). Still tracking at 60 s → halt + HUNT even if the tag is visible.

**Test C0 — cat.** Overlay + `cat.wav`. No walk.

**Test sticky.** Both plugs enabled, gtest or dry: only the locked id is followed.

Tape-measure the real standoff once. If 0.80 m feels far on this floor, lower `r_target` toward 0.70 **after** Test 4, never below `d_go + 0.10`.

---

## Files to add (when implementing)

**Add in `proud_up` (gait math):**

- `include/proud_up/follow_gait.hpp`
- `src/follow_gait.cpp`
- `test/test_follow_gait.cpp`

**Add in `proud_up` (hunter + plugs):**

- `include/proud_up/hunter.hpp`
- `src/hunter.cpp`
- `include/proud_up/target_source.hpp`
- `src/target_saveli.cpp`
- `src/target_cat.cpp`
- `src/masha_hunter_node.cpp`
- `test/test_hunter.cpp`
- `launch/masha_hunter.launch.py`
- `config/masha_hunter.yaml` (`enabled_targets: [saveli]`, tag size, `r_target`, `d_stop`, `d_go`, gains, `enable_walk: false`, `enable_crab: false`, `enable_wander: false`, `wander_vx`, `wander_wz`, `follow_max_s: 60`, `search_timeout_s: 20`, `hunter_pose`, pan window, wav paths, `follow_gait: 5` / select `15`)
- `config/apriltag_36h11_savelij.yaml` (Saveli plug; measured size / id / image remaps)

**Wavs:** `xf_mic_asr_offline/feedback_voice/english/saveli.wav`, `cat.wav`.

**Edit:** `proud_up/CMakeLists.txt`, `proud_up/package.xml` (`tf2_ros`, `tf2_geometry_msgs`, `apriltag_msgs`, `kinematics_msgs` already used by follow-the-cat).

**Gait hook (this project, same files Sprint 3 Step 2 would touch):**

- `driver/controller/controller/move.py` — `FollowGaitGenerator`
- `driver/controller/controller/step_controller.py` — branch `gait == 5`, `gait == 15`, `cmd_gait == 5`

Do **not** rebuild `kinematics.so`. Do **not** pass `gait=5` into `set_step_mode`. Do **not** edit vendor `app/lidar_controller.py` or `example/.../apriltag_*.py`.

C++ rebuild: `colcon build --packages-select proud_up` then `source ~/ros2_ws/install/local_setup.zsh`. Python controller is usually symlink-installed; still restart `move_controller` after editing `move.py`. `need_compile=False` does not help C++.

If Sprint 3 Step 2 lands first, **reuse** `follow_gait.*` and the generator hook; this project then adds the hunter. If this project lands first, Sprint 3 Step 2 reuses the gait and only adds ASK / YOLO person — still do **not** launch `follow_the_cat_node` beside the hunter.

---

## Why not `custom_gait_node`

The clipboard’s 50 Hz node is the right *ambition* (we own the foot curve) aimed at the wrong *socket* (the servo topic).

What the hunter needs from the legs, and where it comes from once we write the gait:

- omnidirectional-enough `vx`, `vy`, and `ωz` — **our** generator, driven by Twist,
- a halt that stands — `halt_legs()`,
- a speed cap the unfinished step cannot overshoot through the LiDAR wall — parameters,
- a walk that is *ours* to change — the map + swing clock, not a second publisher.

Two writers on `/servo_controller` is how a gait fights a dance, and how a “custom gait node” would fight `StepController`. We do not learn that lesson by repeating it.

---

## Do not implement on this turn

This file is the corrected plan. Map is **variant 3**. Client is **`masha_hunter_node`** with pluggable targets (Saveli first, cat next). No node, no generator, no launch, no yaml in `proud_up` until Peter says to build it.
