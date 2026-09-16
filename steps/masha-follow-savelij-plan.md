# Masha follows Saveli — corrected plan

**Status:** plan only. Do **not** implement until Peter asks.  
**Source:** host clipboard paste (2026-09-16) titled “Step 2 Execution Plan: Masha Following Savelij (Mecanum Car)”.  
**This memo:** feasibility against Masha as she actually runs (Jetson Orin NX, ROS 2 Humble, `ROS_DOMAIN_ID=27`, `proud_up`, Aurora 930, LD19, bus servos).  
**Name:** the other robot is **Saveli** in the trio docs; the clipboard used **Savelij**. Same machine. This file uses **Saveli**.  
**Revision (2026-09-16, later same day):** custom gait is **in scope**. The first draft dropped it because a vendor `CmdVelGenerator` walk would already chase a tagged car. That was the *efficient* answer. Peter’s goal is **learning**, so we write the walk ourselves. **Map pick (Peter, same day): variant 3 — omnidirectional tripod.** Detailed design is in this file. Bounce / high-step / period are knobs, not a fourth map.

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
| **2. Saveli follower** | AprilTag → `base_link` \((x,y,\theta)\), PD, **crab `vy` when he strafes**, LiDAR latch, halt that *stands* | Vendor `apriltag_track` / `lidar_app`. Bare `/cmd_vel`. Zero Twist as “stop”. |

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

Crab is **not** a second gait. The generator always accepts `vy`. The follower **turns `vy` on** when Saveli is already ahead and slides left/right, so Masha sidesteps and stays in one line behind him instead of yawing after him. See [Crab policy](#crab-policy-follower-not-gait).

Cost: wrong AEP/PEP signs scuff or trip. First live gait tests keep `vy = 0` until `vx + ωz` is boring (Phase G2), then a hand `vy` Twist, then the follower crab gate.

#### Not a variant (knobs on the chosen map)

Do not spend a fourth generator on these:

- **Bounce / body bob** — stance-`z` amplitude on variant 1 or 3.
- **High-step / stalking period** — `height` and `T`, same map.
- **Rest-to-rest quintic on the whole cycle** — point-to-point, not a walk (lurch–pause). Correct for a single creep step; wrong as a cyclic gait.

---

## Variant 3 design (locked)

Status: design only. Do **not** implement until Peter says to build. Package split stays: C++ math in `proud_up/follow_gait.hpp` (no rclcpp) + gtest; Python `FollowGaitGenerator` copies the same formulas; `StepController` consumes the generator; follower publishes Twist including `vy`.

### What “omnidirectional” means here

The body command is a **planar twist** in `base_link`:

```text
V = (vx, vy, ωz)     # m/s, m/s, rad/s   (generator uses mm/s internally)
```

- `vx > 0` — walk toward the head
- `vy > 0` — crab toward Masha’s **left** (same sign as `base_link` +Y)
- `ωz > 0` — yaw CCW (left turn)

A tripod gait with only `vx` and `ωz` is still a tripod. What we add is the **no-slip map** from this twist onto six AEP/PEP points, including the lateral term. Without that map, `linear.y` on `/controller/cmd_vel` would either be ignored or fall through to vendor `CmdVelGenerator`.

The gait does **not** decide when to crab. It only knows the twist it was given this cycle. When to set `vy ≠ 0` is the follower policy below.

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

### Crab policy (follower, not gait)

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

So crab is **activated when Saveli moves left or right and we are already looking at him**. It is not activated when he is 40° off to the side (that is Test 1: turn in place). Lost tag / LiDAR latch: `vy = 0` and `halt_legs()`, same as today.

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
| `vx_max` | 0.05 m/s | follower |
| `vy_max` | 0.04 m/s | follower; 0 until crab gate |
| `wz_max` | 0.3 rad/s | follower |
| `r_target` | 0.80 m | follower |
| `θ_crab` | 15° | crab gate |
| `y_on` / `y_off` | 0.06 / 0.03 m | crab hysteresis |
| `enable_crab` | **false** until Test G2 `vy` is boring | yaml |

### Bring-up order for this map

1. **G0** — `follow_gait.hpp` + gtest (no robot). Includes `vy > 0` → AEP `+Y`, `ωz > 0` → outside foot longer, swing `z-dot ≈ 0`, stance speed at wrap ≠ 0, CoM in triangle at `φ = 0, π`.
2. **G1** — hook, Traveling `gait: 5`, `stride 0` / `V = 0`: tripod in place, soft landings, halt `-2`.
3. **G2a** — `gait: 15`, `vx = 0.03`, halt; then `ωz` only, halt.
4. **G2b** — `vy = 0.03` only, halt. She must **translate left**, not yaw. This is the gait proof of crab. Tape a line on the floor.
5. **Follow C** — `enable_crab:=false`. Tests 1–2 as written (`vx`, `ωz`).
6. **Follow Test 3b** — `enable_crab:=true`. Drive Saveli **sideways** on the Mecanum. Masha sidesteps, stays behind him, heading does not chase him around. If he leaves the 15° cone, she yaws first. Cover the tag → stand.

### Failure modes (this map)

| Symptom | Likely cause |
|---|---|
| Crab is a spin | `vy` sign flipped, or follower is writing `ωz` from `y` instead of `vy` |
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
| `camera_link` | `depth_cam_link` / `rgb_camera_link`, parent `link4` | Lock arm at camera-forward rest pose. TF detections into `base_link`. |
| `r = √(x²+z²)`, `θ = atan2(x,z)` | Optical: X right, Z forward. Body: X forward, Y left | Optical only inside the detector. Control: `r = hypot(x,y)`, `θ = atan2(y,x)` in `base_link`. |
| `r_target = 0.60`, stop `0.50`, go `0.55` | Halt delay + lidar offset + body ~0.4 m wide | Start: `r_target = 0.80 m`, `d_stop = 0.55 m`, `d_go = 0.70 m` (all `base_link`). First `vx` cap **0.05 m/s**. |
| P-only `vx = Kp er`, `ωz = Kp eθ` | Hexapod lags a full step | PD + deadband. No I on first bring-up (windup during the unfinished step). |
| Three nodes in series | Safety must override | One follower node: tracker, PID, lidar, halt, watchdog. Gait lives in `controller`, selected by `gait=5` / `15`. |
| 60° LiDAR sector | Vendor follow uses 90° | Parameter, default **90°**. Ignore non-finite / `< 0.2 m`. |
| Test 3: match Mecanum strafe | Variant 3 gait accepts `vy`; follower **gates** it | G2b: hand `vy` Twist. Follow C: `vx`,`ωz`. Test 3b: crab when `|θ|<15°` and `|y|` grows. Lost tag → halt. |

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
  3. x,y in base_link;  r = hypot(x,y);  θ = atan2(y,x)
  4. optional: depth at tag pixel as range sanity check
  5. frontal /scan → d_min in base_link
  6. once: Traveling gait=15  (select our generator for cmd_vel)
  7. if lost / lidar stop / watchdog: halt_legs()
     else: PD(e_x, e_θ) → vx, ωz;  if crab gate: PD(e_y) → vy
           Twist (vx, vy, ωz) on /controller/cmd_vel
           │
           ▼
/controller
  cmd_gait==5  →  FollowGaitGenerator @ 50 Hz   (SE(2) AEP/PEP + swing quintic)
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
x, y              # metres, ignore z (tag height)
r     = hypot(x, y)
θ     = atan2(y, x)                 # rad, +θ = tag to Masha’s left
e_x   = x - r_target                # along our nose (≈ range when |θ| small)
e_y   = y                           # left/right of our nose — crab error
e_θ   = θ
```

PD (first bring-up; I-term off). Crab gate is in [Variant 3 design](#crab-policy-follower-not-gait); `enable_crab` default **false**.

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

- `include/proud_up/savelij_follow.hpp` + `src/savelij_follow.cpp` — no rclcpp: PD, deadband, lidar hysteresis, crab gate (`θ_crab`, `y_on`/`y_off`), optical→body heading tests with fake poses.
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

### Phase E — knobs on the chosen variant (after C/D, not a gate)

Height, period, optional bounce. Do **not** switch to a second map in this phase. Do **not** add a second servo node. Do **not** pass `5` into `kinematics.set_step_mode`.

After Test 2 is boring: Test 3b, `enable_crab:=true`. Lost-tag halt remains the backstop.

---

## Tests

### Gait (Phases G0–G2)

Joystick / pad on Masha unplugged. No Saveli required.

**Test G0 — gtest.** All invariants in Phase G0. Fail the build if swing `z-dot` at touchdown is not ~0, or if cycle-end stance speed *is* ~0.

**Test G1 — in-place cycle.** Traveling `gait: 5`, `stride: 0`. Chosen grouping visible (tripod A/B, or one-leg wave). Soft landings. No body pause at wrap. `gait: -2` → `DEFAULT_POSE`. Repeat. No servo fight (one publisher).

**Test G2 — slow `cmd_vel`.** `gait: 15` then `vx = 0.03` for ~3 s, halt. Then `ωz` only, halt. She must translate / yaw and then **stand**, not march. Variant 3 only, after that is boring: `vy` only, then halt.

### Follow (clipboard Phase 5, corrected)

Joystick on Saveli. Masha’s pad unplugged. `enable_walk` only after Phase B. Walk is **gait=5**.

**Test 1 — Stationary heading.** Saveli still, offset ~20–30° at ~1 m. Masha turns in place (`ωz` only) until `|θ| < 5°`, then **halt** (stand). She must not step in place after alignment.

**Test 2 — Straight follow.** Drive Saveli forward slowly. Masha holds ~0.80 m. No contact. Lost tag (you cover it) → stand within one step.

**Test 3 — Gentle curve.** Saveli yaws. Masha updates `ωz` and `vx`. `enable_crab:=false`: if Saveli **strafes** out of the camera, she **stops** (pass).

**Test 3b — Crab / one line.** `enable_crab:=true`. Drive Saveli sideways (Mecanum). Masha crabs (`vy`) and stays behind him; she does **not** spin in place to face the tag. If `|θ| > θ_crab` she yaws first. Cover tag → stand.

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
- `config/savelij_follow.yaml` (topics, tag id, **measured** size, `r_target`, `d_stop`, `d_go`, gains, `enable_walk: false`, `enable_crab: false`, `θ_crab`, `y_on`/`y_off`, `follow_gait: 5` / select `15`)
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

This file is the corrected plan. Map is **variant 3** (omnidirectional tripod + crab gate). No node, no generator, no launch, no yaml in `proud_up` until Peter says to build it.
