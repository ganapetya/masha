# Follow the cat — Masha-grounded Sprint 2 plan

**Status:** plan only. Do **not** implement the node until Peter asks.  
**Syllabus (Gemini):** [`quarter-01-sprint-02-plan-v2.md`](quarter-01-sprint-02-plan-v2.md)  
**This memo:** how that syllabus maps onto Masha as she actually runs (Jetson Orin NX, ROS 2 Humble, `proud_up`, Aurora, bus servos).

## Goal

Build `follow_the_cat_node` as a ROS 2 Humble C++ node on Masha: detect a white card in the Aurora RGB image, point the arm-mounted camera at it with bus-servo pulses, then optionally walk a short, bounded step toward it. Keep the syllabus math (pixel → camera ray → yaw/pitch, mutex, gtest). Replace every invented topic, launch, and “neck servo” with what this Jetson actually runs.

## Learner comments (standing rule)

Whatever program we build for this sprint — headers, `.cpp`, tests, launch, yaml — must be **commented richly so a learner can learn from the source**, not only from this memo.

Write comments for a student who knows Java/Python better than C++/ROS and is using this week to learn frames, Eigen, mutexes, and Masha’s bus servos:

- **Why, not restating the line.** Explain the idea: why best-effort QoS, why the mutex is released before `toCvCopy`, why servo 22 is tilt and 23 is not, why `enable_walk` defaults false.
- **Formulas next to code.** Pixel→ray, `atan2` yaw/pitch, pulse = rest ± angle × ticks/rad. Name frames (`optical`, `base_link`) and units (px, rad, pulse).
- **Masha specifics.** Topic names, servo ids, pulse clamps, `ROS_DOMAIN_ID`, Aurora 15 fps — so the file itself contradicts the syllabus where the syllabus is wrong.
- **Concurrency and ownership.** Note which thread runs the callback vs the timer, what the lock guards, and whether a pointer is unique or shared.
- **Safety.** Call out walk abort, zero Twist on shutdown, and joint clamps at the command site.
- **Tests.** Each gtest says which invariant it proves (center pixel → +Z ray, clamp, etc.).

Do not spam `// increment i`. Prefer a short block comment on each function/phase and inline notes on the non-obvious lines. Match existing `proud_up` tone: factual, no narration of the edit.

This rule applies to later sprints too unless Peter says otherwise.

## Corrections vs the syllabus

The v2 program is pedagogically right and hardware-wrong in several places:

- **Package.** `proud_up` already exists at `/home/ubuntu/ros2_ws/src/proud_up`. Add the new executable there. Do not invent a second C++ package.
- **Launch.** There is no `masha_bringup` / `robot.launch.py`. Boot stack is `ros2 launch bringup bringup.launch.py` (`BRINGUP_PROFILE=slim` by default). Camera + gait are already up after power-on. Do **not** run `~/.stop_ros.sh` just to start this node — that kills the whole stack.
- **Run command.** After `colcon build --packages-select proud_up` and `source ~/ros2_ws/install/local_setup.zsh`: `ros2 launch proud_up follow_the_cat.launch.py` (or `ros2 run proud_up follow_the_cat_node`).
- **Image topic.** `/depth_cam/rgb/image_raw` (Aurora 930 remapped in `peripherals/launch/include/aurora930.launch.py`). ~15 fps.
- **QoS.** Aurora is best-effort. Copy `proud_up_node`: `KeepLast(5)` + `best_effort()`. Default reliable subscriptions often see zero frames.
- **Intrinsics.** Subscribe to `/depth_cam/rgb/camera_info`. Do **not** hardcode 640×480 / `fx=fy=525`. USB-cam yaml is a fallback only (`peripherals/config/camera_info.yaml` is 640×480, fx≈522, fy≈525, cx≈340, cy≈240). Aurora `resolution_mode_index=2` is not guaranteed 640×480; object-tracking’s working size is 640×400-class. Always use `msg.width` / `msg.height` and live `K`.
- **Servo topic / type.** Publisher: `servo_controller` (`/servo_controller`), message `servo_controller_msgs/msg/ServosPosition`, `position_unit = "pulse"`. Same helper shape as `servo_controller/bus_servo_control.py` and `proud_up/pose_commander.cpp`.
- **There is no neck.** Camera is fixed on **link4** (`rospider_description/urdf/depth_camera.urdf.xacro`, `camera_connect_joint`). Joint map: id 19 = `joint1` yaw (pan), 20–22 = arm pitch chain, **23 = `joint5` wrist yaw — not tilt**, 24 = gripper.
- **Do not command servo 23 for tilt.** Existing color track holds 23 and 24 at 500 and pans with 19 (`example/color_track/color_track_node.py`). Tilt this week: hold proud-up pose on 20/21/23/24 and apply pitch on **id 22** (wrist pitch, camera parent).
- **Walk command.** `/controller/cmd_vel` (`geometry_msgs/msg/Twist`), **not** `/cmd_vel`. `move_controller` clamps `linear.x` to ±0.12 m/s. Syllabus 0.05 m/s is legal.
- **Domain.** `ROS_DOMAIN_ID=27`.
- **C++ rebuild.** `need_compile=False` only helps Python. This node needs `colcon build`.

## Reuse from Week 1 (`proud_up`)

Copy these patterns; do not rewrite them:

- Image callback stores `sensor_msgs::msg::Image::ConstSharedPtr` under `std::mutex` / `std::lock_guard` (`proud_up/src/proud_up_node.cpp`).
- 10 Hz wall timer owns the state machine (never do OpenCV + servo publish inside the subscription if it can block DDS).
- `cv_bridge::toCvCopy(msg, "bgr8")`.
- Camera-forward rest pose (ActionGroups `init_horizontal` / proud_up yaml): 19=500, 20=810, 21=180, 22=150, 23=500, 24=500. object_tracking enter is close (20=720, 21=130, 22=150). Start from **proud_up** numbers so gaze matches the boot snapshot.
- gtest via `ament_add_gtest` like `proud_up/test/test_camera_frame.cpp`.
- Ready wait (optional): `/controller_manager/init_finish` and `/init_pose/init_finish`.

Eigen 3.4 is already on the Jetson (`libeigen3-dev`, `ros-humble-eigen3-cmake-module`).

## Node design

**Name:** `follow_the_cat_node`  
**Package:** `proud_up`

Split testable math from the ROS node:

- `include/proud_up/follow_the_cat.hpp` + `src/follow_the_cat.cpp` — no rclcpp: detect white blob, pixel→ray, yaw/pitch, rad→pulse.
- `src/follow_the_cat_node.cpp` — subscriptions, timer, publishers, services.
- `test/test_follow_the_cat.cpp` — offline gtest.
- `launch/follow_the_cat.launch.py` + `config/follow_the_cat.yaml`.

**Phases:** `IDLE → TRACKING → MOVING → DONE` (plus `LOST` back to `IDLE`).

- `IDLE`: rest pose, zero `cmd_vel`, wait for a blob.
- `TRACKING`: 10 Hz servo updates; require ~2 s of continuous lock with the blob near image center before walking.
- `MOVING`: `linear.x = 0.05` on `/controller/cmd_vel` for **at most 3 s**, then zero twist → `DONE`. Abort immediately if the blob is lost.
- `DONE`: hold; `~/start` (`std_srvs/Trigger`) resets to `IDLE`.

Walk is gated by parameter `enable_walk` **default false**. First live bring-up is gaze-only. Turn walk on after pan/tilt signs are verified on the real arm.

**Services (Masha app pattern, lighter than phone enter/exit):**

- `~/start`, `~/stop` (`std_srvs/Trigger`)
- `~/init_finish` — reports current phase (same idea as proud_up)

**Debug:** publish `~/image_result` (`sensor_msgs/Image`, bgr8) with contour, center, and phase text. View on the dummy HDMI / NoMachine; slim boot has no `web_video_server`.

**Concurrency:**

- Callback thread: lock, store latest image + camera_info pointers, return.
- Timer thread: lock, copy pointers, unlock, then `toCvCopy` / detect / control. Never hold the mutex across OpenCV.

## Geometry → pulses (this week’s math, this robot’s joints)

Optical frame (ROS): X right, Y down, Z forward.

```
X = (u - cx) / fx
Y = (v - cy) / fy
ray = (X, Y, 1).normalized()
yaw   = atan2(X, Z)                         // pan, servo 19
pitch = atan2(-Y, hypot(X, Z))              // tilt, servo 22
```

Pulse law (from `servo_controller/joint_position_controller.py`; ids 19 and 22 are **not** flipped):

```
ticks_per_rad = 1000 / (240 * π/180)   ≈ 238.73
pulse_19 = 500 + yaw_sign   * yaw   * ticks_per_rad
pulse_22 = 150 + pitch_sign * pitch * ticks_per_rad
```

Clamp 19 to **200–800** (color_track). Clamp 22 to a tight band around 150 (e.g. 80–250) so the arm does not fold. Duration per command ~0.05–0.1 s (color_track uses 0.02 s; slightly slower is safer).

`yaw_sign` / `pitch_sign` are parameters, default +1. **Day 7 on hardware** is when we flip them. color_track increases pulse 19 when the target is to the right of center — verify, do not assume the optical-frame yaw sign matches.

Quaternion check (Day 3, not sent to hardware): `Eigen::AngleAxisd(yaw, UnitZ()) * Eigen::AngleAxisd(pitch, UnitY())`. Log `norm≈1` and that a center pixel gives ~identity.

This is **not** full camera-from-base extrinsics (Week 3) and **not** arm IK (that is how vendor `color_track` does vertical). Week 2 is a 2-DoF gaze around the proud-up pose.

## Detection (Day 1, honest limits)

Pipeline: BGR → gray → Gaussian blur → `threshold` (parameter, start ~200) → morph open → `findContours` → largest contour with area in `[min_area, max_area]` and `approxPolyDP` 4–6 vertices **or** aspect in `[0.4, 2.5]`. Centroid = contour moments (or min-area rect center).

White + Aurora auto-exposure is brittle (driver `exposure_time=10`, `gain_value=10`). If lock is flaky: raise threshold, require min area, hold a high-contrast card, or temporarily use a bright lamp. Full LAB invariance is Week 5; do not pull in `lab_tool` yet.

## Safety (non-negotiable on a live hexapod)

- Never start walk unless `enable_walk:=true` **and** 2 s lock **and** |yaw| small.
- Always publish zero Twist on `stop`, `DONE`, destructor, and SIGINT.
- Do not run while `object_tracking`, `color_track`, `intelligent_kick`, or joystick own `servo_controller` / `cmd_vel`. Slim profile does not start those; do not switch to `profile:=full`.
- Floor clear, 0.05 m/s × 3 s ≈ 15 cm, once per `~/start`.
- Do not command leg servos (ids 1–18). Only 19–24.
- Keep `proud_up_node` idle (`Done`) if it was used at boot; it is one-shot and then silent.

## Day-by-day on Masha (map to the syllabus)

Keep the 1 hr/day shape; change the artifacts.

1. **Day 1 — capture + white blob.** Library `detect_white_card(cv::Mat) -> optional<Pixel>`. Node: best-effort sub, timer, `~/image_result`. No servos.
2. **Day 2 — pixels → ray.** `CameraIntrinsics` from `camera_info` (fallback only if missing). `pixel_to_ray(u,v) -> Eigen::Vector3d`.
3. **Day 3 — yaw/pitch + quaternion.** Pure functions + rest-pose pulse mapping. Still no live motion, or optional `dry_run` log of pulses.
4. **Day 4 — state machine + mutex.** Phases, `TargetState {u,v,stamp,seen}` guarded; timer 10 Hz.
5. **Day 5 — hardware.** Servo 19/22 publish; walk behind `enable_walk`. Verify signs with the card held still, then moving, **then** walk.
6. **Day 6 — gtest.** Center pixel → ray ≈ (0,0,1); right pixel → yaw > 0 (optical); 500/150 pulses at zero angles; clamp; detect on a synthetic white rectangle.
7. **Day 7 — integrate.** `colcon build --packages-select proud_up`, launch node against live slim bringup, tune threshold/signs, one walk.

## Files to add/touch (when implementing)

- Add: `include/proud_up/follow_the_cat.hpp`, `src/follow_the_cat.cpp`, `src/follow_the_cat_node.cpp`, `test/test_follow_the_cat.cpp`, `launch/follow_the_cat.launch.py`, `config/follow_the_cat.yaml`
- Edit: `proud_up/CMakeLists.txt` (executable, Eigen, `geometry_msgs`, gtest), `proud_up/package.xml`
- Comment every new file per **Learner comments** above (formulas, threads, Masha topic/servo facts).
- Do not edit vendor `app/`, `example/color_track`, or bringup unless a launch include is later requested.

## Live run (Day 7)

```bash
# slim stack should already be up from start_app_node.service
source ~/.zshrc
cd ~/ros2_ws
colcon build --symlink-install --packages-select proud_up
source install/local_setup.zsh

ros2 launch proud_up follow_the_cat.launch.py
# later:
ros2 launch proud_up follow_the_cat.launch.py --ros-args -p enable_walk:=true
```

Sanity: `ros2 topic hz /depth_cam/rgb/image_raw` and `ros2 topic echo /depth_cam/rgb/camera_info --once` before blaming detection.

## Out of scope (later weeks)

- True cat / YOLO / ONNX (Weeks 6–7)
- LAB color picker (Week 5)
- Full extrinsics / TF tree camera→base (Week 3)
- Vendor arm IK (`/arm_kinematics/set_pose_target`)
- Multi-agent / Saveli (Week 8)

## After the week

Write `/opt/src/learning-bots-sharing/quarter-01-week-02-result.md` only when Peter says the week is complete. Update stale `~/CURRENT_PLAN.md` (it still says Week 1). Commit robot code to `ganapetya/masha` via `~/git-push-all.sh` only with Peter’s OK.
