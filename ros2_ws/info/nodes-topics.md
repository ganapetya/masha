# Masha ROS graph: nodes and topics

**Machine:** Hiwonder ROSpider hexapod (this unit is called Masha)  
**Brain:** Jetson Orin NX, ROS 2 Humble, domain `ROS_DOMAIN_ID=27`  
**Written from:** live `ros2 node/topic` dump + launch/source on 2026-08-29  
**Related:** `info/startup.md` (what boots), `info/desktop.md` (GUI vs terminal), `info/voice.md` (sherpa voice)

Foxglove’s Topic / Node graph looks huge because it is showing **everything at once**: motion, sensors, parameter plumbing, TF internals, duplicate helper nodes, and the Foxglove bridge itself. Most of those names are not independent robot functions. This file sorts that graph.

Live profile when this was written (`~/ros2_ws/.typerc`):

| Env | Value | Effect |
|---|---|---|
| `BRINGUP_PROFILE` | `slim` | Motion + sensors at boot |
| `BRINGUP_JOYSTICK` | `true` | USB gamepad node is on |
| `BRINGUP_VOICE` | `true` | Sherpa voice stack is on |
| `DEPTH_CAMERA_TYPE` | `aurora` | Deptrum Aurora 930 |
| `LIDAR_TYPE` | `LD19` | LD19 lidar |
| Extra (not bringup) | `foxglove_bridge` | You launched this to inspect the graph |

---

## How to read the Foxglove panel

Three things inflate the picture.

### 1. One process, several ROS node names

ROS 2 counts every `rclpy.Node` / `rclcpp::Node`, not every OS process. On Masha, gait and servos each spawn extra nodes **inside** the main process:

| OS process (what `ps` shows) | ROS names Foxglove lists |
|---|---|
| `move_controller` | `/controller`, `/step_controller`, `/joint_control` |
| `servo_controller` (`controller_manager.py`) | `/controller_manager`, `/servo_manager` |
| `joystick_control` | `/joystick_control` **and** a `/controller_client` |
| `voice_control_move` | `/voice_control_move` **and** another `/controller_client` |

That is why ROS prints: *nodes in the graph share an exact name*. Two `/controller_client` nodes is expected with joystick + voice.

### 2. Foxglove itself is a node

`/foxglove_bridge` connects to **every node’s parameter services** so the UI can list them. In the graph it looks like a star with dozens of edges. That is the inspector, not the robot.

Ignore `/foxglove_bridge` when you are trying to understand walking, camera, or lidar.

### 3. Always-on plumbing topics

Every node publishes `/rosout` and `/parameter_events`. Many also expose `describe_parameters` / `get_parameters` / `set_parameters`. Foxglove draws those too. They are ROS housekeeping, not robot I/O.

Also ignore:

| Name | Why it is noise |
|---|---|
| `/_ros2cli_daemon_*` | Temporary CLI helper |
| `/transform_listener_impl_*` | Internal TF2 listener object |
| `/static_transform_publisher_<random>` | Real, but the random suffix is just `tf2_ros` not giving them a name |
| `/odom_rf2o` with no `rf2o` node | EKF **subscribes** to it; the laser-odometry node is **not** started by default, so the topic exists with no publisher |

---

## 1. Essential vs minor

“Essential” here means: **needed for the body to stand, walk, and know its own pose**. Sensors are essential for perception and later SLAM/Nav, but the hexapod can still hold pose and walk from `cmd_vel` without camera or voice.

### 1.1 Essential — hardware, servos, gait, pose

These are the core of `bringup.launch.py` → `controller.launch.py`. If they die, the robot is dead as a robot.

| Node | Process / package | Why it is essential |
|---|---|---|
| `/ros_robot_controller` | `ros_robot_controller` | USB/serial to the **STM32**. IMU raw, battery, buttons, buzzer, OLED, bus-servo pulses. Nothing else can move a servo without this. |
| `/controller_manager` | `servo_controller` | Joint/servo broker. Takes `/servo_controller` commands, talks to STM32 via `/servo_manager`, publishes joint/servo state. |
| `/servo_manager` | same process as above | Low-level pulse publisher to `/ros_robot_controller/bus_servo/set_position`. |
| `/controller` | `move_controller` | Public gait API: `cmd_vel`, traveling, pose, legs, action sets. |
| `/step_controller` | same process | Gait loop (tripod/ripple), inverse kinematics of the legs, writes servo targets. |
| `/joint_control` | same process | Maps kinematic joint angles → servo pulses. |
| `/init_pose` | `controller/init_pose` | Plays the `init` `.d6a` action so the hexapod stands after boot. Stays resident. |
| `/robot_state_publisher` | `robot_state_publisher` | URDF → `/tf` and `/tf_static` (leg/arm links). |
| `/joint_state_publisher` | `joint_state_publisher` | Relays `/controller_manager/joint_states` → `/joint_states` for TF. |
| `/ekf_filter_node` | `robot_localization` | Fuses IMU (and optional laser odom) → `/odom` and TF `odom` → `base_footprint`. |
| `/imu_calib` | `imu_calib/apply_calib` | Applies `imu_calib.yaml` to raw IMU. |
| `/imu_filter` | `imu_filter_madgwick` | Madgwick orientation → `/imu`. |
| `/arm_kinematics` | `search_kinematics_solutions` | Arm/gripper IK/FK services. Needed for any arm motion, including joystick. |

### 1.2 Essential for perception (always on in bringup, not required just to walk)

| Node | Why |
|---|---|
| `/aurora/aurora` | Deptrum Aurora 930. RGB, depth, IR, point cloud on `/depth_cam/...`. Largest idle CPU load. |
| two `static_transform_publisher_*` | Camera optical frames: `depth_cam_link` → `depth_camera_link` / `rgb_camera_link`. |
| `/LD19` | Lidar driver. Publishes `/scan_raw`. |
| `/scan_to_scan_filter_chain` | Crops/filters lidar → `/scan` (this is the scan SLAM/Nav/avoidance should use). |

### 1.3 Minor — running now, not required for walking

Enabled by `.typerc` flags or by you, not by the slim motion core.

| Node | Why it is minor | How it got here |
|---|---|---|
| `/joystick_control` | Gamepad teleop. Robot walks without a pad. | `BRINGUP_JOYSTICK=true` |
| `/asr_node` | Wake word + speech-to-text. | `BRINGUP_VOICE=true` + USB mic |
| `/voice_control_move` | Phrase → walk / turn / stop / dance. | same |
| `/controller_client` (×2) | Thin publishers used by joystick and voice. Not a controller. | spawned inside those two nodes |
| `/foxglove_bridge` | WebSocket inspector. | you launched it |
| `/startup` (`startup_check`) | Boot beep + OLED SSID/IP; may start voice. Short-lived / helper. | bringup |
| `/oled_show` | Writes SSID/IP then typically exits. | controller launch |

### 1.4 Graph noise — ignore in Foxglove

| Node / topic | Role |
|---|---|
| `/_ros2cli_daemon_*` | `ros2` CLI background daemon |
| `/transform_listener_impl_*` | TF2 internal |
| `/parameter_events`, `/rosout` | Every node |
| `*/describe_parameters`, `*/get_parameters`, `*/set_parameters` | Parameter API, not robot function |
| `/diagnostics` | EKF / laser_filters health |

### 1.5 Optional stacks — **not** in this Foxglove snapshot

These appear only when you launch them (and usually you must **stop** `start_app_node.service` first if the launch includes another motion tree — see `info/startup.md` §13).

| Domain | Typical nodes | When |
|---|---|---|
| Phone / web | `rosbridge_websocket`, `rosapi`, `web_video_server` | `profile:=full` or `rosbridge` / `web_video` |
| Demo apps | `perform_actions`, `lidar_app`, `line_following`, `intelligent_kick`, `self_balancing`, `hand_gesture`, `hand_trajectory` | `start_apps:=true` or `full` |
| Color tracking | `object_tracking` | its own launch (not in `start_app`) |
| SLAM | `slam_toolbox` (+ robot tree) | desktop SLAM / `slam.launch.py` |
| Navigation | Nav2: `map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`, … | desktop Navigation |
| 3D SLAM | `rtabmap` | `rtabmap_slam` / `rtabmap_navigation` |
| Laser odom | `rf2o_laser_odometry` | `rf2o_laser_odometry.launch.py` (feeds `/odom_rf2o`) |
| Competition | `competition`, `cross_bridge`, `narrow_slit_traversal`, `automatic_pick` / `pick_and_place`, `yolo_node`, `apriltag_recognition` | `competition.launch.py` |
| Large models | `vocal_detect`, `agent_process`, `tts_node` + LLM example nodes | `large_models` / `large_models_examples` |
| Example demos | dozens of OpenCV / MediaPipe / gait / YOLO nodes | `example` package, manual |

---

## 2. Classification by functional domain

Same nodes as above, grouped by **what they do**, not by how important they are.

```
                         Foxglove / rosbridge / web_video     (inspect / phone)
                                      │
USB mic ──► asr_node ──► voice_control_move ─┐
Gamepad ──► joystick_control ────────────────┤
Nav2 / apps / competition ───────────────────┤
                                             ▼
                              /controller  (gait API)
                                      │
                              /step_controller  +  /joint_control
                                      │
                              /servo_controller  topic
                                      ▼
                         /controller_manager  +  /servo_manager
                                      │
                                      ▼
                         /ros_robot_controller  ──USB──► STM32 ──► bus servos
                                      │
                                      ├── imu_raw ──► imu_calib ──► imu_filter ──► /imu
                                      │                                              │
                                      │                                              ▼
Aurora ──► /depth_cam/*                                                 ekf_filter_node ──► /odom, tf
LD19   ──► /scan_raw ──► filter ──► /scan  (EKF wants /odom_rf2o if rf2o is launched)
                                      │
                         robot_state_publisher ◄── /joint_states ◄── joint_state_publisher
                                                                      ▲
                                                         controller_manager/joint_states
```

### Domain map

| Domain | Live nodes | Optional nodes |
|---|---|---|
| **A. Low-level board I/O** | `/ros_robot_controller` | — |
| **B. Servo / joints** | `/controller_manager`, `/servo_manager`, `/init_pose` | MoveIt-style `*_controller/follow_joint_trajectory` already hosted here |
| **C. Hexapod gait** | `/controller`, `/step_controller`, `/joint_control`, `/controller_client` | body_control examples (`tripod_gait`, `ripple_gait`, …) |
| **D. Arm IK** | `/arm_kinematics` | MoveIt (`robot_moveit_config`) |
| **E. IMU / odom / TF** | `/imu_calib`, `/imu_filter`, `/ekf_filter_node`, `/robot_state_publisher`, `/joint_state_publisher`, camera static TFs | `/odom_publisher` (commented out), `/rf2o_laser_odometry` |
| **F. Depth camera** | `/aurora/aurora` | `usb_cam` if `DEPTH_CAMERA_TYPE=usb_cam` |
| **G. Lidar** | `/LD19`, `/scan_to_scan_filter_chain` | other lidar launches (`sclidar`) |
| **H. Teleop** | `/joystick_control` | `teleop_key_control` |
| **I. Voice (sherpa)** | `/asr_node`, `/voice_control_move` | iFlytek `mic_init` used by competition; large-model `vocal_detect` |
| **J. Demo apps** | — | `perform_actions`, `lidar_controller`, `line_following`, `intelligent_kick`, `self_balancing`, `hand_gesture`, `hand_trajectory`, `object_tracking` |
| **K. SLAM** | — | `slam_toolbox`, `rtabmap`, `map_save` |
| **L. Navigation** | — | Nav2 lifecycle nodes in `nav2_container` |
| **M. Competition** | — | `competition`, `cross_bridge`, `narrow_slit_traversal`, `pick_and_place`/`automatic_pick`, YOLO |
| **N. Large models** | — | `agent_process`, `vocal_detect`, `tts_node`, LLM/VLLM examples |
| **O. Vision examples** | — | color, AprilTag, KCF, MediaPipe, garbage YOLO, track-and-grab, … |
| **P. Inspect / remote** | `/foxglove_bridge` | `rosbridge_websocket`, `web_video_server` |
| **Q. Boot UI** | `startup_check`, `oled_show` | — |

---

## 3. Role of each node (detail)

### 3.A Low-level board I/O

#### `/ros_robot_controller` — STM32 bridge

**Package:** `ros_robot_controller`  
**Source:** `src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py`  
**Device:** `/dev/ttyACM0`

This is the only node that speaks the controller-board protocol. Everything that moves a servo, beeps, or writes the OLED ends here.

**Publishes**

| Topic | Type | Meaning |
|---|---|---|
| `/ros_robot_controller/imu_raw` | `sensor_msgs/Imu` | Uncalibrated IMU from the board |
| `/ros_robot_controller/battery` | `std_msgs/UInt16` | Battery millivolts (Foxglove often shows this) |
| `/ros_robot_controller/button` | `ButtonState` | Board buttons |
| `/ros_robot_controller/joy` | `sensor_msgs/Joy` | USB gamepad frames (if a pad is plugged into the board path) |
| `/ros_robot_controller/sbus` | `Sbus` | RC SBUS, unused on a typical Masha boot |

**Subscribes**

| Topic | Meaning |
|---|---|
| `/ros_robot_controller/bus_servo/set_position` | Pulse targets for bus servos (legs + arm) |
| `/ros_robot_controller/bus_servo/set_state` | Torque / limits / ID ops |
| `/ros_robot_controller/pwm_servo/set_state` | PWM servos (if fitted) |
| `/ros_robot_controller/set_buzzer` | Beep |
| `/ros_robot_controller/set_oled` | Two-line OLED text |
| `/ros_robot_controller/set_led` | Board LED |
| `/ros_robot_controller/set_motor` | DC motors (not the hexapod legs) |
| `/ros_robot_controller/enable_reception` | Enable/disable incoming serial parse |

**Services:** `/ros_robot_controller/bus_servo/get_state`, `pwm_servo/get_state`, `init_finish`.

If this node is missing, IMU, battery, OLED, and all servo motion stop.

---

### 3.B Servo / joints

#### `/controller_manager` — servo broker

**Package:** `servo_controller` (executable `servo_controller`)  
**Source:** `src/driver/servo_controller/servo_controller/controller_manager.py`

Knows the 18 leg joints (`coxa/femur/tibla` × LF, LM, LR, RF, RM, RR) plus arm `joint1`–`joint5` and gripper `r_joint`. Converts rad/deg/pulse commands into servo IDs.

**Subscribes:** `/servo_controller` (`ServosPosition`), `/joint_controller` (`JointState`).  
**Publishes:** `/controller_manager/joint_states`, `/controller_manager/servo_states` (~50 Hz).  
**Actions:** `/leg_controller/follow_joint_trajectory`, `/arm_controller/follow_joint_trajectory`, `/gripper_controller/follow_joint_trajectory`.

This is **not** ROS 2 Control’s `controller_manager`. Same name, different code.

#### `/servo_manager` — pulse writer

Created inside `/controller_manager`. Publishes `/ros_robot_controller/bus_servo/set_position`. Clients the board `get_state` service. You never talk to it directly.

#### `/init_pose`

**Source:** `src/driver/controller/controller/init_pose.py`  
Bringup starts it with `action_name:=init`. After `controller_manager` is ready it plays the **init** action group from `~/software/actionset_editor/ActionGroups` so the body stands. Then it sits idle, still advertising `init_finish`.

---

### 3.C Hexapod gait

This is the stack you command when you want the robot to walk.

#### `/controller` (`move_controller`)

**Source:** `src/driver/controller/controller/move_controller.py`

Public API of the body. Other nodes should publish here rather than writing servos themselves (apps often cheat and publish `/servo_controller` too).

| Interface | Direction | Role |
|---|---|---|
| `/controller/cmd_vel` | in | `geometry_msgs/Twist`. Linear x/y + yaw. Uses the last gait (tripod/ripple) settings. **Main teleop / Nav2 / voice command topic.** |
| `/controller/traveling` | in | Full gait packet: gait id, stride, height, direction, rotation, period, step count. |
| `/controller/set_pose_euler` | in | Absolute body pose (xyz mm + RPY). |
| `/controller/pose_transform_euler` | in | Relative body pose increment. |
| `/controller/set_leg_absolute` / `set_leg_relatively` | in | One foot in Cartesian space. |
| `/controller/run_actionset` | in | Play a `.d6a` action group. |
| `/controller/set_pose_1` | service | Built-in named poses (`DEFAULT_POSE`, etc.). |
| `/action_complete` | in/out | Action-group finished flag. |
| `/perform_actions/actions` | out | Forwards special action groups to the demo player (only useful if that demo is running). |
| `/servo_controller` | out | Direct servo stream used by the action-group player. |

Gait ids used in this codebase: **1 = ripple**, **2 = tripod**.

#### `/step_controller`

**Source:** `src/driver/controller/controller/step_controller.py`  
Instantiated by `MoveController`, so it is the same process.

Runs the gait generators (`MovingGenerator`, `CmdVelGenerator`, `PoseTransformer`) in a loop and publishes servo targets. Also listens to `/step_controller/cmd_param` (`interfaces/CmdParam`) to switch **built-in poses** and gait height/period (joystick and some apps use this).

#### `/joint_control`

**Source:** `src/driver/kinematics/kinematics/x_joint_control.py`  
Created by `StepController`. Converts kinematic joint radians to bus-servo ticks using `kinematics/config.py` (`SERVOS` table: center, ticks, direction, offset). Publishes `/servo_controller`.

#### `/controller_client` (helper, not a controller)

**Source:** `src/driver/controller/controller/controller_client.py`

A convenience `Node` that only **publishes** the `/controller/*` topics and calls `set_pose_1`. Joystick, voice, apps, and competition each construct one. That is why Foxglove shows two (or more) nodes with this name. Treat them as “someone’s remote control”, not as extra brains.

---

### 3.D Arm kinematics

#### `/arm_kinematics`

**Source:** `src/driver/arm_kinematics/arm_kinematics/search_kinematics_solutions_node.py`

Reads `/controller_manager/servo_states` so it knows the current arm. Other nodes call services instead of doing IK themselves.

| Service | Role |
|---|---|
| `~/set_pose_target` | Inverse kinematics: Cartesian pose → joint/servo command |
| `~/get_current_pose` | Forward kinematics |
| `~/set_joint_value_target` | Direct joint angles |
| `~/get_link` / `~/set_link` | DH / link geometry |
| `~/get_joint_range` / `~/set_joint_range` | Joint limits |

Joystick arm mode, pick-and-place, intelligent kick, and track-and-grab all depend on this.

---

### 3.E IMU, odometry, TF

Pipeline:

```
board IMU
  /ros_robot_controller/imu_raw
        → /imu_calib  (apply imu_calib.yaml)
  /imu_corrected
        → /imu_filter  (Madgwick, no magnetometer, world ENU)
  /imu
        → /ekf_filter_node  (2D EKF)
  /odom  +  tf: odom → base_footprint
```

#### `/imu_calib`

Remaps `raw` → `ros_robot_controller/imu_raw`, `corrected` → `imu_corrected`. Config: `src/peripherals/config/imu_calib.yaml`.

#### `/imu_filter`

`imu_filter_madgwick`. `publish_tf` is **false** in launch; orientation is on `/imu` only. `use_mag: false`.

#### `/ekf_filter_node`

`robot_localization` EKF, `src/driver/controller/config/ekf.yaml`.

- `two_d_mode: true`
- `imu0: imu` — yaw + yaw rate (gravity removed)
- `odom0: odom_rf2o` — vx + vyaw **if** rf2o is running
- Output remapped to `/odom`
- Broadcasts TF `odom` → `base_footprint`
- Services: `/set_pose`, `/enable`, `/toggle`

Without rf2o, odom is IMU-only (heading/rate). Translation will not walk-odometry-track until rf2o or another odom source is added.

`odom_publisher` (`controller/odom_publisher_node.py`) is **commented out** in `odom_publisher.launch.py`. Do not expect `/odom/raw` from the gait node unless `odom_enable` is turned on.

#### `/robot_state_publisher`

Loads `rospider.xacro`. Publishes `/robot_description`, dynamic `/tf` for all links from `/joint_states`, and static URDF TFs on `/tf_static`.

#### `/joint_state_publisher`

`source_list: ['/controller_manager/joint_states']`, 20 Hz. Republishes as `/joint_states` so `robot_state_publisher` can build the tree.

#### Camera `static_transform_publisher_*`

From `aurora930.launch.py`:

- `depth_cam_link` → `depth_camera_link`  (RPY −π/2, 0, −π/2)
- `depth_cam_link` → `rgb_camera_link`    (same)

Optical-frame convention so depth/RGB images match ROS camera frames.

---

### 3.F Depth camera

#### `/aurora/aurora`

**Package:** `deptrum-ros-driver-aurora930` (third-party workspace)  
**Launch:** `src/peripherals/launch/include/aurora930.launch.py`

Remapped onto the robot-standard prefix `/depth_cam`:

| Topic | Type |
|---|---|
| `/depth_cam/rgb/image_raw` | `sensor_msgs/Image` |
| `/depth_cam/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `/depth_cam/depth/image_raw` | `sensor_msgs/Image` |
| `/depth_cam/depth/camera_info` | `sensor_msgs/CameraInfo` |
| `/depth_cam/ir/image` | `sensor_msgs/Image` |
| `/depth_cam/points2` | `sensor_msgs/PointCloud2` |

Also publishes camera `/tf_static`. This node is the usual reason the Jetson is hot while “idle”.

If `.typerc` had `DEPTH_CAMERA_TYPE=usb_cam`, this would be `usb_cam` instead (RGB only).

---

### 3.G Lidar

#### `/LD19`

**Package:** `ldlidar_stl_ros2` (third-party). Publishes `/scan_raw`.

#### `/scan_to_scan_filter_chain`

**Package:** `laser_filters`. Config `src/peripherals/config/lidar_filters_config.yaml`.  
Remaps: `scan` ← `/scan_raw`, `scan_filtered` → `/scan`.

**Use `/scan` for everything downstream** (Nav2, SLAM, avoidance, line-follow obstacle check). `/scan_raw` is the uncropped puck.

---

### 3.H Teleop

#### `/joystick_control`

**Source:** `src/peripherals/peripherals/joystick_control.py`

Reads `/ros_robot_controller/joy`. Left stick walks (`ControllerClient` → `/controller/traveling` / `cmd_vel`). Other buttons change gait/pose (`/step_controller/cmd_param`), beep, or move the arm via `/arm_kinematics/set_pose_target`.

Not required. Disable with `BRINGUP_JOYSTICK=false` (or omit; slim default is off unless you set the env).

Optional sibling, not running: `teleop_key_control` (keyboard Twist).

---

### 3.I Voice (sherpa-onnx)

Canonical design: `info/voice.md`. Two nodes only.

#### `/asr_node`

PulseAudio default source → sherpa-onnx. Wake: **Hello / Hi / Shalom Masha**. Then listens for motion phrases. Publishes `/asr_node/voice_words` (`std_msgs/String`). Plays “I’m here” on the USB speaker.

#### `/voice_control_move`

Subscribes to `/asr_node/voice_words`. Maps phrases (`go forward`, `turn left`, `stop`, `dance`, `come here`, Chinese equivalents) onto `/controller/cmd_vel` and action groups. Owns a `/controller_client`.

---

### 3.J Demo apps (optional, `start_app.launch.py`)

Idle until a phone/`Trigger` **enter** + **set_running**. They still occupy CPU if started.

| Node | Role | Main I/O |
|---|---|---|
| `perform_actions` | Play canned body shows / action groups | sub `/perform_actions/actions`; pub `/servo_controller`, `/action_complete` |
| `lidar_controller` (`lidar_app`) | Lidar avoid / lidar follow | sub `/scan`; pub `/controller/cmd_vel` |
| `line_following` | Color line follow + lidar stop | sub `/depth_cam/rgb/image_raw`, `/scan`; pub `cmd_vel`, `~/image_result` |
| `intelligent_kick` | Find colored ball, walk, kick | RGB + arm IK + `cmd_vel` |
| `self_balancing` | Keep body level from IMU | sub `/ros_robot_controller/imu_raw`; pub `/servo_controller` |
| `hand_trajectory` | MediaPipe palm/landmarks | RGB → `~/gesture`, `~/points`, `~/image_result` |
| `hand_gesture` | Gesture → walk / actions | sub `/hand_trajectory/gesture`; pub `cmd_vel` |
| `object_tracking` | Color blob follow | **not** in `start_app`; own launch. RGB → `cmd_vel` |

Common app services: `~/enter`, `~/exit`, `~/set_running`, `~/init_finish`, plus color/threshold setters.

---

### 3.K SLAM (optional)

Launch: `src/slam/launch/slam.launch.py` (desktop SLAM icon). Starts **another** robot tree — stop bringup first.

| Node | Role |
|---|---|
| `slam_toolbox` (`sync_slam_toolbox_node`) | 2D lidar SLAM. Subscribes `/scan`, odom TF. Publishes `/map`. |
| `map_save` | Helper to write `src/slam/maps/*.yaml` + `.pgm` |
| `rtabmap` | RGB-D/lidar 3D SLAM (`rtabmap_slam.launch.py`) |

---

### 3.L Navigation (optional)

Launch: `src/navigation/launch/navigation.launch.py`. Composable nodes in `/nav2_container`. `cmd_vel` is remapped to **`/controller/cmd_vel`**.

| Node | Role |
|---|---|
| `map_server` | Load saved map (`map_01` default) |
| `amcl` | Particle-filter localization vs map + `/scan` |
| `planner_server` | Global path (NavFn / Smac) |
| `controller_server` | Local control; **TEB** if `use_teb:=true` (default), else DWB |
| `smoother_server` | Path smoothing |
| `behavior_server` | Recoveries (spin, backup, wait) |
| `bt_navigator` | Behavior-tree NavigateToPose |
| `waypoint_follower` | Waypoint list |
| `velocity_smoother` | Accel limiting on cmd_vel |
| `lifecycle_manager_navigation` / `_localization` | Bring the above up |

RTAB-Map navigation is a parallel launch (`rtabmap_navigation.launch.py`) that skips AMCL and uses RTAB-Map’s pose.

---

### 3.M Competition (optional)

Launch: `src/competition/launch/competition.launch.py`. Orchestrator plus three skills.

| Node | Role |
|---|---|
| `competition` | Voice/task sequencer. Listens `/asr_node/voice_words`. Calls enter services of the skills. Tracks `/narrow_slit_traversal/finish`, `/cross_bridge/finish`, `/automatic_pick/finish`. |
| `narrow_slit_traversal` | Lidar side-distance gap crossing (step down, align, shift, traverse). |
| `cross_bridge` | Depth + lidar: climb, align, walk a beam. |
| `pick_and_place` / `automatic_pick` | YOLO/AprilTag + depth + arm IK: pick an object and place it. |
| `yolo_node` (competition) | TensorRT engine `competition.engine`. |
| `apriltag_recognition` | Tag detector from `example`. |

---

### 3.N Large models (optional)

`ros2 launch large_models start.launch.py`:

| Node | Role |
|---|---|
| `vocal_detect` | Wake + ASR for the LLM path (separate from sherpa `/asr_node`) |
| `agent_process` | Sends text (+ optional camera frame) to the configured LLM/VLM |
| `tts_node` | Speaks the agent reply |

`large_models_examples` then adds task nodes: `llm_control_move`, `llm_color_track`, `vllm_track`, `vllm_navigation`, `automatic_transport`, function-calling `llm_control`, etc. All of those **command the same** `/controller` API.

---

### 3.O Vision / body examples (optional)

`example` package. Manual launches. They are teaching/demo nodes, not bringup.

| Group | Nodes | Idea |
|---|---|---|
| OpenCV | `color_detect`, `color_recognition`, `color_position`, `color_track`, `kcf`, `apriltag_*`, `ar` | Color / tag / tracker on `/depth_cam/rgb/image_raw` |
| MediaPipe | `face_track`, `hand_detect`, `hand_gesture`, `finger_trajectory`, `pose_control` | Body/hand from RGB |
| RGB-D | `cross_bridge`, `prevent_falling`, `track_and_grab`, `object_classification`, `object_volume_measurement` | Depth geometry |
| YOLO | `yolo_node`, `yolo_detect_demo`, `garbage_classification` | TensorRT/Ultralytics on RGB; `~/object_detect` |
| Transport | `automatic_pick`, `navigation_transport`, `intelligent_pick`, `intelligent_transport` | Nav2 + arm |
| Body control | `tripod_gait`, `ripple_gait`, `forward_and_rorate`, `left_and_right`, `diagonally`, `speed_control`, `broken_line_walk`, `square_walk`, `body_ik`, `height_adjustment`, `posture_adjustment`, `body_wave`, `body_circle`, `forward_back`, `across_the_steps` | Scripted walks via `ControllerClient` |

---

### 3.P Inspect / remote

| Node | Port | Role |
|---|---|---|
| `/foxglove_bridge` | 8765 (default) | What you are looking at. Subscribes on demand; also grabs all parameter services. |
| `rosbridge_websocket` + `rosapi` | 9090 | Hiwonder phone app. Off on slim. |
| `web_video_server` | 8080 | MJPEG of any `sensor_msgs/Image`. Off on slim. |

---

### 3.Q Boot helpers

| Node | Role |
|---|---|
| `startup_check` (`/startup`) | Sleeps ~50 s, beep, OLED SSID + IP. If `enable_voice` and `/dev/ring_mic` exist, `os.system` launches the voice launch. |
| `oled_show` | Same OLED write, then exits. |

Neither is part of the control loop.

---

## 4. Live topic catalog (this Foxglove session)

Grouped. Types from `ros2 topic list -t` on 2026-08-29. Skip `/rosout` and `/parameter_events`.

### Command the body

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/controller/cmd_vel` | `geometry_msgs/Twist` | voice, joystick client, Nav2, apps | `/controller` | **Primary walk command** |
| `/controller/traveling` | `kinematics_msgs/Traveling` | `controller_client` | `/controller` | Full gait packet |
| `/controller/set_pose_euler` | `kinematics_msgs/Pose` | clients | `/controller` | Absolute body pose |
| `/controller/pose_transform_euler` | `kinematics_msgs/TransformEuler` | clients | `/controller` | Relative body pose |
| `/controller/set_leg_absolute` | `kinematics_msgs/LegPosition` | clients | `/controller` | One foot, world/body Cartesian |
| `/controller/set_leg_relatively` | `kinematics_msgs/LegPosition` | clients | `/controller` | One foot, delta |
| `/controller/run_actionset` | `interfaces/msg/RunActionSet` | clients | `/controller` | `.d6a` name |
| `/step_controller/cmd_param` | `interfaces/CmdParam` | joystick | `/step_controller` | Named pose + gait height/period |
| `/perform_actions/actions` | `interfaces/RunActionSet` | `/controller` | `perform_actions` (if running) | Special shows |
| `/action_complete` | `std_msgs/Bool` | `/controller` (and perform_actions) | `/controller` | Action group done |

### Servo / joints

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/servo_controller` | `servo_controller_msgs/ServosPosition` | gait, init, joystick, voice, apps | `/controller_manager` |
| `/joint_controller` | `sensor_msgs/JointState` | `/init_pose` | `/controller_manager` |
| `/controller_manager/joint_states` | `sensor_msgs/JointState` | `/controller_manager` | `/joint_state_publisher` |
| `/controller_manager/servo_states` | `servo_controller_msgs/ServoStateList` | `/controller_manager` | arm IK, joystick |
| `/joint_states` | `sensor_msgs/JointState` | `/joint_state_publisher` | `/robot_state_publisher` |
| `/ros_robot_controller/bus_servo/set_position` | `ServosPosition` | `/servo_manager` | `/ros_robot_controller` |

### IMU / odom / TF

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/ros_robot_controller/imu_raw` | `sensor_msgs/Imu` | board | `/imu_calib` |
| `/imu_corrected` | `sensor_msgs/Imu` | `/imu_calib` | `/imu_filter` |
| `/imu` | `sensor_msgs/Imu` | `/imu_filter` | `/ekf_filter_node` |
| `/odom` | `nav_msgs/Odometry` | `/ekf_filter_node` | Nav2 / Foxglove / you |
| `/odom_rf2o` | `nav_msgs/Odometry` | **none by default** | EKF (empty unless rf2o launched) |
| `/set_pose` | `PoseWithCovarianceStamped` | you / RViz | EKF |
| `/tf` | `tf2_msgs/TFMessage` | EKF, robot_state_publisher, (imu_filter capable) | everyone |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher, Aurora, camera static TFs | everyone |
| `/robot_description` | `std_msgs/String` | robot_state_publisher | joint_state_publisher, Foxglove |

### Camera

| Topic | Type | Publisher |
|---|---|---|
| `/depth_cam/rgb/image_raw` | `Image` | `/aurora/aurora` |
| `/depth_cam/rgb/camera_info` | `CameraInfo` | same |
| `/depth_cam/depth/image_raw` | `Image` | same |
| `/depth_cam/depth/camera_info` | `CameraInfo` | same |
| `/depth_cam/ir/image` | `Image` | same |
| `/depth_cam/points2` | `PointCloud2` | same |

### Lidar

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/scan_raw` | `LaserScan` | `/LD19` | filter |
| `/scan` | `LaserScan` | filter | voice “come here”, apps, SLAM, Nav2 |

### Board extras

| Topic | Type | Publisher / subscriber |
|---|---|---|
| `/ros_robot_controller/battery` | `UInt16` | board → Foxglove |
| `/ros_robot_controller/button` | `ButtonState` | board |
| `/ros_robot_controller/joy` | `Joy` | board → joystick |
| `/ros_robot_controller/sbus` | `Sbus` | board |
| `/ros_robot_controller/set_buzzer` | `BuzzerState` | voice, joystick, startup → board |
| `/ros_robot_controller/set_oled` | `OLEDState` | startup, oled_show → board |
| `/ros_robot_controller/set_led` | `LedState` | → board |
| `/ros_robot_controller/set_motor` | `MotorsState` | → board |
| `/ros_robot_controller/enable_reception` | `Bool` | → board |
| `/ros_robot_controller/bus_servo/set_state` | `SetBusServoState` | servo tools → board |
| `/ros_robot_controller/pwm_servo/set_state` | `SetPWMServoState` | → board |

### Voice

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/asr_node/voice_words` | `std_msgs/String` | `/asr_node` | `/voice_control_move` (and competition if launched) |

### Health

| Topic | Type | Publisher |
|---|---|---|
| `/diagnostics` | `DiagnosticArray` | EKF, laser_filters |

---

## 5. Useful services and actions (live)

Services worth remembering; skip the `describe_parameters` forest.

| Service | Type | Who |
|---|---|---|
| `/controller/set_pose_1` | `kinematics_msgs/SetPose1` | Named built-in poses |
| `/arm_kinematics/set_pose_target` | `SetRobotPose` | Arm IK |
| `/arm_kinematics/get_current_pose` | `GetRobotPose` | Arm FK |
| `/ros_robot_controller/bus_servo/get_state` | `GetBusServoState` | Read pulses |
| `*/init_finish` | `std_srvs/Trigger` | Vendor “I am up” handshake |
| `/set_pose` | `robot_localization/SetPose` | Reset EKF pose |
| `/enable`, `/toggle` | EKF | Pause/resume filter |

**Actions** (all on `/controller_manager`):

- `/leg_controller/follow_joint_trajectory`
- `/arm_controller/follow_joint_trajectory`
- `/gripper_controller/follow_joint_trajectory`

---

## 6. What to pin in Foxglove

If the goal is “understand the robot”, hide everything except:

**Must watch**

- `/controller/cmd_vel`
- `/servo_controller`
- `/joint_states`
- `/imu`
- `/odom`
- `/tf` + `/tf_static` (TF tree panel)
- `/scan`
- `/depth_cam/rgb/image_raw` (and depth if you care)
- `/ros_robot_controller/battery`
- `/asr_node/voice_words` (only if using voice)

**Safe to hide**

- `/parameter_events`, `/rosout`
- every `*/get_parameters` edge
- `/foxglove_bridge`
- `/controller_client` (look at `/controller/*` instead)
- `/odom_rf2o` until you launch rf2o
- `/ros_robot_controller/sbus`, `set_motor`, PWM unless you use that hardware

---

## 7. Refresh this snapshot

```bash
source ~/.zshrc
ros2 node list
ros2 topic list -t
ros2 node info /controller
```

Bringup owner:

```bash
systemctl status start_app_node.service
tr '\0' '\n' < /proc/$(systemctl show -p MainPID --value start_app_node.service)/environ \
  | grep -E 'BRINGUP_|LIDAR|DEPTH_CAMERA|ASR_'
```

If you later start SLAM, Nav2, or competition, those extra nodes are §3.K–3.O; they are not missing from a healthy slim boot.
