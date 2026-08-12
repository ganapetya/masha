# Hiwonder ROSpider Workspace Memo

**Last updated:** 2026-08-12  
**Machine:** Hiwonder ROSpider (hexapod) on NVIDIA Jetson Orin NX  
**Primary code path (runtime):** `/home/ubuntu/ros2_ws`  
**This Grok worktree (dev):** `/home/ubuntu/.grok/worktrees/ubuntu-ros2-ws/masha-workspace`  
**Git remote:** `http://192.168.11.206:3000/JetSpider/ros2_ws.git` (branch `main`, tip often `jp6.0`)

---

## 1. What this robot is

**ROSpider** (also referred to as JetSpider in some assets) is a **6-legged hexapod** with:

- Bus servos for legs (and arm/gripper on equipped variants)
- STM32-based **ros_robot_controller** board (UART/serial protocol)
- LiDAR, depth camera, IMU, optional microphone array
- OLED, buzzer, LEDs, joystick / gamepad support
- Optional robotic arm + gripper for pick/place / competition tasks

It runs **ROS 2 Humble** on **Ubuntu 22.04** (JetPack 6.x / L4T R36.4.3).

### Hardware snapshot (this unit)

| Item | Value |
|------|--------|
| SoC | NVIDIA Jetson Orin NX (16GB), model p3767-0000 |
| JetPack | 6.2 (L4T 36.4.3), kernel 5.15.148-tegra |
| CUDA | 12.6 / TensorRT 10.7 / cuDNN 9.3 |
| OpenCV | 4.11.0 (CUDA enabled) |
| PyTorch | 2.7.0 (aarch64 wheels under `~/Downloads`) |
| Disk | NVMe ~116G, ~59% used (~46G free) |
| Machine type env | `MACHINE_TYPE=ROSpider` |
| LiDAR | `LIDAR_TYPE=LD19` |
| Depth camera | `DEPTH_CAMERA_TYPE=aurora` (Deptrum Aurora 930) |
| Mic | `MIC_TYPE=xf` (iFlytek offline ASR package present) |
| ASR mode | `ASR_MODE=online` (can switch offline) |
| Language | `ASR_LANGUAGE=Chinese` |
| ROS domain | `ROS_DOMAIN_ID=27` |
| Version string | `\|V1.1.0\|App_ROS2_1.0\|2026-02-06\|` |

---

## 2. Critical layout: two copies of source

| Path | Role |
|------|------|
| `~/ros2_ws` | **Live robot workspace.** Built (`build/`, `install/`, `log/`). Autostart, launch files, and many hardcoded paths point here. |
| `~/.grok/worktrees/ubuntu-ros2-ws/masha-workspace` | **Grok/dev worktree** of the same git repo. Good for agent edits; does **not** automatically replace runtime unless you sync/build into `~/ros2_ws`. |

**Runtime hardcoding note:** Many launch files check `need_compile`:

- `need_compile=False` (default in `.typerc`) → paths like `/home/ubuntu/ros2_ws/src/...` are used directly (source tree, not install share).
- `need_compile=True` → uses `get_package_share_directory(...)`.

So **editing only the Grok worktree will not change robot behavior** until changes are applied under `~/ros2_ws/src` (and rebuild if using install mode or C++ packages).

Shell bootstrap:

```
~/.zshrc
  → source ~/ros2_ws/.zshrc
      → source ~/ros2_ws/.robotrc
          → source ~/ros2_ws/.typerc   # env: LIDAR, CAMERA, MACHINE, ASR, ROS_DOMAIN_ID
          → source /opt/ros/humble/local_setup.zsh
          → source ~/ros2_ws/install/local_setup.zsh
          → source ~/third_party/third_party_ws/install/local_setup.zsh
          → source ~/third_party/aurora_ws/install/local_setup.zsh
          → source ~/third_party/orbbec_ws/install/local_setup.zsh
          → source ~/third_party/rtabmap_ws/install/local_setup.zsh
```

CUDA is on `PATH` via `.robotrc` (`/usr/local/cuda`).

---

## 3. Home directory map (`~`)

```
/home/ubuntu/
├── ros2_ws/                 # MAIN ROS2 workspace (source + build + install)
├── third_party/             # Large deps & secondary ROS workspaces (~11G)
│   ├── third_party_ws/      # lidar drivers, web_video_server, teb, apriltag, etc.
│   ├── aurora_ws/           # Deptrum Aurora 930 depth camera driver
│   ├── orbbec_ws/           # Orbbec camera stack (optional alternate camera)
│   ├── rtabmap_ws/          # 3D SLAM (RTAB-Map)
│   ├── Open3D/              # Point cloud / 3D (built)
│   ├── opencv/ + opencv_contrib/  # Custom OpenCV build
│   ├── ultralytics/         # YOLO training/inference stack
│   ├── yolo/                # Training assets / engines
│   ├── sherpa-onnx/         # Offline ASR/TTS (large models offline path)
│   ├── rtw89/               # WiFi driver sources
│   ├── ch341ser_linux/      # USB-serial driver
│   └── msc/                 # iFlytek MSC data blobs
├── software/                # Qt desktop tools (not colcon packages)
│   ├── actionset_editor/    # Servo action groups (*.d6a) — 63 files
│   ├── servo_tool/          # Direct servo debug UI
│   ├── lab_tool/            # LAB color threshold calibration
│   ├── collect_picture/     # Dataset image capture
│   ├── tool/                # App mode switcher / system tool
│   └── roLabelImg/          # Image labeling (YOLO datasets)
├── large_models/            # Standalone LLM/ASR/TTS demos + config (outside ros2_ws)
├── wifi_manager/            # WiFi AP/STA + button_scan + remote services
├── Desktop/                 # ROSpider.desktop, slam/navigation shortcuts, tools
├── Downloads/               # aarch64 wheels: torch, torchvision, pycuda, onnxruntime_gpu
├── .ollama/                 # Ollama local LLM host (binary present; model list may be empty)
├── .ros/                    # ROS home (logs, etc.)
└── .grok/                   # Grok Build agent install + worktrees
```

### Desktop / autostart services

| Service | Purpose |
|---------|---------|
| `start_app_node.service` | Boot: `ros2 launch bringup bringup.launch.py` (main stack) |
| `wifi.service` | WiFi management |
| `button_scan.service` | Physical button handling |
| `expand_rootfs.service` | One-shot rootfs expand |
| `set_default_device.service` | Audio sink selection (USB mic/speaker) |

Udev rules of interest: `99-ttyACM0.rules`, `lidar.rules`, `xf_mic.rules`, `99-usb-cam.rules`, `99-deptrum-libusb.rules`, `99-obsensor-libusb.rules`, `angstrong-camera.rules`.

---

## 4. ROS 2 package architecture (`ros2_ws/src`)

### Layer diagram (conceptual)

```
┌─────────────────────────────────────────────────────────────┐
│  Apps / Competition / Large-model demos / Examples           │
│  app | competition | large_models_examples | example         │
├─────────────────────────────────────────────────────────────┤
│  Navigation / SLAM                                           │
│  navigation | slam                                           │
├─────────────────────────────────────────────────────────────┤
│  Perception peripherals                                      │
│  peripherals (lidar, depth cam, joy, teleop, imu filter)     │
│  xf_mic_asr_offline (+ msgs)                                 │
├─────────────────────────────────────────────────────────────┤
│  Motion stack                                                │
│  controller (move/step/odom/oled)                            │
│  kinematics (+ kinematics_msgs)  [hexapod IK, kinematics.so] │
│  arm_kinematics (+ msgs)         [arm IK]                    │
│  servo_controller (+ msgs)                                   │
│  ros_robot_controller (+ msgs)   [STM32 serial board]        │
│  sdk (common, pid, led, button, fps)                         │
├─────────────────────────────────────────────────────────────┤
│  Shared interfaces                                           │
│  interfaces (ColorDetect, RunActionSet, SetPose2D, ...)      │
├─────────────────────────────────────────────────────────────┤
│  Simulation / description                                    │
│  rospider_description (URDF/xacro/meshes)                    │
│  robot_moveit_config                                         │
├─────────────────────────────────────────────────────────────┤
│  Bringup                                                     │
│  bringup (startup_check + full stack launch)                 │
└─────────────────────────────────────────────────────────────┘
```

### Package catalog

#### `bringup`
- **Entry point for the robot at boot.**
- `bringup.launch.py` starts:
  - `startup_check`
  - `controller` (full motion stack)
  - depth camera + lidar
  - `rosbridge_server` websocket + `web_video_server`
  - `app/start_app.launch.py` (feature apps)
  - joystick control
  - `init_pose` action

#### `driver/` (motion + hardware)

| Package | Role |
|---------|------|
| `ros_robot_controller` | Serial SDK to STM32 board: bus servos, PWM servos, IMU, buzzer, LED, OLED, keys, motors |
| `ros_robot_controller_msgs` | Board message/service types |
| `servo_controller` | High-level servo + **action group** playback (`.d6a` files) |
| `servo_controller_msgs` | `ServosPosition`, etc. |
| `kinematics` | Hexapod inverse kinematics (`kinematics.so` binary + Python wrappers) |
| `kinematics_msgs` | `Traveling`, `LegPosition`, `Pose`, `TransformEuler`, … |
| `arm_kinematics` | Arm IK (`.so` + Python) for manipulator |
| `arm_kinematics_msgs` | Arm pose/service APIs |
| `controller` | **Core gait / motion node stack** |
| `sdk` | Shared helpers: `common`, `pid`, `led`, `button`, `fps` |

**Controller internals (most important motion code):**

- `move_controller.py` — ROS API: `cmd_vel`, `traveling`, leg pose, body pose, action sets
- `step_controller.py` — gaits: **RIPPLE** vs **TRIPOD**, pose transform, action runners
- `move.py` — step generators / cmd_vel generators
- `odom_publisher_node.py` + EKF (`robot_localization`) — odometry fusion with IMU
- `oled_show.py`, `init_pose.py`, `build_in_pose.py`, `pose_transformer.py`

**Action groups path (hardcoded in several places):**  
`/home/ubuntu/software/actionset_editor/ActionGroups/`  
Examples: `init.d6a`, `kick.d6a`, `climb_stairs*.d6a`, `garbage_pick*.d6a`, `navigation_pick*.d6a`, place-by-waste-type, flutters, etc. (**63** groups).

#### `peripherals`
Launches hardware drivers selected by env vars:

- **Depth camera:** `DEPTH_CAMERA_TYPE=aurora` → `aurora930.launch.py`; else USB cam
- **LiDAR:** `LIDAR_TYPE=LD19` → LD19 launch under `launch/include/`
- IMU filter, joystick, keyboard teleop, RViz views

#### `app` (product features, often phone/tablet driven)
- `self_balancing` — IMU posture balance
- `lidar_controller` — obstacle avoid / follow / guard modes
- `line_following` — color line follow
- `object_tracking` — color track
- `intelligent_kick` — ball/color kick
- `hand_gesture` — MediaPipe-style hand models (ONNX under `app/model/`)
- `hand_trajectory`, `perform_actions`
- Pattern: `enter` / `set_running` / heartbeat services (see `app/common.py` Heart + ColorPicker)

#### `example` (course / demos)
- `body_control/` — walk, IK, height, posture, wave, circle, gaits
- `opencv_example/` — color, AprilTag, AR, KCF
- `yolo_detect/` — TensorRT engines for garbage / general detect
- `mediapipe_example/` — hand/face/pose
- `rgbd_example/` — track_and_grab, cross_bridge, object_classification, volume measurement, prevent_falling
- `garbage_classification/`, `navigation_transport/`, `color_track/`, `intelligent_transport/`

#### `competition` (contest pipeline)
Orchestrated tasks (voice + nav + vision):

- `narrow_slit_traversal` — narrow corridor
- `cross_bridge` — balance beam / bridge
- `pick_and_place` / automatic pick services
- `competition.py` — coordinator (nav transport style, listens for finish flags)
- Model: `competition/models/competition.engine`
- Launch: `ros2 launch competition competition.launch.py`

#### `slam` / `navigation`
- 2D SLAM (laser) + map saver; maps in `slam/maps/` (`map_01`, `map_03`, `map_09`, `map_011`)
- RTAB-Map 3D SLAM/nav launches for depth-camera variant
- Nav2-based 2D navigation with custom params under `navigation/config/`

#### `large_models` + `large_models_examples`
- Nodes: `agent_process`, `tts_node`, `vocal_detect`
- Config: online (Aliyun DashScope / OpenAI / OpenRouter) vs offline (Ollama `qwen3:1.7b` + sherpa-onnx ASR/TTS)
- Examples: LLM move, color track, visual patrol, VLLM camera describe/track/nav/transport, function calling
- Standalone demos also in `~/large_models/`

#### `xf_mic_asr_offline`
- iFlytek mic array offline ASR (C++ + Python, many wav prompts, MSC resources)
- Voice control move launch

#### `interfaces`
Shared app-level msgs/srvs: color detect, ROI, objects, `RunActionSet`, `SetPose2D`, `SetInt64`, etc.

#### `simulations`
- `rospider_description` — full URDF (base, legs, arm, gripper, lidar, depth cam, IMU)
- `robot_moveit_config` — MoveIt for arm

---

## 5. Data / control flow (typical)

```
Sensors (LiDAR / Aurora depth / IMU / camera / mic)
        │
        ▼
peripherals + ros_robot_controller
        │
        ▼
controller (step/gait/odom) ──► servo_controller ──► bus servos (legs/arm)
        ▲
        │ cmd_vel / traveling / action sets
Apps, Nav2, Competition, LLM demos
```

**Key topics/services (non-exhaustive):**

- Motion: `controller/cmd_vel`, `controller/traveling`, `servo_controller`, `action_complete`
- Action sets: `RunActionSet` messages; groups from ActionGroups dir
- App pattern: `/<app>/enter`, `/<app>/set_running`, heartbeat `SetBool`
- Odom: `odom/raw` → EKF → `odom`
- Web: rosbridge websocket + `web_video_server` for app/UI streaming

---

## 6. Build & run cheat sheet

Command catalog lives in workspace root file: **`command`** (very useful; Chinese + English comments).

```bash
# Stop all ROS
~/.stop_ros.sh

# Full rebuild (on robot, in ~/ros2_ws)
cd ~/ros2_ws
colcon build --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# Single package
colcon build --symlink-install --packages-select <pkg>

# Full stack (same as systemd)
ros2 launch bringup bringup.launch.py

# Common apps (examples)
ros2 launch app self_balancing_node.launch.py debug:=true
ros2 launch app lidar_node.launch.py debug:=true
ros2 launch slam slam.launch.py
ros2 launch navigation navigation.launch.py map:=map_01
ros2 launch competition competition.launch.py
```

After changing **Python** with `need_compile=False`, restart the node/launch (no rebuild required if running from `src`).  
After changing **msgs / C++ / installed assets**, rebuild the affected package and re-source `install/local_setup.zsh`.

---

## 7. ML / vision assets on disk

| Location | Contents |
|----------|----------|
| `src/example/example/yolo_detect/models/` | `best*.engine`, garbage + competition TensorRT engines |
| `src/competition/models/competition.engine` | Contest detector |
| `src/app/app/model/` | Hand palm/landmark ONNX |
| `src/large_models_examples/.../resources/models/` | NanoTrack engines, FastSAM |
| `~/third_party/yolo/` | Training scripts + pt/onnx/engine |
| `~/third_party/ultralytics/` | Ultralytics source |
| `~/third_party/sherpa-onnx/` | Offline speech models path referenced by config |
| `~/Downloads/*aarch64*.whl` | Torch / torchvision / pycuda / onnxruntime_gpu wheels |

---

## 8. Third-party ROS packages (`third_party_ws`)

Includes: `ldlidar_stl_ros2`, `sllidar_ros2`, `sclidar_ros2`, `web_video_server`, `async_web_server_cpp`, `apriltag_ros`, `imu_calib`, `laser_filters`, `rf2o_laser_odometry`, `teb_local_planner`, `costmap_converter`, `vision_opencv`, `ascamera_listener`, plus camera-specific workspaces (aurora, orbbec).

System ROS packages also present: full **Nav2**, **MoveIt**, rosbridge, etc. under `/opt/ros/humble`.

---

## 9. Desktop tools (`~/software`)

| Tool | Use |
|------|-----|
| `actionset_editor` | Record/edit servo choreography (`.d6a`); Desktop “ROSpider” launcher |
| `servo_tool` | Low-level bus servo ID/position debug |
| `lab_tool` | Calibrate LAB color ranges → YAML used by color apps |
| `collect_picture` | Capture training images |
| `tool` | Switch app/voice modes, system helpers |
| `roLabelImg` | Annotate images for YOLO |

---

## 10. WiFi / remote

`~/wifi_manager/`: `wifi.py`, `button_scan.py`, `remote.py`, `find_device.py` + matching systemd units. Used for AP mode, discovery, and remote ops on classroom networks.

---

## 11. Development guidance for agents

1. **Prefer editing under the active worktree**, then **sync or apply to `~/ros2_ws`** before testing on hardware.
2. Respect **hardcoded `/home/ubuntu/ros2_ws/...` and `/home/ubuntu/software/...` paths** — do not assume install-space only.
3. Motion changes usually touch: `driver/controller`, `driver/kinematics`, `driver/servo_controller`, action groups.
4. Vision/competition changes: `competition/`, `example/.../rgbd_example`, `example/yolo_detect`, ROI YAMLs in `example/config/` and `competition/config/`.
5. Env toggles in **`.typerc`**: camera type, lidar type, ASR online/offline, language, `ROS_DOMAIN_ID`.
6. Do not commit secrets; LLM API keys in `large_models/.../config.py` and `~/large_models/config.py` are currently empty placeholders.
7. Binary blobs: `kinematics.so`, TensorRT `.engine`, MSC `msc/` dirs — treat as platform-specific artifacts.
8. Reference command book: workspace root **`command`** file.

---

## 12. Quick identity card

```
Product:     Hiwonder ROSpider (hexapod ROS2 robot)
Brain:       Jetson Orin NX 16GB + JetPack 6.2
Middleware:  ROS 2 Humble
Firmware:    STM32 ros_robot_controller over serial
Locomotion:  6-leg IK + ripple/tripod gaits + action groups
Sensors:     LD19 LiDAR + Aurora depth cam + IMU (+ xf mic)
Apps:        balance, lidar modes, line follow, track, kick, gestures
Advanced:    Nav2 SLAM/nav, RTAB-Map, YOLO/TRT, LLM voice agents
Contest:     narrow slit, cross bridge, pick/place pipeline
Tools:       actionset_editor, lab_tool, servo_tool, collect_picture
Code:        ~/ros2_ws  (runtime)  |  this worktree (agent sandbox)
```

---

*This memo is intentionally dense so future sessions can orient without re-walking the full filesystem.*
