# Masha

Masha is a **Hiwonder ROSpider** hexapod. This repository is her software image: ROS 2 Humble on a Jetson Orin NX. Git root is the robot home directory (`/home/ubuntu`). Develop on a Linux host, then `git pull` and `colcon build` on the Jetson.

Sister robot **Savelij** will live in a separate repo. Shared world-level notes belong in [robots-world](https://github.com/ganapetya/robots-world).

A denser filesystem memo is in [`ROSpider_WORKSPACE_MEMO.md`](ROSpider_WORKSPACE_MEMO.md). Launch recipes live in [`ros2_ws/command`](ros2_ws/command).

---

## Hardware

| | |
|---|---|
| Body | 6-leg hexapod + bus servos; arm/gripper on this unit |
| Brain | NVIDIA Jetson Orin NX 16GB (p3767), JetPack 6.2 / L4T 36.4.3 |
| MCU | STM32 **ros_robot_controller** over USB serial (`/dev/ttyACM0`) |
| OS | Ubuntu 22.04, ROS 2 Humble, CUDA 12.6, TensorRT 10.7 |
| LiDAR | LD19 (`LIDAR_TYPE=LD19`) |
| Depth camera | Deptrum Aurora 930 (`DEPTH_CAMERA_TYPE=aurora`) |
| IMU | On the controller board, fused in `robot_localization` |
| Mic | WonderEchoPro (iFlytek package still present) |
| UI | OLED, buzzer, LEDs, joystick / gamepad |

Runtime switches are in `ros2_ws/.typerc`: machine type, lidar, camera, mic, ASR online/offline, language, `ROS_DOMAIN_ID=27`.

---

## Architecture

```
Sensors (LiDAR, Aurora, IMU, camera, mic)
        │
        ▼
peripherals + ros_robot_controller          # drivers / STM32 serial
        │
        ▼
controller ──► servo_controller ──► legs / arm
  gait (ripple / tripod), IK, odom, action groups (*.d6a)
        ▲
        │  cmd_vel / traveling / RunActionSet
apps, Nav2, competition, LLM demos
```

Boot: systemd `start_app_node.service` → `ros2 launch bringup bringup.launch.py`  
(startup check, motion stack, camera + lidar, rosbridge, web video, app suite, joystick, init pose).

`need_compile=False` (default) means many launches read **`/home/ubuntu/ros2_ws/src/...` directly**. Do not rename that path on the robot. Python edits take effect after a node restart; msgs / C++ / installed assets need `colcon build`.

```
~/.zshrc → ros2_ws/.robotrc → .typerc
         → /opt/ros/humble
         → ros2_ws/install
         → third_party/{third_party_ws,aurora_ws,orbbec_ws,rtabmap_ws}/install
```

---

## What this repo contains

| Path | Role |
|------|------|
| `ros2_ws/src/` | All robot ROS 2 packages |
| `ros2_ws/info/` | Operator notes (`startup.md`, `desktop.md`) |
| `software/` | Qt tools + **action groups** (`ActionGroups/*.d6a`) — required at runtime |
| `wifi_manager/` | AP/STA, physical button, remote discovery |
| `git-push-all.sh` | Add / commit / push (hides nested vendor `.git` dirs) |
| `.gitignore` | Ships source only: no `build/`, `install/`, `third_party/`, `*.engine` |

**Not in git** (stay on the Jetson image): `~/third_party` (~11 GB: OpenCV, Open3D, camera/SLAM workspaces, sherpa-onnx), `ros2_ws/build|install|log`, TensorRT `*.engine` (rebuild on this JetPack).

On the Jetson the live trees **are** `~/ros2_ws` and `~/software`. A host clone can live anywhere; on Masha keep those two paths.

---

## Modules (`ros2_ws/src`)

**Bringup** — `bringup`: boot launch + `startup_check`.

**Motion** (`driver/`)

| Package | Role |
|---------|------|
| `ros_robot_controller` | Serial SDK to STM32: servos, IMU, buzzer, LED, OLED, keys |
| `servo_controller` | High-level servos + playback of `.d6a` action groups |
| `kinematics` | Hexapod IK (`kinematics.so` + Python) |
| `arm_kinematics` | Arm IK |
| `controller` | Gaits, `cmd_vel` / traveling, odom + EKF, OLED |
| `sdk` | Shared helpers (pid, led, button, fps) |
| `*_msgs` | Matching message/service packages |

Action groups path (hardcoded in several nodes):  
`/home/ubuntu/software/actionset_editor/ActionGroups/`

**Perception** — `peripherals` (lidar / Aurora / joy / teleop / IMU filter), `xf_mic_asr_offline`.

**Maps** — `slam` (2D laser + RTAB-Map 3D), `navigation` (Nav2 + TEB/DWB params).

**Apps** — `app`: self-balance, lidar avoid/follow/guard, line follow, color track, kick, hand gesture. Phone/tablet pattern: `/<app>/enter`, `/<app>/set_running`.

**Examples / contest / LLM** — `example` (gaits, OpenCV, YOLO, MediaPipe, RGB-D pick), `competition` (narrow slit, bridge, pick/place), `large_models` + `large_models_examples` (online DashScope/OpenAI or offline Ollama + sherpa).

**Shared / sim** — `interfaces` (color, ROI, `RunActionSet`, poses), `rospider_description` (URDF/xacro/meshes), `robot_moveit_config`.

**Desktop tools** (`~/software`, not colcon): `actionset_editor`, `servo_tool`, `lab_tool` (LAB color), `collect_picture`, `tool`, `roLabelImg`.

---

## Build and run (on Masha)

```bash
~/.stop_ros.sh
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.zsh

ros2 launch bringup bringup.launch.py
# or a single app, see ros2_ws/command
```

TensorRT engines must be exported **on this Jetson**. `kinematics.so` is aarch64 — the host can edit and view URDF/RViz, not run the full stack.

---

## Git

```bash
# after edits on Masha
~/git-push-all.sh "what changed"
```

Remote: `git@github.com:ganapetya/masha.git`  
SSH key (Masha → GitHub): `~/.ssh/id_ed25519_github_robots_world`  
Host → Masha SSH snippet: `Desktop/host-masha-ssh.config` (X11 forward for clipboard). Do not put passwords in SSH config; use a host key + `ssh-copy-id`.

Vendor trees still have their own `.git` (LAN Gitea). `git-push-all.sh` hides them during `git add` so this repo is one tree, not submodules.
