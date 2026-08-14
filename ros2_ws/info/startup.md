# Masha / ROSpider startup

**Machine:** Hiwonder ROSpider hexapod (this unit is called Masha)  
**Brain:** NVIDIA Jetson Orin NX, Ubuntu 22.04, JetPack 6.2, ROS 2 Humble  
**Written from:** live units and launch files on 2026-08-14  
**Related:** `~/ros2_ws/info/desktop.md`, `~/ROSpider_WORKSPACE_MEMO.md`, `~/ros2_ws/.typerc`

This is what starts when you **switch the robot on**. Silent and motionless does **not** mean software is idle. Almost everything below stays running until you stop it or shut down.

---

## 1. Boot sequence (order)

```
Power on
  → Jetson UEFI / L4T bootloader
  → Linux kernel 5.15.148-tegra
  → systemd  (default.target → graphical.target)
      ├─ NVIDIA Jetson platform services
      ├─ Ubuntu core (network, ssh, docker, …)
      ├─ Robot systemd services  (wifi, buttons, bringup, …)
      └─ GDM display manager
           → autologin user `ubuntu`
           → GNOME on X11  (HDMI dummy plug pretends there is a 1080p monitor)
                ├─ PulseAudio, Update Notifier, nvpmodel tray, …
                └─ (in parallel) start_app_node.service
                     → source ~/.zshrc → ros2_ws/.robotrc → .typerc
                     → ros2 launch bringup bringup.launch.py
                          ├─ motion + IMU + servos + OLED
                          ├─ Aurora depth camera
                          ├─ LD19 lidar
                          ├─ rosbridge + web video
                          ├─ ALL demo apps (kick, balance, line, gesture, lidar)
                          ├─ joystick
                          ├─ init pose (`init` action group)
                          └─ after ~50 s + if /dev/ring_mic exists:
                               voice stack (WonderEcho Pro + voice_control_move)
```

Environment used by every ROS process (from `~/ros2_ws/.typerc`):

| Variable | Current value | Meaning |
|---|---|---|
| `MACHINE_TYPE` | `ROSpider` | Hexapod body |
| `LIDAR_TYPE` | `LD19` | Lidar driver launch |
| `DEPTH_CAMERA_TYPE` | `aurora` | Deptrum Aurora 930 |
| `MIC_TYPE` | `WonderEchoPro` | iFlytek/WonderEcho circular array |
| `ASR_LANGUAGE` | `English` | Wake / command language |
| `ASR_MODE` | `online` | Large-model ASR path (voice nodes still start offline kit) |
| `ROS_DOMAIN_ID` | `27` | DDS domain; other machines must match |
| `need_compile` | `False` | Launches read sources under `/home/ubuntu/ros2_ws/src` |

---

## 2. Robot systemd services

These live in `/etc/systemd/system/` and are **enabled** for `multi-user.target` unless noted. They start as soon as the network stack is up, **before** (or beside) the desktop.

| Service | Script / command | What it does |
|---|---|---|
| **`start_app_node.service`** | `ros2 launch bringup bringup.launch.py` (after sourcing `~/.zshrc`) | **Main robot stack.** Starts motion, sensors, phone APIs, and every demo app. `DISPLAY=:0`. `Restart=no` — if this dies, it does not come back by itself. |
| **`wifi.service`** | `~/wifi_manager/wifi.py` | Puts the Wi‑Fi radio in AP or STA mode. Default AP SSID is `WN-` + first 8 chars of the Jetson serial. Blinks the Wi‑Fi LED (GPIO 24). Reads `/etc/wifi/wifi_conf.py`. |
| **`button_scan.service`** | `~/wifi_manager/button_scan.py` | Watches two physical buttons. **GPIO 25 (short):** wipe `/etc/wifi/*` and restart `wifi.service` (back to AP). **GPIO 4 (long hold):** beep and `sudo halt` (power off). |
| **`find_device.service`** | `~/wifi_manager/find_device.py` | UDP listener on **port 9027**. The Hiwonder phone app broadcasts `LOBOT_NET_DISCOVER`; this answers with robot type, SSID, image size so the app can find Masha on the LAN. |
| **`remote.service`** | `~/wifi_manager/remote.py` | TCP listener on **port 9026**. Phone app sends JSON `{setwifi: {ssid, passwd}}`; this writes STA config to `/etc/wifi/wifi_conf.py` and restarts `wifi.service`. |
| **`set_default_device.service`** | sleep 10s, then `~/ros2_ws/.set_default_device.sh` | One-shot. Sets PulseAudio **default speaker** to the GeneralPlus USB audio device and **default mic** to the iFlytek XFM-DP. Then exits (`Restart=no`). |
| **`jtop.service`** | `/usr/local/bin/jtop --force` | Background daemon for the `jtop` Jetson stats tool (temps, clocks, GPU). Does not drive the robot. |
| **`expand_rootfs.service`** | `~/ros2_ws/.expand_rootfs.sh` | **Disabled.** One-shot first-boot disk expand. Not part of a normal power-on. |
| **`ollama.service`** | `ollama serve` | **Disabled.** Local LLM server. Not started at boot. |

Check them:

```bash
systemctl status start_app_node wifi button_scan find_device remote set_default_device jtop
```

---

## 3. Display, desktop, remote access

The Jetson has **no real monitor**. An **HDMI dummy plug** (EDID name `HDP-V104`, labeled something like “HDMI UHD”) is plugged into HDMI so the GPU thinks a 1920×1080 screen exists. That is why GNOME starts on a headless robot.

| Service / process | What it does |
|---|---|
| **`gdm.service`** | GNOME Display Manager. `/etc/gdm3/custom.conf` has `AutomaticLoginEnable=True` / `AutomaticLogin=ubuntu`. Wayland is off (`WaylandEnable=false`). |
| **Xorg + `gnome-shell`** | Full Ubuntu desktop on the fake 1080p display. Needed for some GUI tools and for NoMachine. Extra CPU/GPU overhead. |
| **`nxserver.service`** | NoMachine remote desktop, TCP **4000**. This is how you see the GNOME desktop from another PC. |
| **`ssh.service`** | OpenSSH on port **22**. Preferred way to operate without the GUI. |
| **`docker.service` + `containerd`** | Docker engine. Enabled at boot even if you are not using containers. |
| **PulseAudio** (user service) | Sound server. Default **output** = USB speaker (GeneralPlus). Default **input** = iFlytek mic array. |
| **`update-notifier`** (autostart, ~60 s after login) | Ubuntu “check for updates” helper. It launches **`update-manager --no-update --no-focus-on-map`**, the Software Updater GUI. That process is **not** part of the robot. On this headless/dummy-HDMI setup it often **busy-loops at ~90% CPU**. Safe to kill. |
| **`nvpmodel_indicator`** | NVIDIA tray applet for power mode (this board is **10W**). Can spawn many copies; they do not move the robot. |
| **`gnome-software` / `packagekitd`** | Ubuntu Software / package kit. Desktop only. |

Desktop icons (`~/Desktop/`) are **not** started at boot. You click them later (Tool, ROSpider action editor, SLAM, Navigation). See `~/info/desktop.md`.

---

## 4. NVIDIA / Jetson platform services (always on)

These come with JetPack. They are not ROSpider apps. Short list of the ones that matter:

| Service | What it does |
|---|---|
| **`nvpmodel.service`** | Sets the power mode (here: **10W**). Runs once and exits. |
| **`nvpower.service`** | CPU/GPU/EMC power and clock policy. |
| **`nvfancontrol.service`** | Fan curve. |
| **`nvargus-daemon.service`** | Camera ISP stack (CSI cameras). Aurora is USB, so this is mostly unused here. |
| **`nvs-service.service`** | NVIDIA sensor HAL. |
| **`nvphs.service`** | Power/thermal hint service. |
| **`nv-tee-supplicant.service`** | OP-TEE trusted execution helper. |
| **`nvzramconfig.service`** | zram swap (~18 GB on this image). |
| **`nv-l4t-usb-device-mode.service`** | USB gadget / `l4tbr0` (`192.168.55.1`) so a PC can reach the Jetson over USB. |
| **`nvfb` / `nvfb-early`** | Framebuffer / splash. |
| **`nvweston.service`** | Weston compositor helper (GNOME/Xorg is what you actually use). |

---

## 5. Main ROS launch: `bringup.launch.py`

Started by **`start_app_node.service`**:

```text
ros2 launch bringup bringup.launch.py
```

File: `~/ros2_ws/src/bringup/launch/bringup.launch.py`

It includes **everything below** in one process tree (parent is `ros2 launch`). That is why CPU stays high when the legs are still.

### 5.1 Startup check

| Node | Package | What it does |
|---|---|---|
| **`startup_check`** | `bringup` | Waits ~50 s, beeps the buzzer once, writes **SSID** and **IP** on the OLED. In a side thread: if `/dev/ring_mic` exists, runs `ros2 launch xf_mic_asr_offline startup_test.launch.py` (voice stack, §6). |

### 5.2 Motion stack (`controller.launch.py`)

| Node / launch | What it does |
|---|---|
| **`ros_robot_controller`** | Talks to the **STM32** board over USB serial (`/dev/ttyACM0`). IMU raw data, buzzer, OLED, buttons on the controller board, low-level I/O. |
| **`robot_state_publisher`** | Publishes TF from the hexapod URDF (`rospider.xacro`). Tells ROS where each link is. |
| **`joint_state_publisher`** | Publishes `/joint_states` from the model (and servo feedback). Always ticking. |
| **`imu_calib` (`apply_calib`)** | Applies the saved IMU calibration (`imu_calib.yaml`) to `/ros_robot_controller/imu_raw` → `/imu_corrected`. |
| **`imu_filter` (Madgwick)** | Turns corrected accel/gyro into a filtered orientation on `/imu`. |
| **`ekf_filter_node`** | `robot_localization` EKF. Fuses IMU (and odom if enabled) into `/odom` and TF `odom` → `base_footprint`. |
| **`servo_controller`** | Bus-servo driver. Sends positions/durations to all leg and arm servos. Holds pose even when you are not walking. |
| **`search_kinematics_solutions`** | Arm inverse-kinematics server. Other nodes call it to place the gripper. |
| **`move_controller`** | Gait / walking controller. Listens for travel / `cmd_vel`-style commands and turns them into servo steps (ripple/tripod etc.). Idle still means the node is spinning. |
| **`oled_show`** | Extra OLED helper (SSID/IP style). `startup_check` also writes the OLED. |
| **`init_pose`** | Started from bringup with `action_name:=init`. Plays the **`init` action group** (`.d6a`) so the hexapod stands in the default pose after boot. |

`odom_publisher` is **commented out** in `odom_publisher.launch.py`; odom currently comes from the EKF, not that node.

### 5.3 Depth camera (`depth_camera.launch.py` → Aurora)

Because `DEPTH_CAMERA_TYPE=aurora`:

| Node | What it does |
|---|---|
| **`aurora930_node`** (`/aurora/aurora`) | Deptrum **Aurora 930** driver. Streams RGB, IR, depth, and point cloud. Remapped to `/depth_cam/...`. This is the **largest always-on CPU load** (~70% of one core) even when nobody is looking at the camera. |

### 5.4 Lidar (`lidar.launch.py` → LD19)

| Node | What it does |
|---|---|
| **`LD19`** (`ldlidar_stl_ros2_node`) | LD19 lidar driver. Publishes raw scans on `/scan_raw`. The puck is spinning the whole time the robot is on. |
| **`scan_to_scan_filter_chain`** | `laser_filters`. Crops/filters the scan (`lidar_filters_config.yaml`) and publishes `/scan`. |

### 5.5 Phone / web APIs

| Node | Port | What it does |
|---|---|---|
| **`rosbridge_websocket`** + **`rosapi`** | **9090** | WebSocket bridge. The Hiwonder phone/tablet app talks ROS through this. |
| **`web_video_server`** | **8080** | Serves camera topics as MJPEG. Example: `http://<IP>:8080/stream?topic=/depth_cam/rgb/image_raw`. |

### 5.6 Joystick

| Node | What it does |
|---|---|
| **`joystick_control`** | Reads a USB gamepad (`/joy`) and sends motion / arm commands to `controller/cmd_vel` and the arm IK. Always running so a pad works without starting another launch. Costs a noticeable amount of CPU even with no pad. |

---

## 6. Demo apps started at boot (`start_app.launch.py`)

All of these start together. They sit idle until the phone app (or a service call) **enables** them, but the processes stay resident. Several still subscribe to camera/lidar/IMU.

| Node | What it does |
|---|---|
| **`perform_actions`** | Plays built-in body motions / action sequences (twists, canned shows) when asked. Used by the app “perform action” buttons. |
| **`lidar_controller`** (`lidar_app`) | **Lidar obstacle avoidance** and **lidar follow**. Uses `/scan`. Off until enabled. |
| **`line_following`** | **Color line following** with the camera (and lidar for obstacles). Off until enabled. |
| **`intelligent_kick`** | **Smart kick**: sees a colored ball with the camera, walks to it, kicks with a leg/arm. Heavy when enabled; still loaded at boot. |
| **`self_balancing`** | **IMU self-balance**: uses `/imu` and kinematics to keep the body level on a slope/tilt. Loaded at boot. |
| **`hand_gesture`** | **Gesture control**: maps recognized hand gestures to walk / actions. |
| **`hand_trajectory`** | MediaPipe hand tracker. Publishes fingertip / trajectory points on the camera image for `hand_gesture`. |

`object_tracking` exists as a launch file but is **not** included in `start_app.launch.py`, so it does **not** start at boot.

---

## 7. Voice stack (started ~50 s after bringup, if the mic is present)

`startup_check` only launches this when `/dev/ring_mic` exists (udev rule for the WonderEcho / iFlytek array).

Launch: `ros2 launch xf_mic_asr_offline startup_test.launch.py`

Because `MIC_TYPE=WonderEchoPro` (not `xf`), `mic_init.launch.py` starts the WonderEcho path, not the old 6-mic iFlytek ASR nodes.

| Node | What it does |
|---|---|
| **`wonder_echo_pro_node`** (`awake` / ASR front-end) | Serial protocol on `/dev/ring_mic`. Detects wake-up and a small set of **on-device commands** (English: go forward/backward, turn left/right, move left/right, dance, come here). Publishes `~/voice_words`. |
| **`voice_control_move`** | Turns those words into motion: walk, turn, dance action, or “come here” (lidar follow). Uses `/scan` for the follow behavior. |

The **iFlytek XFM-DP** USB device is the **microphone** (listen only). Playback goes to the **GeneralPlus USB speaker** (separate gadget on the same USB hub). The circular mic array does **not** speak.

If `MIC_TYPE` were `xf`, boot would instead start `awake_node`, `asr_node`, and `voice_control` (offline iFlytek ASR). That path is **not** used on this image.

---

## 8. Hardware those apps talk to

| Hardware | Role at boot |
|---|---|
| STM32 `ros_robot_controller` (`/dev/ttyACM0`) | Servos, IMU, buzzer, OLED |
| Bus servos (legs + arm) | Held in `init` pose |
| Deptrum Aurora 930 | Depth + RGB streaming |
| LD19 lidar | Scanning |
| WonderEcho Pro / iFlytek XFM-DP | Listening for wake / commands |
| GeneralPlus USB audio (`1b3f:2008`) | Speaker for TTS / prompts |
| HDMI dummy plug `HDP-V104` | Fake monitor so GNOME/NoMachine work |
| Wi‑Fi + GPIO 24 LED | AP/STA + status blink |
| GPIO 25 / GPIO 4 buttons | Reset Wi‑Fi / shutdown |

---

## 9. Network ports that are open after a normal boot

| Port | Process | Use |
|---|---|---|
| 22 | `sshd` | SSH |
| 4000 | NoMachine `nxd` | Remote desktop |
| 8080 | `web_video_server` | Camera HTTP/MJPEG |
| 9090 | `rosbridge_websocket` | Phone app ↔ ROS |
| 9026 | `remote.py` | Phone sets Wi‑Fi |
| 9027 UDP | `find_device.py` | Phone discovers robot |

---

## 10. What does **not** start at power-on

| Thing | When it starts |
|---|---|
| SLAM (`slam.sh` / desktop icon) | Manual |
| Nav2 navigation (`navigation.sh`) | Manual |
| Large-model / LLM demos (`large_models`) | Manual |
| `ollama serve` | Service disabled |
| Action-set editor, Tool, lab_tool, servo_tool | Desktop click |
| Object tracking app | Separate launch, not in `start_app` |
| Rootfs expand | Service disabled |

---

## 11. Why it runs hot while “doing nothing”

Typical always-on cost after boot (order-of-magnitude, one sample):

| Process | Approx. CPU | Why |
|---|---|---|
| `update-manager` | ~90% **if stuck** | Ubuntu Software Updater bug; not the robot. `pkill -f update-manager` |
| `aurora930_node` | ~70% | Depth camera always streaming |
| `joystick_control` | ~20% | Gamepad node always spinning |
| `ros_robot_controller` | ~18% | STM32 / IMU loop |
| `intelligent_kick` | ~14% | Vision demo loaded |
| `joint_state_publisher` | ~13% | Joints published forever |
| `self_balancing` | ~11% | Demo loaded |
| Arm IK + `servo_controller` | ~20% together | Servo / IK loops |
| Lidar + rosbridge + the rest | a few % each | Always on |

GPU (`GR3D`) is often **0%** at idle. Heat is **CPU + camera + the always-on ROS tree**, plus a full GNOME session on the dummy HDMI plug.

---

## 12. Useful commands

```bash
# What systemd started for the robot
systemctl status start_app_node wifi button_scan find_device remote set_default_device

# ROS nodes that came up with bringup
ros2 node list

# Stop the whole robot stack (legs freeze in last pose; sensors stop)
sudo systemctl stop start_app_node

# Start it again
sudo systemctl start start_app_node

# Kill the stuck Ubuntu updater only
pkill -f update-manager

# Live Jetson load / temps
tegrastats
jtop
```
