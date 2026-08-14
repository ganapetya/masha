# Masha / ROSpider startup

**Machine:** Hiwonder ROSpider hexapod (this unit is called Masha)  
**Brain:** NVIDIA Jetson Orin NX, Ubuntu 22.04, JetPack 6.2, ROS 2 Humble  
**Written from:** live units and launch files on 2026-08-14  
**Verified slim boot:** 2026-08-14 11:28 IDT (`start_app_node` PID 7233, `BRINGUP_PROFILE=slim`)  
**Related:** `~/ros2_ws/info/desktop.md`, `~/ROSpider_WORKSPACE_MEMO.md`, `~/ros2_ws/.typerc`, `~/ros2_ws/command`

This is what starts when you **switch the robot on**. Silent and motionless does **not** mean software is idle. Almost everything below stays running until you stop it or shut down.

**Mode recipes** (phone, camera stream, joystick, voice, one demo, SLAM, Nav, competition, Qt tools) are in **[§13](#13-optional-startup-variants)**.

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
                          ├─ init pose (`init` action group)
                          └─ extras only if BRINGUP_PROFILE=full (or flags):
                               rosbridge + web video, demo apps, joystick,
                               voice (~50 s later if /dev/ring_mic exists)
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
| `BRINGUP_PROFILE` | `slim` | `slim` = motion+sensors (default). `full` = vendor phone/demo tree. |

### Live slim boot (2026-08-14)

`start_app_node.service` restarted at 11:28:24 with `BRINGUP_PROFILE=slim`. Journal printed `BRINGUP_PROFILE: slim`. `startup_check` started, slept ~50 s, exited cleanly (beep + OLED). `oled_show` also exited cleanly right after writing.

**Present:** `/ros_robot_controller`, `/controller` + `/step_controller` (`move_controller`), `/servo_manager`, `/arm_kinematics`, `/aurora/aurora`, `/LD19`, `/ekf_filter_node`, `/imu_calib`, `/imu_filter`, `/init_pose`.

**Topics present:** `/controller/cmd_vel`, `/scan`, `/scan_raw`, `/depth_cam/rgb/image_raw`, `/imu`, `/odom`.

**Absent (correct on slim):** `perform_actions`, `lidar_app` / `lidar_controller`, `line_following`, `intelligent_kick`, `self_balancing`, `hand_gesture`, `hand_trajectory`, `joystick_control`, `rosbridge_websocket`, `web_video_server`, `wonder_echo_pro_node`, `voice_control_move`.

**Ports:** 22, 4000, 9026, 9027 open. **8080 and 9090 closed.** `/dev/ring_mic` existed (`ttyCH341USB0`) and voice still did **not** start.

---

## 2. Robot systemd services

These live in `/etc/systemd/system/` and are **enabled** for `multi-user.target` unless noted. They start as soon as the network stack is up, **before** (or beside) the desktop.

| Service | Script / command | What it does |
|---|---|---|
| **`start_app_node.service`** | `ros2 launch bringup bringup.launch.py` (after sourcing `~/.zshrc`) | **Main robot stack.** Default **`BRINGUP_PROFILE=slim`**: motion + sensors + init pose. `full` also starts phone APIs and every demo app. `DISPLAY=:0`. `Restart=no` — if this dies, it does not come back by itself. |
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
| **`update-notifier`** | **Disabled** (2026-08-14). Used to launch `update-manager` ~60 s after login; that GUI often busy-looped at ~90% CPU. Override: `~/.config/autostart/update-notifier.desktop`. Timers `update-notifier-download` / `update-notifier-motd` disabled. |
| **`nvpmodel_indicator`** | **Disabled.** NVIDIA tray applet. Power mode is already set by `nvpmodel.service` (10W). It was forking ~21 copies. |
| **`gnome-software` / `packagekitd`** | **Disabled / masked.** Ubuntu Software store. `packagekit.service` is masked so it cannot come back. |

Desktop icons (`~/Desktop/`) are **not** started at boot. You click them later (Tool, ROSpider action editor, SLAM, Navigation). See `~/ros2_ws/info/desktop.md`.

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

Default profile is **`slim`** (from `BRINGUP_PROFILE` in `~/ros2_ws/.typerc`). Slim starts motion + sensors + init pose. The old vendor tree is **`profile:=full`**. Launch arguments override the profile; `auto` (the flag default) follows it.

| Argument | `slim` | `full` | Meaning |
|---|---|---|---|
| `profile` | `slim` | `full` | Master switch. Env: `BRINGUP_PROFILE`. |
| `start_apps` | off | on | Include `start_app.launch.py` (all demo apps). |
| `joystick` | off | on | USB gamepad node. |
| `rosbridge` | off | on | Phone WebSocket on **:9090**. |
| `web_video` | off | on | MJPEG server on **:8080**. |
| `voice` | off | on | `startup_check` may launch the voice stack if `/dev/ring_mic` exists. |

Launch-argument defaults also read optional env vars from `.typerc`: `BRINGUP_PROFILE`, `BRINGUP_START_APPS`, `BRINGUP_JOYSTICK`, `BRINGUP_ROSBRIDGE`, `BRINGUP_WEB_VIDEO`, `BRINGUP_VOICE`. Unset / `auto` follows the profile.

```bash
# persist vendor boot — edit .typerc, then:
sudo systemctl restart start_app_node.service

# one-shot (stop the service first; do not run a second bringup beside it)
sudo systemctl stop start_app_node.service
ros2 launch bringup bringup.launch.py profile:=full

# slim + only the camera HTTP stream (one-shot)
ros2 launch bringup bringup.launch.py web_video:=true
```

The parent is still one `ros2 launch` process tree. Slim is lighter because the demo/phone/voice extras are not started. Copy-paste recipes for every mode: **[§13](#13-optional-startup-variants)**.

### 5.1 Startup check

| Node | Package | What it does |
|---|---|---|
| **`startup_check`** | `bringup` | Waits ~50 s, beeps the buzzer once, writes **SSID** and **IP** on the OLED. Parameter **`enable_voice`** (default false; bringup sets it from the `voice` flag). Only then, if `/dev/ring_mic` exists, it runs `ros2 launch xf_mic_asr_offline startup_test.launch.py` (voice stack, §7). |

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

### 5.5 Phone / web APIs (`full` or `rosbridge` / `web_video`)

Off on **`slim`**.

| Node | Port | What it does |
|---|---|---|
| **`rosbridge_websocket`** + **`rosapi`** | **9090** | WebSocket bridge. The Hiwonder phone/tablet app talks ROS through this. |
| **`web_video_server`** | **8080** | Serves camera topics as MJPEG. Example: `http://<IP>:8080/stream?topic=/depth_cam/rgb/image_raw`. |

### 5.6 Joystick (`full` or `joystick:=true`)

Off on **`slim`**.

| Node | What it does |
|---|---|
| **`joystick_control`** | Reads a USB gamepad (`/joy`) and sends motion / arm commands to `controller/cmd_vel` and the arm IK. Costs a noticeable amount of CPU even with no pad. |

---

## 6. Demo apps (`start_app.launch.py`) — `full` or `start_apps:=true`

**Not started on `slim`.** On `full`, all of these start together. They sit idle until the phone app (or a service call) **enables** them, but the processes stay resident. Several still subscribe to camera/lidar/IMU.

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

## 7. Voice stack (`full` or `voice:=true`, then only if the mic is present)

Off on **`slim`**. On `full`, `startup_check` launches this ~50 s after bringup **only** when `/dev/ring_mic` exists (udev rule for the WonderEcho / iFlytek array). Competition launches its own mic stack when you start that launch; it does not need boot voice.

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
| WonderEcho Pro / iFlytek XFM-DP | Idle on slim. Listening for wake / commands only if voice is enabled |
| GeneralPlus USB audio (`1b3f:2008`) | Speaker for TTS / prompts |
| HDMI dummy plug `HDP-V104` | Fake monitor so GNOME/NoMachine work |
| Wi‑Fi + GPIO 24 LED | AP/STA + status blink |
| GPIO 25 / GPIO 4 buttons | Reset Wi‑Fi / shutdown |

---

## 9. Network ports that are open after a normal boot

| Port | Process | Use | When |
|---|---|---|---|
| 22 | `sshd` | SSH | Always |
| 4000 | NoMachine `nxd` | Remote desktop | Always |
| 8080 | `web_video_server` | Camera HTTP/MJPEG | `full` or `web_video:=true` |
| 9090 | `rosbridge_websocket` | Phone app ↔ ROS | `full` or `rosbridge:=true` |
| 9026 | `remote.py` | Phone sets Wi‑Fi | systemd (not ROS) |
| 9027 UDP | `find_device.py` | Phone discovers robot | systemd (not ROS) |

---

## 10. What does **not** start at power-on

| Thing | When it starts |
|---|---|
| Phone demos (`start_app.launch.py`) | `BRINGUP_PROFILE=full` or `start_apps:=true` |
| rosbridge / web video / joystick / boot voice | `full` or the matching flag |
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
| `update-manager` | ~90% **if stuck** | Autostart **disabled** (see §3). If it reappears: `pkill -x update-manager` |
| `aurora930_node` | ~70% | Depth camera always streaming |
| `joystick_control` | ~20% | Gamepad node; **slim does not start this** |
| `ros_robot_controller` | ~18% | STM32 / IMU loop |
| `intelligent_kick` | ~14% | Vision demo; **slim does not start this** |
| `joint_state_publisher` | ~13% | Joints published forever |
| `self_balancing` | ~11% | Demo; **slim does not start this** |
| Arm IK + `servo_controller` | ~20% together | Servo / IK loops |
| Lidar + the rest | a few % each | Lidar always on. rosbridge only on `full` / `rosbridge:=true` |

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

# Confirm which bringup profile the service actually has
MAIN_PID=$(systemctl show -p MainPID --value start_app_node.service)
tr '\0' '\n' < /proc/$MAIN_PID/environ | grep -E 'BRINGUP_|need_compile|ROS_DOMAIN'

# What this launch file can take
ros2 launch bringup bringup.launch.py --show-args
```

---

## 13. Optional startup variants

One rule: **only one motion/sensor tree at a time**. `start_app_node.service` already owns `ros2 launch bringup bringup.launch.py`. Do not start a second bringup (or SLAM/Nav/`debug:=true` app launches) beside it — they fight for the STM32, camera, and lidar.

| Kind | When to use | How |
|---|---|---|
| **Persist** | Next reboot / every `systemctl start` | Edit `~/ros2_ws/.typerc`, then `sudo systemctl restart start_app_node.service` |
| **One-shot** | This session only, keep `.typerc` slim | `sudo systemctl stop start_app_node.service`, then `ros2 launch …` in the foreground |
| **Overlay** | Extra node on the **running slim** stack | Leave the service up; launch a node that does **not** include `controller.launch.py` |
| **Replace** | SLAM, Nav, competition, `debug:=true` apps | Stop the service first (those launches start their own robot tree) |

Always `source ~/.zshrc` (or use a login shell) so `.typerc` is loaded. After a one-shot / replace session, return to default slim with `sudo systemctl start start_app_node.service`.

`ros2 launch bringup bringup.launch.py --show-args` lists every flag.

### 13.1 Persist: what boots every time

Edit `~/ros2_ws/.typerc` (or use desktop **Tool** for camera/lidar/ASR — Tool does **not** have a `BRINGUP_PROFILE` combobox). Then restart:

```bash
sudo systemctl restart start_app_node.service
```

| Goal | `.typerc` lines |
|---|---|
| **Developer platform (default)** | `export BRINGUP_PROFILE=slim` |
| **Vendor phone / all demos** | `export BRINGUP_PROFILE=full` |
| Slim + browser camera (`:8080`) | `slim` and `export BRINGUP_WEB_VIDEO=true` |
| Slim + Hiwonder phone bridge (`:9090`) | `slim` and `export BRINGUP_ROSBRIDGE=true` |
| Slim + USB gamepad | `slim` and `export BRINGUP_JOYSTICK=true` |
| Slim + boot voice (if `/dev/ring_mic`) | `slim` and `export BRINGUP_VOICE=true` |
| Slim + all demo apps, no phone | `slim` and `export BRINGUP_START_APPS=true` |

Unset those `BRINGUP_*` flags (or set them to `auto`) to follow the profile again.

Desktop **Tool → Apply** restarts `start_app_node` and `find_device`. After you change `BRINGUP_PROFILE` by hand, Apply is enough; you do not need a reboot.

### 13.2 One-shot: this session only

Stop the service so you do not get two trees:

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service
```

Then pick one:

```bash
# default slim (same as the service)
ros2 launch bringup bringup.launch.py

# old vendor appliance (phone + demos + joystick + voice-if-mic)
ros2 launch bringup bringup.launch.py profile:=full

# slim + MJPEG for a browser / VLC
ros2 launch bringup bringup.launch.py web_video:=true
# stream: http://<IP>:8080/stream?topic=/depth_cam/rgb/image_raw

# slim + phone WebSocket, no demos
ros2 launch bringup bringup.launch.py rosbridge:=true
# app: ws://<IP>:9090

# slim + gamepad
ros2 launch bringup bringup.launch.py joystick:=true

# slim + boot voice (~50 s later if /dev/ring_mic exists)
ros2 launch bringup bringup.launch.py voice:=true

# slim + every demo app (kick, line, gesture, lidar, balance, perform)
ros2 launch bringup bringup.launch.py start_apps:=true

# mix: camera in the browser + gamepad, still no demos
ros2 launch bringup bringup.launch.py web_video:=true joystick:=true

# vendor tree but skip the demo bundle
ros2 launch bringup bringup.launch.py profile:=full start_apps:=false
```

Ctrl+C stops that launch. Bring the default back:

```bash
sudo systemctl start start_app_node.service
```

### 13.3 Overlay: one extra tool on running slim

Leave `start_app_node` up. These do **not** start a second controller (omit `debug:=true`).

```bash
source ~/.zshrc

# keyboard drive (SSH / extra terminal)
ros2 launch peripherals teleop_key_control.launch.py

# one demo node only — then enter / set_running as in desktop.md §8
ros2 launch app lidar_node.launch.py
ros2 launch app line_following_node.launch.py
ros2 launch app intelligent_kick_node.launch.py
ros2 launch app self_balancing_node.launch.py
ros2 launch app hand_gesture.launch.py
ros2 launch app object_tracking_node.launch.py   # not in start_app.launch.py

# voice without rebuilding bringup (mic must exist)
ros2 launch xf_mic_asr_offline voice_control_move.launch.py
# or the same stack startup_check would start:
# ros2 launch xf_mic_asr_offline startup_test.launch.py
```

Enable a demo after it is loaded:

```bash
# lidar: 1=avoid  2=follow  3=guard
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 1}"

ros2 service call /line_following/enter std_srvs/srv/Trigger {}
ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: true}"

ros2 service call /intelligent_kick/enter std_srvs/srv/Trigger {}
ros2 service call /intelligent_kick/set_running std_srvs/srv/SetBool "{data: true}"

ros2 service call /hand_gesture/enter std_srvs/srv/Trigger {}
ros2 service call /hand_gesture/set_running std_srvs/srv/SetBool "{data: true}"
```

Drive without a GUI:

```bash
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

### 13.4 Isolated debug: one app with its own robot tree

`debug:=true` on an app launch **includes** `controller.launch.py` (and often lidar/camera). Stop bringup first.

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service

ros2 launch app lidar_node.launch.py debug:=true
ros2 launch app line_following_node.launch.py debug:=true
ros2 launch app intelligent_kick_node.launch.py debug:=true
ros2 launch app object_tracking_node.launch.py debug:=true
ros2 launch app self_balancing_node.launch.py debug:=true
ros2 launch app hand_gesture.launch.py debug:=true
```

Line-follow / kick / track: click the debug image to pick a color, then `set_running`.

### 13.5 SLAM

Conflicts with bringup. Desktop icon **SLAM** runs `~/ros2_ws/src/bringup/scripts/slam.sh` (stops the service, opens gnome-terminal tabs). Needs NoMachine / a display.

SSH / tmux:

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service

# 2D mapping
ros2 launch slam slam.launch.py enable_save:=false
# other terminals:
ros2 launch peripherals teleop_key_control.launch.py
rviz2 -d ~/ros2_ws/src/slam/rviz/slam_desktop.rviz

# save
cd ~/ros2_ws/src/slam/maps
ros2 run nav2_map_server map_saver_cli -f "map_01" \
  --ros-args -p map_subscribe_transient_local:=true

# 3D (Aurora)
ros2 launch slam rtabmap_slam.launch.py
ros2 launch slam rviz_rtabmap.launch.py
```

### 13.6 Navigation

Conflicts with bringup. Desktop icon **Navigation** runs `~/ros2_ws/src/bringup/scripts/navigation.sh` (`map:=map_01`).

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service

ros2 launch navigation navigation.launch.py map:=map_01
rviz2 -d ~/ros2_ws/src/navigation/rviz/navigation_desktop.rviz

# 3D
ros2 launch navigation rtabmap_navigation.launch.py
ros2 launch navigation rviz_rtabmap_navigation.launch.py
```

### 13.7 Competition

`competition.launch.py` pulls in `slam/include/robot.launch.py` plus mic. **Stop bringup first.**

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service
ros2 launch competition competition.launch.py

ros2 service call /narrow_slit_traversal/enter std_srvs/srv/Trigger {}
ros2 service call /cross_bridge/enter std_srvs/srv/Trigger {}
ros2 service call /automatic_pick/pick std_srvs/srv/Trigger {}
ros2 service call /automatic_pick/place std_srvs/srv/Trigger {}
```

Competition starts its own mic. You do not need `voice:=true` on bringup.

### 13.8 Large-model / LLM demos

Set `ASR_MODE` / `ASR_LANGUAGE` in `.typerc` first (`online` or `offline`). These launches are their own stacks — stop bringup unless you know the example only adds a node.

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service

ros2 launch large_models_examples llm_control_move.launch.py
ros2 launch large_models_examples llm_color_track.launch.py
ros2 launch large_models_examples llm_visual_patrol.launch.py
ros2 launch large_models_examples vllm_with_camera.launch.py
ros2 launch large_models_examples vllm_navigation.launch.py map:=map_01
```

`ollama.service` is disabled. Offline mode needs Ollama started by hand if the example requires it.

### 13.9 Sensors / IMU / URDF only

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service   # if the device is already claimed

ros2 launch peripherals depth_camera.launch.py
ros2 launch peripherals lidar_view.launch.py
ros2 launch peripherals imu_view.launch.py
ros2 launch rospider_description display.launch.py

# IMU calib
ros2 launch ros_robot_controller ros_robot_controller.launch.py
ros2 run imu_calib do_calib --ros-args \
  -r imu:=/ros_robot_controller/imu_raw \
  --param output_file:=/home/ubuntu/ros2_ws/src/peripherals/config/imu_calib.yaml
```

Vendor gait/IK classroom launches live in `~/ros2_ws/command` (`example forward_and_rorate`, `body_ik`, …). Treat them as **replace** (stop bringup).

### 13.10 Qt / desktop tools (NoMachine)

These do not replace bringup. Slim can stay up. Need `DISPLAY` (NoMachine `:4000` or the dummy HDMI session).

| Tool | Start | Needs |
|---|---|---|
| **Tool** (camera / lidar / ASR in `.typerc`) | desktop icon or `zsh ~/software/tool/tool.sh` | then Apply to restart bringup |
| **ROSpider** action-group editor | desktop icon or `zsh ~/software/actionset_editor/actionset_editor.sh` | slim bringup if you want to play `.d6a` on the robot |
| **servo_tool** | `zsh ~/software/servo_tool/servo_tool.sh` | stop conflicting servo nodes if it cannot open the bus |
| **lab_tool** | `zsh ~/software/lab_tool/lab_tool.sh` | camera topic (slim is enough) |
| **collect_picture** | `zsh ~/software/collect_picture/collect_picture.sh` | camera topic |
| **RViz** (camera / scan) | `rviz2` | slim already publishes `/scan` and `/depth_cam/...` |

More GUI ↔ CLI mapping: `~/ros2_ws/info/desktop.md`. The long vendor cheat sheet is `~/ros2_ws/command`.

### 13.11 Quick chooser

| I want… | Start |
|---|---|
| Quiet developer boot (motion + Aurora + lidar) | Default. Nothing to do. |
| See the camera in a browser | Persist `BRINGUP_WEB_VIDEO=true`, or one-shot `web_video:=true` |
| Hiwonder phone app | `profile:=full`, or slim + `rosbridge:=true` (and `start_apps` / `web_video` if the app needs them) |
| USB gamepad | `joystick:=true` |
| Voice walk commands | `voice:=true`, or overlay `voice_control_move.launch.py` |
| One vision demo | Overlay the matching `ros2 launch app …` (no `debug`) |
| Phone-style “everything” | `profile:=full` |
| Map the room | §13.5 — stop bringup, SLAM icon or `slam.launch.py` |
| Navigate a saved map | §13.6 — stop bringup, Navigation icon or `navigation.launch.py` |
| Contest tasks | §13.7 — stop bringup, `competition.launch.py` |
| LLM voice/vision demos | §13.8 |
| Edit actions / LAB / servos | §13.10 NoMachine + Qt |
| Back to default slim | `sudo systemctl start start_app_node.service` |
