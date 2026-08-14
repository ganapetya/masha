# ROSpider Desktop, Remote Access & Terminal Equivalents

**Machine:** Hiwonder ROSpider (Jetson Orin NX)  
**Last updated:** 2026-08-12  
**Purpose:** Map desktop / NoMachine / phone-app interfaces to terminal commands so you can operate without the GUI.

Related docs:

- `~/ros2_ws/info/startup.md` — what starts at power-on
- `~/ROSpider_WORKSPACE_MEMO.md` — full workspace architecture
- `~/ros2_ws/command` — vendor launch/service cheat sheet

---

## 1. Remote access stack

| Interface | Port | Protocol | Role |
|-----------|------|----------|------|
| SSH | `22` | TCP | Terminal / automation (preferred) |
| NoMachine (NX) | `4000` | TCP (default) | Full remote desktop (GUI tools, RViz, Qt apps) |
| rosbridge WebSocket | `9090` | WS | Phone/tablet Hiwonder app ↔ ROS |
| web_video_server | `8080` | HTTP | MJPEG camera streams for browser/VLC |
| Phone WiFi config | `9026` | TCP | App sets STA SSID/password |
| Device discovery | `9027` | UDP | App finds robot (`LOBOT_NET_DISCOVER`) |

### Observed listening ports (typical with bringup up)

- `22` — sshd  
- `4000` — NoMachine `nxd`  
- `8080` — `web_video_server`  
- `9090` — `rosbridge_websocket`  
- `9026` — `remote.py` (wifi set from phone)  
- NoMachine locals: `127.0.0.1:12001`, `7001`, `25001`, etc.

### Network (example snapshot)

| Interface | Address | Notes |
|-----------|---------|--------|
| wlan0 | e.g. `192.168.0.156/24` | Main STA WiFi |
| l4tbr0 | `192.168.55.1/24` | Jetson USB device mode bridge |
| docker0 | `172.17.0.1/16` | Docker |

### Client URLs (replace IP)

```text
NoMachine:  connect to <IP>:4000
Camera:     http://<IP>:8080/
            http://<IP>:8080/stream?topic=/depth_cam/rgb/image_raw
rosbridge:  ws://<IP>:9090
SSH:        ssh ubuntu@<IP>
```

**Note:** Docs sometimes mention VLC; it is not a vendor service. People open HTTP camera streams in VLC. **NoMachine** is the installed remote desktop (`nxserver` active/enabled).

### NoMachine

- Install path: `/usr/NX/`
- Service: active and enabled
- Config: `/usr/NX/etc/server.cfg` (default NXTCPPort 4000)

---

## 2. Desktop launchers (`~/Desktop/`)

| Icon | Exec | Type | Terminal equivalent |
|------|------|------|---------------------|
| **Tool** | `zsh ~/software/tool/tool.sh` | Qt GUI | Edit `~/ros2_ws/.typerc` + restart services (see §3) |
| **ROSpider** | `zsh ~/software/actionset_editor/actionset_editor.sh` | Qt GUI | Actionset editor; play `.d6a` via ROS (see §4) |
| **SLAM** | `bash ~/ros2_ws/src/bringup/scripts/slam.sh` | Multi gnome-terminal | §5 |
| **Navigation** | `bash ~/ros2_ws/src/bringup/scripts/navigation.sh` | Multi gnome-terminal | §6 |
| Terminal | `gnome-terminal` | Desktop | SSH or local shell |
| NVIDIA links | xdg-open URLs | Browser | Ignore for robot control |

Desktop file sources (also under bringup for some):

- `~/Desktop/tool.desktop`
- `~/Desktop/ROSpider.desktop`
- `~/Desktop/slam.desktop`
- `~/Desktop/navigation.desktop`
- Scripts: `~/ros2_ws/src/bringup/scripts/{slam,navigation}.sh`  
  Service templates: `~/ros2_ws/src/bringup/scripts/start_app_node.service`

---

## 3. Tool GUI → terminal (config + apply)

**Path:** `~/software/tool/main.py`  
**Wrapper:** `~/software/tool/tool.sh` → `source ~/.zshrc; python3 .../main.py`

### What the GUI edits

File: **`~/ros2_ws/.typerc`**

| Variable | Options (as in Tool comboboxes) |
|----------|----------------------------------|
| `DEPTH_CAMERA_TYPE` | `aurora`, `usb_cam` |
| `LIDAR_TYPE` | `LD19` |
| `MACHINE_TYPE` | `ROSpider` |
| `ASR_LANGUAGE` | `Chinese`, `English` |
| `ASR_MODE` | `online`, `offline` |
| `MIC_TYPE` | `xf`, `WonderEchoPro` |
| `VERSION` | display-only from typerc |
| WiFi AP name | derived `WN-<serial>` via serial number |

Also shows: OS, kernel, disk, memory, WLAN IP, Ethernet IP.

### What **Apply** does

```bash
sudo systemctl restart start_app_node.service
sudo systemctl restart find_device.service
```

**Save** only writes `.typerc` (does not restart until Apply).

### Terminal workflow (no GUI)

```bash
nano ~/ros2_ws/.typerc
# set e.g. ASR_LANGUAGE=English

sudo systemctl restart start_app_node.service
sudo systemctl restart find_device.service

# verify env on main process
MAIN_PID=$(systemctl show -p MainPID --value start_app_node.service)
tr '\0' '\n' < /proc/$MAIN_PID/environ | grep -E 'ASR_LANGUAGE|ASR_MODE|LIDAR|DEPTH_CAMERA'
```

**Note:** Tool resolves typerc as `~/software/tool/../../ros2_ws/.typerc` → `~/ros2_ws/.typerc`.  
Grok worktree `.typerc` is separate; runtime uses **`~/ros2_ws/.typerc`**.

---

## 4. Software Qt tools (`~/software/`)

All wrappers: `source $HOME/.zshrc` then `python3 .../main.py`.

| Tool | Paths | Needs DISPLAY | Purpose |
|------|-------|---------------|---------|
| **actionset_editor** | `actionset_editor/main.py`, `actionset_editor.sh` | Yes | Edit/play servo action groups (`.d6a`) |
| **servo_tool** | `servo_tool/main.py`, `servo_tool.sh` | Yes | Bus servo ID/limit/position debug |
| **lab_tool** | `lab_tool/main.py`, `lab_tool.sh` | Yes | LAB color calibration → `lab_config.yaml` |
| **collect_picture** | `collect_picture/main.py`, `.sh` | Yes | Capture images from camera topic |
| **tool** | `tool/main.py`, `tool.sh` | Yes | Machine config (see §3) |
| **roLabelImg** | `roLabelImg/` | Yes | Dataset labeling for YOLO |

### Action groups (actionset editor data)

```text
~/software/actionset_editor/ActionGroups/
```

~63 `*.d6a` files, e.g. `init.d6a`, `kick.d6a`, `climb_stairs*.d6a`, `garbage_pick*.d6a`, `navigation_pick*.d6a`, place-by-waste-type, flutters.

Hardcoded in motion code as ActionGroups path (controller / step_controller).

### Launch GUIs without NoMachine (if local X session exists)

```bash
export DISPLAY=:0
source ~/.zshrc
python3 ~/software/lab_tool/main.py
```

Otherwise use **NoMachine** desktop session.

### LAB config

```text
~/software/lab_tool/lab_config.yaml
```

Colors under `lab.Mono.*` with min/max LAB ranges (black, blue, green, orange, …).

### Vendor command file references

```bash
python3 ~/software/lab_tool/main.py
python3 ~/software/collect_picture/main.py
python3 ~/software/servo_tool/main.py
python3 ~/software/actionset_editor/main.py
```

---

## 5. SLAM desktop script → terminal

**Desktop:** `slam.desktop`  
**Script:** `~/ros2_ws/src/bringup/scripts/slam.sh`

Opens 4 `gnome-terminal` tabs:

1. Stop bringup + start 2D SLAM  
2. Teleop keyboard (after 10s)  
3. RViz slam_desktop config (after 10s)  
4. `ros2 run slam map_save` (after 10s)

### Headless / SSH equivalent

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service

# A — mapping
ros2 launch slam slam.launch.py enable_save:=false

# B — drive
ros2 launch peripherals teleop_key_control.launch.py

# C — RViz (optional; needs display)
rviz2 -d /home/ubuntu/ros2_ws/src/slam/rviz/slam_desktop.rviz

# D — optional helper node (exposes save path behavior)
ros2 run slam map_save
```

### Save map (CLI)

```bash
cd ~/ros2_ws/src/slam/maps
ros2 run nav2_map_server map_saver_cli -f "map_01" \
  --ros-args -p map_subscribe_transient_local:=true
```

Existing maps: `map_01`, `map_03`, `map_09`, `map_011` (`.pgm` + `.yaml`).

### map_save node behavior

`~/ros2_ws/src/slam/slam/map_save.py` — on save service, runs `map_saver_cli -f map_01` into `~/ros2_ws/src/slam/maps`.

### 3D SLAM (depth camera)

```bash
ros2 launch slam rtabmap_slam.launch.py
ros2 launch slam rviz_rtabmap.launch.py   # display
```

---

## 6. Navigation desktop script → terminal

**Desktop:** `navigation.desktop`  
**Script:** `~/ros2_ws/src/bringup/scripts/navigation.sh`

Tabs:

1. Stop bringup, sleep 10, RViz navigation_desktop  
2. `ros2 launch navigation navigation.launch.py map:=map_01`

### Terminal equivalent

```bash
source ~/.zshrc
sudo systemctl stop start_app_node.service

ros2 launch navigation navigation.launch.py map:=map_01

# optional RViz
rviz2 -d /home/ubuntu/ros2_ws/src/navigation/rviz/navigation_desktop.rviz
```

### 3D navigation

```bash
ros2 launch navigation rtabmap_navigation.launch.py
ros2 launch navigation rviz_rtabmap_navigation.launch.py
```

---

## 7. Bringup / systemd (always-on stack)

### Boot service

```text
/etc/systemd/system/start_app_node.service
```

```ini
[Service]
User=ubuntu
ExecStart=/bin/zsh -c 'source home/ubuntu/.zshrc; ros2 launch bringup bringup.launch.py;'
```

Enabled; starts full robot stack at boot.

### Related services

| Service | Role |
|---------|------|
| `start_app_node.service` | Main ROS bringup |
| `wifi.service` | WiFi AP/STA manager |
| `find_device.service` | UDP discovery port 9027 |
| `remote.service` / remote.py | TCP 9026 WiFi config from phone |
| `button_scan.service` | Physical button |
| `set_default_device.service` | USB audio sink selection |
| `expand_rootfs.service` | One-shot disk expand |

### Lifecycle commands

```bash
sudo systemctl status start_app_node.service --no-pager
sudo systemctl stop start_app_node.service
sudo systemctl start start_app_node.service
sudo systemctl restart start_app_node.service

# Kill leftover ROS processes (vendor helper)
~/.stop_ros.sh
# which essentially: kill ros-related PIDs
```

### What bringup starts (`bringup.launch.py`)

1. `startup_check` — buzz, OLED SSID/IP; if mic `/dev/ring_mic`, launches xf mic test  
2. `controller` — board, servos, arm IK, move_controller, EKF, IMU filter, OLED  
3. Depth camera (`DEPTH_CAMERA_TYPE`)  
4. LiDAR (`LIDAR_TYPE`)  
5. rosbridge websocket + web_video_server  
6. `start_app.launch.py` — perform_actions, lidar, line_following, hand_gesture, intelligent_kick, self_balancing  
7. Joystick control  
8. `init_pose` (action `init`)

### Env bootstrap

```text
~/.zshrc
  → ~/ros2_ws/.zshrc
      → ~/ros2_ws/.robotrc
          → ~/ros2_ws/.typerc     # ASR_LANGUAGE, cameras, need_compile, ROS_DOMAIN_ID
          → /opt/ros/humble
          → ~/ros2_ws/install
          → ~/third_party/{third_party_ws,aurora_ws,orbbec_ws,rtabmap_ws}/install
```

`need_compile=False` → many launch files load from `/home/ubuntu/ros2_ws/src/...`  
Node executables still often from `install/` via ament.

---

## 8. Phone / app interfaces as CLI

Apps under bringup use a common pattern:

```text
/<app>/enter          Trigger
/<app>/exit           Trigger
/<app>/set_running    SetBool or SetInt64
/<app>/heartbeat      SetBool  (keep-alive while app is active)
```

### Examples (stack must be running)

```bash
source ~/.zshrc

# Lidar modes: 1=avoid, 2=follow, 3=guard
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 1}"
ros2 service call /lidar_app/heartbeat std_srvs/srv/SetBool "{data: true}"

# Line following
ros2 service call /line_following/enter std_srvs/srv/Trigger {}
ros2 service call /line_following/set_running std_srvs/srv/SetBool "{data: true}"

# Intelligent kick
ros2 service call /intelligent_kick/enter std_srvs/srv/Trigger {}
ros2 service call /intelligent_kick/set_running std_srvs/srv/SetBool "{data: true}"

# Hand gesture
ros2 service call /hand_gesture/enter std_srvs/srv/Trigger {}
ros2 service call /hand_gesture/set_running std_srvs/srv/SetBool "{data: true}"
```

### Discover live APIs

```bash
ros2 node list
ros2 topic list
ros2 service list | grep -E 'enter|set_running|heartbeat'
ros2 interface show interfaces/msg/RunActionSet
ros2 interface show interfaces/srv/SetInt64
```

### Motion without GUI

```bash
# Drive forward slowly (rate)
ros2 topic pub /controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Stop
ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{}"
```

### Key topics (when bringup is up)

| Topic | Role |
|-------|------|
| `/controller/cmd_vel` | Body velocity command |
| `/controller/traveling` | Gait / traveling params |
| `/controller/run_actionset` | Play action group |
| `/servo_controller` | Servo position commands |
| `/scan`, `/scan_raw` | LiDAR |
| `/depth_cam/rgb/image_raw` | RGB from Aurora |
| `/depth_cam/depth/image_raw` | Depth |
| `/imu`, `/imu_corrected` | IMU |
| `/odom` | Fused odometry |
| `/ros_robot_controller/*` | Board I/O (buzzer, OLED, battery, bus servo) |
| `/line_following/image_result` | App debug image |
| `/action_complete` | Action group finished |

### App nodes typically present under bringup

`lidar_app`, `line_following`, `hand_gesture`, `hand_trajectory`, `intelligent_kick`, `self_balancing`, `perform_actions`, plus controller/sensor stack.

Debug launches (from `~/ros2_ws/command`):

```bash
ros2 launch app self_balancing_node.launch.py debug:=true
ros2 launch app lidar_node.launch.py debug:=true
ros2 launch app line_following_node.launch.py debug:=true
ros2 launch app object_tracking_node.launch.py debug:=true
ros2 launch app intelligent_kick_node.launch.py debug:=true
ros2 launch app hand_gesture.launch.py debug:=true
```

---

## 9. Competition (CLI)

```bash
ros2 launch competition competition.launch.py

ros2 service call /narrow_slit_traversal/enter std_srvs/srv/Trigger {}
ros2 service call /cross_bridge/enter std_srvs/srv/Trigger {}
ros2 service call /automatic_pick/pick std_srvs/srv/Trigger {}
ros2 service call /automatic_pick/place std_srvs/srv/Trigger {}
ros2 service call /automatic_pick/is_up_steps std_srvs/srv/Trigger {}
```

---

## 10. WiFi manager (`~/wifi_manager/`)

| File | Role |
|------|------|
| `wifi.py` | Main WiFi AP/STA daemon (systemd `wifi.service`) |
| `wifi_conf.py` | User config (mode, SSIDs, passwords) |
| `/etc/wifi/wifi_conf.py` | System conf (phone remote may write here) |
| `remote.py` | TCP server **9026** — JSON `setwifi` → write conf → restart wifi |
| `find_device.py` | UDP **9027** — replies `ROSPIDER_<W>_<H>:<sn>` on `LOBOT_NET_DISCOVER` |
| `button_scan.py` | Hardware button service |
| `wifi.log` | Log file |

### wifi_conf fields (example shape)

```python
WIFI_MODE = 2                 # 1=AP, 2=STA, 3=AP+eth share
WIFI_AP_PASSWORD = '...'
WIFI_STA_SSID = '...'
WIFI_STA_PASSWORD = '...'
```

AP SSID default pattern: `WN-` + first 8 of device serial (from device-tree).

```bash
# Restart WiFi stack (careful — can drop your SSH session)
sudo systemctl restart wifi.service
sudo systemctl status wifi.service --no-pager
```

---

## 11. Camera / web video without GUI

With bringup running:

```text
http://<IP>:8080/
http://<IP>:8080/stream?topic=/depth_cam/rgb/image_raw
http://<IP>:8080/stream?topic=/depth_cam/depth/image_raw
http://<IP>:8080/stream?topic=/line_following/image_result
```

List image topics:

```bash
ros2 topic list | grep image
```

Manual camera-only launch (if not using full bringup):

```bash
ros2 launch peripherals depth_camera.launch.py
ros2 launch peripherals lidar_view.launch.py
```

---

## 12. Large models / voice (Tool ASR mode)

Vendor notes in `command`: use **Tool** to switch voice mode online/offline before launching LLM examples.

Terminal equivalent: set in `.typerc`:

```bash
export ASR_MODE=online    # or offline
export ASR_LANGUAGE=English  # or Chinese
```

Then restart `start_app_node.service`.

Offline stack uses Ollama + sherpa-onnx under `~/third_party/sherpa-onnx`; config in:

```text
~/ros2_ws/src/large_models/large_models/large_models/config.py
~/large_models/config.py
```

Example launches (after mode set):

```bash
ros2 launch large_models_examples llm_control_move.launch.py
ros2 launch large_models_examples llm_color_track.launch.py
ros2 launch large_models_examples vllm_with_camera.launch.py
```

---

## 13. Rebuild (when GUI/tools are not enough)

Only needed for msgs/C++/package layout changes — **not** for `.typerc` language/camera switches.

```bash
sudo systemctl stop start_app_node.service
# or ~/.stop_ros.sh

cd ~/ros2_ws
colcon build --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

# single package:
colcon build --symlink-install --packages-select app

source ~/.zshrc
sudo systemctl start start_app_node.service
```

Runtime source of truth: **`~/ros2_ws`**. Grok worktree is a separate checkout for editing.

---

## 14. Quick decision guide

| Goal | Use |
|------|-----|
| Change language / camera / ASR mode | Edit `~/ros2_ws/.typerc` + restart services |
| Phone app features (lidar, line, kick) | `ros2 service call` while bringup up |
| Drive robot | `/controller/cmd_vel` or teleop launch |
| SLAM / Nav | Stop bringup; run slam/navigation launches (tmux) |
| Watch camera | Browser `http://IP:8080` |
| Edit actions / servos / LAB colors | NoMachine + Qt tools under `~/software/` |
| Full GUI desktop | NoMachine port 4000 |
| Automation / CI-like | SSH only |

---

## 15. File index (quick paths)

```text
~/Desktop/*.desktop
~/software/tool/
~/software/actionset_editor/   (+ ActionGroups/)
~/software/servo_tool/
~/software/lab_tool/
~/software/collect_picture/
~/software/roLabelImg/
~/ros2_ws/src/bringup/launch/bringup.launch.py
~/ros2_ws/src/bringup/scripts/slam.sh
~/ros2_ws/src/bringup/scripts/navigation.sh
~/ros2_ws/src/bringup/scripts/start_app_node.service
~/ros2_ws/.typerc
~/ros2_ws/command
~/wifi_manager/
/etc/systemd/system/start_app_node.service
/usr/NX/                          # NoMachine
```

---

## 16. Example: full terminal session without desktop

```bash
# Login
ssh ubuntu@<ROBOT_IP>

# Check stack
systemctl is-active start_app_node.service
source ~/.zshrc
ros2 node list | head

# Lidar avoid mode
ros2 service call /lidar_app/enter std_srvs/srv/Trigger {}
ros2 service call /lidar_app/set_running interfaces/srv/SetInt64 "{data: 1}"

# Or stop bringup and map
sudo systemctl stop start_app_node.service
ros2 launch slam slam.launch.py enable_save:=false
# other terminal: teleop + map save
```

---

*Collected from desktop files, bringup scripts, tool/main.py, wifi_manager, live ports/services, and `~/ros2_ws/command`.*
