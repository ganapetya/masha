# Debug mixed C++ / Python ROS 2 on Masha

**Machine:** Masha — Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, `ROS_DOMAIN_ID=27`  
**Workspace:** `~/ros2_ws`  
**Checked on Masha:** 2026-09-18  
**IDE:** Visual Studio Code on the **host**, Remote-SSH into Masha. Do not install a second GUI IDE on the Jetson.

This is how to debug `proud_up` (hunter is C++; launch is Python) and the rest of the overlay (many nodes are Python). Use the layers in order. Do **not** start with gdb on a walking spider.

Related:

- Hunter start recipe: `~/steps/masha-follow-savelij-plan.md`
- Gaits / Traveling vs `cmd_vel`: `~/steps/MASHA_GAITS.md`
- Bringup: `~/ros2_ws/info/startup.md`

---

## 1. What is already on Masha (2026-09-18)

| Tool | Path / version | Status |
|---|---|---|
| gdb | `/usr/bin/gdb` 12.1 | **installed** |
| gdbserver | `/usr/bin/gdbserver` | **installed** |
| Python | 3.10.12 | **installed** |
| colcon | `~/.local/bin/colcon` | **installed** |
| VS Code **server** | `~/.vscode-server/` | **installed** (Remote-SSH already used) |
| VS Code **app** (`code`) | — | not on Masha (correct; it lives on the host) |
| C/C++ / Python / CMake extensions | — | **missing** (only Copilot + Chinese pack) |
| clangd | — | **missing** |
| debugpy | — | **missing** |
| `~/ros2_ws/.vscode/` | — | **missing** |
| proud_up build type | `CMAKE_BUILD_TYPE=Release` | gdb will show little until RelWithDebInfo |

Host that was SSH'd in: `192.168.0.218`. Masha LAN: `192.168.0.156` (`ubuntu-desktop`).

---

## 2. Split the tree before you pick a debugger

The hunter is **not** half Python. Launch files start processes; the bug is almost never inside them.

| Piece | Language | What it is | How to debug |
|---|---|---|---|
| `masha_hunter.launch.py` | Python | Process starter | Read the log. Almost never step it. |
| `masha_hunter.yaml` | YAML | Parameters. Launch extra dict wins. | `ros2 param get` |
| `masha_hunter_node.cpp` | C++ | Wiring: TF, topics, timers, servos, audio | gdb **after** walk is off |
| `hunter.cpp` / `hunter.hpp` | C++ | Policy. **No ROS.** | `colcon test` first |
| `target_*.cpp` | C++ | Saveli TF plug / cat YOLO plug | gtest + overlay |
| `controller`, `bringup`, most `app/` | Python | Other nodes | pdb / VS Code Python |
| `follow_the_cat.hpp` | C++ | Pinhole + gaze, shared with hunter | gtest `test_follow_the_cat` |

Two-layer hunter design (this is why tests work without the robot):

```
hunter.hpp / hunter.cpp     BRAIN   tick() in, Halt/Twist out
masha_hunter_node.cpp       BODY    sensors, TF, pubs
```

---

## 3. IDE: yes, Visual Studio Code

Use **VS Code on the host** + **Remote-SSH** into Masha, folder `/home/ubuntu/ros2_ws`.

That is already how this Jetson is opened (`~/.vscode-server` exists). Keep it.

Do **not** install:

- Visual Studio (Windows, x64, wrong machine)
- CLion on the Jetson (heavy, ARM license hassle)
- A second VS Code GUI on Masha (the remote server is enough)

You edit and set breakpoints in the host window. The compiler, gdb, and `ros2` run **on Masha**.

---

## 4. Install — host (laptop)

Do this once on the machine you sit at.

### 4.1 VS Code

1. Install VS Code: https://code.visualstudio.com/
2. Open the Extensions view (`Ctrl+Shift+X`).
3. Install **Remote - SSH** (`ms-vscode-remote.remote-ssh`).

### 4.2 SSH into Masha

From a host terminal, confirm:

```bash
ssh ubuntu@192.168.0.156
# or whatever hostname you already use
```

If that works, in VS Code:

1. `F1` → **Remote-SSH: Connect to Host…**
2. Pick `ubuntu@192.168.0.156` (add it if missing: `~/.ssh/config` on the **host**).
3. **File → Open Folder** → `/home/ubuntu/ros2_ws`

Example host `~/.ssh/config` (edit the HostName if your LAN IP changed):

```
Host masha
    HostName 192.168.0.156
    User ubuntu
```

Then `F1` → Connect to Host → `masha`.

### 4.3 What stays on the host

- The VS Code window
- Remote-SSH extension
- Your SSH key

Do **not** install gdb, ROS, or colcon on the host for this workflow. Those run on Masha.

---

## 5. Install — Masha (Jetson)

Open a terminal **inside the Remote-SSH window** (bottom panel). That shell is already on Masha as `ubuntu`. A normal Masha zsh already sources Humble + overlay + `.typerc` via `.robotrc`. Re-source anyway in a bare terminal:

```bash
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh
source ~/ros2_ws/.typerc
# ROS_DOMAIN_ID must print 27
echo $ROS_DOMAIN_ID
```

### 5.1 System packages (gdb is already there)

```bash
sudo apt-get update
sudo apt-get install -y gdb gdbserver clangd-14 clang-format-14
sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-14 100
clangd --version
gdb --version
```

`gdb` / `gdbserver` were already installed on 2026-09-18. Re-running apt is safe.

### 5.2 Python debugger (for Python **nodes**, not launch files)

```bash
python3 -m pip install --user debugpy
python3 -c "import debugpy; print(debugpy.__version__)"
```

`pdb` is in the stdlib and works with no extra package:

```bash
python3 -m pdb /path/to/some_node.py
```

### 5.3 VS Code extensions on the **remote** (Masha)

Extensions must be installed in the Remote window, not only on the host. Bottom-left of VS Code should say `SSH: masha` (or the host name). Then:

**GUI (preferred)**

1. Extensions (`Ctrl+Shift+X`)
2. Search each id below
3. Click **Install in SSH: masha** (not “Install” on the local host)

Install these:

| Extension | Id | Why |
|---|---|---|
| C/C++ | `ms-vscode.cpptools` | IntelliSense + **gdb** breakpoints in `.cpp` |
| Python | `ms-python.python` | `.py` nodes and launch |
| Python Debugger | `ms-python.debugpy` | VS Code Python launch/attach |
| CMake Tools | `ms-vscode.cmake-tools` | `proud_up` is ament_cmake |
| CMake language | `twxs.cmake` | syntax of `CMakeLists.txt` |

Optional:

| Extension | Id | Why |
|---|---|---|
| ROS | `ms-iot.vscode-ros` | node/topic peek |
| clangd | `llvm-vs-code-extensions.vscode-clangd` | Go to Definition via `compile_commands.json`. If you install this, disable cpptools IntelliSense so they do not fight (`C_Cpp.intelliSenseEngine: Disabled`). |

**CLI from the same Remote-SSH terminal** (the `code` command exists only inside a VS Code terminal, not a raw SSH):

```bash
code --install-extension ms-vscode.cpptools
code --install-extension ms-python.python
code --install-extension ms-python.debugpy
code --install-extension ms-vscode.cmake-tools
code --install-extension twxs.cmake
# optional:
# code --install-extension ms-iot.vscode-ros
# code --install-extension llvm-vs-code-extensions.vscode-clangd
```

Reload the window (`F1` → **Developer: Reload Window**) after installs.

If `ms-vscode.cpptools` fails to download its ARM64 binary on Jetson, use **clangd** instead of Microsoft IntelliSense, and keep gdb as the debugger (`miDebuggerPath`: `/usr/bin/gdb`).

### 5.4 Rebuild hunter with symbols + compile_commands

Current proud_up cache is **Release**. gdb line numbers will be thin. Rebuild:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select proud_up --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.zsh
ln -sf ~/ros2_ws/build/proud_up/compile_commands.json ~/ros2_ws/compile_commands.json
```

`RelWithDebInfo` = optimized **and** `-g`. Fast enough to walk, gdb can still map lines.

VS Code C++ / clangd look for `compile_commands.json` at the workspace root. The symlink above is that file.

Binary gdb will attach to:

```
~/ros2_ws/install/proud_up/lib/proud_up/masha_hunter_node
```

---

## 6. Layered debug (use in this order)

How to plant a red dot:

- **VS Code:** open the file on Masha (Remote-SSH), click the gutter left of the line number. F5 uses `launch.json`. Hover the red dot → **Edit Breakpoint** for a condition (use this on 20 Hz functions).
- **gdb:** `break Namespace::Class::method` then `continue`. `info break` lists them. `delete 2` removes breakpoint 2.
- Line numbers below are **Masha 2026-09-18** after the comment pass. If they drifted, search the **function name**.

**Do not** put an unconditional breakpoint on a 20 Hz function (`control_tick_inner`, `Hunter::tick`, `apply_legs`) while `enable_walk:=true`. The timer stops; last Twist may keep walking. Walk **off**, or use a **condition** (`hit.has_value()`, `out.play_name`).

Symptom → first stop (read this table, then the layer):

| What you see | Layer | Break / watch here |
|---|---|---|
| gtest FAIL on Halt / crab / PD | 1 | the failing `TEST` + `Hunter::tick` |
| No `/masha_hunter_node` | 2 | no breakpoint — launch log |
| Hunt forever, empty detections | 2 then 4 | `saveli_from_tf` |
| Tag in rqt, still Hunt | 4 | `saveli_from_tf` (stale TF) then `tick` Hunt+hit |
| NAME silent | 3 then 4 | log `NAME play` / `MISSING`; `play_once` |
| Named, never Follow | 4 | `notify_name_done` / `tick` Name branch |
| Twist in log, no `cmd_vel` | 4 | `apply_legs` (`enable_walk` / `dry_run`) |
| `cmd_vel` but legs idle | 5 | `StepController.cmd_vel` (`cmd_gait` must be 5) |
| No crab (`linear.y=0`) | 1 then 4 | `follow_twist` |
| Sudden stand | 4 | `halt_legs` / `watchdog_tick` |
| SIGSEGV at start | 4 | constructor; do **not** subscribe `apriltag_msgs` |

### Layer 1 — policy, no robot (C++ tests)

**Breakpoints: yes. Prefer this layer.** No ROS, no legs. You debug `Hunter::tick` with fake poses.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
colcon test --packages-select proud_up --event-handlers console_direct+
```

`src/proud_up/test/test_hunter.cpp` calls `Hunter::tick()` with fake `TargetHit` poses. Typical sequence (same as the live node):

```
h.start()                          Idle → Hunt
h.tick(t, saveli_at(x,y), d_min)   Hunt → Name
h.notify_name_done()
h.tick(...)                        Follow: SelectGait15 then Twist
```

If Halt/crab/PD is wrong, it fails **here**.

**Where to set the breakpoint**

| Symptom / question | File | Function | Line (now) | Inspect when it hits |
|---|---|---|---|---|
| Which test failed? | `src/proud_up/test/test_hunter.cpp` | the `TEST(Hunter, …)` that failed | 64+ | step into `h.tick` |
| Phase / Halt vs Twist | `src/proud_up/src/hunter.cpp` | `Hunter::tick` | 296 | `phase_`, `hit`, `o.legs`, `o.twist` |
| Hunt → Name vs skip NAME | same | `Hunter::tick` Hunt `if (hit)` | 339 | `skip_name`, `named_this_lock_` |
| vx / wz / crab | same | `Hunter::follow_twist` | 192 | `e.x`, `e.y`, `e.theta`, `crab_`, `t.vy` |
| Sticky vs yaml order | same | `pick_target` | 114 | `sticky_id`, `h->id` |
| LiDAR stop chatter | same | `Hunter::tick` Follow `d_stop`/`d_go` | 399 | `d_min`, `lidar_latched_`, `phase_` |
| Saveli plug rejects tag | `src/proud_up/src/target_saveli.cpp` | `SaveliSource::detect` | 14 | `in.have_saveli_pose`, `in.tag_id` |

Run **one** test under gdb (VS Code: open `test_hunter.cpp`, breakpoint on the `TEST` line, debug the `test_hunter` binary — or gdb):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.zsh
# after colcon build --packages-select proud_up
gdb --args ./build/proud_up/test_hunter \
  --gtest_filter=Hunter.RangeErrorCommandsForwardTwist
(gdb) break proud_up::Hunter::tick
(gdb) break proud_up::Hunter::follow_twist
(gdb) run
```

Useful filters: `Hunter.DeadbandHaltsNotZeroTwist`, `Hunter.CrabGateUsesYNotThetaAlone`, `Hunter.FollowMissCoastsTwistUntilLostTimeout`.

Do **not** launch the hunter node for Layer 1.

### Layer 2 — ROS graph, no debugger

**Breakpoints: none.** The “breakpoint” is a `ros2 topic echo` / `tf2_echo`. If you stop in gdb here you have skipped the cheap check.

Slim bringup stays up. Do **not** run `~/.stop_ros.sh`. Do **not** also launch `follow_the_cat`, `masha_interaction` FOLLOW, joystick, Nav2, or a second `/controller/cmd_vel` writer.

Terminal 1:

```bash
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh
source ~/ros2_ws/.typerc
ros2 launch proud_up masha_hunter.launch.py \
  enabled_targets:=saveli \
  enable_walk:=false \
  enable_crab:=true \
  vx_max:=0.12
```

First lock is safer with **walk off**. Twist is logged; legs stay in hunter pose.

Terminal 2:

```bash
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh
source ~/ros2_ws/.typerc
ros2 service call /masha_hunter_node/start std_srvs/srv/Trigger
```

Watch:

```bash
ros2 node list
ros2 topic echo /apriltag/apriltag_detections
ros2 topic echo /controller/cmd_vel
ros2 run tf2_ros tf2_echo base_link saveli_tag
ros2 param get /masha_hunter_node enable_walk
ros2 topic echo /masha_hunter_node/image_result   # overlay; rqt_image_view is easier
```

How to read it:

| Observation | Meaning |
|---|---|
| Detections empty every frame | Tag too far / no white margin / not 36h11 id 0 |
| `tf2_echo` fails | AprilTag node not up, or frame is `tag36h11:0` not `saveli_tag` |
| `linear.y` nonzero | Crab is gated |
| `linear.y=0` and only `angular.z` | Bearing too large for crab, or crab off |
| `enable_walk` false | Twist logged only; no `cmd_vel` |
| Overlay `hunt` forever | No fresh TF (stale pose is not a hit) |

If Layer 2 already answers the question, **stop**. Do not open gdb yet.

**If you still want a one-shot C++ stop after the graph looks wrong:** `on_start` (fires once per `~/start`). Not `control_tick`.

### Layer 3 — logs in the node

**Breakpoints: none.** The log line *is* the breakpoint. Read the launch terminal; then if you must stop in the editor, break on that same `RCLCPP_*` line (it fires at the same moment as the print).

`enable_walk:=false` is the debug build for wiring: Twist is logged, `cmd_vel` is not published.

| Log text | File | Function | Line (now) | Meaning |
|---|---|---|---|---|
| `masha_hunter enable_walk=…` | `masha_hunter_node.cpp` | constructor | 507 | Node is up. If `enable_walk=false`, no `cmd_vel`. |
| `NAME saveli=… (MISSING)` | same | constructor | 515 | Clip path wrong; Name will time out with no sound. |
| `start: HUNT (head sweep)` | same | `on_start` | 1062 | `~/start` reached the node. Break here if this never prints. |
| `no Saveli TF yet` | same | `saveli_from_tf` | 697 | Tag not in view, or frame not `saveli_tag` / `tag36h11:0`. |
| `Saveli TF stale` | same | `saveli_from_tf` | 716 | Last TF older than `pose_max_age` (0.40 s). Not a hit. |
| `NAME play …` | same | `control_tick_inner` | 843 | Policy asked for the clip. Next stop: `WavPlayer::play_once`. |
| `twist vx=… vy=… crab=` | same | `apply_legs` | 878 | Policy Twist. If this prints and `cmd_vel` is empty, `dry_run` or walk still off. |
| `FOLLOW→HUNT lost` | same | `control_tick_inner` | 825 | Lost lock; pan continues from current servo 19. |
| `walk watchdog` | same | `watchdog_tick` | 1040 | No `cmd_vel` for 0.3 s; Traveling stand. |
| `detect_tick:` / `control_tick:` exception | same | wrappers | 573 / 742 | OpenCV / TF threw; node stays up. |

### Layer 4 — gdb on the C++ node

**Breakpoints: yes, on the node, walk off.** Only after RelWithDebInfo (§5.4). A breakpoint in `control_tick()` freezes the 20 Hz timer: last Twist may keep walking, or the walk watchdog stands the legs. Do not gdb Follow with `enable_walk:=true` until Halt is proven.

**First three stops (do these before hunting a 20 Hz function)**

| Order | File | Function | Line (now) | Why this one |
|---|---|---|---|---|
| 1 | `src/proud_up/src/masha_hunter_node.cpp` | `on_start` | 1048 | Fires **once** per `~/start`. Confirms the service reached C++. Inspect `enable_walk`. |
| 2 | same | `saveli_from_tf` | 688 | Every control tick, but return early if no TF. Inspect `tag_frame`, `age`, `in.have_saveli_pose`. Condition: leave unconditional only with walk off. |
| 3 | `src/proud_up/src/hunter.cpp` | `Hunter::tick` | 296 | The policy. Inspect `phase_`, `hit`, `o.legs`, `o.play_name`. **Condition** `hit.has_value()` so empty Hunt ticks do not stop you 20 times a second. |

**Then pick by symptom**

| Symptom | File | Function | Line (now) | Inspect |
|---|---|---|---|---|
| Wrong / missing TF name | `masha_hunter_node.cpp` | `saveli_tf_frame` | 667 | return `saveli_tag` vs `tag36h11:0` vs empty |
| Cat pixel, no base pose | same | `fill_cat_base_pose` | 635 | `hit.range_m`, `base.point.x/y` |
| YOLO never runs | same | `detect_tick_inner` | 582 | `run_cat`, `image` — no-op if cat not in `enabled_targets` |
| Head sweep wrong | same | `apply_head` | 896 | `out.pan_head`, `arm.id19`, `elapsed` |
| Gaze in Follow (pixel targets) | same | `apply_head` Follow branch | 920 | `gaze_id19_`, `err` |
| No `cmd_vel` | same | `apply_legs` | 856 | `out.legs`, `enable_walk`, `dry_run_` |
| Gait 15 never sent | same | `select_gait15` | 983 | should run **once** per walk bout, before first Twist |
| Stand (gait 0 then -2) | same | `halt_legs` | 1000 | who called it: Idle, lost, watchdog, `~/stop` |
| Watchdog stand | same | `watchdog_tick` | 1021 | `walking_active_`, time since `last_walk_cmd_at_` |
| NAME clip | same | `WavPlayer::play_once` | 229 | `wav`, `file_exists` |
| Hunt → Name | `hunter.cpp` | `Hunter::tick` | 339 | `skip_name`, `o.play_name` |
| Name → Follow | same | `Hunter::tick` Name | 368 | `name_playing_`, `notify_name_done` |
| Follow PD / crab | same | `Hunter::follow_twist` | 192 | `e`, `t.vx/vy/wz`, `crab_` |
| Follow → Hunt at 60 s | same | `Hunter::tick` `left <= 0` | 386 | `follow_t0_` |
| LiDAR Stopped | same | `Hunter::tick` `d_min < d_stop` | 399 | `d_min` |
| Coast on brief TF miss | same | `Hunter::tick` `if (!hit)` in Follow | 436 | `o.twist == last_twist_`, not Halt yet |

gdb names (anonymous-namespace helpers need the file, not a pretty `MashaHunterNode::` if the compiler inlined; methods below are the class):

```
(gdb) break MashaHunterNode::on_start
(gdb) break MashaHunterNode::saveli_from_tf
(gdb) break proud_up::Hunter::tick
(gdb) break proud_up::Hunter::follow_twist
(gdb) break MashaHunterNode::apply_legs
(gdb) condition 3 hit.has_value()
(gdb) continue
```

When `tick` hits: `p phase_`, `p hit.has_value()`, `p o.legs`, `p o.twist.vx`. Step **into** `follow_twist` only after `phase_` is Follow.

**A. Run the node under gdb** (launch file not involved; you must pass params yourself or use yaml):

```bash
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh
source ~/ros2_ws/.typerc
ros2 run --prefix 'gdb -ex run --args' proud_up masha_hunter_node --ros-args \
  --params-file ~/ros2_ws/src/proud_up/config/masha_hunter.yaml \
  -p enable_walk:=false
```

**B. Attach to a running hunter** (launch as usual, walk off):

```bash
pgrep -a masha_hunter
sudo gdb -p $(pgrep -n masha_hunter_node)
```

Useful gdb:

```
(gdb) bt
(gdb) info threads
(gdb) break proud_up::Hunter::tick
(gdb) break MashaHunterNode::control_tick_inner
(gdb) continue
```

`sudo` may be required to attach (`ptrace_scope`). If attach is denied:

```bash
cat /proc/sys/kernel/yama/ptrace_scope
# 1 = only attach children. Temporary:
sudo sysctl kernel.yama.ptrace_scope=0
```

Put it back to `1` when finished.

**C. VS Code F5 (after extensions in §5.3)**

Create `~/ros2_ws/.vscode/launch.json` on Masha (paste below). Open `masha_hunter_node.cpp`, red-dot **`on_start` (line 1048)** first — not `control_tick`. **Run and Debug → gdb hunter (walk off)**. Call `~/start` from another terminal; it should stop once. Then add `saveli_from_tf` / `Hunter::tick` with a condition.

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "gdb hunter (walk off)",
      "type": "cppdbg",
      "request": "launch",
      "program": "${workspaceFolder}/install/proud_up/lib/proud_up/masha_hunter_node",
      "args": [
        "--ros-args",
        "--params-file",
        "${workspaceFolder}/src/proud_up/config/masha_hunter.yaml",
        "-p", "enable_walk:=false"
      ],
      "cwd": "${workspaceFolder}",
      "environment": [
        { "name": "ROS_DOMAIN_ID", "value": "27" }
      ],
      "externalConsole": false,
      "MIMode": "gdb",
      "miDebuggerPath": "/usr/bin/gdb",
      "setupCommands": [
        { "description": "pretty print", "text": "-enable-pretty-printing", "ignoreFailures": true }
      ]
    },
    {
      "name": "attach hunter",
      "type": "cppdbg",
      "request": "attach",
      "program": "${workspaceFolder}/install/proud_up/lib/proud_up/masha_hunter_node",
      "processId": "${command:pickProcess}",
      "MIMode": "gdb",
      "miDebuggerPath": "/usr/bin/gdb"
    }
  ]
}
```

The launch configuration does **not** source `setup.zsh` for you. Start VS Code with Remote-SSH **after** a login that already has Humble on `PATH`, or prepend in a wrapper. If F5 says `ros2` / plugin not found, run from a terminal that already sourced Humble:

```bash
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh
source ~/ros2_ws/.typerc
# then F5 from that same Remote window
```

A `c_cpp_properties.json` that uses the compile_commands symlink:

```json
{
  "configurations": [
    {
      "name": "Masha ROS 2 Humble",
      "compileCommands": "${workspaceFolder}/compile_commands.json",
      "cppStandard": "c++17",
      "intelliSenseMode": "linux-gcc-arm64"
    }
  ],
  "version": 4
}
```

### Layer 5 — Python nodes (controller, bringup, app)

**Breakpoints: yes, on the Python node, not the launch file.** Use this when Layer 4 already publishes `cmd_vel` / Traveling but the legs do not match.

Launch files (`*.launch.py`) are not the bug. Debug the **node** they start.

Find the installed script:

```bash
ros2 pkg executables controller
```

**Where to set the breakpoint (hunter → legs)**

| Symptom | File | Function | Line (now) | Inspect |
|---|---|---|---|---|
| Twist arrives? | `src/driver/controller/controller/move_controller.py` | `cmd_vel_callback` | 194 | `msg.linear.x/y`, `msg.angular.z` after the clamp (vx cap 0.12, vy cap **0.10**) |
| Generator rebuilt? | `src/driver/controller/controller/step_controller.py` | `cmd_vel` | 422 | `self.cmd_gait` — must be **5** after hunter’s gait=15. `linear_x` is mm/s (`*1000`) |
| gait=15 ignored as walk | same | `set_step_mode` | 482 | `gait == 15` branch sets `cmd_gait = 5` and **returns without walking**. Next `cmd_vel` uses `FollowGaitGenerator` |
| 50 Hz loop using stale generator | same | `loop` | 124 | `self.cmd_gait`, current generator type |
| Launch args not applied | `src/proud_up/launch/masha_hunter.launch.py` | `_launch_setup` | 36 | **Do not** step this for Follow bugs. Print `enable_walk` if the node came up with the wrong flag. |

**pdb:**

```bash
source /opt/ros/humble/setup.zsh
source ~/ros2_ws/install/setup.zsh
source ~/ros2_ws/.typerc
python3 -m pdb $(ros2 pkg prefix controller)/lib/controller/move_controller
```

In pdb: `b step_controller.py:422` then `c`. Same idea in VS Code: open the **source** file under `src/driver/controller/…` (not a copy in `install/`), red-dot those lines, attach to the running `move_controller` process.

**VS Code Python** (after `debugpy`): a launch config needs `ROS_DOMAIN_ID=27` and the same `source` environment. Without domain 27 you attach to a silent graph.

Do not step `masha_hunter.launch.py` to find out why Saveli is not followed. That bug is TF, yaml, or `hunter.cpp`.

---

## 7. What not to do

- gdb / breakpoint in `control_tick` with `enable_walk:=true` while Follow is live.
- Two writers on `/controller/cmd_vel` or servos 19/22 (`follow_the_cat`, joystick, Nav2, `lidar_app`).
- `~/.stop_ros.sh` “to make debug quieter” — that kills slim bringup (camera, lidar, STM32).
- Debugging the **launch file** for a policy/TF bug.
- Rebuilding `Debug` (`-O0`) for walk tests — too slow on the 20 Hz loop. Use `RelWithDebInfo`.
- Installing extensions on the **host** only, then wondering why C++ IntelliSense is dead on Masha. Install **in SSH: masha**.

---

## 8. Checklist (first time)

On the **host:**

1. VS Code + Remote-SSH
2. Connect → open `/home/ubuntu/ros2_ws`

On **Masha** (Remote-SSH terminal):

3. `sudo apt-get install -y gdb gdbserver clangd-14`
4. `python3 -m pip install --user debugpy`
5. Install extensions in §5.3 (Install in SSH)
6. `colcon build --packages-select proud_up --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
7. Symlink `compile_commands.json` to the workspace root
8. Optional: add `.vscode/launch.json` from §6 layer 4C

Then debug in this order: **gtest → topic/TF/overlay → logs → gdb (walk off) → Python pdb**.

First C++ red dot on a live node: `on_start`. First Python red dot if `cmd_vel` is live but legs are not: `StepController.cmd_vel`.

---

## 9. Hunter-specific reminders

- Zero `Twist` on `/controller/cmd_vel` **starts a gait in place**. Halt is Traveling `gait=0` then `gait=-2`.
- Saveli pose is TF (`saveli_tag` or fallback `tag36h11:0`), not `apriltag_msgs` (layout mismatch SIGSEGV).
- `pose_max_age` (0.40 s): old TF is not a live hit; Follow coasts then Hunt.
- Launch args do not hot-reload. Ctrl-C, relaunch.
- Comments that walk the 20 Hz tick: `~/ros2_ws/src/proud_up/src/masha_hunter_node.cpp` file header, then `Hunter::tick()` in `hunter.cpp`.
