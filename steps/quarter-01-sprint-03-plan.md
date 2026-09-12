# Voice, dance, and ZMP — Masha-grounded Sprint 3 plan

**Status:** plan only. Do **not** implement the node until Peter asks.  
**Syllabus (Gemini):** [`quarter-01-sprint-03-guru-source.md`](quarter-01-sprint-03-guru-source.md)  
**This memo:** how that syllabus maps onto Masha as she actually runs (Jetson Orin NX, ROS 2 Humble, `proud_up`, sherpa-onnx, bus servos).  
**Start date:** 2026-09-12

## Goal

Build `masha_interaction_node` as a ROS 2 Humble **C++ node inside the existing `proud_up` package**. After wake, she hears “who is your master?”, answers **“I guess you think you are my master”** on the USB speaker, halts gait, then plays a rhythmic `master_dance.d6a`. While she dances, a ZMP estimator (cart-table / linear inverted pendulum) publishes the Zero Moment Point and the support polygon; the ZMP must stay inside the polygon **and** must actually travel (this is not a timid stand-in-place).

Keep Gemini’s teaching: hexapod support polygons, homogeneous TF, a dedicated interaction state machine, ActionSet playback, ZMP. Replace every invented topic, service, package, spoken line, and Bluetooth path with what this Jetson runs.

## Learner comments (standing rule)

Same rule as Sprint 2 (`quarter-01-sprint-02-masha-plan.md`). Headers, `.cpp`, tests, launch, yaml must be commented so a learner who knows Java/Python better than C++/ROS can follow frames, Eigen, mutexes, and Masha’s topics from the source:

- **Why, not restating the line.** Why `gait=-2` is the halt, why `aplay` is not on the callback thread, why IMU accel is not already in `base_link`.
- **Formulas next to code.** Cart-table \(p_x = x - (z/g)\ddot x\), convex hull, point-in-polygon. Name frames (`base_link`, `imu_link`, `end_LF`) and units (m, m/s², rad).
- **Masha specifics.** Real topic names, `ROS_DOMAIN_ID=27`, `tibla_*` spelling, `/controller/run_actionset` is a **topic** not a service.
- **Concurrency.** Speech callback vs 50 Hz ZMP timer; what the mutex guards; playback on a worker thread.
- **Safety.** Halt before dance; abort to `stop` / `init_pose` if ZMP leaves the polygon. Do not “fix” instability by slowing the choreography in software.
- **Tests.** Each gtest names the invariant (static ZMP = CoM projection; accel shifts ZMP opposite \(\ddot x\); point inside/outside a triangle).

Do not spam `// increment i`. Match existing `proud_up` tone.

## Corrections vs the syllabus

The Gemini program is pedagogically right and hardware-wrong in several places. Peter’s overrides are in this table too.

| Gemini syllabus | What Masha actually has | What we do |
|---|---|---|
| C++ sherpa inside `peripherals` | `peripherals` is ament_python (joystick, teleop, IMU TF). Sherpa is Python `xf_mic_asr_offline/scripts/asr_node.py`. Canonical: `ros2_ws/info/voice.md` — **do not add a second recognizer**. | Keep sherpa in `asr_node`. New C++ is **`proud_up`**, like Sprint 2. |
| New ROS package | Sprint 2 already builds C++ in `proud_up`. | **No new package.** |
| Topic `/voice_asr_result` | `/asr_node/voice_words` (`std_msgs/String`) | Subscribe there. |
| Idle phrase `"Masha, who is your master?"` | Wake is **Hello / Hi / Shalom Masha**. Commands only for 120 s after wake. `extract_command()` currently publishes only walk/dance/stop; unknown speech is dropped. Idle full sentence will **not** wake her (no greeting token). | Wake first, then the query. Add master-query aliases in `extract_command`. |
| Reply “Peter Kovgan is my master” on Bluetooth via `paplay` | Peter’s line: **“I guess you think you are my master”**. Playback is `aplay -D pulse` on the Pulse **default sink** (GeneralPlus **USB** speaker) via `voice_play.py`. No Bluetooth path in this stack. `aplay` on the listen thread used to freeze ASR after reboot. | New wav. `aplay` on a **worker thread** with timeout. |
| `/cmd_vel` clears “gait buffer” | Walk topic is `/controller/cmd_vel`. The halt that actually stops legs is `kinematics_msgs/Traveling` with **`gait=-2`** on `/controller/traveling` (`move_controller.set_traveling_callback` → `DEFAULT_POSE`). Zero Twist alone is not enough. | Publish halt Traveling **and** zero Twist. |
| Service `yahboomcar_msgs::srv::RunActionSet` | **No such package.** Playback is topic `/controller/run_actionset`, type `interfaces/msg/RunActionSet` (`string action_path`, `bool interrupt`). Basename of the `.d6a`, no extension. | C++ publisher of that message. |
| TF `world → odom → base_link → coxa → femur → tibia` | `odom → base_footprint → base_link → leg_center_{LF,LM,LR,RF,RM,RR} → coxa_* → femur_* → tibla_*` (vendor spelling) `→ end_*`. | Use real link names. |
| `/voltage` | `/ros_robot_controller/battery` (`std_msgs/UInt16`) | Echo that. |
| `.typerc`: `ASR_OFFLINE=true`, `MIC_TYPE=usb_array` | `ASR_MODE=online` is the large-model path. Voice is still sherpa. `MIC_TYPE=usb` and **unused**. `BRINGUP_VOICE=true` already. | Do not change those vars. |
| `colcon build … peripherals servo_controller` | Voice Python is symlink-installed. Action groups are files on disk. | `colcon build --packages-select proud_up` only. |
| Conservative dance / “reduce playback speed” as the ZMP fix | Peter wants **real ZMP** and a **rhythmic** dance. Slowing the `.d6a` in software is not the design. | Live ZMP + abort if it leaves the polygon. Choreography stays energetic. |
| STM32 `/dev/ttyACM0` vs X3 `ttyUSB1` | Correct: `/dev/ttyACM0`. | Keep that troubleshooting note. |

Vendor gait numbers: **1 = ripple**, **2 = tripod**, **−2 = halt / DEFAULT_POSE**.

Existing voice “dance” already plays `twist.d6a` from `voice_control_move.py`. This sprint is a **different phrase** and a **dedicated C++ node**, not a second ASR.

## Architecture

```
USB 6-mic  (iFlytek XFM-DP, PulseAudio default source)
    → asr_node.py  (sherpa-onnx, only listener)
         → /asr_node/voice_words
              ├─ voice_control_move.py     (existing walk / twist-dance / stop)
              └─ masha_interaction_node    (NEW, package proud_up)
                   1. match canonical "who is your master"
                   2. Traveling gait=-2  +  zero /controller/cmd_vel
                   3. aplay master_response.wav  (worker thread)
                   4. interfaces/msg/RunActionSet → /controller/run_actionset
                   5. 50 Hz ZMP: IMU + TF feet → ~/zmp vs ~/support_polygon
```

Do not launch a second `asr_node`. Slim bringup + `BRINGUP_VOICE=true` already starts the Python voice stack.

### Spoken protocol

1. “Hello Masha” (or Hi / Shalom) → she says “I’m here”.
2. Wait for the clip (~2.2 s ignore window — do not talk over it).
3. “Who is your master?”
4. She answers **“I guess you think you are my master”**, then dances.

Sherpa aliases (non-exhaustive): `who is your master`, `who's your master`, `who your master`, `who is ur master`, `whose your master`. Canonical published string: `who is your master`.

`voice_control_move` logs unmatched phrases and does not move. Optionally add the canonical string to its ignore list so the log is clean.

### Safety interlock (before dance)

```
Traveling { gait: -2, time: 1.0, steps: 0 }  →  /controller/traveling
Twist {}                                    →  /controller/cmd_vel
then RunActionSet { action_path: "master_dance", interrupt: true }
                                            →  /controller/run_actionset
```

`move_controller` maps `action_path == "stop"` to stop the group + `init_pose`. Use that for the ZMP abort.

### Audio

Wav: `~/ros2_ws/src/xf_mic_asr_offline/feedback_voice/english/master_response.wav`  
Exact spoken text: **I guess you think you are my master**

Play with `aplay -D pulse` (fallback `default`, then `plughw:CARD=Device` like `voice_play.py`), **worker thread**, ~5 s timeout. Never `system("paplay …")` on the ROS callback. Never block `rclcpp` spin.

## ZMP (required)

There is no vendor ZMP topic. This sprint **adds** the estimator in `proud_up`.

### Math

Cart-table / linear inverted pendulum. CoM \((x, y, z)\) and horizontal CoM acceleration \((\ddot x, \ddot y)\), gravity \(g = 9.80665\,\mathrm{m/s}^2\):

\[
p_x = x - \frac{z}{g}\,\ddot x, \qquad
p_y = y - \frac{z}{g}\,\ddot y
\]

\(p = (p_x, p_y)\) is the ZMP in the stance plane. If \(\ddot x = \ddot y = 0\), this is the CoM projection (Day 1 static stability).

Support polygon = convex hull of **grounded** foot tips `end_LF … end_RR` in **one** frame (`base_link`). A foot is grounded if its \(z\) is within a contact band of the lowest foot (parameter, start ~0.01 m).

DoD: \(p\) stays inside that polygon for the whole dance. The dance must still **move** \(p\) (weight shift, body roll/twist). A near-static stand does not satisfy ZMP work.

### Sensors on this robot

| Quantity | Source |
|---|---|
| Foot tips | TF `base_link` → `end_{LF,LM,LR,RF,RM,RR}` (`robot_state_publisher` + `/joint_states`) |
| Body orientation / accel | `/imu` (`sensor_msgs/Imu`, after `imu_filter`) |
| Link masses | URDF inertials in `rospider_description/urdf/base.urdf.xacro` (`base_link` ≈ 0.427 kg; each `leg_segment` has coxa/femur/tibla/end masses) |
| CoM | mass-weighted mean of link origins from TF. Document the approximation in comments. |

IMU linear acceleration is at `imu_link`. Transform into `base_link` before using \(\ddot x, \ddot y\). Subtract gravity using orientation — or use the filter’s linear accel if it is already gravity-compensated. **Measure on Day 7; do not assume.**

### Library vs node (Sprint 2 split)

- `include/proud_up/zmp.hpp` + `src/zmp.cpp` — **no rclcpp**: CoM from (mass, position) pairs, cart-table ZMP, 2D convex hull, point-in-polygon, min distance to edge.
- `src/masha_interaction_node.cpp` — subscriptions, 50 Hz timer, publishers, state machine, `aplay` worker.
- `test/test_zmp.cpp` — offline gtest.

### Debug topics (node-private `~/`)

| Topic | Type | Meaning |
|---|---|---|
| `~/zmp` | `geometry_msgs/PointStamped` | \(p\) in `base_link` |
| `~/support_polygon` | `geometry_msgs/PolygonStamped` | hull of grounded feet |
| `~/zmp_inside` | `std_msgs/Bool` | point-in-polygon |
| `~/zmp_margin` | `std_msgs/Float64` | signed distance to edge (m) |

Foxglove/RViz: polygon + point. No new `.msg` package.

### Dance is not timid

- Peter authors `~/software/actionset_editor/ActionGroups/master_dance.d6a` with **visible CoM travel**: body roll/pitch, twist, alternating load. Musical timing, not padded delays.
- Amplitude references (do not slow them by default): `twist_fast.d6a`, `twist.d6a`.
- **No global slowdown parameter** as the design.
- **Abort, don’t pre-slow:** if `zmp_inside` is false for N consecutive ticks (parameter, start 5 ticks at 50 Hz ≈ 100 ms), publish `RunActionSet { action_path: "stop", interrupt: true }` and log the excursion.
- Day 7 success: ZMP **moves** (distance from polygon centroid is not ~0) **and** stays inside. A near-miss is a `.d6a` retune, not a software sleep.

## Node design

**Name:** `masha_interaction_node`  
**Package:** `proud_up`  
**Launch:** `ros2 launch proud_up masha_interaction.launch.py`

Phases: `LISTEN → IDENTIFY → HALT → SPEAK → DANCE → IDLE`

- `LISTEN`: subscribe only; ignore speech until canonical master query (ASR already requires wake).
- `IDENTIFY`: match `who is your master` (and aliases if ASR ever publishes raw-ish text).
- `HALT`: Traveling `gait=-2` + zero Twist; short wait (~0.3–0.5 s) so DEFAULT_POSE starts.
- `SPEAK`: start `aplay` worker; do not wait the full wav on the timer if it would stall ZMP setup — waiting for the line to finish **before** dance starts is OK (the reply is the point).
- `DANCE`: publish `RunActionSet` `master_dance`; run ZMP timer; abort on polygon exit.
- `IDLE`: ZMP timer can keep publishing at a lower rate or stop; ready for the next query while ASR stays awake.

Concurrency (copy Sprint 2):

- Callback thread: lock, store latest speech string / IMU / (optional) nothing heavy; return.
- Timer thread: lock, copy, unlock, then TF lookup, ZMP, publish. Never hold the mutex across `lookupTransform` if it can block — copy IMU under lock, TF outside.
- Playback: dedicated `std::thread` or `std::async`; join with timeout on shutdown.

Parameters (`config/masha_interaction.yaml`): action name `master_dance`, wav path, contact band, ZMP rate, abort ticks, `g` , link-mass table or a “use body CoM only” fallback.

Services (Masha app pattern, optional): `~/init_finish` (`std_srvs/Trigger`). Not required for Day 7 if logs are clear.

## Day-by-day on Masha

Keep Gemini’s 1 hr/day shape; change the artifacts.

### Day 1 — Masha’s gait architecture (not a textbook table)

Read **[`MASHA_GAITS.md`](MASHA_GAITS.md)** (same folder; copy also at `ros2_ws/info/MASHA_GAITS.md`). That is the Day 1 text: how Traveling vs `cmd_vel` vs `.d6a` actually run, the 20 ms loop, leg ids LF…RF, halt `gait=-2`. Then: support polygon vs CoM; tripod (1,3,5)=(LF,LR,RM) vs (2,4,6)=(LM,RR,RF). Eigen: point-in-triangle. No new node yet.

### Day 2 — FK / TF + ZMP formula

Real chain: `base_link → leg_center_* → coxa_* → femur_* → tibla_* → end_*`.

Checklist (one leg, e.g. LF):

- [ ] Coxa yaw (horizontal)
- [ ] Femur pitch
- [ ] Tibla pitch
- [ ] Product \(T_{\mathrm{end}}^{\mathrm{base}} = T_{\mathrm{center}} T_{\mathrm{coxa}} T_{\mathrm{femur}} T_{\mathrm{tibla}} T_{\mathrm{end}}\)
- [ ] Live: `ros2 run tf2_ros tf2_echo base_link end_LF` while standing

Jacobian / \(\det(J)\) is a notebook check, not a live servo loop this week.

Start `zmp.cpp`: cart-table, hull, point-in-polygon, gtest with synthetic feet.

### Day 3 — ASR wiring

Confirm existing voice: `ros2 launch xf_mic_asr_offline startup_test.launch.py` (or boot with `BRINGUP_VOICE=true`). Debug: `/tmp/masha-asr.log`.

Edit `asr_node.py` `extract_command` / `COMMAND_PHRASES` so the master query publishes `who is your master`.

Scaffold `masha_interaction_node` in `proud_up` that only subscribes to `/asr_node/voice_words` and logs.

Do not change `ASR_MODE` / `MIC_TYPE` in `.typerc`.

### Day 4 — ActionSet + wav

Peter: Qt `~/software/actionset_editor`, save **energetic** `master_dance.d6a` under `ActionGroups/`. grok-build: `RunActionSet` publisher (dry-run log first if needed). Record `master_response.wav`.

Do not reimplement IK in C++; vendor `kinematics.so` already feeds gait. Action groups are pulse sequences for the bus servos.

### Day 5 — state machine

Phases above. Mutex between speech callback and timer. HALT then SPEAK then DANCE. Start ZMP timer when DANCE starts.

### Day 6 — live ZMP + gtest

Wire `/imu` + tf2 into the library. Publish `~/zmp`, `~/support_polygon`, `~/zmp_inside`, `~/zmp_margin`. gtest: static ZMP = CoM projection; \(+\ddot x\) moves \(p_x\) the other way; inside/outside a triangle; hull of a square.

### Day 7 — hardware

```bash
source ~/.zshrc
cd ~/ros2_ws
colcon build --symlink-install --packages-select proud_up
source install/local_setup.zsh
ros2 launch proud_up masha_interaction.launch.py
```

Slim stack + voice should already be up. Do **not** run `~/.stop_ros.sh` just to start this node.

Sanity: `ros2 topic echo /asr_node/voice_words`, `ros2 topic echo /masha_interaction_node/zmp`, battery `ros2 topic echo /ros_robot_controller/battery --once`. Floor clear. Confirm spoken line, dance, ZMP trace inside polygon and not stuck at centroid.

## Files to add/touch (when implementing)

**Add in `proud_up`:**

- `include/proud_up/zmp.hpp`
- `src/zmp.cpp`
- `src/masha_interaction_node.cpp`
- `test/test_zmp.cpp`
- `launch/masha_interaction.launch.py`
- `config/masha_interaction.yaml`

**Edit:** `proud_up/CMakeLists.txt` (executable, `zmp` lib, gtest, `tf2_ros`, `tf2_geometry_msgs`, `kinematics_msgs`, `interfaces`, `std_msgs`), `proud_up/package.xml`.

**Voice:** `xf_mic_asr_offline/scripts/asr_node.py` (aliases); wav `feedback_voice/english/master_response.wav`.

**Dance:** `software/actionset_editor/ActionGroups/master_dance.d6a` (Peter).

Optional: ignore list in `voice_control_move.py`.

Do not edit vendor `app/`, `peripherals` Python, or bringup unless a launch include is later requested. Do not rebuild `servo_controller` for this.

## Out of scope

- Second sherpa / iFlytek IAT / serial keyword engine
- New ROS package
- Bluetooth as the design
- `yahboomcar_msgs`
- Replacing `voice_control_move`
- Slowing the dance “to be safe”
- Full multi-body ZMP with measured ground-reaction (cart-table + TF feet is this week)
- Camera / cat (Sprint 2) and later-week YOLO

## Troubleshooting (Masha, not X3)

| Symptom | Probable cause | Resolution |
|---|---|---|
| Voice not detected | Second `asr_node` stole the mic; or she is asleep | One voice launch. Say Hello Masha first. `pactl list sources`. `/tmp/masha-asr.log` |
| Query heard in log but nothing published | `extract_command` has no master alias | Add aliases; look for `cmd=` in the debug log |
| C++ node silent | Subscribed to `/voice_asr_result` | Must be `/asr_node/voice_words`. `ROS_DOMAIN_ID=27` |
| Gait keeps walking into the dance | Only zero Twist was sent | Also publish Traveling `gait=-2` |
| No motion | Wrong `RunActionSet` type/topic; missing `.d6a` | Topic `/controller/run_actionset`; file `ActionGroups/master_dance.d6a` |
| No audio | `paplay` / Bluetooth / blocking `aplay` | USB Pulse sink; worker-thread `aplay -D pulse` |
| Service timeout / no servos | STM32 gone | `/dev/ttyACM0` (`ros_robot_controller`). Not `ttyUSB1` |
| ZMP always origin / always outside | Wrong frame, gravity not removed, feet not in `base_link` | Check TF, IMU units, contact band |
| Battery | Not `/voltage` | `/ros_robot_controller/battery` |

## After the week

Write a progress report only when Peter says the sprint is complete. Push to `ganapetya/learning-bots-sharing` only with Peter’s OK. Robot code goes to `ganapetya/masha` via `~/git-push-all.sh`, also only with Peter’s OK.

Do not write `quarter-01-week-03-result.md` under the old week-3 (camera extrinsics) name — this sprint is **Sprint 3 (voice / ZMP)**, not Quarter 1 Week 3 of the original syllabus.

### Progress-report template

```md
# Sprint 3 progress report

- Date:
- Hardware: Jetson Orin NX, ROSpider (Masha), ROS_DOMAIN_ID=27
- Package: proud_up (`masha_interaction_node`)

## DoD

- [ ] Wake then “who is your master” published on /asr_node/voice_words
- [ ] Spoken reply: “I guess you think you are my master”
- [ ] Halt: Traveling gait=-2 and zero /controller/cmd_vel
- [ ] master_dance.d6a ran via /controller/run_actionset
- [ ] ~/zmp and ~/support_polygon published; ZMP stayed inside and moved

## Tests

- gtest test_zmp: (paste output)

## Interfaces used (not the syllabus names)

- …

## What Gemini got wrong and what we used

- …

## Notes for Sprint 4

- …
```
