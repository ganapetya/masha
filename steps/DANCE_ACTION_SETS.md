# How Masha dances — action groups as they are recorded

This is the **Sprint 3 Day 3** working note (calendar swapped 2026-09-13: dance today, ASR on Day 4). It is about **this robot**, not a generic hexapod tutorial.

Read this at the desk, then use the Qt editor on Masha. Ask grok-build about any button while you work. Saving `master_dance.d6a` is today’s deliverable. No C++ node, no wav, no sherpa on this day.

Sprint plan: `~/steps/quarter-01-sprint-03-plan.md`  
Gaits (do not mix with this): `~/steps/MASHA_GAITS.md`

Copies: `~/steps/DANCE_ACTION_SETS.md`

---

## 1. This is not a gait

Masha has three motion machines. The dance is the third.

| Machine | What it is | Typical trigger |
|---|---|---|
| **Gait** | Cyclic stepping, IK of six foot tips, 20 ms loop | `/controller/traveling`, `/controller/cmd_vel` |
| **Body pose** | Six feet planted; body leans over them | `/controller/pose_transform_euler` |
| **Action group** | Open-loop **pulse** sequence from a `.d6a` SQLite file | Qt editor (today) or `/controller/run_actionset` (later) |

An action group has **no IK, no support polygon, no ZMP**. Each row says “move servos 1–24 to these pulses in this many milliseconds.” The STM32 interpolates. If all six feet leave the floor, she falls. ZMP is a later-day estimator *while* this file plays, not something the file contains.

Voice “dance” already plays `twist.d6a`. Sprint 3’s dance is a **new** file, `master_dance.d6a`, triggered later by the C++ interaction node after “who is your master”.

Gaits and action groups both end on **the same** `/servo_controller` topic. They must not run together. Halt the gait (`gait=-2`) before the editor or before playback.

---

## 2. Today’s file

| | |
|---|---|
| Path | `/home/ubuntu/software/actionset_editor/ActionGroups/master_dance.d6a` |
| Runtime name | `master_dance` (basename, **no** `.d6a`) |
| Later ROS topic | `/controller/run_actionset` (`interfaces/msg/RunActionSet`: `action_path`, `interrupt`) |
| Do **not** overwrite | `twist.d6a`, `twist_fast.d6a`, `twist_l.d6a`, `twist_f.d6a`, `init.d6a`, `init_pose.d6a`, `wave.d6a` |

Amplitude / timing references (open them, do not save over them):

| File | Rows | Total time | Character |
|---|---|---|---|
| `init.d6a` | 1 | 0.5 s | Neutral stand (good first/last row) |
| `init_pose.d6a` | 1 | 1.0 s | Slightly different stand |
| `twist.d6a` | 25 | 13.2 s | Weight-shift / twist. Pattern: ~700 ms move + ~100 ms hold |
| `twist_fast.d6a` | 25 | 8.9 s | Same poses as `twist`, shorter times |
| `wave.d6a` | 13 | 4.4 s | One-side lift + coxa wiggle |

---

## 3. What is stored (the `.d6a` format)

A `.d6a` is **SQLite**, one table named `ActionGroup`:

```
Index INTEGER PRIMARY KEY
Time  INT                 -- milliseconds for this row (min 20, max 30000)
Servo1 INT … Servo24 INT  -- pulse 0–1000
```

- One **row** = one **keyframe**.
- `Time` is how long the STM32 takes to **arrive** at that row from the previous pulses. It is not a sleep after the pose, and it is not a quintic. Do not pad with huge times “to be safe.”
- Pulses are **physical bus IDs**, not IK `leg_id` and not joint radians.
- Servos **19–24** are the arm + gripper. Park them unless the dance uses the arm. Rest used by `twist` / `init`:

  `19=500, 20=720, 21=130, 22=150, 23=500, 24=500`

Inspect a file from a shell (after save):

```bash
python3 - <<'PY'
import sqlite3
p='/home/ubuntu/software/actionset_editor/ActionGroups/master_dance.d6a'
c=sqlite3.connect(p)
rows=c.execute('select * from ActionGroup').fetchall()
print('rows', len(rows), 'total_ms', sum(r[1] for r in rows))
for r in rows:
    print('idx', r[0], 't', r[1], 's1-18', r[2:20], 'arm', r[20:26])
PY
```

Playback (ROS, **not** today) reads those rows in order, publishes `ServosPosition` with `position_unit='pulse'` and `duration = Time/1000`, then `sleep(Time/1000)`. `/action_complete` is `false` while running, `true` when finished.

---

## 4. Safety (do this first)

1. Floor clear. Hands or a stand ready under the chassis.
2. If she is walking, halt the **gait** (the editor does not do this for you):

```bash
source ~/.zshrc
ros2 topic pub --once /controller/traveling kinematics_msgs/msg/Traveling \
  "{gait: -2, time: 1.0, steps: 0, interrupt: true}"
```

3. Slim ROS may stay up. The editor, on start, publishes `/ros_robot_controller/enable_reception false` so the ROS serial node **stops parsing** `/dev/ttyACM0`, then the Qt process talks to the STM32 itself. On **Quit** it turns reception back on.
4. Do **not** run Nav2, `perform_actions`, or another `.d6a` player at the same time as the editor.
5. Do **not** click **Manual** until you are supporting the body. Manual unloads **all 24** servos. She will collapse.
6. **Reset servo** sends every ID to pulse 500 in 1 s. That is not a stand pose. Avoid it.
7. **Erase single** / **All erase** delete `.d6a` files on disk. Do not use them.

---

## 5. Launch the UI

Desktop icon **ROSpider**, or:

```bash
zsh ~/software/actionset_editor/actionset_editor.sh
```

That is `python3 ~/software/actionset_editor/main.py`. Click **English** (radio). Chinese labels are listed below only so a leftover 中文 button is still findable.

Code: `~/software/actionset_editor/main.py`, layout `Ui.py`. Action files directory is hardcoded next to the script: `…/actionset_editor/ActionGroups/`.

---

## 6. Screen map (English)

The window is three regions:

1. **Servo panel (IDs 1–24)** — slider + number box per bus servo. Dragging a slider **moves that servo live** (20 ms move). Typing a number and pressing **Enter** sends all 24 typed values (500 ms). Labels next to some sliders: U/D (up/down), F/B (front/back), L/R, O/C (open/close on the gripper).
2. **Keyframe table** — columns: play-icon, Index, Time, ID:1 … ID:24. Each row is one keyframe. Click a row to select it. The small icon in column 0 **plays that one row** onto the robot and copies its pulses into the sliders.
3. **Buttons** — edit the table, save/load files, play.

**Duration** (default `1000`, allowed 20–30000) is the `Time` written into the **next** Add / Read angle / Update / Insert. **Total duration** is the sum of table Times, in seconds.

**Lock time** (checkbox on the time field): when you play a single row, do not overwrite the Duration box from that row.

**Loop**: **Run** repeats the table from the row you started on until you press Stop.

### Buttons you will use today

| English | 中文 | What it actually does |
|---|---|---|
| **Manual** | 手掰编程 | Unload (torque off) servos 1–24. Pose by hand. Support the body first. |
| **Read angle** | 角度回读 | Read current pulses from the bus, **append a new table row**, `Time` = Duration box. This is “store the real leg positions.” If any servo read returns empty, it aborts. |
| **Add action** | 添加动作 | Append a row from the **current sliders** (not from the bus). |
| **Update action** | 更新动作 | Overwrite the **selected** row from the current sliders + Duration. |
| **Insert action** | 插入动作 | Insert a row **above** the selection, from current sliders. |
| **Delete action** | 删除动作 | Remove the selected row. |
| **Action upward / down** | 上移 / 下移动作 | Swap the selected row with its neighbour. |
| **Open action file** | 打开动作文件 | Load a `.d6a` into the table (replaces the table). |
| **Save action file** | 保存动作文件 | Write the **table** as SQLite `.d6a`. File dialog defaults to `ActionGroups/`. |
| **Integrate file** | 串联动作文件 | Append another `.d6a` onto the **end** of the current table (concatenate). |
| **Run** | 运行 | Play the **table** from the selected row, live, whether or not you have saved. Toggles to Stop. |
| **Run action** | 动作运行 | Play the **saved file** named in the combo box (`Action list`), from disk, in a background thread. |
| **Stop** | 动作停止 | Stop that disk playback. |
| **Reset servo** | 舵机回中 | All sliders → 500, move there in 1 s. Avoid. |

### Buttons to leave alone today

| English | Why |
|---|---|
| **Read deviation / Download deviation** | Servo EEPROM offset calibration. Not choreography. A bad download changes every future pose. |
| **Erase single / All erase** | Deletes `.d6a` files under `ActionGroups/`. |
| Combo-box **Action list** + erase | Same: disk delete. |

Right-click the table → “Delect all action” clears the **table** (not disk) after a confirm dialog.

---

## 7. Two ways to capture a pose

### A. Hand-pose (record real legs)

This is the path the day is for.

1. Support the chassis.
2. **Manual** — servos go limp.
3. Bend the legs to the pose you want. Keep a support triangle on the floor if this pose is meant to be standing.
4. Set **Duration** (ms) for how long the *playback* should take to reach this pose.
5. **Read angle** — a new row appears with live pulses.
6. Torque is still off until you **Run** a row or drag a slider (those send pulses and the servos hold again).

If Read angle says it failed, a servo did not answer. Do not save a half-row; try again.

### B. Sliders (edit in software)

1. Open a template (`twist` or `init`) or click a table row (icon play loads it into sliders).
2. Drag IDs that belong to the joints you want (see §8). The robot **moves as you drag**.
3. Set **Duration**.
4. **Add action** (new row) or **Update action** (replace selected row).

Use A for new shapes, B for small corrections and for mirroring a left-leg pulse onto the right.

---

## 8. Bus IDs ↔ legs

Editor columns are **physical servo IDs**. IK `leg_id` 1–6 (LF…RF) is a different numbering; do not type those into the table.

| Leg | coxa (yaw) | femur | tibla (vendor spelling) |
|---|---|---|---|
| **LF** left front | **5** | **3** | **1** |
| **LM** left middle | **11** | **9** | **7** |
| **LR** left rear | **17** | **15** | **13** |
| **RF** right front | **6** | **4** | **2** |
| **RM** right middle | **12** | **10** | **8** |
| **RR** right rear | **18** | **16** | **14** |
| Arm + gripper | 19–24 | | |

`init.d6a` stand (legs only), useful as first and last row:

```
tibla  L/R:  1=700  2=300     7=700  8=300    13=700  14=300
femur  L/R:  3=300  4=700     9=300 10=700    15=300  16=700
coxa   L/R:  5=500  6=500    11=500 12=500    17=500  18=500
```

Left and right femur/tibia are **mirrors** around 500 (left 300 ↔ right 700). Coxa 500 is “straight out” in these files. Pulse 0–1000; the board clamps.

Tripod reminder (for *which feet stay down*, not for IK): stance triangles LF+LR+RM and LM+RR+RF. A dance keyframe that lifts a whole triangle plus another foot is how she falls. ZMP work on Days 6–7 needs the point to **move** inside the hull — so do shift weight; do not lift everyone.

---

## 9. Choreography rules (Sprint 3)

Peter’s design, not a software slowdown:

- **Energetic.** Visible CoM travel: body roll / pitch / twist, alternating load. A timid stand fails the later ZMP DoD (ZMP must move, not sit on the centroid).
- **Musical timing.** `twist` uses ~700 ms to move and ~100 ms to hold. Start there. Minimum 20 ms. Do not insert multi-second gaps.
- **At least three feet down** on every standing keyframe.
- **End in a stand** close to `init.d6a` so a gait can start after `/action_complete`.
- **Park the arm** (19–24 at rest) unless you deliberately include it.
- **No quintic** on `.d6a` rows. No extra sleeps in the 20 ms gait loop. Those are out of scope for action groups.
- If she tips on **Run**, retune **poses**, not playback speed as a global knob.

---

## 10. Session sequence (this day)

1. Halt gait if needed (§4). Launch editor. **English**.
2. **Open action file** → `twist.d6a` or `init.d6a`.
3. **Save action file** immediately as `master_dance.d6a` (before any edit). Confirm the path is `ActionGroups/master_dance.d6a`.
4. Build keyframes with Manual + Read angle and/or sliders + Add/Update.
5. After every few rows, select the first row and **Run** the table. Watch the feet.
6. Last row ≈ `init` stand.
7. **Save action file** again (same name).
8. One full **Run** on the floor.
9. **Quit** the editor (restores ROS serial reception). Tell grok-build — inspect with the Python snippet in §3.

Suggested first dance if you want a short original loop rather than a `twist` edit: stand → roll left (load LF+LR+LM) → stand → roll right → twist coxae (5/6, 11/12, 17/18) → stand. Keep tibias from folding under.

---

## 11. Done when

- [ ] File exists: `/home/ubuntu/software/actionset_editor/ActionGroups/master_dance.d6a`
- [ ] `twist` / `init` files still exist and were not overwritten
- [ ] Table has **more than one** row; `Time` values are musical, not padded
- [ ] Servo1–24 columns present; arm 19–24 at rest unless intended
- [ ] Last row is a stable stand
- [ ] One live **Run** completed without falling
- [ ] Editor quit (serial reception back to ROS)

Not this day: ASR aliases, `master_response.wav`, `masha_interaction_node`, ZMP library, `/controller/run_actionset` from C++.

---

## 12. Later (not today) — play from ROS

After the editor is quit and reception is on:

```bash
source ~/.zshrc
# halt gait first if she is walking
ros2 topic pub --once /controller/traveling kinematics_msgs/msg/Traveling \
  "{gait: -2, time: 1.0, steps: 0, interrupt: true}"

ros2 topic pub --once /controller/run_actionset interfaces/msg/RunActionSet \
  "{action_path: 'master_dance', interrupt: true}"

ros2 topic echo /action_complete
```

`action_path: stop` aborts the group and plays `init_pose`. Names `dance_1` / `dance_2` / `dance_3` are forwarded to `/perform_actions/actions` — do **not** use those names for this file.

---

## 13. Code map (if a button misbehaves)

| Piece | Path |
|---|---|
| Qt UI | `~/software/actionset_editor/main.py` |
| Direct serial player (editor) | `~/software/actionset_editor/action_group_controller.py` |
| ROS player | `ros2_ws/src/driver/servo_controller/servo_controller/action_group_controller.py` |
| ROS façade | `controller/move_controller.py` `run_actionset_callback` |
| Servo ID YAML | `ros2_ws/src/driver/servo_controller/config/servo_controller.yaml` |

The editor does **not** go through ROS topics while it is open. Closing it is what gives the bus back to `ros_robot_controller`.
