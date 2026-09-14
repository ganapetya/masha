# Master dialog + dance (Sprint 3 slice)

**Date on Masha:** 2026-09-14, comments and topic grounding 2026-09-14  
**Parent sprint:** [`quarter-01-sprint-03-plan.md`](quarter-01-sprint-03-plan.md)  
**Status:** the C++ node, ASR aliases, ignore window, and both mp3 files are on disk. Read this as a map of the code that is here, not as a plan of work still to do.

This memo used to describe a play that had not been written. The play is written. A few lines in the old memo named rooms that do not exist on this robot. Those lines are corrected below.

---

## What she does

Someone says **“Masha, who is your master”**. She answers **“I guess you think you are my master”**, stands, then plays the action group `master_dance` while music runs. When the group ends (or time runs out, or someone says **stop**), the music dies and she waits again.

Wake is still **Hello / Hi / Shalom Masha** in `asr_node.py`. The master line is a *command*, not a new greeting. The leading name is address. Walk and the old voice “dance” (`twist.d6a`) still need the greeting if she is asleep. The master line is taken from idle — see `_should_take_command` in `asr_node.py`.

This slice is not ZMP and not follow-gait.

---

## How to read the code

Start here, in this order. Each file is a person in the same room.

| Order | File | Role |
|---|---|---|
| 1 | `proud_up/src/masha_interaction_node.cpp` | Stage manager. Phases, halt, speech, dance cue, music. |
| 2 | `proud_up/include/proud_up/interaction_phrases.hpp` | The tiny ear: master question and “stop”. No ROS. |
| 3 | `xf_mic_asr_offline/scripts/asr_node.py` | The real ear. Publishes canonical strings on `/asr_node/voice_words`. |
| 4 | `xf_mic_asr_offline/scripts/voice_control_move.py` | The walker. Ignores `who is your master` so it does not steal the legs. |
| 5 | `controller/move_controller.py` | The body. Halt gaits, and the `run_actionset` postcard. |
| 6 | `servo_controller/action_group_controller.py` | The pianist. Reads a `.d6a` SQLite roll and publishes servo pulses. |

Then the small wiring: `launch/masha_interaction.launch.py`, `config/masha_interaction.yaml`, `test/test_interaction_phrases.cpp`.

---

## The spoken path (as the code does it)

```
“Who is your master” / “Masha, who is your master”
        asr_node publishes  "who is your master"
                     →  Idle hears it (logged as IDENTIFY, not a phase)
                     →  Halt: zero Twist, Traveling gait=0, then gait=-2
                     →  Speak: master_response.mp3  (no file → no dance)
                     →  Dance: RunActionSet master_dance  +  music
                     →  wait /action_complete false then true
                        (or dance_timeout_s, or “stop”)
                     →  Idle
```

Sherpa aliases, already in both `asr_node.COMMAND_PHRASES` and `kMasterAliases`:

- `who is your master`
- `who's your master` → punctuation stripped → `who s your master`
- `who your master`
- `who is ur master`
- `whose your master`

`voice_control_move` ignores only the **canonical** string `who is your master`. That is what ASR publishes. The C++ matcher still accepts the aliases in case a raw transcript ever arrives.

**stop** during Halt / Speak / Dance sets `abort_`. What happens next depends on the phase:

| Phase when “stop” arrives | What the code does |
|---|---|
| Halt | return to Idle (stand may already have been sent) |
| Speak | kill aplay, `halt_legs()`, Idle — no `RunActionSet stop` (the dance has not started) |
| Dance | kill music, `RunActionSet action_path=stop`, `halt_legs()`, Idle |

`MoveController` treats `stop` as: halt the group, then play `init_pose`. Do not name our dance `stop` / `dance_1` / `dance_2` / `dance_3`.

---

## Architecture (reuse, as wired)

```
USB 6-mic  →  asr_node.py (sherpa-onnx, only listener)
                 → /asr_node/voice_words
                      ├─ voice_control_move.py   (walk / twist-dance / stop)
                      └─ masha_interaction_node  (package proud_up)
                           1. match "who is your master" while Idle
                           2. Halt: zero cmd_vel, Traveling gait=0 then gait=-2
                           3. play master_response.mp3 on a worker thread
                              (ffmpeg → wav cache, then aplay -D pulse)
                           4. postcard /controller/run_actionset  master_dance
                              and start music in the same breath
                           5. wait /action_complete false, then true
                           6. stop music → Idle
```

Do **not** launch a second `asr_node`. Do **not** launch `follow_the_cat_node` next to this node. Do **not** change `ASR_MODE` / `MIC_TYPE` in `.typerc`.

A ROS *topic* is a postcard. A *service* is a registered letter. `/controller/run_actionset` is a postcard (`interfaces/msg/RunActionSet`: `action_path`, `interrupt`). There is no `RunActionSet` service on this robot. Do not copy `controller_client.run_actionset()` — that helper sets `repeat` and `default_path`, fields of `kinematics_msgs/msg/RunActionSet`, which is a different letter.

---

## Why not Python `voice_control_move`?

The parent sprint assigned this to a C++ node in `proud_up`. `voice_control_move` stays the walk / `twist` player. The master phrase is on its ignore list (`INTERACTION_IGNORED`) so it does not log-unmatched. Music and the spoken line are **mp3**. `voice_play.play()` is wav-name-only and kills `aplay` at **3 s** (`voice_play.py`, `timeout=3`). That would clip the reply and cannot run a 14.3 s track.

---

## State machine (the enum, not the old memo)

The C++ `Phase` enum is four names: `Idle`, `Halt`, `Speak`, `Dance`.

The old memo wrote `LISTEN → IDENTIFY → HALT → SPEAK → DANCE → IDLE`. In the code, LISTEN and IDLE are the same `Idle`. IDENTIFY is a log line inside `on_speech`, then we walk into `Halt`. There is no fifth state.

| Phase | What the code does |
|---|---|
| Idle | Subscribe `/asr_node/voice_words`. Ignore until a master match. Accept `stop` only if we are already in Halt / Speak / Dance. |
| Halt | First tick: `halt_legs()`. Then wait `halt_wait_s` (0.4 s). Missing spoken mp3 → Idle, **no dance**. Else start the player and enter Speak. |
| Speak | Wait until the player finishes, or `speak_timeout_s` (8.0 s). Then `start_dance()`. |
| Dance | Stay until (`saw_action_running` **and** `action_complete`) or timeout 16.5 s or abort. Then stop music, Idle. |

ASK / WAIT_YES / FOLLOW / ZMP stay in the parent sprint. The enum has a comment where they can hang; they are not wired.

`~/init_finish` (`/masha_interaction_node/init_finish`) is the same “are you up?” doorbell the other `proud_up` nodes offer. The reply message is the current phase name.

Concurrency, as written:

- Speech / `action_complete` callbacks: take the mutex, write flags, return. No `aplay`, no sleep.
- Timer `tick()` (50 ms): copy flags, drop the mutex, then publish / decide.
- Playback: a `std::thread` that `posix_spawn`s aplay or ffplay. `fork()` is not used — this process is already multithreaded.

---

## Halt (standing still is not walking at speed zero)

`halt_legs()` publishes three postcards, in this order:

1. A zero `Twist` on `/controller/cmd_vel` — so a leftover walk does not keep a generator.
2. `Traveling {gait: 0, time: 1.0, interrupt: true}` — stop the stepping loop.
3. `Traveling {gait: -2, time: 1.0, interrupt: true}` — snap to `DEFAULT_POSE`.

`steps` is left at its message default (0). A *lone* zero Twist would start `CmdVelGenerator` at speed 0 and she would march in place. That is why `follow_the_cat` never sends a zero Twist without Traveling. We send the Twist first, then the stand last so the stand wins if both arrive in one tick.

`MoveController.set_traveling_callback`: gait `> 0` walks, gait `0` stops, gait `-2` is `DEFAULT_POSE`.

---

## Action groups (the piano roll)

A `.d6a` is SQLite, one table `ActionGroup`. Each row is a keyframe: milliseconds, then pulses for servos 1–24. `ActionGroupController.run_action` opens the file, publishes `ServosPosition` for each row, sleeps that row’s time, and on the `status_pub` it was given writes **false** when it starts and **true** when the cursor runs off the end.

| | |
|---|---|
| File | `~/software/actionset_editor/ActionGroups/master_dance.d6a` |
| Rows | **25** |
| Total time | **14.3 s** (14300 ms) |
| Runtime name | `master_dance` (basename, no `.d6a`) |
| Cue | `/controller/run_actionset` |

The old memo said 21 rows / 13.5 s. That was the plan. The file on disk is 25 / 14.3. `dance_timeout_s` is 16.5 — a backstop, not a sleep that “is” the dance.

First and last **leg** rows are all pulse **500** (not `init.d6a` stand: coxa/femur/tibia 700, 300, 300, 700, …). Mid-range femur/tibia is not a stand. Floor-clear, hands ready. If she sags, retune the `.d6a` in the Qt editor — do not “fix” it by slowing playback. Arm 19–24 start parked (`500, 720, 130, 150, 500, 500`); later rows swing id 19 (250 / 500 / 750) and id 24 (gripper).

---

## The completion mailbox (Masha naming, live-checked)

On a running robot (`ros2 topic list`, 2026-09-14):

| Topic | Who writes | Who reads |
|---|---|---|
| `/controller/run_actionset` | interaction node (and `controller_client`) | `/controller` (`~/run_actionset`) |
| `/action_complete` | `/controller` (`ActionGroupController` via `status_pub`) | `/controller` itself, and now `masha_interaction_node` |
| `/controller/action_complete` | nobody | empty street |

ROS 2 name expansion, node name `controller`, namespace `/`:

```
'action_complete'     →  /action_complete          (no tilde: namespace only)
'~/run_actionset'     →  /controller/run_actionset (tilde: private)
'~/action_complete'   →  /controller/action_complete  (we do not use this)
```

The old memo inverted this. It claimed the player wrote `/controller/action_complete` and that `/action_complete` stayed quiet. Live `ros2 topic info` shows the opposite: one publisher, node `controller`, topic `/action_complete`.

The C++ node and the YAML now subscribe to **`/action_complete`**. We wait for **false then true** (`saw_action_running_`), so a leftover true from the previous group does not end this dance on the first tick. Timeout 16.5 s is the backstop if a Bool never arrives.

Do **not** `ros2 topic echo /controller/action_complete` and wait. Echo `/action_complete`.

---

## Audio (both clips are mp3)

One `AudioPlayer` in the C++ node. Source of truth is **mp3**. `aplay` cannot play mp3, so **on first play** (worker thread, not node start) ffmpeg writes a wav next to the mp3 if the wav is missing or older:

```
ffmpeg -y -hide_banner -loglevel error -i src.mp3 -ar 44100 -ac 2 cache.wav
```

Then `aplay -q -D pulse` on that wav. If pulse cannot start, **ffplay** on the mp3. We do **not** walk the three-device list in `voice_play.py` (`pulse`, `default`, `plughw:CARD=Device,DEV=0`). Generated wavs are cache only (`sounds/.gitignore`).

Do **not** use `paplay`. Do **not** call `system()` on the ROS callback. Do **not** go through `voice_play.py`.

### Spoken reply

| | |
|---|---|
| Path | `~/ros2_ws/src/xf_mic_asr_offline/feedback_voice/english/master_response.mp3` |
| Cache wav | same dir, `master_response.wav` (generated) |
| Exact text | **I guess you think you are my master** |
| Missing file | Log error, return to Idle. **Do not dance** without the line. |

### Dance music

| | |
|---|---|
| Path | `~/ros2_ws/src/proud_up/sounds/master_dance.mp3` |
| Cache wav | `~/ros2_ws/src/proud_up/sounds/master_dance.wav` (generated) |
| Start | When `RunActionSet` is published (Speak has already finished) |
| Stop | `/action_complete` true after a false, or `stop`, or `dance_timeout_s`, or node shutdown |
| Loop | **Yes**, until dance ends (`music_loop: true`) |
| Missing file | Dance proceeds silent. Do not block the machine. |

ASR leak: the USB mic will hear the music. What the code actually does:

1. `asr_node._accept_command`: after the master command, `ignore_until = now + 20.0` seconds. That **20 is a hardcoded number**, not a ROS parameter. The clock starts when the phrase is accepted, **not** when the music later stops. `stop` still uses 0.8 s. Other walk prompts use 1.6 s.
2. `voice_control_move` ignore-list: canonical `who is your master`. The ignore window in (1) is the real guard against music-hallucinated “go forward”.
3. Interaction node in Speak / Dance / Halt accepts only `stop` as a new command.

---

## Files (as they stand)

**This memo:** `~/steps/MASTER_DIALOG_DANCE.md`

**In `proud_up`:**

- `include/proud_up/interaction_phrases.hpp` — alias match, no rclcpp
- `src/masha_interaction_node.cpp` — state machine, halt, mp3 player
- `launch/masha_interaction.launch.py`
- `config/masha_interaction.yaml`
- `sounds/master_dance.mp3` (wav cache generated next to it)
- `test/test_interaction_phrases.cpp` — six tests, alias → canonical
- `CMakeLists.txt` / `package.xml` — executable, gtest, `interfaces`, `std_msgs`

**Voice, already edited for this slice:**

- `xf_mic_asr_offline/scripts/asr_node.py` — `COMMAND_PHRASES` master aliases; 20 s `ignore_until`
- `xf_mic_asr_offline/scripts/voice_control_move.py` — `INTERACTION_IGNORED`

**On disk, not code:**

- `feedback_voice/english/master_response.mp3`
- `proud_up/sounds/master_dance.mp3`
- `~/software/actionset_editor/ActionGroups/master_dance.d6a`

**Do not edit for this slice:** `app/`, `peripherals` Python, `bringup` (no auto-launch), `kinematics.so`, `twist.d6a` / `init.d6a`. Do not retune `master_dance.d6a` in software.

---

## Parameters (`config/masha_interaction.yaml`)

These match `declare_parameter` in the node.

- `asr_topic: /asr_node/voice_words`
- `traveling_topic: /controller/traveling`
- `cmd_vel_topic: /controller/cmd_vel`
- `run_actionset_topic: /controller/run_actionset`
- `action_complete_topic: /action_complete`
- `action_name: master_dance`
- `master_response_mp3:` absolute path to the spoken mp3
- `music_mp3:` path under `proud_up/sounds/`
- `music_loop: true`
- `halt_wait_s: 0.4`
- `speak_timeout_s: 8.0`
- `dance_timeout_s: 16.5`

---

## What the old memo got wrong

A short ledger, so the code and this page stay friends.

| Old memo | Code / live robot |
|---|---|
| `masha_interaction_node` not written; mp3s missing | Node, both mp3s, and the `.d6a` are on disk |
| Phases `LISTEN → IDENTIFY → … → IDLE` | Enum: `Idle, Halt, Speak, Dance`. IDENTIFY is a log. |
| Halt is only `{gait: -2, steps: 0}` | Zero Twist, then gait 0, then gait -2. `steps` left at default. |
| aplay devices pulse, then default, then plughw | `aplay -D pulse`, then ffplay on the mp3 |
| ffmpeg at node start | ffmpeg on first play, in the worker thread |
| Wait `/controller/action_complete == true` | Live mailbox is `/action_complete`. Wait false, then true. |
| `master_dance.d6a` 21 rows / 13.5 s | 25 rows / 14.3 s |
| `ignore_until` “parameter, ~20 s after music stops” | Hardcoded 20.0 s from the moment the command is accepted |
| `stop` during SPEAK/DANCE always sends `RunActionSet stop` | Only Dance sends it. Speak kills audio and stands. Halt just returns to Idle. |
| `voice_control_move` ignore-list includes aliases | Ignore-list is the canonical string only |

If an earlier floor test stopped the music about two seconds after the body finished, that was the 16.5 s timeout, not the Bool. The subscription is now on the mailbox the pianist actually writes.

---

## Out of scope (this slice)

- ZMP estimator / `zmp.hpp` / Foxglove polygon
- Step 2: `follow_offer` clip, yes/no, gait=5, YOLO
- Second sherpa, Bluetooth, `yahboomcar_msgs`, `paplay`
- Replacing `voice_control_move` or the existing voice “dance” → `twist`
- Auto-start from `bringup.launch.py`
- Recording either mp3
- Commits/push (`~/git-push-all.sh` only with Peter’s OK)

---

## Launch (on Masha)

Do **not** start a second copy of the voice launch. A second `asr_node` steals the USB mic.

### After a normal boot

Slim bringup already starts voice (`BRINGUP_VOICE=true`). You only need the interaction node:

```bash
source ~/.zshrc
ros2 launch proud_up masha_interaction.launch.py
```

Leave that terminal open. Healthy line:

```text
masha_interaction asr=/asr_node/voice_words action=master_dance complete=/action_complete ...
```

If it warns that `master_response.mp3` is missing, the dance will not run. Music missing is OK (silent dance).

### Voice is not up

Terminal 1 — voice (ASR + walk / `twist` dance):

```bash
source ~/.zshrc
ros2 launch xf_mic_asr_offline startup_test.launch.py
```

Wait until `I am Masha`, `recording from pulse`, `I am ready`.

Terminal 2 — master dialog + `master_dance`:

```bash
source ~/.zshrc
ros2 launch proud_up masha_interaction.launch.py
```

### Rebuild (only if you changed C++)

Python voice files are symlink-installed: edit, then **restart the voice launch**. C++ needs:

```bash
source ~/.zshrc
cd ~/ros2_ws
colcon build --symlink-install --packages-select proud_up
source install/local_setup.zsh
```

Then restart `masha_interaction.launch.py`.

### Restart voice without killing the whole robot

Find the launch, interrupt it, start it again. Do **not** run `~/.stop_ros.sh` just for this.

```bash
pgrep -af 'startup_test.launch.py'
# SIGINT that pid, wait until asr_node.py is gone, then:
source ~/.zshrc
ros2 launch xf_mic_asr_offline startup_test.launch.py
```

### Talk

No Hello needed for this phrase. Say clearly:

**Masha, who is your master**

She answers **I guess you think you are my master**, then plays `master_dance` with music.

To run it again: the microphone ignores the speaker for **20 s from the moment the phrase was accepted** (speak + dance usually fill most of that). After 120 s of silence she sleeps; the master line still starts this sequence. Walk / “dance” (`twist.d6a`) still need **Hello Masha** if she is asleep.

**stop** during the spoken line or the dance kills music (and, if the dance had started, the action group).

### Debug

```bash
tail -f /tmp/masha-asr.log
# look for: ok who is your master
ros2 topic echo /asr_node/voice_words
ros2 topic echo /controller/run_actionset --once
ros2 topic echo /action_complete
```

---

## Verification

1. `colcon build --symlink-install --packages-select proud_up`
2. Slim stack + voice already up. `ros2 launch proud_up masha_interaction.launch.py`
3. `ros2 topic echo /asr_node/voice_words` — the query publishes `who is your master` (idle is OK)
4. USB speaker: spoken `master_response.mp3`, then dance music (or warning if dance mp3 absent)
5. `ros2 topic echo /controller/run_actionset --once` shows `master_dance`
6. `ros2 topic echo /action_complete` goes false then true; music stops. Log line `dance complete — music stopped` (not the timeout warning)
7. `stop` mid-dance kills music and plays `init_pose`
8. Existing “dance” still plays `twist.d6a`
9. gtest `test_interaction_phrases` (6 tests)

---

## Execution gate

| Step | When |
|---|---|
| Write `~/steps/MASTER_DIALOG_DANCE.md` | Done, then grounded against the code 2026-09-14 |
| C++ / ASR / audio wiring | Done (`test_interaction_phrases` 6 tests) |
| Completion topic pointed at `/action_complete` | Done (was `/controller/action_complete`, an empty street) |
| Live floor test of the Bool path | Re-run after this topic fix; earlier music stop may have been the 16.5 s timeout |
