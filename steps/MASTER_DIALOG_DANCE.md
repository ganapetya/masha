# Master dialog + dance (Sprint 3 slice)

**Date on Masha:** 2026-09-14  
**Parent sprint:** [`quarter-01-sprint-03-plan.md`](quarter-01-sprint-03-plan.md)  
**Status:** implemented 2026-09-14. Waiting on `master_response.mp3` (required) and `master_dance.mp3` (optional). Live floor test after those files land.

## Context

Peter authored `master_dance.d6a` (Sprint 3 Day 3). The next live behaviour is:

1. Peter (already awake): **“Masha, who is your master”**
2. Masha speaks **“I guess you think you are my master”**
3. Masha plays action group **`master_dance`**
4. **While she dances, music plays** (mp3 supplied later)

This is Sprint 3 Step 1, Days 4–5, plus one new requirement the parent plan does **not** have: **dance-time music**. It is **not** ZMP (Days 6–7) and **not** Step 2 follow-gait.

What is already true on Masha:

| Piece | State |
|---|---|
| `~/software/actionset_editor/ActionGroups/master_dance.d6a` | Exists. 21 rows, **13.5 s** total |
| `masha_interaction_node` | **Not written.** `proud_up` still has only `proud_up_node` + `follow_the_cat_node` |
| ASR `extract_command` | Walk / dance / stop only. Master query is dropped |
| Spoken line | **Missing.** Source file is `master_response.mp3` (not wav) |
| Dance music | **Missing** (Peter will drop an mp3 later) |
| Voice “dance” | Still plays `twist.d6a` via `voice_control_move` — leave that phrase alone |

Wake is unchanged: **Hello / Hi / Shalom Masha**, then 120 s of commands. “Masha, who is your master” is the **command** (the leading name is address). It does **not** replace the greeting wake.

## Spoken protocol (this slice)

```
“Who is your master” / “Masha, who is your master”
                     →  “I guess you think you are my master”
                     →  halt gait
                     →  master_dance.d6a  +  music until dance ends
                     →  IDLE
```

Hello Masha is **not** required for this phrase (taken from idle). Walk / “dance” (`twist`) still need the greeting if she is asleep. See **Launch** below.

Sherpa aliases (canonical string published on `/asr_node/voice_words`):

- `who is your master`
- `who's your master` → same (punctuation stripped → `who s your master`)
- `who your master`
- `who is ur master`
- `whose your master`

`stop` during SPEAK/DANCE aborts: kill music, `RunActionSet action_path=stop`, Traveling `gait=-2`.

## Architecture (reuse, do not invent)

```
USB 6-mic  →  asr_node.py (sherpa-onnx, only listener)
                 → /asr_node/voice_words
                      ├─ voice_control_move.py   (walk / twist-dance / stop)
                      └─ masha_interaction_node  (NEW, package proud_up)
                           1. match canonical "who is your master"
                           2. Traveling gait=-2  +  zero /controller/cmd_vel
                           3. play master_response.mp3  (worker thread; ffmpeg→wav then aplay)
                           4. RunActionSet master_dance  +  start music
                           5. wait /controller/action_complete == true
                           6. stop music → IDLE
```

Do **not** launch a second `asr_node`. Do **not** launch `follow_the_cat_node` next to this node. Do **not** change `ASR_MODE` / `MIC_TYPE` in `.typerc`. Build only `proud_up`.

## Why not Python `voice_control_move`?

The parent sprint already assigned this to a **C++** node in `proud_up` (same pattern as Sprint 2). `voice_control_move` stays the walk / `twist` player. The master phrase must be on its **ignore list** so it does not log-unmatched and so a later “yes” does not get stolen. Music and the spoken line are **mp3**, need a worker-thread player with a **timeout longer than 3 s**, and cannot go through `voice_play.play()` — that API is wav-name-only and hard-timeouts `aplay` at **3 s**, which would clip the reply and cannot run a 13.5 s track.

## State machine (this slice only)

Phases: `LISTEN → IDENTIFY → HALT → SPEAK → DANCE → IDLE`

| Phase | What happens |
|---|---|
| LISTEN | Subscribe `/asr_node/voice_words`. Ignore until canonical master query. |
| IDENTIFY | Match aliases (also works if ASR already published the canonical string). |
| HALT | Traveling `{gait: -2, time: 1.0, steps: 0, interrupt: true}` on `/controller/traveling` **and** zero Twist on `/controller/cmd_vel`. Wait ~0.3–0.5 s so `DEFAULT_POSE` starts. Copy `follow_the_cat_node::halt_legs` (gait 0 then −2 is also fine). |
| SPEAK | Worker-thread play of `master_response.mp3` (same player as dance music: ffmpeg→wav cache, then `aplay -D pulse`). Wait until the line finishes **before** dance (the reply is the point). Timeout ~8 s, not 3 s. Fallback: `ffplay -nodisp -autoexit` on the mp3; aplay devices `pulse`, then `default`, then `plughw:CARD=Device,DEV=0`. |
| DANCE | Publish `interfaces/msg/RunActionSet {action_path: "master_dance", interrupt: true}` on `/controller/run_actionset`. **Same moment:** start music. Stay until `/controller/action_complete` is **true**, or abort on `stop`, or timeout (`dance_timeout` ≈ 13.5 s + 3 s margin). |
| IDLE | Music stopped. Ready for the next query while ASR stays awake. |

ASK / WAIT_YES / FOLLOW / ZMP stay in the parent sprint plan. This node should be shaped so those phases can be added later (enum + comments), but they are **not** wired now.

Concurrency (copy Sprint 2):

- Speech callback: lock, store string, return. No `aplay`, no sleep.
- Timer thread: lock, copy phase + flags, unlock, then publish / decide.
- Playback: dedicated `std::thread`; `SIGTERM` the child on stop/shutdown; join with timeout.

## Audio (both clips are mp3)

One player in the C++ node. Source of truth is **mp3**. `aplay` cannot play mp3, so on node start (or first use) convert with `ffmpeg -y -i src.mp3 -ar 44100 -ac 2 cache.wav` if the wav is missing or older than the mp3. Play the cache with worker-thread `aplay -D pulse` (USB Pulse sink is already the default: GeneralPlus). Fallback: `ffplay -nodisp -autoexit` on the mp3. Generated wavs are cache only — do not hand-edit them; do not commit them as the asset.

Do **not** use `paplay` as the design (Sprint 3 already rejected Bluetooth/`paplay`). Do **not** call `system()` on the ROS callback. Do **not** go through `voice_play.py` (3 s timeout, **wav-name-only** API).

### Spoken reply

| | |
|---|---|
| Drop path | `~/ros2_ws/src/xf_mic_asr_offline/feedback_voice/english/master_response.mp3` |
| Cache wav | same dir, `master_response.wav` (generated) |
| Exact text | **I guess you think you are my master** |
| How | Peter drops the mp3. Not a wav. Not `voice_play`. |
| Missing file | Log error, return to IDLE. **Do not dance** without the line. |

### Dance music (new vs parent sprint)

Peter will supply this mp3 later. Dance must still run if it is missing (log a warning, silent dance).

| | |
|---|---|
| Drop path | `~/ros2_ws/src/proud_up/sounds/master_dance.mp3` |
| Cache wav | `~/ros2_ws/src/proud_up/sounds/master_dance.wav` (generated) |
| Start | When `RunActionSet` is published (SPEAK has already finished) |
| Stop | `/controller/action_complete == true`, or `stop`, or `dance_timeout`, or node shutdown |
| Loop | **Yes**, until dance ends. A short clip still fills 13.5 s. A long clip is killed at dance end. Parameter `music_loop: true` |
| Missing file | Dance proceeds. Do not block the state machine |

ASR leak: the USB mic will hear the music. Mitigations:

1. After accepting the master command, `asr_node` sets `ignore_until` to now + **reply + dance** (parameter, start ~20 s), not the usual 1.6 s. `stop` still uses the short 0.8 s window.
2. `voice_control_move` ignore-list: `who is your master` (and aliases). It must **not** treat music-hallucinated “go forward” as a walk **during** this sequence — the ignore window in (1) is the real guard; the ignore-list is so the published canonical string is not “unmatched”.
3. Interaction node accepts only `stop` in SPEAK/DANCE.

## Action-complete topic (Masha quirk)

`ActionGroupController` publishes completion on the `status_pub` passed by `MoveController`:

```python
self.status_pub = self.create_publisher(Bool, 'action_complete', 1)  # node name = controller
# → actual topic /controller/action_complete
self.create_subscription(Bool, '/action_complete', ...)             # different topic
```

So `/action_complete` is **not** what the `.d6a` player emits today. The C++ node must subscribe to **`/controller/action_complete`** (parameter, that default). Timeout is the backstop if a Bool never arrives.

Do **not** name the action `dance_1` / `dance_2` / `dance_3` / `stop` — those are special-cased in `run_actionset_callback` (`perform_actions` or halt+`init_pose`). Basename is **`master_dance`**. Message type is **`interfaces/msg/RunActionSet`** (`string action_path`, `bool interrupt`). Do **not** use `kinematics_msgs/msg/RunActionSet` (different fields: `repeat`, `default_path`). Do **not** copy `controller_client.run_actionset()`, which sets those extra fields.

## Files

**This memo:** `~/steps/MASTER_DIALOG_DANCE.md`

**Add in `proud_up` (when executing):**

- `include/proud_up/interaction_phrases.hpp` — alias match, no rclcpp
- `src/masha_interaction_node.cpp` — state machine, halt, mp3 player (ffmpeg→wav + aplay)
- `launch/masha_interaction.launch.py`
- `config/masha_interaction.yaml`
- `sounds/` — drop box for `master_dance.mp3` (gitkeep; mp3 itself may come later)
- `test/test_interaction_phrases.cpp` — alias → canonical

**Edit:**

- `proud_up/CMakeLists.txt` — executable, gtest, `interfaces`, `std_msgs`
- `proud_up/package.xml` — those depends
- `xf_mic_asr_offline/scripts/asr_node.py` — `COMMAND_PHRASES` master aliases; longer `ignore_until` after that command
- `xf_mic_asr_offline/scripts/voice_control_move.py` — ignore list for the master canonical string

**Peter supplies (not code):**

- `feedback_voice/english/master_response.mp3` (spoken line; **mp3, not wav**)
- later: `proud_up/sounds/master_dance.mp3`

**Do not edit:** `app/`, `peripherals` Python, `bringup` (no auto-launch this slice), `servo_controller`, `kinematics.so`, `twist.d6a` / `init.d6a`. Do not retune `master_dance.d6a` in software.

## Parameters (`config/masha_interaction.yaml`)

- `action_name: master_dance`
- `master_response_mp3:` absolute path to the spoken mp3
- `music_mp3:` path under `proud_up/sounds/` (cache wav derived next to it)
- `music_loop: true`
- `halt_wait_s: 0.4`
- `speak_timeout_s: 8.0`
- `dance_timeout_s: 16.5`  (13.5 s file + margin)
- `action_complete_topic: /controller/action_complete`
- `asr_topic: /asr_node/voice_words`

## Learner comments

Same Sprint 2/3 rule: why `gait=-2` is the halt, why `aplay` is off the callback, why `/controller/run_actionset` is a **topic** not a service, why music is killed from `/controller/action_complete` not a sleep of 13.5 s, why the spoken line is **mp3** (ffmpeg cache, not `voice_play` wav), why `voice_play`’s 3 s timeout is unusable here. No `// increment i`.

## Live-test risk (choreography, not this code)

`master_dance.d6a` first and last **leg** rows are all pulse **500** (not `init.d6a` stand: tibias 700/300, femurs 300/700). Mid-range femur/tibia is not a stand. Floor-clear, hands ready. If she sags on the first or last row, retune the `.d6a` in the Qt editor — do not “fix” it by slowing playback. Arm 19–24 are mostly parked; later rows wiggle id 19.

## Out of scope (this slice)

- ZMP estimator / `zmp.hpp` / Foxglove polygon
- Step 2: `follow_offer` clip, yes/no, gait=5, YOLO
- Second sherpa, Bluetooth, `yahboomcar_msgs`, `paplay`
- Replacing `voice_control_move` or the existing voice “dance” → `twist`
- Auto-start from `bringup.launch.py`
- Recording/generating either mp3
- Commits/push (`~/git-push-all.sh` only with Peter’s OK)

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
masha_interaction asr=/asr_node/voice_words action=master_dance ...
```

If it warns that `master_response.mp3` is missing, the dance will not run. Music missing is OK (silent dance).

### Voice is not up (you killed it, or boot did not start it)

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

To run it again: wait ~**20 s** after the music stops (ASR ignores the speaker during the line + dance), then say the same phrase. No restart. After 120 s of silence she sleeps; the master line still wakes that sequence. Walk / “dance” (`twist.d6a`) still need **Hello Masha** if she is asleep.

**stop** during the spoken line or the dance kills music and the action group.

### Debug

```bash
tail -f /tmp/masha-asr.log
# look for: ok who is your master
ros2 topic echo /asr_node/voice_words
ros2 topic echo /controller/run_actionset --once
```

## Verification (when executing)

1. `colcon build --symlink-install --packages-select proud_up`
2. Slim stack + voice already up. `ros2 launch proud_up masha_interaction.launch.py`
3. `ros2 topic echo /asr_node/voice_words` — the query publishes `who is your master` (idle is OK; Hello is not required for this phrase)
4. USB speaker: spoken `master_response.mp3`, then dance music (or warning if dance mp3 absent)
5. `ros2 topic echo /controller/run_actionset --once` shows `master_dance`
6. `ros2 topic echo /controller/action_complete` goes false then true; music stops
7. `stop` mid-dance kills music and plays `init_pose`
8. Existing “dance” still plays `twist.d6a`
9. gtest `test_interaction_phrases`

## Execution gate

| Step | When |
|---|---|
| Write `~/steps/MASTER_DIALOG_DANCE.md` | Done |
| C++ / ASR / audio wiring | Done (`colcon build --packages-select proud_up`; gtest `test_interaction_phrases` 6/6) |
| Live floor test | Done 2026-09-14 (spoken mp3 + dance mp3 + interaction node) |
