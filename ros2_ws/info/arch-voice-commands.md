# Masha voice commands — architecture

How speech becomes motion. Read this to follow the process from the microphone to the legs, including which process and thread does each step.

Related: `info/voice.md` (rules and how to run), `info/startup.md` (boot).

---

## 1. What she does

1. Sleeps and listens for **Hello Masha** / **Hi Masha** / **Shalom Masha**.
2. Plays **I’m here** (`awake.wav`).
3. For **120 seconds**, listens for motion phrases.
4. Executes the phrase (walk, turn, stop, dance).
5. Each accepted command restarts the 120-second window.
6. After 120 seconds with no accepted command, she sleeps. Say the name again.

Speech-to-text is **sherpa-onnx only**. There is no iFlytek IAT on this path, no serial keyword engine, and no second recognizer.

---

## 2. The line (hardware → legs)

```
USB 6-mic  (iFlytek XFM-DP, PulseAudio default source)
    │  arecord -D pulse, 16 kHz, mono S16
    ▼
asr_node  (process)
    │  sherpa-onnx transcribes
    │  identity.py matches wake
    │  extract_command() maps text → canonical phrase
    │  publishes std_msgs/String
    ▼
/asr_node/voice_words
    ▼
voice_control_move  (process)
    │  match_command()
    │  play_async() spoken reply
    │  ControllerClient.traveling()  or  ActionGroupController.run_action()
    ▼
/controller/traveling     /controller/cmd_vel     /servo_controller
    ▼
move_controller / step_controller / servo_controller
    ▼
STM32  →  bus servos  →  legs
```

Two ROS nodes. Nothing else in the voice stack.

Playback is a side path: `voice_play.play_async()` → `aplay` on the PulseAudio **default sink** (GeneralPlus USB speaker).

---

## 3. How the stack is started

Boot: `start_app_node.service` → `ros2 launch bringup bringup.launch.py`.

If `BRINGUP_VOICE=true` (or `voice:=true`) **and** `/dev/ring_mic` exists, `startup_check` starts a **non-daemon** thread that blocks on:

```text
ros2 launch xf_mic_asr_offline startup_test.launch.py
```

That launch includes:

| Launch file | Starts |
|---|---|
| `startup_test.launch.py` | `mic_init.launch.py` + `voice_control_move.py` |
| `mic_init.launch.py` | `asr_node.py` only |

So after boot you have **two Python processes**:

| Process | Executable | ROS node name(s) |
|---|---|---|
| ASR | `…/xf_mic_asr_offline/asr_node.py` | `/asr_node` |
| Move | `…/xf_mic_asr_offline/voice_control_move.py` | `/voice_control_move` and a nested `/controller_client` |

Python files are **symlink-installed**. Edit under `src/`, restart the launch, no colcon.

Do not start a second copy. A second `arecord` steals the mic.

---

## 4. Who listens, who executes

### 4.1 Listen (ASR)

`asr_node` is the only listener.

| It listens to | How | It does **not** |
|---|---|---|
| PulseAudio default source (USB mic) | child process `arecord -D pulse` | open `/dev/ring_mic` or a raw ALSA capture while Pulse owns the mic |
| Its own sherpa stream | 100 ms PCM chunks | call the motion stack |
| Wake phrases | `identity.is_wake_phrase` / greeting+name hold | play wavs on this thread |
| Motion phrases | `extract_command()` | walk |

Output is one topic: **`/asr_node/voice_words`** (`std_msgs/msg/String`).

| Published string | Meaning |
|---|---|
| `唤醒成功(wake-up-success)` | First wake of this session. Move node ignores it (special word). |
| `go forward` / `go backward` / `turn left` / `turn right` / `move left` / `move right` / `come here` / `dance` / `stop` | Canonical command. Move node executes it. |

`asr_node` also offers `/asr_node/init_finish` (`std_srvs/Trigger`) so the move node can wait until ASR is up.

### 4.2 Execute (move)

`voice_control_move` never opens the microphone.

| It listens to | How | It does |
|---|---|---|
| `/asr_node/voice_words` | ROS subscription → `words_callback` | stash the string in `self.words` |
| `/asr_node/init_finish` | client, once at startup | wait, then play `running.wav` |

| Canonical phrase | Motion | Also publishes |
|---|---|---|
| `go forward` / `come here` | `traveling(gait=2, stride=40, direction=0, steps=6)` | `cmd_vel.linear.x = +0.12` |
| `go backward` | same, `direction=180` | `cmd_vel.linear.x = -0.12` |
| `turn left` | `traveling(stride=0, rotation=+18, steps=6)` | `cmd_vel.angular.z = +0.4` |
| `turn right` | `rotation=-18` | `cmd_vel.angular.z = -0.4` |
| `move left` / `move right` | `direction=90` / `270` | `cmd_vel.linear.y` |
| `dance` | `ActionGroupController.run_action('twist')` | — |
| `stop` | `traveling(gait=-2)` halt | zero `Twist` |
| special words | nothing | — |

`ControllerClient.traveling()` publishes `kinematics_msgs/Traveling` on **`/controller/traveling`**.  
`/controller` (`move_controller`) + `/step_controller` turn that into gait. Servos go out through `/servo_controller`.

About five seconds after a walk/turn starts, the move loop sends `traveling(gait=-2)` to halt unless another command arrived. Dance is an action group, not that timed halt.

---

## 5. Thread map

This is the part that used to break after reboot: **playback must never sit on the listen thread**, and **sleep must not depend on the ROS executor**.

### 5.1 `asr_node` process

```
asr_node.py
├─ main thread          MultiThreadedExecutor.spin()
│                         ROS: /asr_node/init_finish only
│                         Does not record. Does not play. Does not time the session.
│
├─ thread masha-audio   audio_loop()          ← THE LISTENER
│                         owns arecord stdout
│                         sherpa decode (stream + clip)
│                         wake / command decision
│                         publish /asr_node/voice_words
│                         MUST NOT call blocking aplay
│
├─ thread masha-session session_loop()        ← THE CLOCK
│                         every 0.2 s: if now > session_deadline → sleep
│                         sets awake_flag = False, need_stream_reset = True
│                         does not touch sherpa (audio thread resets the stream)
│
└─ thread masha-play    voice_play.play()     ← spawned per clip, daemon
                          aplay -D pulse (then default, then plughw)
                          3 s timeout, lock so two wavs do not overlap
```

`arecord` is a **child OS process**, not a Python thread. `masha-audio` reads its stdout in 100 ms frames.

Sherpa itself uses `num_threads=2` inside the recognizer. Those are ONNX Runtime worker threads, not ROS.

Shared flags (set/read across threads; CPython assignment is atomic):

| Flag | Set by | Read by | Meaning |
|---|---|---|---|
| `awake_flag` | audio (`_keep_awake`), session (`False`) | audio, session | command mode |
| `session_deadline` | audio (`now + 120`) | session | when to sleep |
| `ignore_until` | audio (wake +2.2 s, command +0.8/1.6 s) | audio | drop PCM (speaker echo) |
| `need_stream_reset` | session | audio | reset sherpa on the audio thread |
| `last_activity` | audio | audio | still-in-session fallback |

### 5.2 `voice_control_move` process

```
voice_control_move.py
├─ main thread          rclpy.spin()
│                         words_callback() on /asr_node/voice_words
│                         only stores self.words (and optional sleep buzzer)
│                         must stay unblocked so callbacks keep arriving
│
├─ daemon thread        main()                ← THE EXECUTOR
│                         poll self.words
│                         play_async(reply wav)
│                         traveling() / run_action() / cmd_vel
│                         after ~5 s, auto-halt if still in a walk
│
├─ thread masha-play    voice_play.play()     ← spawned per reply
│
└─ nested node          ControllerClient
                          publishers only (traveling, cmd_vel, …)
                          created in __init__; not added to the executor
```

`play()` is `play_async`. The executor thread publishes motion **immediately**. It does not wait for the speaker.

### 5.3 Why the split exists

After reboot, `aplay -D plughw:CARD=Device` on the USB speaker can stall in the kernel and ignore `SIGKILL`. If that call ran inside `audio_loop`:

- the mic pipe was no longer being read
- commands spoken after **I’m here** never reached sherpa
- the old ROS timer could still flip her to sleep
- later speech showed up as `idle 'dance'` and was dropped

Now: listen thread never calls `aplay`. Session timeout is a dedicated Python thread. Playback prefers **pulse** (the default sink is already the USB speaker) with a 3 s timeout.

---

## 6. ASR state machine

```
                    start
                      │
                      ▼
                 ┌─────────┐
        wake     │  IDLE   │◄──────── session_loop: deadline passed
     ───────────►│  sleep  │
                 └────┬────┘
                      │ Hello / Hi / Shalom + Masha
                      │ (or name within 2.8 s of a greeting)
                      ▼
                 ┌─────────┐
     command     │  AWAKE  │◄──── each accepted command
     ───────────►│  120 s  │      restarts the deadline
                 └────┬────┘
                      │
                      └── ignore_until (2.2 s after wake, 0.8–1.6 s after command)
                          audio is read and discarded (don’t hear the speaker)
```

**Idle.** Always streaming-decode. Wake if the text is a greeting+name (including sherpa spellings like `hello masam`, `hello marsha`). A 2.4 s rolling clip is decoded as well so “Hello” + pause + “Masha” still wakes (`GREET_HOLD_SECONDS = 2.8`).

**Awake.** Same streaming decode as idle. If `extract_command()` returns a phrase, accept it. A 2.4 s clip and a short energy burst (VAD) are backups, not the only path.

**Still in session.** If `awake_flag` was just cleared but `last_activity` is still inside 120 s, a recognized command is still taken (`still in session, taking …`). After that window, commands in idle are ignored until the next wake.

**Ignore window.** After wake (~2.2 s) and after a command (0.8 s for stop, 1.6 s otherwise) PCM is thrown away so **I’m here** / **go** do not come back in as text.

---

## 7. One command, step by step

Example: “Hello Masha” … wait … “go forward”.

| t | Thread / process | What happens |
|---|---|---|
| 0 | `masha-audio` | `arecord` chunk → sherpa → `hello masam` (or similar) |
| 0 | `masha-audio` | `identity` says wake. `_keep_awake()`. Publish `唤醒成功(wake-up-success)`. `play_async('awake')`. Set `ignore_until = now+2.2`. Reset sherpa stream. |
| 0 | `masha-play` | `aplay` **I’m here** (~1.7 s). Listen thread is already back in the read loop. |
| 0 | move `spin` | `words_callback`: special word, stash only. |
| 0 | move `main()` | sees special word, `continue` — no motion. |
| 0–2.2 s | `masha-audio` | reads mic, discards samples (speaker echo). |
| 3 s | you | say **go forward** |
| 3 s | `masha-audio` | stream text contains `forward` / `go fervent` / … → `extract_command` → `go forward` |
| 3 s | `masha-audio` | `_accept_command`: publish `go forward`, restart 120 s, ignore 1.6 s, reset stream. Log `ok go forward`. |
| 3 s | move `spin` | `words_callback` → `self.words = 'go forward'` |
| 3 s | move `main()` | `play_async('go')` **and immediately** `traveling(… direction=0, steps=6)` + `cmd_vel` |
| 3 s | `/controller` | hexapod walks ~6 steps |
| ~8 s | move `main()` | `time_stamp` expired → `traveling(gait=-2)` halt |
| ≤123 s | `masha-session` | if no new command, `awake_flag = False`, log `sleep` |

If you speak during **I’m here**, that audio is in the ignore window and is dropped. Wait for the clip, then command.

---

## 8. Phrase mapping

Two independent tables. ASR decides what to publish; move decides what to do.

**ASR (`extract_command`)** — substring first, then token hints (`forward` / `back` / `left` / `right` / `dance` / `stop`), including common sherpa misses (`go fervent` → `go forward`, `dan's` → `dance`).

**Move (`match_command`)** — alias list to the same canonical set. Unknown strings are logged `unmatched command` and do not move her.

Wake matching lives only in `identity.py` (`WAKE_PHRASES`, aliases, greeting token + name token). Commands never wake her by themselves.

---

## 9. Audio devices

| Role | Device | Who opens it |
|---|---|---|
| Mic | Pulse default source `alsa_input.usb-iflytek_XFM-DP-…` | `arecord -D pulse` from `masha-audio` |
| Speaker | Pulse default sink `alsa_output.usb-GeneralPlus_USB_Audio_Device-…` | `aplay -D pulse` from a `masha-play` thread |

Fallback record devices: `plughw:CARD=XFMDPV0018`, then `hw:`. Fallback play: Pulse `default`, then `plughw:CARD=Device`. Prefer Pulse after reboot.

Wav files: `src/xf_mic_asr_offline/feedback_voice/english/*.wav` (`awake`, `running`, `go`, `back`, `turn_left`, `turn_right`, `move_left`, `move_right`, `come`, `dance`, `stop`).

---

## 10. File map

| Path | Role |
|---|---|
| `xf_mic_asr_offline/scripts/asr_node.py` | Listener, sherpa, wake, command publish |
| `xf_mic_asr_offline/scripts/voice_control_move.py` | Subscriber, gait / dance |
| `xf_mic_asr_offline/xf_mic_asr_offline/identity.py` | Name and wake spellings |
| `xf_mic_asr_offline/xf_mic_asr_offline/voice_play.py` | `play` / `play_async` |
| `xf_mic_asr_offline/launch/startup_test.launch.py` | Voice launch |
| `xf_mic_asr_offline/launch/mic_init.launch.py` | Starts `asr_node` |
| `driver/controller/controller/controller_client.py` | Publishes `Traveling` / `cmd_vel` |
| `~/software/actionset_editor/ActionGroups/twist.d6a` | Dance |
| `~/third_party/sherpa-onnx/sherpa-onnx-streaming-zipformer-en-2023-06-21/` | English model |

Language comes from `ASR_LANGUAGE` in `ros2_ws/.typerc` (English on this robot).

---

## 11. Debug

| Place | What you see |
|---|---|
| `/tmp/masha-asr.log` | `start`, `idle` / `awake` text, `idle-clip` / `awake-clip`, `wake`, `cmd peak= raw= cmd=`, `sleep` |
| `asr_node` ROS log | `heard Masha: …`, `ok go forward (still listening)`, `sleep (say Hello Masha to wake)` |
| `voice_control_move` ROS log | `words:…`, `unmatched command`, `I am ready` |

Healthy boot line: `I am Masha`, `Wake on: Hello Masha / Hi Masha / Shalom Masha`, `recording from pulse`, `I am ready`.

A command that was heard but not done: look for `ok …` in ASR (published) vs `words:…` in move (received). If ASR stays `idle 'dance'` she was asleep. If you see `ok dance` and `words:dance` but no motion, the fault is the controller / servos, not voice.

---

## 12. Do not

- Put blocking `aplay` / `play()` back on `audio_loop` or on `rclpy.spin`.
- Move `session_loop` back onto a ROS timer (a blocked executor would stop sleep; more importantly, sleep must not share a thread with decode/play).
- Launch a second voice stack.
- Record with raw ALSA while Pulse owns the mic.
- Add another ASR, a serial wake node, or a keyword-flash service.
- Speak the command over **I’m here**.
