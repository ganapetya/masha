# Masha voice — architecture

This is the only voice design. Read this before touching wake or commands.

## Rule

Speech-to-text is **sherpa-onnx**. There is no second recognizer, no serial keyword engine, and no vendor SDK on this path. Do not add one back.

## What she does

1. Listen for **Hello Masha** / **Hi Masha** / **Shalom Masha**.
2. Play **I’m here**.
3. For 120 seconds, listen for: `go forward`, `go backward`, `turn left`, `turn right`, `stop`, `dance`, `come here`.
4. Walk. Stay in command mode another 120 seconds after each accepted command.
5. After 120 seconds of silence, sleep. Say the name again.

## The line

```
USB mic (PulseAudio default source)
    → asr_node  (sherpa-onnx)
         → /asr_node/voice_words
              → voice_control_move
                   → hexapod gait
```

Two ROS nodes. Nothing else.

| Piece | Role |
|---|---|
| PulseAudio default source | The microphone. Record with `pulse`. |
| PulseAudio default sink | The speaker. Play wavs with `aplay`. |
| `asr_node.py` | Wake + commands. One recognizer. Plays **I’m here** on a side thread so listen never blocks. Session timeout is its own thread, not a ROS timer. |
| `identity.py` | Name and greeting spellings. |
| `voice_control_move.py` | Canonical phrase → walk / turn / stop / dance. |
| `feedback_voice/english/*.wav` | Spoken replies. |

Launch: `ros2 launch xf_mic_asr_offline startup_test.launch.py`  
Bringup starts that launch when `BRINGUP_VOICE=true` and the USB mic is present.

Python nodes are symlink-installed. Edit, restart the launch, no colcon.

## How to run

```bash
source ~/.zshrc
ros2 launch xf_mic_asr_offline startup_test.launch.py
```

Healthy start: `I am Masha`, `Wake on: Hello Masha / Hi Masha / Shalom Masha`, `recording from pulse`, `I am ready`.

Debug file: `/tmp/masha-asr.log` (`idle`, `wake`, `cmd peak= raw= cmd=`).

## Do not

- Launch a second copy of the voice stack (it steals the mic).
- Record with a raw ALSA device while PulseAudio owns it.
- Add another ASR, a serial wake node, or a keyword-flash service.
- Speak the command over **I’m here** (she ignores the speaker for ~2 s).

Wait for **I’m here**, then say the command. After reboot, a stuck `aplay` must not sit on the listen thread.
