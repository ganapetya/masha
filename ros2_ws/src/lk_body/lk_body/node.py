"""ROS node. This is the only module that imports rclpy."""

import asyncio
import base64
import json
import random
import logging
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from interfaces.msg import RunActionSet
from rclpy.node import Node
from std_msgs.msg import Bool

from lk_body.envfile import load
from lk_body.listen import HoldMic
from lk_body.playback import PcmPlayer
from lk_body.poses import jaw_while, play_once

MIMICS = Path("/home/ubuntu/ros2_ws/src/lk_body/config/mimics.json")
ACTIONS = Path("/home/ubuntu/software/actionset_editor/ActionGroups")
MUTE_S = 0.3
log = logging.getLogger("lk_body")


class LkBody(Node):
    def __init__(self):
        super().__init__("lk_body")
        self._complete = True
        self._complete_lock = threading.Lock()
        self.create_subscription(Bool, "/action_complete", self._on_complete, 10)
        self._actions = self.create_publisher(RunActionSet, "/controller/run_actionset", 10)
        self._mic = HoldMic()
        self._player = PcmPlayer()
        self._queue: queue.Queue = queue.Queue()
        self._names: queue.Queue = queue.Queue()
        self._stop_line = threading.Event()
        self._on_played = None
        self.create_timer(0.05, self._flush_actions)
        self._muted_until = 0.0
        self._discard = False
        self._mimics = json.loads(MIMICS.read_text(encoding="utf-8"))
        self._speaking = self._mimics["speaking"]["action"]
        self._involuntary_on = bool(self._mimics.get("involuntary_animocity", True))
        self._involuntary = [
            name for name in (self._mimics.get("involuntary") or [])
            if isinstance(name, str) and (ACTIONS / f"{name}.d6a").is_file()
        ]
        self._worker = threading.Thread(target=self._play_loop, daemon=True)
        self._worker.start()

    def _on_complete(self, msg: Bool) -> None:
        with self._complete_lock:
            self._complete = bool(msg.data)

    def _idle(self) -> bool:
        with self._complete_lock:
            return self._complete

    def publish_action(self, name: str) -> None:
        self._names.put(name)

    def _flush_actions(self) -> None:
        while True:
            try:
                name = self._names.get_nowait()
            except queue.Empty:
                return
            msg = RunActionSet()
            msg.action_path = name
            msg.interrupt = False
            self._actions.publish(msg)

    def startup_status(self) -> tuple:
        if not _running("move_controller"):
            return False, "controller_down"
        if shutil.which("ffmpeg") is None or shutil.which("aplay") is None or not _pulse():
            return False, "speaker_missing"
        jaw = ACTIONS / f"{self._speaking}.d6a"
        if not jaw.is_file():
            return False, f"file_missing {self._speaking}"
        return True, ""

    def on_down(self) -> str:
        if self._player.playing() or time.monotonic() < self._muted_until:
            self._discard = True
            return ""
        self._discard = False
        if not self._mic.start():
            return "mic_busy"
        return ""

    def on_up(self) -> bytes:
        wav = self._mic.stop()
        drop = self._discard or self._player.playing() or time.monotonic() < self._muted_until
        self._discard = False
        if drop:
            return b""
        return wav

    def enqueue(self, pcm: bytes, actions: list | None = None, language: str = "") -> None:
        names = []
        for name in actions or []:
            if not isinstance(name, str):
                continue
            base = Path(name).name
            if base.endswith(".d6a"):
                base = base[:-4]
            if not base or base == self._speaking or base in names:
                continue
            if not (ACTIONS / f"{base}.d6a").is_file():
                log.info("file_missing %s", base)
                continue
            names.append(base)
        self._stop_line.set()
        self._player.stop()
        self._queue.put((pcm, names, language or ""))

    def _play_loop(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            pcm, names, language = item
            self._stop_line.clear()
            self._player.play(pcm, _speech_volume(language))

            def still() -> bool:
                return self._player.playing() and not self._stop_line.is_set()

            jaw_while(still, self.publish_action, self._idle, self._speaking)
            self._player.stop()
            if pcm and not self._stop_line.is_set():
                self._note_played()
            self._muted_until = time.monotonic() + MUTE_S
            if not names and self._involuntary_on and self._involuntary:
                names = [random.choice(self._involuntary)]
            for name in names:
                if self._stop_line.is_set():
                    break
                play_once(self.publish_action, self._idle, name)

    def _note_played(self) -> None:
        notify = self._on_played
        if notify is not None:
            notify()

    def shutdown(self) -> None:
        self._stop_line.set()
        self._player.stop()
        self._mic.stop()
        self._queue.put(None)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="lk_body %(message)s")
    env = load()
    url = env.get("MASHA_HUB_WS", "")
    token = env.get("MASHA_ATTACH_TOKEN", "")
    if not url or len(token) < 32:
        log.error("dial url or attach token is missing")
        raise SystemExit(1)
    rclpy.init()
    node = LkBody()
    from lk_body.session import run as run_session

    def _registered() -> None:
        ready_pose = node._mimics.get("ready") or {}
        ready_name = ready_pose.get("action") if isinstance(ready_pose, dict) else ""
        if isinstance(ready_name, str) and (ACTIONS / f"{ready_name}.d6a").is_file():
            node.publish_action(ready_name)

    async def _on_speak(down: bool, socket) -> None:
        await _speak(node, socket, down)

    thread = threading.Thread(
        target=lambda: asyncio.run(
            run_session(
                url,
                token,
                "masha",
                _register(node),
                node.startup_status,
                node.enqueue,
                _on_speak,
                lambda callback: setattr(node, "_on_played", callback),
                rclpy.ok,
                _registered,
            )
        ),
        daemon=True,
    )
    thread.start()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def _register(node: LkBody) -> dict:
    speaking = node._mimics["speaking"]
    return {
        "type": "register",
        "speaking": {"action": speaking["action"], "intent": speaking["intent"]},
        "stances": node._mimics["stances"],
    }


async def _speak(node: LkBody, socket, down: bool) -> None:
    if down:
        reason = node.on_down()
        if reason:
            await socket.send(json.dumps({"type": "status", "ready": False, "reason": reason}))
        return
    # The finger lifts on the last word. Keep the mic for one second so that word is in the wav.
    await asyncio.sleep(1.0)
    wav = node.on_up()
    if not wav:
        await socket.send(json.dumps({"type": "audio", "data": "", "mime": "audio/wav"}))
        return
    await socket.send(
        json.dumps(
            {
                "type": "audio",
                "data": base64.b64encode(wav).decode("ascii"),
                "mime": "audio/wav",
            }
        )
    )



def _speech_volume(language: str) -> int:
    """Pulse stream volume. 65536 is the level she had. English quieter, Norsk louder."""
    gain = {"en": 0.8, "nb": 1.3}.get(language, 1.0)
    return int(65536 * gain)


def _running(mark: str) -> bool:
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if mark in raw:
            return True
    return False


def _pulse() -> bool:
    try:
        result = subprocess.run(["pactl", "info"], capture_output=True, timeout=2)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and b"Default Sink:" in result.stdout


if __name__ == "__main__":
    main()
