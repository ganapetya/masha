"""Enter and leave her talk mode. Bringup stays up."""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

STARTUP_MARK = "startup_test.launch.py"
BRINGUP_MARK = "bringup.launch.py"
CONTROLLER_MARK = "move_controller"
BODY_MARK = "lk_body_node"
PID_PATH = Path("/tmp/lk_body.pid")
ROS = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/ubuntu/ros2_ws/install/setup.bash && "
    "export ROS_DOMAIN_ID=27 && "
)


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args == ["enter"]:
        return enter()
    if args == ["leave"]:
        return leave()
    print("usage: lk_bot_mode enter|leave", file=sys.stderr)
    return 2


def enter() -> int:
    stop_startup()
    if not controller_up():
        print("controller is down", file=sys.stderr)
        return 1
    if body_running():
        return 0
    proc = subprocess.Popen(
        ["bash", "-lc", ROS + "exec ros2 run lk_body lk_body_node"],
        start_new_session=True,
    )
    PID_PATH.write_text(str(proc.pid), encoding="utf-8")
    time.sleep(0.5)
    if proc.poll() is not None:
        print("lk_body did not stay up", file=sys.stderr)
        return 1
    return 0


def leave() -> int:
    stop_body()
    start_startup()
    return 0


def controller_up() -> bool:
    return any(CONTROLLER_MARK in cmd for _pid, cmd in _commands())


def body_running() -> bool:
    return any(BODY_MARK in cmd for _pid, cmd in _commands())


def stop_startup() -> None:
    _signal_marked(STARTUP_MARK, protect=BRINGUP_MARK)


def stop_body() -> None:
    _signal_marked(BODY_MARK, protect=BRINGUP_MARK)
    if PID_PATH.is_file():
        try:
            pid = int(PID_PATH.read_text(encoding="utf-8").strip())
            os.killpg(pid, signal.SIGTERM)
        except (OSError, ValueError):
            pass
        PID_PATH.unlink(missing_ok=True)


def start_startup() -> None:
    if any(STARTUP_MARK in cmd for _pid, cmd in _commands()):
        return
    subprocess.Popen(
        ["bash", "-lc", ROS + "exec ros2 launch xf_mic_asr_offline startup_test.launch.py"],
        start_new_session=True,
    )


def _signal_marked(mark: str, protect: str) -> None:
    for pid, cmd in _commands():
        if mark not in cmd:
            continue
        try:
            pgid = os.getpgid(pid)
        except OSError:
            continue
        group = [text for opid, text in _commands() if _pgid(opid) == pgid]
        if any(protect in text for text in group):
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
            continue
        try:
            os.killpg(pgid, signal.SIGTERM)
        except OSError:
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass


def _pgid(pid: int):
    try:
        return os.getpgid(pid)
    except OSError:
        return None


def _commands():
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        yield int(entry.name), raw


if __name__ == "__main__":
    raise SystemExit(main())
