"""Play 22050 Hz mono PCM on Pulse. ffmpeg and aplay share one process group."""

import os
import signal
import subprocess
import threading


class PcmPlayer:
    def __init__(self):
        self._proc = None
        self._lock = threading.Lock()

    def playing(self) -> bool:
        with self._lock:
            proc = self._proc
        return proc is not None and proc.poll() is None

    def play(self, pcm: bytes) -> None:
        self.stop()
        proc = subprocess.Popen(
            "ffmpeg -hide_banner -loglevel error -f s16le -ar 22050 -ac 1 -i pipe:0 -f wav - | aplay -q -D pulse",
            shell=True,
            stdin=subprocess.PIPE,
            start_new_session=True,
        )
        if proc.stdin is not None:
            try:
                proc.stdin.write(pcm)
                proc.stdin.close()
            except BrokenPipeError:
                pass
        with self._lock:
            self._proc = proc

    def stop(self) -> None:
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except OSError:
            proc.terminate()
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                proc.kill()
