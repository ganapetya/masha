"""Play 22050 Hz mono PCM on Pulse. pacat is one process group."""

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

    def play(self, pcm: bytes, volume: int = 65536) -> None:
        self.stop()
        proc = subprocess.Popen(
            [
                "pacat",
                "--playback",
                "--rate",
                "22050",
                "--format",
                "s16le",
                "--channels",
                "1",
                "--volume",
                str(int(volume)),
                "--latency-msec",
                "50",
                "--client-name",
                "lk_body",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        with self._lock:
            self._proc = proc

        def feed() -> None:
            try:
                if proc.stdin is None:
                    return
                view = memoryview(pcm)
                step = 4096
                for start in range(0, len(view), step):
                    proc.stdin.write(view[start : start + step])
                    proc.stdin.flush()
                proc.stdin.close()
            except (BrokenPipeError, ValueError, OSError):
                pass

        threading.Thread(target=feed, daemon=True).start()

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
