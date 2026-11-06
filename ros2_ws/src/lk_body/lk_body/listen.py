"""Hold-to-talk capture. A pause while the button is down stays in the one wav."""

import io
import os
import signal
import subprocess
import threading
import wave

RATE = 16000


class HoldMic:
    def __init__(self, source="default"):
        self._source = source
        self._proc = None
        self._reader = None
        self._chunks = []
        self._lock = threading.Lock()

    def start(self) -> bool:
        with self._lock:
            if self._proc is not None:
                return True
            try:
                proc = subprocess.Popen(
                    [
                        "ffmpeg",
                        "-hide_banner",
                        "-loglevel",
                        "error",
                        "-f",
                        "pulse",
                        "-i",
                        self._source,
                        "-ac",
                        "1",
                        "-ar",
                        str(RATE),
                        "-f",
                        "s16le",
                        "pipe:1",
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except OSError:
                return False
            self._proc = proc
            self._chunks = []
            self._reader = threading.Thread(target=self._read, args=(proc,), daemon=True)
            self._reader.start()
            return proc.poll() is None

    def _read(self, proc) -> None:
        stream = proc.stdout
        if stream is None:
            return
        while True:
            block = stream.read(4096)
            if not block:
                break
            self._chunks.append(block)

    def stop(self) -> bytes:
        with self._lock:
            proc = self._proc
            reader = self._reader
            self._proc = None
            self._reader = None
        if proc is None:
            return b""
        _stop_group(proc)
        if reader is not None:
            reader.join(timeout=2)
        pcm = b"".join(self._chunks)
        self._chunks = []
        return _wav(pcm)


def _stop_group(proc) -> None:
    if proc.poll() is not None:
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


def _wav(pcm: bytes) -> bytes:
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(RATE)
        handle.writeframes(pcm)
    return buffer.getvalue()
