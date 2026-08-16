#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os
import threading
import subprocess

wav_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline/feedback_voice'
# pulse first: after reboot plughw on the USB speaker can stall in the kernel
# and ignore SIGKILL, which used to freeze the listen thread.
PLAY_DEVICES = (
    'pulse',
    'default',
    'plughw:CARD=Device,DEV=0',
)
_PLAY_LOCK = threading.Lock()


def get_path(f, language='Chinese'):
    if language == 'Chinese':
        return os.path.join(wav_path, f + '.wav')
    return os.path.join(wav_path, 'english', f + '.wav')


def play(voice, volume=80, language='Chinese'):
    path = get_path(voice, language)
    if not os.path.isfile(path):
        print('missing wav', path)
        return
    with _PLAY_LOCK:
        for device in PLAY_DEVICES:
            try:
                result = subprocess.run(
                    ['aplay', '-q', '-D', device, path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=3,
                    check=False,
                )
                if result.returncode == 0:
                    return
            except Exception:
                continue
        try:
            from speech import speech
            speech.set_volume(volume)
            speech.play_audio(path)
        except BaseException as e:
            print('error', e)


def play_async(voice, volume=80, language='Chinese'):
    threading.Thread(
        target=play,
        args=(voice, volume, language),
        daemon=True,
        name='masha-play',
    ).start()


if __name__ == '__main__':
    play('ok')
    play('running', language='English')
    play('running')
