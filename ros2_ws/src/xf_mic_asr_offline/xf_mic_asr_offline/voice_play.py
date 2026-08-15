#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os
import subprocess

wav_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline/feedback_voice'
PLAY_DEVICES = (
    'plughw:CARD=Device,DEV=0',
    'pulse',
    'default',
)


def get_path(f, language='Chinese'):
    if language == 'Chinese':
        return os.path.join(wav_path, f + '.wav')
    return os.path.join(wav_path, 'english', f + '.wav')


def play(voice, volume=80, language='Chinese'):
    path = get_path(voice, language)
    if not os.path.isfile(path):
        print('missing wav', path)
        return
    for device in PLAY_DEVICES:
        try:
            result = subprocess.run(
                ['aplay', '-q', '-D', device, path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=8,
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


if __name__ == '__main__':
    play('ok')
    play('running', language='English')
    play('running')
