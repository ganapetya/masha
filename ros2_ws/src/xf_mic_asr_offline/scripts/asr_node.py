#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
import os
import re
import time
import difflib
import threading
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from xf_mic_asr_offline import identity, voice_play

# Voice is sherpa-onnx on PulseAudio only. See ros2_ws/info/voice.md.


SAMPLE_RATE = 16000
# Stay in command mode this long after the last wake or command.
LISTEN_SECONDS = 120.0
# Hold a greeting across a short pause so "Hello" + "Masha" still wakes.
GREET_HOLD_SECONDS = 2.8
IDLE_CLIP_SECONDS = 2.4
ASR_DEBUG_LOG = '/tmp/masha-asr.log'
SHERPA_ROOT = os.path.expanduser('~/third_party/sherpa-onnx')
RECORD_DEVICES = (
    'pulse',
    'plughw:CARD=XFMDPV0018,DEV=0',
    'hw:CARD=XFMDPV0018,DEV=0',
)
COMMAND_PHRASES = (
    ('go forwards', 'go forward'),
    ('go forward', 'go forward'),
    ('move forward', 'go forward'),
    ('walk forward', 'go forward'),
    ('go ahead', 'go forward'),
    ('go fervent', 'go forward'),
    ('go forwent', 'go forward'),
    ('go foreword', 'go forward'),
    ('go forword', 'go forward'),
    ('go forth', 'go forward'),
    ('go ford', 'go forward'),
    ('go for', 'go forward'),
    ('go fer', 'go forward'),
    ('goforward', 'go forward'),
    ('go backwards', 'go backward'),
    ('go backward', 'go backward'),
    ('move backward', 'go backward'),
    ('go back', 'go backward'),
    ('move back', 'go backward'),
    ('turn to the left', 'turn left'),
    ('turn to the right', 'turn right'),
    ('turn left', 'turn left'),
    ('turn right', 'turn right'),
    ('move left', 'move left'),
    ('move right', 'move right'),
    ('come here', 'come here'),
    ('come over', 'come here'),
    ('跳个舞吧', 'dance'),
    ('左平移', 'move left'),
    ('右平移', 'move right'),
    ('前进', 'go forward'),
    ('后退', 'go backward'),
    ('左转', 'turn left'),
    ('右转', 'turn right'),
    ('过来', 'come here'),
    ('停下', 'stop'),
    ('dance', 'dance'),
    ('stop', 'stop'),
)

_FORWARD_HINTS = (
    'forward', 'forwards', 'fervent', 'forwent', 'foreword', 'forword',
    'fourward', 'forth', 'ahead', 'onward', 'onwards',
)
_BACKWARD_HINTS = ('backward', 'backwards', 'back')
_FORWARD_PREFIXES = ('for', 'fer', 'fwd', 'ward')
SPEECH_PEAK = 0.05
SILENCE_CHUNKS = 5   # 0.5 s of quiet ends an utterance
MIN_SPEECH_CHUNKS = 4
MAX_SPEECH_CHUNKS = 40  # 4 s cap


def _close_word(token, target, ratio=0.72):
    if not token or abs(len(token) - len(target)) > 3:
        return False
    return difflib.SequenceMatcher(None, token, target).ratio() >= ratio


def _looks_forward(token):
    if token in _FORWARD_HINTS:
        return True
    return _close_word(token, 'forward') or _close_word(token, 'forwards')


def _model_files(language):
    if language == 'Chinese':
        base = os.path.join(SHERPA_ROOT, 'sherpa-onnx-streaming-zipformer-zh-xlarge-int8-2025-06-30')
        return {
            'tokens': os.path.join(base, 'tokens.txt'),
            'encoder': os.path.join(base, 'encoder.int8.onnx'),
            'decoder': os.path.join(base, 'decoder.onnx'),
            'joiner': os.path.join(base, 'joiner.int8.onnx'),
        }
    base = os.path.join(SHERPA_ROOT, 'sherpa-onnx-streaming-zipformer-en-2023-06-21')
    return {
        'tokens': os.path.join(base, 'tokens.txt'),
        'encoder': os.path.join(base, 'encoder-epoch-99-avg-1.int8.onnx'),
        'decoder': os.path.join(base, 'decoder-epoch-99-avg-1.int8.onnx'),
        'joiner': os.path.join(base, 'joiner-epoch-99-avg-1.int8.onnx'),
    }


def extract_command(text):
    if not text:
        return ''
    low = text.lower()
    low = re.sub(r'[^a-z0-9\u4e00-\u9fff\s]', ' ', low)
    low = re.sub(r'\s+', ' ', low).strip()
    for phrase, command in COMMAND_PHRASES:
        if phrase in low:
            return command
    tokens = low.split()
    token_set = set(tokens)
    if 'dance' in token_set:
        return 'dance'
    if 'stop' in token_set:
        return 'stop'
    if token_set & set(_BACKWARD_HINTS):
        return 'go backward'
    if any(_looks_forward(tok) for tok in tokens):
        return 'go forward'
    if 'go' in token_set and any(tok.startswith(('for', 'fer')) for tok in tokens):
        return 'go forward'
    if 'left' in token_set:
        return 'turn left'
    if 'right' in token_set:
        return 'turn right'
    return ''


class ASRNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.awake_flag = False
        self.session_deadline = 0.0
        self.ignore_until = 0.0
        self.last_partial = ''
        self.last_activity = 0.0
        self.pending_greeting_until = 0.0
        self.cmd_chunks = []
        self.idle_chunks = []
        self.declare_parameter('confidence', 18)
        self.declare_parameter('seconds_per_order', 5)

        self.seconds_per_order = min(max(int(self.get_parameter('seconds_per_order').value), 4), 6)
        self.language = os.environ.get('ASR_LANGUAGE', 'English')

        self.control = self.create_publisher(String, '~/voice_words', 1)
        self.recognizer = None
        self.stream = None
        self._load_recognizer()

        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.2, self.session_timer, callback_group=timer_cb_group)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        threading.Thread(target=self.audio_loop, daemon=True).start()
        self.get_logger().info('\033[1;32mI am %s\033[0m' % identity.ROBOT_NAME)
        self.get_logger().info('\033[1;32mWake on: %s\033[0m' % identity.wake_phrase_log())
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self._debug_log('start name=%s wake=%s' % (identity.ROBOT_NAME, identity.wake_phrase_log()))

    def _load_recognizer(self):
        import sherpa_onnx

        files = _model_files(self.language)
        missing = [p for p in files.values() if not os.path.isfile(p)]
        if missing:
            raise FileNotFoundError('sherpa-onnx model files missing: %s' % missing)

        self.get_logger().info('loading sherpa-onnx %s model...' % self.language)
        self.recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=files['tokens'],
            encoder=files['encoder'],
            decoder=files['decoder'],
            joiner=files['joiner'],
            num_threads=2,
            sample_rate=SAMPLE_RATE,
            feature_dim=80,
            enable_endpoint_detection=True,
            decoding_method='greedy_search',
            provider='cpu',
        )
        self._reset_stream()
        self.get_logger().info('\033[1;32msherpa-onnx ready\033[0m')

    def _reset_stream(self):
        self.stream = self.recognizer.create_stream()
        self.last_partial = ''

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _publish_words(self, text):
        msg = String()
        msg.data = text
        self.control.publish(msg)

    def _debug_log(self, line):
        try:
            with open(ASR_DEBUG_LOG, 'a', encoding='utf-8') as fh:
                fh.write('%.3f %s\n' % (time.time(), line))
        except Exception:
            pass

    def _keep_awake(self, seconds=LISTEN_SECONDS):
        self.awake_flag = True
        self.last_activity = time.time()
        self.session_deadline = time.time() + seconds

    def _text_is_wake(self, text):
        if identity.is_wake_phrase(text):
            return True
        if identity.has_name(text) and time.time() < self.pending_greeting_until:
            return True
        return False

    def _note_partial(self, text):
        if identity.has_greeting(text):
            self.pending_greeting_until = time.time() + GREET_HOLD_SECONDS
        if identity.has_name(text) and time.time() < self.pending_greeting_until:
            return True
        return identity.is_wake_phrase(text)

    def _trigger_wake(self, raw_text):
        already = self.awake_flag
        self.get_logger().info(
            '\033[1;32mheard %s: %r\033[0m' % (identity.ROBOT_NAME, raw_text))
        self._debug_log('wake %r' % raw_text)
        self.pending_greeting_until = 0.0
        self._keep_awake()
        self.cmd_chunks = []
        self.idle_chunks = []
        self._reset_stream()
        if not already:
            self._publish_words('唤醒成功(wake-up-success)')
        voice_play.play('awake', language='English')
        self.ignore_until = time.time() + 0.5

    def _open_recorder(self):
        last_err = ''
        for device in RECORD_DEVICES:
            try:
                proc = subprocess.Popen(
                    ['arecord', '-q', '-D', device, '-f', 'S16_LE', '-c', '1',
                     '-r', str(SAMPLE_RATE), '-t', 'raw'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
                time.sleep(0.05)
                if proc.poll() is not None:
                    err = (proc.stderr.read() or b'').decode('utf-8', errors='ignore')
                    last_err = '%s: %s' % (device, err.strip())
                    continue
                self.get_logger().info('recording from %s' % device)
                return proc
            except Exception as exc:
                last_err = '%s: %s' % (device, exc)
        raise RuntimeError('arecord failed: %s' % last_err)

    def _transcribe_chunk(self, samples):
        self.stream.accept_waveform(SAMPLE_RATE, samples)
        while self.recognizer.is_ready(self.stream):
            self.recognizer.decode_stream(self.stream)
        text = (self.recognizer.get_result(self.stream) or '').strip()
        if self.language != 'Chinese':
            text = text.lower()
        endpoint = False
        try:
            endpoint = bool(self.recognizer.is_endpoint(self.stream))
        except Exception:
            endpoint = False
        return text, endpoint

    def _decode_clip(self, audio):
        stream = self.recognizer.create_stream()
        stream.accept_waveform(SAMPLE_RATE, audio)
        tail = np.zeros(int(SAMPLE_RATE * 0.4), dtype=np.float32)
        stream.accept_waveform(SAMPLE_RATE, tail)
        stream.input_finished()
        while self.recognizer.is_ready(stream):
            self.recognizer.decode_stream(stream)
        text = (self.recognizer.get_result(stream) or '').strip()
        if self.language != 'Chinese':
            text = text.lower()
        return text

    def _handle_command_audio(self, chunks):
        audio = np.concatenate(chunks)
        peak = float(np.max(np.abs(audio))) if audio.size else 0.0
        text = self._decode_clip(audio)
        cmd = extract_command(text)
        self.get_logger().info(
            'command clip peak=%.3f raw=%r cmd=%r' % (peak, text, cmd))
        self._debug_log('cmd peak=%.3f raw=%r cmd=%r' % (peak, text, cmd))
        if self._note_partial(text) or self._text_is_wake(text):
            self._trigger_wake(text)
            return
        if cmd:
            self._accept_command(cmd)

    def audio_loop(self):
        chunk_bytes = int(SAMPLE_RATE * 0.1) * 2
        proc = None
        last_log = 0.0
        last_clip = 0.0
        while rclpy.ok():
            try:
                if proc is None or proc.poll() is not None:
                    if proc is not None:
                        try:
                            proc.kill()
                        except Exception:
                            pass
                    proc = self._open_recorder()
                raw = proc.stdout.read(chunk_bytes)
                if not raw:
                    time.sleep(0.02)
                    continue
                samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                if time.time() < self.ignore_until:
                    self.cmd_chunks = []
                    self.idle_chunks = []
                    continue

                if not self.awake_flag:
                    self.cmd_chunks = []
                    self.idle_chunks.append(samples)
                    idle_need = int(IDLE_CLIP_SECONDS / 0.1)
                    if len(self.idle_chunks) > idle_need:
                        self.idle_chunks = self.idle_chunks[-idle_need:]
                    text, endpoint = self._transcribe_chunk(samples)
                    if text and text != self.last_partial:
                        self.last_partial = text
                        now = time.time()
                        if now - last_log > 0.8:
                            last_log = now
                            self.get_logger().info('idle heard: %r' % text)
                            self._debug_log('idle %r' % text)
                    if self._note_partial(text) or self._text_is_wake(text):
                        self._trigger_wake(text)
                        continue
                    # Whole-utterance clip catches "Hello"+"Masha" after a reset.
                    now = time.time()
                    greeting_hold = now < self.pending_greeting_until
                    idle_peak = float(np.max(np.abs(samples))) if samples.size else 0.0
                    if (len(self.idle_chunks) >= 12 and now - last_clip > 0.5
                            and (greeting_hold or idle_peak > 0.03 or text)):
                        last_clip = now
                        clip_text = self._decode_clip(np.concatenate(self.idle_chunks))
                        if clip_text and clip_text != self.last_partial:
                            self.get_logger().info('idle clip: %r' % clip_text)
                            self._debug_log('idle-clip %r' % clip_text)
                            last_log = now
                        if self._note_partial(clip_text) or self._text_is_wake(clip_text):
                            self._trigger_wake(clip_text)
                            continue
                    cmd = extract_command(text)
                    recently = (time.time() - self.last_activity) < LISTEN_SECONDS
                    if cmd and recently:
                        self.get_logger().info('still in session, taking %s' % cmd)
                        self._accept_command(cmd)
                        continue
                    # Keep the stream if we just heard "hello" so "Masha" can land.
                    greeting_hold = time.time() < self.pending_greeting_until
                    if endpoint and text and not greeting_hold and not self._text_is_wake(text):
                        self._reset_stream()
                    continue

                # Awake: wait for a spoken burst, then decode that utterance.
                # Fixed 5 s windows were mostly silence and late, and sherpa
                # often wrote "go fervent" for "go forward".
                peak = float(np.max(np.abs(samples))) if samples.size else 0.0
                if peak >= SPEECH_PEAK:
                    self.cmd_chunks.append(samples)
                    if len(self.cmd_chunks) >= MAX_SPEECH_CHUNKS:
                        self._handle_command_audio(self.cmd_chunks)
                        self.cmd_chunks = []
                    continue
                if not self.cmd_chunks:
                    continue
                self.cmd_chunks.append(samples)
                trailing_quiet = 0
                for chunk in reversed(self.cmd_chunks):
                    chunk_peak = float(np.max(np.abs(chunk))) if chunk.size else 0.0
                    if chunk_peak < SPEECH_PEAK:
                        trailing_quiet += 1
                    else:
                        break
                spoke = len(self.cmd_chunks) - trailing_quiet
                if trailing_quiet >= SILENCE_CHUNKS and spoke >= MIN_SPEECH_CHUNKS:
                    self._handle_command_audio(self.cmd_chunks)
                    self.cmd_chunks = []
                elif len(self.cmd_chunks) >= MAX_SPEECH_CHUNKS:
                    self._handle_command_audio(self.cmd_chunks)
                    self.cmd_chunks = []
            except Exception as exc:
                self.get_logger().error('audio loop: %s' % exc)
                time.sleep(0.3)
                proc = None

    def _accept_command(self, cmd):
        self._keep_awake()
        self._publish_words(cmd)
        self.get_logger().info('\033[1;32mok %s (still listening)\033[0m' % cmd)
        # Stop finishes quickly; other commands talk over the prompt wav.
        self.ignore_until = time.time() + (0.8 if cmd == 'stop' else 1.6)
        self.cmd_chunks = []
        self._reset_stream()

    def session_timer(self):
        if self.awake_flag and time.time() > self.session_deadline:
            self.awake_flag = False
            self.get_logger().info('sleep (say Hello Masha to wake)')
            self._reset_stream()


def main():
    node = ASRNode('asr_node')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
