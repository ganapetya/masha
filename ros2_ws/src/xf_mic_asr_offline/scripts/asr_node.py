#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
import os
import re
import time
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from xf_mic_asr_offline import voice_play

# iFlytek MSC trial is expired (11212). Use sherpa-onnx on the 6-mic USB
# stream, one fixed-length capture after the wake beep.


SAMPLE_RATE = 16000
SHERPA_ROOT = os.path.expanduser('~/third_party/sherpa-onnx')
RECORD_DEVICES = (
    'plughw:CARD=XFMDPV0018,DEV=0',
    'hw:CARD=XFMDPV0018,DEV=0',
)
COMMAND_PHRASES = (
    ('go forwards', 'go forward'),
    ('go forward', 'go forward'),
    ('move forward', 'go forward'),
    ('walk forward', 'go forward'),
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
    tokens = set(low.split())
    if 'dance' in tokens:
        return 'dance'
    if 'stop' in tokens:
        return 'stop'
    if 'forward' in tokens:
        return 'go forward'
    if 'backward' in tokens or 'backwards' in tokens:
        return 'go backward'
    return ''


class ASRNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.awake_flag = False
        self.busy = False
        self.first_listen = True
        self.serial_text = ''
        self.session_deadline = 0.0
        self.declare_parameter('confidence', 18)
        self.declare_parameter('seconds_per_order', 4)

        self.seconds_per_order = max(int(self.get_parameter('seconds_per_order').value), 3)
        self.language = os.environ.get('ASR_LANGUAGE', 'English')

        self.control = self.create_publisher(String, '~/voice_words', 1)
        self.recognizer = None
        self._load_recognizer()

        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Bool, '/awake_node/awake_flag', self.awake_flag_callback, 1)
        self.create_subscription(String, '/awake_node/iat', self.iat_callback, 10)

        self.create_client(Trigger, '/awake_node/init_finish').wait_for_service()

        self.create_timer(0.1, self.main, callback_group=timer_cb_group)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

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
            enable_endpoint_detection=False,
            decoding_method='greedy_search',
            provider='cpu',
        )
        self.get_logger().info('\033[1;32msherpa-onnx ready\033[0m')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _publish_words(self, text):
        msg = String()
        msg.data = text
        self.control.publish(msg)

    def awake_flag_callback(self, msg):
        if not msg.data:
            return
        self.awake_flag = True
        self.first_listen = True
        self.serial_text = ''
        self.session_deadline = time.time() + 15.0
        self._publish_words('唤醒成功(wake-up-success)')
        self.get_logger().info('\033[1;32m唤醒成功(wake-up-success)\033[0m')

    def iat_callback(self, msg):
        cmd = extract_command(msg.data)
        if cmd:
            self.serial_text = cmd
            self.get_logger().info('got serial command: %s' % cmd)

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

    def listen_and_recognize(self):
        if self.recognizer is None:
            return ''

        # First listen waits out the wake clip. Later commands in the
        # same session start recording immediately.
        if self.first_listen:
            time.sleep(1.6)
            self.first_listen = False
        cmd = extract_command(self.serial_text)
        if cmd:
            self.serial_text = ''
            return cmd

        chunk_bytes = int(SAMPLE_RATE * 0.1) * 2
        chunks = []
        peak = 0.0
        proc = None
        try:
            proc = self._open_recorder()
            nchunks = int(self.seconds_per_order / 0.1)
            for i in range(nchunks):
                if not rclpy.ok():
                    break
                cmd = extract_command(self.serial_text)
                if cmd:
                    self.serial_text = ''
                    return cmd
                raw = proc.stdout.read(chunk_bytes)
                if not raw:
                    break
                samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                if i < 2:
                    continue
                if samples.size:
                    peak = max(peak, float(np.max(np.abs(samples))))
                    chunks.append(samples)
        except Exception as exc:
            self.get_logger().error('6-mic record failed: %s' % exc)
            return ''
        finally:
            if proc is not None:
                try:
                    proc.terminate()
                    proc.wait(timeout=1)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass

        if not chunks:
            self.get_logger().info('listen peak=%.4f no audio' % peak)
            return ''

        audio = np.concatenate(chunks)
        stream = self.recognizer.create_stream()
        stream.accept_waveform(SAMPLE_RATE, audio)
        tail = np.zeros(int(SAMPLE_RATE * 0.4), dtype=np.float32)
        stream.accept_waveform(SAMPLE_RATE, tail)
        stream.input_finished()
        while self.recognizer.is_ready(stream):
            self.recognizer.decode_stream(stream)
        raw_text = (self.recognizer.get_result(stream) or '').strip()
        if self.language != 'Chinese':
            raw_text = raw_text.lower()
        cmd = extract_command(raw_text)
        self.get_logger().info('listen peak=%.4f raw=%r cmd=%r' % (peak, raw_text, cmd))
        return cmd

    def main(self):
        if not self.awake_flag or self.busy:
            return
        if time.time() > self.session_deadline:
            self.awake_flag = False
            self.first_listen = True
            return
        self.busy = True
        try:
            text = self.listen_and_recognize()
            self.get_logger().info('\033[1;32mresult: %s\033[0m' % text)
            if text:
                self.session_deadline = time.time() + 15.0
                self._publish_words(text)
                self.get_logger().info('\033[1;32mok\033[0m')
                time.sleep(2.8)
                return
            if time.time() > self.session_deadline:
                self.awake_flag = False
                self.first_listen = True
        finally:
            self.busy = False


def main():
    node = ASRNode('asr_node')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
