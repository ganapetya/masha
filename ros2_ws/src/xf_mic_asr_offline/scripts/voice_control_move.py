#!/usr/bin/env python3
# encoding: utf-8

# 语音控制移动(voice control move)
import os
import re
import json
import math
import time
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from controller import controller_client
from xf_mic_asr_offline import voice_play
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from servo_controller.action_group_controller import ActionGroupController

SPECIAL_WORDS = {
    '唤醒成功(wake-up-success)',
    '休眠(Sleep)',
    '失败5次(Fail-5-times)',
    '失败10次(Fail-10-times)',
    '失败10次(Fail-10-times',
}

# Longer phrases first so "go forward" wins over "forward".
COMMAND_ALIASES = (
    ('go forwards', 'go forward'),
    ('go forward', 'go forward'),
    ('move forward', 'go forward'),
    ('walk forward', 'go forward'),
    ('go backward', 'go backward'),
    ('go backwards', 'go backward'),
    ('move backward', 'go backward'),
    ('walk backward', 'go backward'),
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
    ('stop', 'stop'),
    ('dance', 'dance'),
    ('forward', 'go forward'),
    ('backward', 'go backward'),
    ('back', 'go backward'),
    ('come', 'come here'),
)


def match_command(raw):
    if raw is None:
        return None
    text = str(raw).strip()
    if text in SPECIAL_WORDS:
        return text
    normalized = text.lower()
    normalized = re.sub(r'[^a-z0-9\u4e00-\u9fff\s]', ' ', normalized)
    normalized = re.sub(r'\s+', ' ', normalized).strip()
    if not normalized:
        return None
    for phrase, command in COMMAND_ALIASES:
        if phrase == normalized or phrase in normalized:
            return command
    return normalized

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree(laser scanning angle, removing obstructed degrees)
CAR_WIDTH = 0.4  # meter

class VoiceControMovelNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.words = None
        self.running = True
        self.haved_stop = False
        self.lidar_follow = False
        self.start_follow = False
        self.last_status = Twist()
        self.threshold = 3
        self.speed = 0.3
        self.stop_dist = 0.4
        self.count = 0
        self.scan_angle = math.radians(90)
        self.declare_parameter('move', False)
        self.move = self.get_parameter('move').value

        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)

        self.language = os.environ['ASR_LANGUAGE']
        self.controller = controller_client.ControllerClient()
        self.agc_controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/actionset_editor/ActionGroups')
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/asr_node/voice_words', self.words_callback, 1)

        self.client = self.create_client(Trigger, '/asr_node/init_finish')
        self.client.wait_for_service()  # 阻塞等待(blocking wait)
        self.declare_parameter('delay', 0)
        time.sleep(self.get_parameter('delay').value)

        self.get_logger().info('唤醒口令: Hello Masha / Hi Masha / Shalom Masha')
        self.get_logger().info('唤醒后可以说指令(After wake-up say: go forward / turn left / dance / come here)')
        self.get_logger().info('控制指令: 左转 右转 前进 后退 过来 跳个舞吧(Voice command: turn left/turn right/go forward/go backward/come here /dance)')
        self.time_stamp = time.time()
        self.current_time_stamp = time.time()
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.play('running')

        if self.language == 'Chinese':
            self.get_logger().info('\033[1;32m%s\033[0m' % '准备就绪')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'I am ready')




    def get_node_state(self, request, response):
        response.success = True
        return response

    def play(self, name):
        # Never block the motion thread on aplay (USB speaker can hang after reboot).
        voice_play.play_async(name, language=self.language)

    def words_callback(self, msg):
        self.words = match_command(msg.data)
        if self.language == 'Chinese' and self.words not in SPECIAL_WORDS and self.words:
            self.words = self.words.replace(' ', '')
        self.get_logger().info('words:%s' % self.words)
        if self.words == '休眠(Sleep)':
            buzz = BuzzerState()
            buzz.freq = 1000
            buzz.on_time = 0.1
            buzz.off_time = 0.01
            buzz.repeat = 1
            self.buzzer_pub.publish(buzz)

    def main(self):
        while True:
            if self.words is not None:
                words = self.words
                self.words = None
                if words in SPECIAL_WORDS:
                    continue
                twist = Twist()
                matched = True
                if words in ('前进', 'go forward'):
                    self.play('go')
                    self.controller.traveling(gait=2, stride=40, height=20, direction=0, time=0.7, steps=6)
                    self.time_stamp = time.time() + 5
                    twist.linear.x = 0.12
                elif words in ('后退', 'go backward'):
                    self.play('back')
                    self.controller.traveling(gait=2, stride=40, height=20, direction=180, time=0.7, steps=6)
                    self.time_stamp = time.time() + 5
                    twist.linear.x = -0.12
                elif words in ('左转', 'turn left'):
                    self.play('turn_left')
                    self.controller.traveling(gait=2, stride=0, height=20, rotation=18, time=0.7, steps=6)
                    self.time_stamp = time.time() + 5
                    twist.angular.z = 0.4
                elif words in ('右转', 'turn right'):
                    self.play('turn_right')
                    self.controller.traveling(gait=2, stride=0, height=20, rotation=-18, time=0.7, steps=6)
                    self.time_stamp = time.time() + 5
                    twist.angular.z = -0.4
                elif words in ('左平移', 'move left'):
                    self.play('move_left')
                    self.controller.traveling(gait=2, stride=40, height=20, direction=90, time=0.7, steps=6)
                    self.time_stamp = time.time() + 5
                    twist.linear.y = 0.12
                elif words in ('右平移', 'move right'):
                    self.play('move_right')
                    self.controller.traveling(gait=2, stride=40, height=20, direction=270, time=0.7, steps=6)
                    self.time_stamp = time.time() + 5
                    twist.linear.y = -0.12
                elif words in ('跳个舞吧', 'dance'):
                    self.play('dance')
                    self.agc_controller.run_action('twist')
                elif words in ('过来', 'come here'):
                    self.play('come')
                    self.controller.traveling(gait=2, stride=40, height=20, direction=0, time=0.7, steps=6)
                    self.time_stamp = time.time() + 5
                    twist.linear.x = 0.12
                elif words == 'stop':
                    self.play('stop')
                    self.controller.traveling(gait=-2, time=1, steps=0)
                    self.cmd_vel_pub.publish(Twist())
                    self.time_stamp = time.time()
                    self.haved_stop = True
                    self.move = False
                    self.get_logger().info('stopped, still listening')
                    continue
                else:
                    matched = False
                    self.get_logger().info('unmatched command: %s' % words)
                if matched:
                    self.move = True
                    self.haved_stop = False
                    self.cmd_vel_pub.publish(twist)
            else:
                time.sleep(0.01)
            self.current_time_stamp = time.time()
            if self.time_stamp < self.current_time_stamp and not self.haved_stop and self.move:
                self.controller.traveling(gait=-2, time=1, steps=0)
                self.haved_stop = True
                if self.lidar_follow:
                    self.lidar_follow = False
                    self.start_follow = True




def main():
    node = VoiceControMovelNode('voice_control_move')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
