#!/usr/bin/env python3
import os
import re
import sys
import socket
import importlib

def get_cpu_serial_number():
    device_serial_number = open("/proc/device-tree/serial-number")
    serial_num = device_serial_number.readlines()[0][-10:-1]
    return serial_num

def update_globals(module):
    if module in sys.modules:
        mdl = importlib.reload(sys.modules[module])
    else:
        mdl = importlib.import_module(module)
    if "__all" in mdl.__dict__:
        names = mdl.__dict__["__all__"]
    else:
        names = [x for x in mdl.__dict__ if not x.startswith("_")]
    globals().update({k: getattr(mdl, k) for k in names})

def get_typerc():
    with open("/home/ubuntu/ros2_ws/.typerc", "r") as f:
        data = f.read()
        machine = re.findall(r'export MACHINE_TYPE.*?\n', data)[0].split('=')[1].replace('\n', '')
        depth_camera_type  = re.findall(r'export DEPTH_CAMERA_TYPE.*?\n', data)[0].split('=')[1].replace('\n', '')
        app_version = re.findall(r'export VERSION.*?\n', data)[0].split('=')[1].split('|')[2][4:].replace('\n', '')
        f.close()
        return machine, app_version, depth_camera_type

if __name__ == "__main__":
    host = '0.0.0.0'
    port = 9027
    machine_type, APP_VERSION, depth_camera_type = get_typerc()

    robot_type = 'ROSPIDER'
    print(robot_type) 
    sn = get_cpu_serial_number().ljust(32, '0')
    WIFI_AP_SSID = ''.join(["WN-", sn[0:8]])
    WIFI_STA_SSID = ""

    IMG_W = 640
    IMG_H = 480
    if depth_camera_type == 'aurora' :
        IMG_W = 640
        IMG_H = 400
    elif depth_camera_type == 'usb_cam':
        IMG_W = 640
        IMG_H = 480

    path = os.path.split(os.path.realpath(__file__))[0]
    config_file_name = "wifi_conf.py"
    external_config_file_dir_path = path
    external_config_file_path = os.path.join(external_config_file_dir_path, config_file_name)
    if os.path.exists(external_config_file_path):
        sys.path.insert(0, external_config_file_dir_path)
        update_globals(os.path.splitext(config_file_name)[0])

    sn = WIFI_AP_SSID[3:].ljust(32, '0')

    udpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpServer.bind((host, port))
    while True:
        data, addr = udpServer.recvfrom(1024)
        msg = str(data, encoding = 'utf-8')
        full_identity = f"{robot_type}_{IMG_W}_{IMG_H}:{sn}"
        print(full_identity, addr)
        if msg == "LOBOT_NET_DISCOVER":
            full_identity = f"{robot_type}_{IMG_W}_{IMG_H}:{sn}"
            udpServer.sendto(bytes(full_identity + "\n", encoding='utf-8'), addr)
