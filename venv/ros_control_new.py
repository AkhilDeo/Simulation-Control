#!/usr/bin/env python
# //==============================================================================
# /*

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from ambf_client import Client
import rospy
from PyKDL import Rotation, Frame, Vector
import socket
import json
import time
from psm_arm import PSM
from ecm_arm import ECM
import numpy as np
import sys
import signal


UDP_IP = socket.gethostbyname(socket.gethostname())
UDP_PORT = 15002
# print("Start")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))


class RobotData:

    def __init__(self):
        self.measured_js = JointState()
        self.measured_cp = PoseStamped()


robData = RobotData()


def measured_js_cb(msg):
    robData.measured_js = msg


def measured_cp_cb(msg):
    robData.measured_cp = msg


_client = Client()
_client.connect()
print(_client.get_obj_names())
w = _client.get_world_handle()
w.reset_bodies()
psm1 = PSM(_client, 'psm1')
psm2 = PSM(_client, 'psm2')
ecm = ECM(_client, 'CameraFrame')
psm_arms = {"left": psm1,
            "right": psm2}

# The PSMs can be controlled either in joint space or cartesian space. For the
# latter, the `servo_cp` command sets the end-effector pose w.r.t its Base frame.

# T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/2.), Vector(0., 0., -1.3))
# T_e_b = Frame(Rotation.RPY(np.pi / 2.0, 0, np.pi/2.), Vector(0., 0., -1.3))
# print("Setting the end-effector frame of PSM1 w.r.t Base", T_e_b)
# psm1.servo_cp(T_e_b)
# psm1.set_jaw_angle(0.2)
# time.sleep(1.0)
# T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/4.), Vector(0.1, -0.1, -1.3))
# print("Setting the end-effector frame of PSM2 w.r.t Base", T_e_b)
# psm2.servo_cp(T_e_b)
# psm2.set_jaw_angle(0.5)
# time.sleep(1.0)
# # jp = [0., 0.2, -0.3, 0.2]
# jp = [0.0, 0.0, 0.0, 0.0]
# print("Setting ECM joint positions to ", jp)
# ecm.servo_jp(jp)
# print('ECM pose in World', ecm.measured_cp())
# time.sleep(1.0)
# print("TEST ECM cartesian movement", jp)
# jp3 = Rotation.RPY(2, 0, 0)
# jp4 = Vector(0, 1, 0)
# jp2 = Frame(jp3, jp4)
# ecm.servo_cp(jp2)
# time.sleep(5.0)


def signal_handler(signum, frame):
    print("\nCtrl+C clicked!")
    exit(1)


signal.signal(signal.SIGINT, signal_handler)


print("Starting TeleOp")
rate = rospy.Rate(60)
cur_slider = 0.4
while True:
    data, addr = sock.recvfrom(1024)
    if data is not None:
        dataDict = json.loads(data)
        if dataDict['camera'] == 'true':
            ecm.servo_jp([dataDict['yaw'], dataDict['pitch'], dataDict['insert'], dataDict['roll']])
            # cam_rpy = Rotation.RPY(dataDict['pitch'], dataDict['yaw'], dataDict['roll'])
            # cam_xyz = Vector(dataDict['x'], dataDict['y'], dataDict['z'])
            # ecm.servo_cp(Frame(cam_rpy, cam_xyz))
        elif 'x' in dataDict:
            robot_arm = psm_arms[dataDict['arm']]
            if dataDict['arm'] == 'right':
                cmd_rpy = Rotation.RPY(dataDict['pitch'], -1 * dataDict['yaw'] + np.pi, dataDict['roll'] - (np.pi / 4))
                cmd_xyz = Vector(dataDict['x'] + 0.1, dataDict['y'] - 0.1, dataDict['z'] - 1.3)
                robot_arm.servo_cp(Frame(cmd_rpy, cmd_xyz))
            else:
                cmd_rpy = Rotation.RPY(-1 * dataDict['pitch'],
                                       dataDict['yaw'], dataDict['roll'] + (np.pi / 4))
                cmd_xyz = Vector(dataDict['x'], dataDict['y'], dataDict['z'] - 1.3)
                robot_arm.servo_cp(Frame(cmd_rpy, cmd_xyz))
            if dataDict['slider'] != cur_slider:
                robot_arm.set_jaw_angle(dataDict['slider'])
                cur_slider = dataDict['slider']
    rate.sleep()
