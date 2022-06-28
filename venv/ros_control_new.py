#!/usr/bin/env python
# //==============================================================================
# /*

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from ambf_client import Client
import rospy
import math
from PyKDL import Rotation, Frame, Vector
import socket
import json
import time
from psm_arm import PSM
from ecm_arm import ECM
import numpy as np

UDP_IP = socket.gethostbyname(socket.gethostname())
UDP_PORT = 15002
#print("Start")
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
T_e_b = Frame(Rotation.RPY(np.pi / 2.0, 0, np.pi/2.), Vector(0., 0., -1.3))
print("Setting the end-effector frame of PSM1 w.r.t Base", T_e_b)
psm1.servo_cp(T_e_b)
psm1.set_jaw_angle(0.2)
time.sleep(1.0)
T_e_b = Frame(Rotation.RPY(np.pi, 0, np.pi/4.), Vector(0.1, -0.1, -1.3))
print("Setting the end-effector frame of PSM2 w.r.t Base", T_e_b)
psm2.servo_cp(T_e_b)
psm2.set_jaw_angle(0.5)
time.sleep(1.0)
# jp = [0., 0.2, -0.3, 0.2]
jp = [0.0, 0.0, 0.0, 0.0]
print("Setting ECM joint positions to ", jp)
ecm.servo_jp(jp)
print('ECM pose in World', ecm.measured_cp())
time.sleep(5.0)
print("Setting ECM joint positions to ", jp)
ecm.servo_jp(jp)
print('ECM pose in World', ecm.measured_cp())
time.sleep(5.0)

# Servo_jp testing
# psm2.servo_jp([-0.4, -0.22, 1.39, -1.64, -0.37, -0.11])
# psm2.servo_jp([-0.4, -0.22, 1.39, 0, 0, 0])
# time.sleep(0.5)
# psm2.set_jaw_angle(0.5)
# time.sleep(5)
# print("Roll testing")
# psm2.servo_jp([-0.4, -0.22, 1.39, -3.05, 0, 0])
# time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, 3.05, 0, 0])
# time.sleep(5)
# print("Pitch testing")
# psm2.servo_jp([-0.4, -0.22, 1.39, 0, -1.57, 0])
# time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, 0, 1.57, 0])
# time.sleep(5)
# print("Yaw testing")
# psm2.servo_jp([-0.4, -0.22, 1.39, 0, 0, -1])
# time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, 0, 0, 1])
# time.sleep(5)


# psm2.servo_jp([-0.4, -0.22, 1.39, 1, -0.37, -0.11])
# time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, -3, -0.37, -0.11])
# time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, 3, -0.37, -0.11])
# time.sleep(5)
print("Starting TeleOp")
rate = rospy.Rate(300)
cur_slider = 0.4
while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024)
    if data is not None:
        dataDict = json.loads(data)
        robot_arm = psm_arms[dataDict['arm']]
        if 'x' in dataDict:
            # if dataDict['cameraBtn']:
            #     print("Camera Pressed")
            if dataDict['arm'] == 'right':
                cmd_rpy = Rotation.RPY(-1 * dataDict['yaw'] + np.pi, dataDict['pitch'], dataDict['roll'])
                cmd_xyz = Vector(dataDict['x'] + 0.1, dataDict['y'] - 0.1, dataDict['z'] - 1.3)
                T_IK = Frame(cmd_rpy, cmd_xyz)
                robot_arm.servo_cp(T_IK)
            else:
                cmd_rpy = Rotation.RPY(1 * dataDict['yaw'], dataDict['pitch'], dataDict['roll'])
                cmd_xyz = Vector(dataDict['x'], dataDict['y'], dataDict['z'] - 1.3)
                T_IK = Frame(cmd_rpy, cmd_xyz)
                robot_arm.servo_cp(T_IK)



        if dataDict['slider'] != cur_slider:
            robot_arm.set_jaw_angle(dataDict['slider'])
            cur_slider = dataDict['slider']
    rate.sleep()