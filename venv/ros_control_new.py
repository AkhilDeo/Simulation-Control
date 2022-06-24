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
#print(_client.get_obj_names())
w = _client.get_world_handle()
w.reset_bodies()
psm_left = PSM(_client, 'psm1')
psm_right = PSM(_client, 'psm2')
psm_arms = {"left": psm_left,
            "right": psm_right}
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
rate = rospy.Rate(250)
cur_slider = 0.5
while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024)
    if data is not None:
        dataDict = json.loads(data)
        robot_arm = psm_arms[dataDict['arm']]
        if 'x' in dataDict:
            if dataDict['arm'] == 'right':
                # robot_arm.servo_jp(
                #     [dataDict['x'] * 1.25 - 0.5, (dataDict['y'] * -1.25) - 0.1, (dataDict['z'] * -1.25) + 1.39,
                #                     (dataDict['roll'] * -1.5), (dataDict['pitch'] * 1.5), dataDict['yaw']])
                self.cmd_rpy = Rotation.RPY(dataDict['roll'], dataDict['pitch'], dataDict['yaw'])
                self.cmd_xyz = Vector(dataDict['x'], dataDict['y'], dataDict['z'])
                self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)
                robot_arm.servo_cp(self.T_IK)
            else:
                robot_arm.servo_jp(
                    [dataDict['x'] * 1.25 + 0.6, (dataDict['y'] * -1.25) + 0.1, (dataDict['z'] * -1.25) + 1.39,
                     (dataDict['roll'] * -1.5), (dataDict['pitch'] * 1.5), -1 * dataDict['yaw']])

        if dataDict['slider'] != cur_slider:
            robot_arm.set_jaw_angle(dataDict['slider'])
            cur_slider = dataDict['slider']
    rate.sleep()