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
time.sleep(0.5)
print(_client.get_obj_names())
w = _client.get_world_handle()
w.reset_bodies()
time.sleep(0.2)
psm2 = PSM(_client, 'psm2')
time.sleep(0.5)
# psm2.servo_jp([-0.4, -0.22, 1.39, -1.64, -0.37, -0.11])
psm2.servo_jp([-0.4, -0.22, 1.39, 0, 0, 0])
psm2.set_jaw_angle(0.8)
time.sleep(10.0)
psm2.set_jaw_angle(0)
time.sleep(5)
psm2.set_jaw_angle(0.5)
time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, 1, -0.37, -0.11])
# time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, -3, -0.37, -0.11])
# time.sleep(5)
# psm2.servo_jp([-0.4, -0.22, 1.39, 3, -0.37, -0.11])
# time.sleep(5)
print("Starting TeleOp")
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024)
    if data is not None:
        dataDict = json.loads(data)
        if 'x' in dataDict:
            psm2.servo_jp([dataDict['x'] - 0.4, dataDict['y']-0.22, dataDict['z'] + 1.39, dataDict['roll'], dataDict['pitch'], dataDict['yaw']])
        psm2.set_jaw_angle(dataDict['slider'])
            #servo_jp_pub.publish(servo_jp_msg)
    rate.sleep()
