#!/usr/bin/env python
# //==============================================================================
# /*

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import rospy
import math
from PyKDL import Rotation
import socket
import json
import time

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


rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_name = "psm2"
measured_js_name = namespace + arm_name + "/measured_js"
measured_cp_name = namespace + arm_name + "/measured_cp"
servo_jp_name = namespace + arm_name + "/servo_jp"
servo_cp_name = namespace + arm_name + "/servo_cp"

measured_js_sub = rospy.Subscriber(measured_js_name, JointState, measured_js_cb, queue_size=1)
measured_cp_sub = rospy.Subscriber(measured_cp_name, PoseStamped, measured_cp_cb, queue_size=1)

servo_jp_pub = rospy.Publisher(servo_jp_name, JointState, queue_size=1)
servo_cp_pub = rospy.Publisher(servo_cp_name, PoseStamped, queue_size=1)

rate = rospy.Rate(50)

servo_jp_msg = JointState()
servo_jp_msg.position = [0., 0., 1.0, 0., 0., 0.]

servo_cp_msg = PoseStamped()
servo_cp_msg.pose.position.z = -1.0
R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)

servo_cp_msg.pose.orientation.x = R_7_0.GetQuaternion()[0]
servo_cp_msg.pose.orientation.y = R_7_0.GetQuaternion()[1]
servo_cp_msg.pose.orientation.z = R_7_0.GetQuaternion()[2]
servo_cp_msg.pose.orientation.w = R_7_0.GetQuaternion()[3]
print("NOTE!!! For this example to work, please RUN the launch_crtk_interface.py script before hand.")

while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024)
    # data = data.decode()
    if bool(data):
        dataDict = json.loads(data)
        #print(dataDict)
        #print(dataDict['slider'])
        # The following 3 lines move the robot in cartesian space
        # servo_cp_msg.pose.position.x = 0.2 * math.sin(rospy.Time.now().to_sec())
        # servo_cp_msg.pose.position.y = 0.2 * math.cos(rospy.Time.now().to_sec())
        servo_cp_msg.pose.position.x = dataDict['x']
        servo_cp_msg.pose.position.y = dataDict['y']
        servo_cp_msg.pose.position.y = dataDict['z']
        servo_cp_pub.publish(servo_cp_msg)

    rate.sleep()