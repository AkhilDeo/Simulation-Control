#!/usr/bin/env python
# //==============================================================================
# /*

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
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


rospy.init_node("sur_chal_crtk_test")
_client = Client()
_client.connect()
print(_client.get_obj_names())
psm2_handle = _client.get_obj_handle('/ambf/env/psm2/baselink')
namespace = "/CRTK/"
arm_name = "psm2"
measured_js_name = namespace + arm_name + "/measured_js"
measured_cp_name = namespace + arm_name + "/measured_cp"
servo_jp_name = namespace + arm_name + "/servo_jp"
servo_cp_name = namespace + arm_name + "/servo_cp"
jaw_name = namespace + arm_name + '/jaw/' + 'servo_jp'


measured_js_sub = rospy.Subscriber(measured_js_name, JointState, measured_js_cb, queue_size=1)
measured_cp_sub = rospy.Subscriber(measured_cp_name, PoseStamped, measured_cp_cb, queue_size=1)

servo_jp_pub = rospy.Publisher(servo_jp_name, JointState, queue_size=1)
servo_cp_pub = rospy.Publisher(servo_cp_name, PoseStamped, queue_size=1)
jaw_jp_pub = rospy.Publisher(jaw_name, JointState, queue_size=1)

def list_to_sensor_msg_position(jp_list):
    msg = JointState()
    msg.position = jp_list
    return msg

def set_jaw_angle(val):
    msg = list_to_sensor_msg_position([val])
    jaw_jp_pub.publish(msg)

def set_arm_position(x, y, z):
    servo_cp_msg.pose.position.x = x
    servo_cp_msg.pose.position.y = y
    servo_cp_msg.pose.position.z = z
    servo_cp_pub.publish(servo_cp_msg)

rate = rospy.Rate(30)

servo_jp_msg = JointState()
servo_jp_msg.position = [0., 0., 1.0, 0., 0., 0.]

servo_cp_msg = PoseStamped()
servo_cp_msg.pose.position.z = -1.0
R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)
#Will have to repeat same process of creating R_7_0 and get quaternion in loop when I implement rot/orientation

servo_cp_msg.pose.orientation.x = R_7_0.GetQuaternion()[0]
servo_cp_msg.pose.orientation.y = R_7_0.GetQuaternion()[1]
servo_cp_msg.pose.orientation.z = R_7_0.GetQuaternion()[2]
servo_cp_msg.pose.orientation.w = R_7_0.GetQuaternion()[3]
# servo_cp_msg.pose.position.x = -1.00
# servo_cp_msg.pose.position.y = 0.20
# servo_cp_msg.pose.position.z = -0.5

# print("NOTE!!! For this example to work, please RUN the launch_crtk_interface.py script before hand.")

# servo_cp_msg.pose.position.x = -1
# servo_cp_msg.pose.position.y = 0.2
# servo_cp_msg.pose.position.z = -.5
# servo_cp_pub.publish(servo_cp_msg)

while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024)
    if data is not None:
        print(data)
        dataDict = json.loads(data)
        #print(dataDict)
        #print(dataDict['slider'])
        # The following 3 lines move the robot in cartesian space
        # servo_cp_msg.pose.position.x = 0.2 * math.sin(rospy.Time.now().to_sec())
        # servo_cp_msg.pose.position.y = 0.2 * math.cos(rospy.Time.now().to_sec())
        if 'x' in dataDict:
            set_arm_position(dataDict['x'] - 1.05, dataDict['y'] - 0.1, dataDict['z'] - 0.5)
            #set_jaw_angle(dataDict['slider'] * math.pi)
            #psm2_handle.set_joint_pos('toolyawlink-toolgripper1link', dataDict['slider'])
            #servo_jp_pub.publish(servo_jp_msg)
            #time.sleep(0.001)
    rate.sleep()
