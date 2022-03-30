from ambf_client import Client
import time

_client = Client()
_client.connect()
print(_client.get_obj_names())
torus_obj = _client.get_obj_handle('Torus')
torus_obj.set_pos(0, 0, 0)
torus_obj.set_rpy(1.5, 0.7, .0)
time.sleep(5)

cur_pos = torus_obj.get_pos()
cur_rot = torus_obj.get_rot()
cur_rpy = torus_obj.get_rpy()

torus_obj.set_force(5, -5, 10)
torus_obj.set_torque(0, 0, 0.8)
time.sleep(5)

for i in range(0, 5000):
    torus_obj.set_force(5, -5, 10)
    torus_obj.set_torque(0, 0, 0.8)
    time.sleep(0.001)
num_joints = torus_obj.get_num_joints()
children_names = torus_obj.get_children_names()
print(num_joints)
print(children_names)

if num_joints > 1:
    torus_obj.set_joint_pos(0, 0.5)
    torus_obj.set_joint_effort(1, 5)
    time.sleep(2)

_client.clean_up()

