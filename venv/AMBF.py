from ambf_client import Client
import time

_client = Client()
_client.connect()
print(_client.get_obj_names())
psm1_handle = _client.get_obj_handle('/ambf/env/psm1/baselink')
psm2_handle = _client.get_obj_handle('/ambf/env/psm2/baselink')
#torus_obj = _client.get_obj_handle('Torus')
#torus_obj.set_pos(0, 0, 0)
#torus_obj.set_rpy(1.5, 0.7, .0)
psm1_handle.set_pos(0, 0, 0)
psm1_handle.set_rpy(1.5, 0.7, .0)
psm2_handle.set_pos(0, 0, 0)
psm2_handle.set_rpy(1.5, 0.7, .0)
time.sleep(5)

cur_pos_psm1 = psm1_handle.get_pos()
cur_rot_psm1 = psm1_handle.get_rot()
cur_rpy_psm1 = psm1_handle.get_rpy()

cur_pos_psm2 = psm2_handle.get_pos()
cur_rot_psm2 = psm2_handle.get_rot()
cur_rpy_psm2 = psm2_handle.get_rpy()

#torus_obj.set_force(5, -5, 10)
#torus_obj.set_torque(0, 0, 0.8)
time.sleep(5)

#for i in range(0, 5000):
#    torus_obj.set_force(5, -5, 10)
#    torus_obj.set_torque(0, 0, 0.8)
#    time.sleep(0.001)
num_joints_psm1 = psm1_handle.get_num_joints()
num_joints_psm2 = psm2_handle.get_num_joints()
children_names_psm1 = psm1_handle.get_children_names()
children_names_psm2 = psm2_handle.get_children_names()
print("psm1 num joints: " + num_joints_psm1)
print("psm2 num joints: " + num_joints_psm2)
print("psm1 children: " + children_names_psm1)
print("psm2 children: " + children_names_psm2)

if num_joints > 1:
    torus_obj.set_joint_pos(0, 0.5)
    torus_obj.set_joint_effort(1, 5)
    time.sleep(2)

_client.clean_up()

