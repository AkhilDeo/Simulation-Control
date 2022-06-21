from surgical_robotics_challenge.kinematics.psmIK import *
from PyKDL import Frame, Rotation, Vector, Twist
import time


class Camera:
    def __init__(self, client, name):
        self.client = client
        self.name = name
        self.camera_handle = self.client.get_obj_handle(name)
        time.sleep(0.5)

        # Transform of Base in World
        self._T_c_w = None
        # Transform of World in Base
        self._T_w_c = None
        self._pose_changed = True
        self._num_joints = 6
        self._ik_solution = np.zeros([self._num_joints])
        self._last_jp = np.zeros([self._num_joints])

    def is_present(self):
        if self.camera_handle is None:
            return False
        else:
            return True

    def get_T_c_w(self):
        self._update_camera_pose()
        return self._T_c_w

    def get_T_w_c(self):
        self._update_camera_pose()
        return self._T_w_c

    def has_pose_changed(self):
        return self._pose_changed

    def set_pose_changed(self):
        self._pose_changed = True

    def _update_camera_pose(self):
        p = self.camera_handle.get_pos()
        q = self.camera_handle.get_rot()
        P_c_w = Vector(p.x, p.y, p.z)
        R_c_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
        self._T_c_w = Frame(R_c_w, P_c_w)
        self._T_w_c = self._T_c_w.Inverse()
        self._pose_changed = False

    def move_cp(self, T_c_w):
        if type(T_c_w) in [np.matrix, np.array]:
            T_c_w = convert_mat_to_frame(T_c_w)

        self.camera_handle.set_pos(T_c_w.p[0], T_c_w.p[1], T_c_w.p[2])
        rpy = T_c_w.M.GetRPY()
        self.camera_handle.set_rpy(rpy[0], rpy[1], rpy[2])
        self._pose_changed = True

    def move_cv(self, twist, dt):
        if type(twist) in [np.array, np.ndarray]:
            v = Vector(twist[0], twist[1], twist[2]) * dt
            w = Vector(twist[3], twist[4], twist[5]) * dt
        elif type(twist) is Twist:
            v = twist.vel * dt
            w = twist.rot * dt
        else:
            raise TypeError

        T_c_w = self.get_T_c_w()
        T_cmd = Frame(Rotation.RPY(w[0], w[1], w[2]), v)
        self.move_cp(T_c_w * T_cmd)
        pass

    def measured_cp(self):
        return self.get_T_c_w()