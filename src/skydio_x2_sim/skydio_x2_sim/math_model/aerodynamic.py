import numpy as np
from scipy.spatial.transform import Rotation as R

class AerodynamicModel:
    def __init__(self, pms):
        self.pms = pms
        self.P = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0]
        ], dtype=float)

    def _get_rot_and_vbody(self, quat):
        return R.from_quat(quat)

    def cal_induced_drag_wrench(self, quat, omega_m, v_body):
        rot_obj = self._get_rot_and_vbody(quat)
        D_body = -self.pms.k_d * np.sum(omega_m) * (self.P @ v_body)
        D_world = rot_obj.apply(D_body)
        D_M_body = np.zeros(3)
        for i in range(4):
            f_drag_i = -self.pms.k_d * omega_m[i] * (self.P @ v_body)
            r_i = self.pms.motor_positions[i] 
            D_M_body += np.cross(r_i, f_drag_i)
        D_M_world = rot_obj.apply(D_M_body)
        return np.block([D_world, D_M_world])
    
    def cal_thrust(self, omega_m, v_body):
        return self.pms.k_omega*omega_m**2 - self.pms.k_z*omega_m*v_body[2] + self.pms.k_h*(v_body[0]**2 + v_body[1]**2)
