import numpy as np
from scipy.spatial.transform import Rotation as R

class QuadcopterDynamicModel:

    def __init__(self, pms):
        self.pms = pms
        self.P = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0]
        ], dtype=float)
        
    def get_rotation_matrix(self, quat):
        return R.from_quat(quat)
    
    def cal_thrust(self, V_body, omega_m):
        return self.pms.k_omega*omega_m**2 - self.pms.k_z*omega_m*V_body[2] + self.pms.k_h*(V_body[0]**2 + V_body[1]**2)

    def forward_dynamics(self, quat, V_body, Omega, omega_m):
        Rot_obj = self.get_rotation_matrix(quat)
        T = self.cal_thrust(V_body, omega_m) 
        gen_force = self.pms.mixing_matrix @ T
        F_thrust_scalar = gen_force[0] 
        M_control = gen_force[1:]      
        F_thrust_body = np.array([0, 0, F_thrust_scalar])
        F_thrust_world = Rot_obj.apply(F_thrust_body)
        g_vec = np.array([0, 0, self.pms.g])
        dV_world = F_thrust_world / self.pms.m - g_vec
        torque_total = M_control - np.cross(Omega, self.pms.J @ Omega)
        dOmega = self.pms.Jinv @ torque_total 
        return np.block([dV_world, dOmega])


    def forward_dynamics_aero(self, quat, v_body, Omega, omega_m):
        Rot_obj = self.get_rotation_matrix(quat)
        T = self.cal_thrust(v_body, omega_m) 
        gen_force = self.pms.mixing_matrix @ T
        F_thrust_scalar = gen_force[0]
        M_control = gen_force[1:]
        D_body = -self.pms.k_d * np.sum(omega_m) * (self.P @ v_body)
        D_M_body = np.zeros(3)
        for i in range(4):
            f_drag_i = -self.pms.k_d * omega_m[i] * (self.P @ v_body)
            r_i = self.pms.motor_positions[i].copy()
            D_M_body += np.cross(r_i, f_drag_i)
        F_thrust_body = np.array([0, 0, F_thrust_scalar])
        F_thrust_world = Rot_obj.apply(F_thrust_body)
        D_world = Rot_obj.apply(D_body)
        g_vec = np.array([0, 0, self.pms.g])
        dV_world = (F_thrust_world + D_world) / self.pms.m - g_vec
        torque_total = M_control + D_M_body - np.cross(Omega, self.pms.J @ Omega)
        dOmega = self.pms.Jinv @ torque_total
        dOmega_world = Rot_obj.apply(dOmega)
        return np.block([dV_world, dOmega_world])
    
    def solve_motor_speed(self, v_body, T_i):
        a = self.pms.k_omega
        b = -self.pms.k_z * v_body[2]
        c = self.pms.k_h * (v_body[0]**2 + v_body[1]**2) - T_i
        delta = b**2 - 4 * a * c
        delta = np.maximum(0.0, delta)
        omega_i = (-b + np.sqrt(delta)) / (2 * a)
        return omega_i
    
    def solve_motor_speed_simple(self, T_i):
        return np.sqrt(np.maximum(0, T_i) / self.pms.k_omega) 
