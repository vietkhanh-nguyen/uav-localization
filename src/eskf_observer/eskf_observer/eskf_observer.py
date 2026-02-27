import numpy as np

class ErrorStateKF:
    def __init__(self, dt):
        
        self.dt = dt
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # [w, x, y, z]
        self.ab = np.zeros(3)
        self.wb = np.zeros(3)
        
        self.P = np.eye(15) * 0.01 
        self.P[9:12, 9:12] = 0.0001 
        self.P[12:15, 12:15] = 0.0001 
        
        self.g = np.array([0, 0, -9.81])

    def _skew(self, v):
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])

    def quat_to_rot_mat(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])

    def rot_vec_to_rot_mat(self, omega_dt):
        phi = np.linalg.norm(omega_dt)
        if phi < 1e-12: 
            return np.eye(3)
        u = omega_dt / phi
        return np.eye(3)*np.cos(phi) + self._skew(u)*np.sin(phi) + np.outer(u, u)*(1 - np.cos(phi))
    
    def rot_mat_to_quat(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            w, x, y, z = 0.25 * S, (R[2,1]-R[1,2])/S, (R[0,2]-R[2,0])/S, (R[1,0]-R[0,1])/S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w, x, y, z = (R[2,1]-R[1,2])/S, 0.25 * S, (R[0,1]+R[1,0])/S, (R[0,2]+R[2,0])/S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w, x, y, z = (R[0,2]-R[2,0])/S, (R[0,1]+R[1,0])/S, 0.25 * S, (R[1,2]+R[2,1])/S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w, x, y, z = (R[1,0]-R[0,1])/S, (R[0,2]+R[2,0])/S, (R[1,2]+R[2,1])/S, 0.25 * S
        
        q = np.array([w, x, y, z])
        n = np.linalg.norm(q)
        return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])

    def predict(self, am, wm, sigma_an, sigma_wn, sigma_aw, sigma_ww):
        R_mat = self.quat_to_rot_mat(self.q)
        acc_body = am - self.ab
        omega_body = wm - self.wb
        
        acc_world = R_mat @ acc_body + self.g
        self.p += self.v * self.dt + 0.5 * acc_world * (self.dt**2)
        self.v += acc_world * self.dt
        
        angle_step = omega_body * self.dt
        dq = self.rot_mat_to_quat(self.rot_vec_to_rot_mat(angle_step))
        self.q = self._quat_multiply(self.q, dq)
        
        n_q = np.linalg.norm(self.q)
        if n_q > 1e-12:
            self.q /= n_q
        else:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])

        Fx = np.eye(15)
        Fx[0:3, 3:6] = np.eye(3) * self.dt
        Fx[3:6, 6:9] = -R_mat @ self._skew(acc_body) * self.dt
        Fx[3:6, 9:12] = -R_mat * self.dt

        rot_step_T = self.rot_vec_to_rot_mat(angle_step).T
        Fx[6:9, 6:9] = rot_step_T 
        Fx[6:9, 12:15] = -np.eye(3) * self.dt

        Qi = np.zeros((15, 15))
        Qi[3:6, 3:6] = (sigma_an**2 * self.dt**2) * np.eye(3)
        Qi[6:9, 6:9] = (sigma_wn**2 * self.dt**2) * np.eye(3)
        Qi[9:12, 9:12] = (sigma_aw**2 * self.dt) * np.eye(3)
        Qi[12:15, 12:15] = (sigma_ww**2 * self.dt) * np.eye(3)
        
        self.P = Fx @ self.P @ Fx.T + Qi
        self.P = (self.P + self.P.T) / 2


    def update_mag(self, mag_meas, R_noise_scalar):

        m_ref = np.array([1.0, 0.0, 0.0]) 
        R_mat = self.quat_to_rot_mat(self.q)
        m_meas_world = R_mat @ mag_meas
        psi_ref = np.arctan2(m_ref[1], m_ref[0])
        psi_meas = np.arctan2(m_meas_world[1], m_meas_world[0])
        yaw_error = psi_ref - psi_meas
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
        H_heading = np.zeros((1, 15))
        H_heading[0, 6:9] = np.array([0, 0, 1]) @ R_mat
        self._kalman_correct(np.array([yaw_error]), H_heading, R_noise_scalar)

    def update_marvelmind(self, pos_marvel, R_marvel):
        if np.any(np.isnan(pos_marvel)):
            return

        H = np.zeros((3, 15))
        H[:, 0:3] = np.eye(3)
        
        self._kalman_correct(pos_marvel - self.p, H, R_marvel)

    def update_slam(self, pos_slam, quat_slam, R_slam_pos, R_slam_quat):
        if pos_slam is not None and not np.any(np.isnan(pos_slam)):
            H_p = np.zeros((3, 15))
            H_p[:, 0:3] = np.eye(3)
            self._kalman_correct(pos_slam - self.p, H_p, R_slam_pos)
        
        if quat_slam is not None and not np.any(np.isnan(quat_slam)):
            q_error = self._quat_multiply(self._quat_inv(self.q), quat_slam)
            dtheta_obs = 2 * q_error[1:] 
            if q_error[0] < 0: 
                dtheta_obs = -dtheta_obs
            
            H_q = np.zeros((3, 15))
            H_q[:, 6:9] = np.eye(3) 
            self._kalman_correct(dtheta_obs, H_q, R_slam_quat)

    def _kalman_correct(self, innovation, H, R_noise):

        S = H @ self.P @ H.T + R_noise
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        dx = K @ innovation
        self._inject(dx)
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_noise @ K.T
        
        G = np.eye(15)
        G[6:9, 6:9] = np.eye(3) - self._skew(0.5 * dx[6:9])
        self.P = G @ self.P @ G.T
        
        self.P = (self.P + self.P.T) / 2

    def _inject(self, dx):
        self.p += dx[0:3]
        self.v += dx[3:6]
        
        dq = self.rot_mat_to_quat(self.rot_vec_to_rot_mat(dx[6:9]))
        self.q = self._quat_multiply(self.q, dq)
        
        n_q = np.linalg.norm(self.q)
        if n_q > 1e-12:
            self.q /= n_q
        else:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])
            
        self.ab += dx[9:12]
        self.wb += dx[12:15]

    def _quat_multiply(self, q, p):
        qw, qx, qy, qz = q
        pw, px, py, pz = p
        return np.array([
            qw*pw - qx*px - qy*py - qz*pz,
            qw*px + qx*pw + qy*pz - qz*py,
            qw*py - qx*pz + qy*pw + qz*px,
            qw*pz + qx*py - qy*px + qz*pw
        ])

    def _quat_inv(self, q):
        return np.array([q[0], -q[1], -q[2], -q[3]])