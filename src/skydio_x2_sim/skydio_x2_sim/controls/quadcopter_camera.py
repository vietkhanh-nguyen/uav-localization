import mujoco as mj
import numpy as np
import os

class QuadcopterCameraProcessing:

    def __init__(self, model, capture_fps, image_size, camera_name="drone_camera_l"):
        self.drone_cam = mj.MjvCamera()  
        self.drone_cam.type = mj.mjtCamera.mjCAMERA_FIXED
        self.drone_cam.fixedcamid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, camera_name)
        self.mount_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "cam_mount")
        self.capture_interval = 1.0/capture_fps
        self.image_height = image_size[0]
        self.image_width = image_size[1]
        self.time_prev_cap = 0
        self.rgb_array = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        self.rgb_flipped_array = np.zeros_like(self.rgb_array)
        self.depth_array = np.zeros((self.image_height, self.image_width), dtype=np.float32)
        self.init_quat = model.body_quat[self.mount_id].copy()
        self.current_roll = 0.0 
        self.limit_rad = np.deg2rad(60)
        self.offscreen = mj.Renderer(model, height=self.image_height, width=self.image_width)
        self.offscreen.disable_depth_rendering() 

    def capture(self, sim_data, time):

        capture_flag = time - self.time_prev_cap >= self.capture_interval
        if capture_flag:
            self.offscreen.update_scene(sim_data, self.drone_cam)
            self.rgb_array = self.offscreen.render() 
            self.time_prev_cap = time

        return self.rgb_array, capture_flag
    