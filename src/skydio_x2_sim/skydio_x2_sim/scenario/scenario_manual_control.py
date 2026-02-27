# Mujoco and calculation
import numpy as np
from skydio_x2_sim.controls.quadcopter_controller import QuadcopterPIDController
from skydio_x2_sim.controls.quadcopter_camera import QuadcopterCameraProcessing
from skydio_x2_sim.math_model.model_pms import MyX2Params, AeroParams
from skydio_x2_sim.math_model import aerodynamics_cpp 
from skydio_x2_sim.math_model.sensor_model import SensorModel
import mujoco as mj
import pickle
import logging
import os

# ROS2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, MagneticField
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped, Twist, PointStamped
from tf2_ros import TransformBroadcaster
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

LOG_FILE = "outputs/drone_testing_log.log"
os.makedirs("outputs", exist_ok=True)
logging.basicConfig(
    filename=LOG_FILE,
    level=logging.DEBUG, 
    format='%(levelname)s - %(message)s',
    filemode='w'
)
logger = logging.getLogger(__name__)


class ScenarioX2ManualControl:
    
    def __init__(self):
        self.name = "Drone Testing"     
        self.debug_flag = False 
        self.track_data_flag = False
        
        self.R = 5.0            
        self.omega = 1.0        
        self.V_target = self.R * self.omega 
        self.center = np.array([0.0, 0.0]) 
        self.altitude_ref = 5.0 
        self.vx_ref = 0.0
        self.vy_ref = 0.0
        self.t0 = 0.0           

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('mujoco_drone_publisher')
        
        self.dt_ground_truth = 0.02
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.pub_state_true = self.node.create_publisher(Odometry, '/drone/state_true', 10)

        self.pub_imu = self.node.create_publisher(Imu, '/drone/imu', 10)
        self.pub_marvel = self.node.create_publisher(PointStamped, '/drone/marvelmind_pos', 10)
        self.pub_clock = self.node.create_publisher(Clock, '/clock', 10)
        self.pub_image_left = self.node.create_publisher(Image, '/drone/camera/left/image_raw', 10)
        self.pub_image_right = self.node.create_publisher(Image, '/drone/camera/right/image_raw', 10)
        self.pub_mag = self.node.create_publisher(MagneticField, '/drone/magnetic', 10)
        self.bridge = CvBridge()

        self.sub_cmd = self.node.create_subscription(
            Twist, 
            '/drone/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        self.no_teleop = True

    def init(self, sim, model, data):
        sim.cam.type = mj.mjtCamera.mjCAMERA_FIXED
        sim.cam.fixedcamid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, "track")
        sim.model.opt.timestep = 1e-3
        self.dt = model.opt.timestep 
        self.yaw_ref = np.pi 

        self.controller = QuadcopterPIDController(time_step=model.opt.timestep)
        self.drone_camera_l = QuadcopterCameraProcessing(model, capture_fps=30, image_size=[240, 320])
        self.drone_camera_r = QuadcopterCameraProcessing(model, capture_fps=30, image_size=[240, 320], camera_name="drone_camera_r")

        def get_p(name, default_val=0.0):
            if not self.node.has_parameter(name):
                self.node.declare_parameter(name, default_val)

            return self.node.get_parameter(name).value
        
        self.imu_acc_model = SensorModel(
            operating_frequency=get_p('imu.accel.freq', 200), 
            sigma=get_p('imu.accel.sigma', 0.05), 
            bias=get_p('imu.accel.bias', 0.1))
            
        self.imu_gyro_model = SensorModel(
            operating_frequency=get_p('imu.gyro.freq', 200), 
            sigma=get_p('imu.gyro.sigma', 0.005), 
            bias=get_p('imu.gyro.bias', 0.01))
            
        self.mag_model = SensorModel(
            operating_frequency=get_p('mag.freq', 50), 
            sigma=get_p('mag.sigma', 0.01), 
            bias=get_p('mag.bias', 0.05))
            
        self.marvel_pos_model = SensorModel(
            operating_frequency=get_p('marvel.freq', 10), 
            sigma=get_p('marvel.sigma', 0.02), 
            bias=get_p('marvel.bias', 0.01))

        self.counter = 0
        self.sim_steps_per_env_step = 1
        self.integral_error = np.zeros(3)
        self.x2_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "x2")
        self.aero_pms = AeroParams()
        self.aero_pms.motor_positions = MyX2Params.motor_positions
        self.aero_model = aerodynamics_cpp.AerodynamicModel()
        
        self.t0 = sim.data.time 
        self.t_ground_truth_last = sim.data.time
        
        logger.info(f"Scenario started. Target Altitude: {self.altitude_ref}m. Circular Radius: {self.R}m.")

    def _publish_tf(self, clock_msg, pos, quat):

        t_msg = TransformStamped()
        t_msg.header.stamp = clock_msg.clock
        t_msg.header.frame_id = "world"
        t_msg.child_frame_id = "drone_body"
        t_msg.transform.translation.x = float(pos[0])
        t_msg.transform.translation.y = float(pos[1])
        t_msg.transform.translation.z = float(pos[2])
        t_msg.transform.rotation.x = float(quat[1])
        t_msg.transform.rotation.y = float(quat[2])
        t_msg.transform.rotation.z = float(quat[3])
        t_msg.transform.rotation.w = float(quat[0])

        self.tf_broadcaster.sendTransform(t_msg)

    def _publish_odometry(self, clock_msg, pos, quat, lin_vel, ang_vel):
        msg = Odometry()
        msg.header.stamp = clock_msg.clock
        msg.header.frame_id = "world"
        msg.child_frame_id = "drone_body"
        msg.pose.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        msg.twist.twist.linear = Vector3(x=lin_vel[0], y=lin_vel[1], z=lin_vel[2])
        msg.twist.twist.angular = Vector3(x=ang_vel[0], y=ang_vel[1], z=ang_vel[2])
        self.pub_state_true.publish(msg)

    def _publish_imu(self, clock_msg, imu_acc, imu_gyro):
        msg = Imu()
        msg.header.stamp = clock_msg.clock
        msg.header.frame_id = "drone_body" 

        # Angular Velocity
        msg.angular_velocity.x = float(imu_gyro[0])
        msg.angular_velocity.y = float(imu_gyro[1])
        msg.angular_velocity.z = float(imu_gyro[2])
        
        # Linear Acceleration
        msg.linear_acceleration.x = float(imu_acc[0])
        msg.linear_acceleration.y = float(imu_acc[1])
        msg.linear_acceleration.z = float(imu_acc[2])

        msg.orientation_covariance[0] = -1 
        var = np.square(self.imu_gyro_model.sigma)
        msg.angular_velocity_covariance = [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var
        ]
        var = np.square(self.imu_acc_model.sigma)
        msg.linear_acceleration_covariance = [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var
        ]

        self.pub_imu.publish(msg)

    def _publish_mag(self, clock_msg, mag_vec):
        msg = MagneticField()
        msg.header.stamp = clock_msg.clock
        msg.header.frame_id = 'drone_body'
        
        msg.magnetic_field.x = float(mag_vec[0])
        msg.magnetic_field.y = float(mag_vec[1])
        msg.magnetic_field.z = float(mag_vec[2])

        var = np.square(self.mag_model.sigma)
        msg.magnetic_field_covariance = [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var
        ]
        
        self.pub_mag.publish(msg)

    
    def _publish_marvelmind(self, clock_msg, pos):
        msg = PointStamped()
        msg.header.stamp = clock_msg.clock
        msg.header.frame_id = "world" 

        msg.point.x = float(pos[0])
        msg.point.y = float(pos[1])
        msg.point.z = float(pos[2])

        self.pub_marvel.publish(msg)
    
    def _publish_image(self, t, rgb_array, publisher):
        image_msg = self.bridge.cv2_to_imgmsg(rgb_array, encoding="rgb8")
        seconds = int(t)
        nanoseconds = int((t - seconds) * 1e9)
        stamp = Time(sec=seconds, nanosec=nanoseconds)
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "drone_body" 
        publisher.publish(image_msg)


    def cmd_vel_callback(self, msg):
        self.no_teleop = False
        self.vx_ref = msg.linear.x
        self.vy_ref = msg.linear.y
        self.altitude_ref = msg.linear.z 
        self.yaw_ref = msg.angular.z     
    
    def update(self, sim, model, data):
        t = sim.data.time
        body_linvel = np.array(data.sensor('body_linvel').data)
        body_angvel = np.array(data.sensor('body_angvel').data)
        body_linacc = np.array(data.sensor('body_linacc').data)
        global_pos = np.array(data.sensor('global_pos').data)
        global_quat = np.array(data.sensor('global_quat').data)
        global_linvel = np.array(data.sensor('global_linvel').data)
        global_angvel = np.array(data.sensor('global_angvel').data)
        body_mag = np.array(data.sensor('body_mag').data)

        imu_acc, acc_flag = self.imu_acc_model.output(body_linacc, t)
        imu_gyro, gyro_flag = self.imu_gyro_model.output(body_angvel, t)
        marvel_pos, marvel_pos_flag = self.marvel_pos_model.output(global_pos, t)
        mag_vec, mag_flag = self.mag_model.output(body_mag, t)

        state_true = np.block([global_pos, global_quat, global_linvel, body_angvel])


        seconds = int(t)
        nanoseconds = int((t - seconds) * 1e9)
        clock_msg = Clock()
        clock_msg.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.pub_clock.publish(clock_msg)

        if (t - self.t_ground_truth_last) >= self.dt_ground_truth:
            self._publish_odometry(clock_msg, global_pos, global_quat, global_linvel, body_angvel)
            self._publish_tf(clock_msg, global_pos, global_quat)
            self.t_ground_truth_last = t

        if gyro_flag: 
            self._publish_imu(clock_msg, imu_acc, imu_gyro)
            
        if mag_flag:
            self._publish_mag(clock_msg, mag_vec)

        if marvel_pos_flag:
            self._publish_marvelmind(clock_msg, marvel_pos)
        
        vel_ref_2d = np.array([self.vx_ref, self.vy_ref])
        control_input = self.controller.vel_body_control_algorithm(
            state=state_true, 
            vel_ref=vel_ref_2d,
            altitude_ref=self.altitude_ref, 
            yaw_ref=self.yaw_ref            
        )
        desired_rpm = np.sqrt(np.maximum(0, control_input) / self.aero_pms.k_omega) 

        drag_world = self.aero_model.cal_induced_drag_wrench(global_quat, desired_rpm, body_linvel)

        data.ctrl[:] = control_input
        data.xfrc_applied[self.x2_body_id, :] = np.zeros_like(data.xfrc_applied[self.x2_body_id, :])
        data.xfrc_applied[self.x2_body_id, :] = drag_world

        rclpy.spin_once(self.node, timeout_sec=0)

        return

    def render(self, sim_data, time):
        rgb_l, flag_l = self.drone_camera_l.capture(sim_data, time)
        rgb_r, flag_r = self.drone_camera_r.capture(sim_data, time)
        if flag_l and flag_r:
            self._publish_image(time, rgb_l, self.pub_image_left)
            self._publish_image(time, rgb_r, self.pub_image_right)
        

    def finish(self):
        if self.node:
            self.node.destroy_node()
            rclpy.shutdown()