import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import PointStamped, Quaternion, Point, Vector3, PoseStamped
import numpy as np

# Giả định file C++ đã biên dịch thành module eskf_observer_cpp
from .eskf_observer_cpp import ErrorStateKF 

class ESKFNode(Node):
    def __init__(self):
        super().__init__('eskf_node')
        
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Khai báo tham số nhiễu (Sigma)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu.accel.sigma', 0.05),   # Nhiễu gia tốc kế
                ('imu.gyro.sigma', 0.005),   # Nhiễu con quay hồi chuyển
                ('mag.sigma', 0.01),         # Nhiễu từ kế
                ('marvel.sigma', 0.02),      # Nhiễu Marvelmind (GPS trong nhà)
                ('slam.pos.sigma', 0.05),    # Nhiễu vị trí SLAM
                ('slam.quat.sigma', 0.02)    # Nhiễu góc quay SLAM
            ])
            
        self.sig_an = self.get_parameter('imu.accel.sigma').value
        self.sig_wn = self.get_parameter('imu.gyro.sigma').value
        
        # Ma trận hiệp phương sai đo lường (R)
        self.R_mag = np.eye(3) * (self.get_parameter('mag.sigma').value**2)
        self.R_marvel = np.eye(3) * (self.get_parameter('marvel.sigma').value**2)
        self.R_slam_pos = np.eye(3) * (self.get_parameter('slam.pos.sigma').value**2)
        self.R_slam_quat = np.eye(3) * (self.get_parameter('slam.quat.sigma').value**2)

        # 1. IMU Subscriber
        self.sub_imu = self.create_subscription(
            Imu, 
            '/drone/imu', 
            self.imu_callback, 
            10
        )

        # 2. Magnetic Subscriber
        self.sub_mag = self.create_subscription(
            MagneticField, 
            '/drone/magnetic', 
            self.magnetic_callback, 
            10
        )

        # 3. Marvelmind Subscriber
        self.sub_marvel = self.create_subscription(
            PointStamped, 
            '/drone/marvelmind_pos', 
            self.marvel_callback, 
            10
        )
        
        self.sub_slam = self.create_subscription(
            PoseStamped, 
            '/orb_slam3/aligned_pose', 
            self.slam_callback, 
            10
        )

        self.pub_odom = self.create_publisher(Odometry, '/drone/state_est', 10)

        # Khởi tạo bộ lọc ESKF (Backend C++)
        self.eskf = ErrorStateKF(0.001) 
        self.last_imu_time = None
        
        self.get_logger().info("ESKF Node Started. Fusing: IMU + Mag + Marvel + SLAM(Aligned)")

    def imu_callback(self, msg):
        # Chuyển đổi stamp sang float giây
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time
        
        if dt <= 0: return 

        self.eskf.dt = dt
        acc_imu = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro_imu = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Bước dự báo (Prediction)
        self.eskf.predict(acc_imu, gyro_imu, self.sig_an, self.sig_wn, 0.001, 0.0001)
        
        # Publish kết quả ngay sau bước dự báo để có tần số cao (theo IMU)
        self.publish_estimate(msg.header.stamp)

    def slam_callback(self, msg):

        pos_slam = np.array([
            msg.pose.position.x, 
            msg.pose.position.y, 
            msg.pose.position.z
        ])
        
        quat_slam = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x, 
            msg.pose.orientation.y, 
            msg.pose.orientation.z
        ])

        self.eskf.update_slam(
            pos_slam, 
            quat_slam, 
            self.R_slam_pos, 
            self.R_slam_quat
        )

    def magnetic_callback(self, msg):
        mag_vec = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
        # Lấy hiệp phương sai từ tin nhắn hoặc dùng mặc định
        R_mag_scalar = float(msg.magnetic_field_covariance[0])
        if R_mag_scalar == 0: R_mag_scalar = 0.01**2 # Fallback nếu covariance rỗng
            
        self.eskf.update_mag(mag_vec, R_mag_scalar)

    def marvel_callback(self, msg):
        pos_marvel = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.eskf.update_marvelmind(pos_marvel, self.R_marvel)

    def publish_estimate(self, timestamp):
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = "world"
        msg.child_frame_id = "drone_est"

        # Gán Position
        msg.pose.pose.position = Point(x=self.eskf.p[0], y=self.eskf.p[1], z=self.eskf.p[2])
        
        # Gán Orientation
        q_vec = self.eskf.q
        msg.pose.pose.orientation = Quaternion(w=q_vec[0], x=q_vec[1], y=q_vec[2], z=q_vec[3])
        
        # Gán Velocity
        msg.twist.twist.linear = Vector3(x=self.eskf.v[0], y=self.eskf.v[1], z=self.eskf.v[2])
        
        self.pub_odom.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ESKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()