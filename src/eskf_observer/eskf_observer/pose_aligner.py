import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

class PoseAligner(Node):
    def __init__(self):
        super().__init__('pose_aligner')
        
        self.declare_parameter('slam_topic', '/orb_slam3/pose')
        self.declare_parameter('est_topic', '/drone/state_est')
        
        self.sub_slam = self.create_subscription(PoseStamped, self.get_parameter('slam_topic').value, self.slam_callback, 10)
        self.sub_est = self.create_subscription(Odometry, self.get_parameter('est_topic').value, self.est_callback, 10)
        self.pub_aligned = self.create_publisher(PoseStamped, '/orb_slam3/aligned_pose', 10)
        
        self.current_est = None
        self.T_offset = None
        self.initialized = False
        self.last_slam_pos = None
        self.jump_threshold = 1.5 

        self.T_cb = np.array([
            [ 0.0, -1.0,  0.0,  0.01],
            [ 0.0,  0.0, -1.0,  0.07],
            [ 1.0,  0.0,  0.0, -0.15],
            [ 0.0,  0.0,  0.0,  1.0 ]
        ])

    def est_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        T = np.eye(4)
        T[:3, 3] = [pos.x, pos.y, pos.z]
        T[:3, :3] = R.from_quat([ori.x, ori.y, ori.z, ori.w]).as_matrix()
        self.current_est = T

    def slam_callback(self, msg):
        if self.current_est is None:
            return

        pos_raw = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        if self.initialized and self.last_slam_pos is not None:
            if np.linalg.norm(pos_raw - self.last_slam_pos) > self.jump_threshold:
                self.get_logger().warn('New Map/Jump detected. Re-aligning...')
                self.initialized = False
        
        self.last_slam_pos = pos_raw

        T_mc = np.eye(4)
        T_mc[:3, 3] = pos_raw
        T_mc[:3, :3] = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, 
                                    msg.pose.orientation.z, msg.pose.orientation.w]).as_matrix()
        
        T_mb = T_mc @ self.T_cb

        if not self.initialized:
            self.T_offset = self.current_est @ np.linalg.inv(T_mb)
            self.initialized = True
            self.get_logger().info('Map Aligned to World successfully.')

        T_wb = self.T_offset @ T_mb
        
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'world'
        out.pose.position.x, out.pose.position.y, out.pose.position.z = T_wb[:3, 3]
        
        q = R.from_matrix(T_wb[:3, :3]).as_quat()
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
        self.pub_aligned.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = PoseAligner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()