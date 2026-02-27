import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_orb = get_package_share_directory('ros2_orb_slam3')
    
    # ORB-SLAM3 Specific paths
    voc_path = "/home/thundera/Workspace/ROS/mjc_drone_sim/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    settings_path = os.path.join(pkg_orb, 'params', 'sim.yaml')

    return LaunchDescription([
        # 1. ORB-SLAM3 Stereo Node
        Node(
            package='ros2_orb_slam3',
            executable='mono_node_cpp',
            name='orb_slam3_stereo',
            output='screen',
            arguments=[voc_path, settings_path],
            parameters=[{'use_sim_time': True}]
        ),

        # 2. Pose Aligner
        Node(
            package='eskf_observer', 
            executable='pose_aligner',
            name='pose_aligner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'slam_topic': '/orb_slam3/pose',
                'est_topic': '/drone/state_est'
            }]
        )
    ])