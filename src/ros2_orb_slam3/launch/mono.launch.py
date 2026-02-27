import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    home_dir = os.environ['HOME']
    voc_file = os.path.join(home_dir, 'Workspace/ROS/mjc_drone_sim/src/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    pkg_share = get_package_share_directory('ros2_orb_slam3')
    settings_file = os.path.join(pkg_share, 'params', 'sim.yaml')

    return LaunchDescription([
        Node(
            package='ros2_orb_slam3',
            executable='mono_node_cpp',
            name='orb_slam3_mono',
            output='screen',
            parameters=[{
                'voc_file': voc_file,
                'use_sim_time': True,
            }],
            arguments=[voc_file, settings_file],
            remappings=[
                # SỬA LỖI QUAN TRỌNG:
                # Remap từ topic nội bộ '/mono_py_driver/img_msg' sang topic thật của Drone
                ('/mono_py_driver/img_msg', '/drone/camera/image_raw') 
            ]
        )
    ])
