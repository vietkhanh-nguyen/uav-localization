import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_skydio = get_package_share_directory('skydio_x2_sim')
    pkg_orb = get_package_share_directory('ros2_orb_slam3')
    
    # Configuration paths
    config_path = os.path.join(pkg_skydio, 'config', 'sensor_params.yaml')
    rviz_config_path = os.path.join(pkg_skydio, 'rviz', 'drone_viz.rviz')
    
    # ORB-SLAM3 specific paths
    voc_path = "/home/thundera/Workspace/ROS/mjc_drone_sim/src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    settings_path = os.path.join(pkg_orb, 'params', 'sim.yaml')

    return LaunchDescription([
        # 1. MuJoCo Simulator: Master clock source
        Node(
            package='skydio_x2_sim',
            executable='simulation_runner',
            name='mujoco_simulator',
            output='screen',
            parameters=[config_path, {'use_sim_time': False}],
            arguments=['--gui', '--ros-args', '--log-level', 'info']
        ),

        # 2. ESKF Observer: State estimation using IMU/Marvelmind
        Node(
            package='eskf_observer',
            executable='eskf_node',
            name='eskf_observer',
            output='screen',
            parameters=[config_path, {'use_sim_time': True}]
        ),

        # 3. ORB-SLAM3 Stereo: Visual Odometry
        Node(
            package='ros2_orb_slam3',
            executable='mono_node_cpp',
            name='orb_slam3_stereo',
            output='screen',
            arguments=[voc_path, settings_path],
            parameters=[{'use_sim_time': True}]
        ),

        # 4. Pose Aligner: Align SLAM coordinate system to World
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
        ),

        # 5. Keyboard Teleop: Manual control terminal
        Node(
            package='eskf_observer', 
            executable='teleop_node',
            name='keyboard_teleop',
            prefix='xterm -e', 
            parameters=[{'use_sim_time': True}]
        ),

        # 6. RViz2: Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])