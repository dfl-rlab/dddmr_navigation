import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the shared configuration file
    p2p_move_base_share = get_package_share_directory('p2p_move_base')
    config_file = os.path.join(p2p_move_base_share, 'config', 'p2p_nav_wo_localization.yaml')
    rviz_config_file = os.path.join(p2p_move_base_share, 'rviz', 'p2p_move_base_localization.rviz')

    # Declare the launch argument for simulation time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Reference the launch argument value
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,

        Node(
            package='perception_3d',
            executable='laserscan2pointcloud_node',
            name='scan_front',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('scan', '/scan_front'),
                ('point_cloud_from_scan', '/front_cloud')
            ]
        ),

        Node(
            package='perception_3d',
            executable='laserscan2pointcloud_node',
            name='scan_back',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('scan', '/scan_back'),
                ('point_cloud_from_scan', '/back_cloud')
            ]
        ),

        Node(
            package='global_planner',
            executable='occupancy2ground',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='global_planner',
            executable='global_planner_node',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='p2p_move_base',
            executable='p2p_move_base_node',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='p2p_move_base',
            executable='clicked2goal.py',
            output='screen',
            respawn=False,
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])