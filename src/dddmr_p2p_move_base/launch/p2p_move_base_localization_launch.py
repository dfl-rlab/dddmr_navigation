import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the shared configuration file
    p2p_move_base_share = get_package_share_directory('p2p_move_base')
    config_file = os.path.join(p2p_move_base_share, 'config', 'p2p_move_base_localization.yaml')
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

        # 1. mcl_feature node
        Node(
            package='lego_loam_bor',
            executable='mcl_feature',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/lslidar_point_cloud', '/lslidar_point_cloud'),
                ('/odom', '/odom')
            ]
        ),

        # 2. Static TF Publisher (sensor2baselink)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor2baselink',
            arguments=['0.3', '0', '0.38', '-3.1415926535', '0.0', '0', 'base_link', 'laser_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 3. dddmr_pg_map_server node
        Node(
            package='dddmr_pg_map_server',
            executable='dddmr_pg_map_server_node',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        # 4. mcl_3dl node
        Node(
            package='mcl_3dl',
            executable='mcl_3dl',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        # 5. global_planner node
        Node(
            package='global_planner',
            executable='global_planner_node',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        # 6. p2p_move_base node
        Node(
            package='p2p_move_base',
            executable='p2p_move_base_node',
            output='screen',
            respawn=False,
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        # 7. clicked2goal.py script node
        Node(
            package='p2p_move_base',
            executable='clicked2goal.py',
            output='screen',
            respawn=False,
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 8. RViz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])