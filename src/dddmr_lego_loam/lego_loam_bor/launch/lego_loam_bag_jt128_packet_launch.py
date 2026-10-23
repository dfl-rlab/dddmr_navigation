import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the shared configuration file
    lego_loam_share = get_package_share_directory('lego_loam_bor')
    config_file = os.path.join(lego_loam_share, 'config', 'p2p_move_base_localization.yaml')
    rviz_config_file = os.path.join(lego_loam_share, 'rviz', 'p2p_move_base_localization.rviz')


    the_yaml = os.path.join(
      get_package_share_directory('lego_loam_bor'),
      'config',
      'loam_bag_jt128_packet_config.yaml'
    )

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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor2baselink',
            arguments=['0.33', '0', '1.12', '0.0', '0.7853981', '0', 'base_link', 'hesai_lidar_2'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
          package="lego_loam_bor",
          executable="lego_loam_hesai_packet_node",
          output="screen",
          parameters = [the_yaml]
        ),  

        Node(
          namespace='hesai_ros_driver', 
          package='hesai_ros_driver', 
          executable='hesai_ros_driver_node', 
          output='screen',
          parameters = [the_yaml]
        ),

        Node(
          package="rviz2",
          executable="rviz2",
          output="screen",
          arguments=['-d', os.path.join(get_package_share_directory('lego_loam_bor'), 'rviz', 'lego_loam.rviz')]
        ),
    ])