# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    trt_node = Node(
        package='dddmr_semantic_segmentation',
        executable='ddrnet_ros_img_sub.py',
        parameters=[{
            'publish_colored_mask_result': True, #set to False to save cpu up tp 20% and reach more than 15 fps
            'use_class_based_mask_result': False #False use rgb based
        }],
        remappings=[
            ('/camera/camera/color/image_raw', '/camera/camera/color/image_raw')
        ]
    )


    mask2pointcloud_node = Node(
        package='dddmr_semantic_segmentation',
        executable='semantic_segmentation2point_cloud',
        parameters=[{
            'max_distance': 10.0,
            'sample_step': 2,
            'voxel_size': 0.05,
            'exclude_class': [-1]
        }],
        remappings=[
            ('/camera_info', '/camera/camera/depth/camera_info'),
            ('/ddrnet_inferenced_mask', '/ddrnet_inferenced_mask'),
            ('/image_rect_raw', '/camera/camera/depth/image_rect_raw')
        ]
    )
    
    pkg_path = get_package_share_directory('dddmr_semantic_segmentation')
    rviz2_node = Node(
            package="rviz2",
            namespace=namespace,
            executable="rviz2",
            output="screen",
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'bag_semantic_segmentation_launch.rviz')]
    ) 

    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "--loop",
            "/root/dddmr_bags/rs455_rgbd_848x480",
        ],
        output="screen",
    )

    cam2optical = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.0", "0.0", "0.0", "-1.571", "-0.000", "-1.571", "camera_link", "camera_depth_optical_frame"]
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    
    ld.add_action(trt_node)
    ld.add_action(mask2pointcloud_node)
    ld.add_action(rviz2_node)

    ld.add_action(cam2optical)
    ld.add_action(TimerAction(period=8.0, actions=[bag_player])) #wait for engine to be loaded

    return ld