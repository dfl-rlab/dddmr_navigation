import os
import sys
import unittest
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.actions import EmitEvent
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_testing.asserts import assertInStdout

import pytest


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def correct_yaml_format_by_ros2_version(metadata_path, ros_version=None):
    """
    Read metadata.yaml, convert offered_qos_profiles, version, and type_description_hash
    based on ROS 2 version, and save the modified metadata.yaml.
    If ros_version is jazzy or above, sets version to 9, offered_qos_profiles to [],
    and ensures type_description_hash is present.
    If ros_version is humble (or below), sets version to 5 and offered_qos_profiles to "".

    :param metadata_path: Path to metadata.yaml file or directory containing it.
    :param ros_version: ROS 2 distro version string (e.g. 'jazzy', 'humble'). Defaults to $ROS_DISTRO.
    """
    if not metadata_path:
        return
    if os.path.isdir(metadata_path):
        metadata_file = os.path.join(metadata_path, 'metadata.yaml')
    else:
        metadata_file = metadata_path

    if not os.path.exists(metadata_file):
        return

    if ros_version is None:
        ros_version = os.environ.get('ROS_DISTRO', 'humble')

    ros_version = str(ros_version).strip().lower()
    first_char = ros_version[0] if ros_version else 'h'

    with open(metadata_file, 'r') as f:
        data = yaml.safe_load(f)

    if not data or not isinstance(data, dict):
        return

    bag_info = data.get('rosbag2_bagfile_information', {})
    if isinstance(bag_info, dict):
        if first_char >= 'j':
            bag_info['version'] = 9
            if 'custom_data' not in bag_info:
                bag_info['custom_data'] = None
            if 'ros_distro' not in bag_info:
                bag_info['ros_distro'] = ros_version
        else:
            bag_info['version'] = 5

        topics = bag_info.get('topics_with_message_count', [])
        if isinstance(topics, list):
            for topic in topics:
                if isinstance(topic, dict):
                    topic_meta = topic.get('topic_metadata', {})
                    if isinstance(topic_meta, dict):
                        if 'offered_qos_profiles' not in topic_meta or topic_meta['offered_qos_profiles'] in ('', [], None):
                            topic_meta['offered_qos_profiles'] = [] if first_char >= 'j' else ""
                        if first_char >= 'j':
                            if 'type_description_hash' not in topic_meta or topic_meta['type_description_hash'] is None:
                                topic_meta['type_description_hash'] = ""

    with open(metadata_file, 'w') as f:
        yaml.dump(data, f, Dumper=NoAliasDumper, default_flow_style=False, sort_keys=False)


@pytest.mark.launch_test
def generate_test_description():

  ### Argument for rviz
  enable_rviz_arg = DeclareLaunchArgument(
    'enable_rviz',
    default_value='true',
    description='Enable Rviz for developer, otherwise for CI'
  )
  # 2. Reference the config string
  enable_rviz_config = LaunchConfiguration('enable_rviz')

  # for debug
  rviz = Node(
          package="rviz2",
          executable="rviz2",
          output="screen",
          arguments=['-d', os.path.join(get_package_share_directory('perception_3d'), 'rviz', 'p3d_mpl_laserscan_test.rviz')], # <-- FIXED CLOSING BRACKET HERE
          condition=IfCondition(enable_rviz_config)
  )  

  # Declare the launch argument for simulation time
  use_sim_time_arg = DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true'
  )
  
  # Reference the launch argument value
  use_sim_time = LaunchConfiguration('use_sim_time')

  ###
  test_name = 'perception_3d_multilayer_spinning_lidar_hokuyo2d'
  b2s = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='baselink2laser',
    arguments=['0.34', '0.26', '0.0', '0.785398163', '0.0', '0', 'base_link', 'laser'],
    parameters=[{'use_sim_time': use_sim_time}]
  )

  b2s2 = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='baselink2laser2',
    arguments=['-0.34', '0.26', '0.0', '-2.38', '0.0', '0', 'base_link', 'laser2'],
    parameters=[{'use_sim_time': use_sim_time}]
  )

  config_yaml = os.path.join(
    get_package_share_directory('perception_3d'),
    'test', 'config',
    test_name+'.yaml'
  )

  p3d = Node(
    package='perception_3d',
    executable='laserscan2pointcloud_node',
    name='scan_front',
    output='screen',
    respawn=False,
    parameters=[config_yaml, {'use_sim_time': use_sim_time}],
    remappings=[
      ('scan', '/scan_front'),
      ('point_cloud_from_scan', '/front_cloud')
    ]
  ) 

  l2p = Node(
    package='perception_3d',
    executable='laserscan2pointcloud_node',
    name='scan_back',
    output='screen',
    respawn=False,
    parameters=[config_yaml, {'use_sim_time': use_sim_time}],
    remappings=[
      ('scan', '/scan_back'),
      ('point_cloud_from_scan', '/back_cloud')
    ]
  )

  o2g = Node(
    package='global_planner',
    executable='occupancy2ground',
    output='screen',
    respawn=False,
    parameters=[config_yaml, {'use_sim_time': use_sim_time}]
  )

  gpl = Node(
    package='global_planner',
    executable='global_planner_node',
    output='screen',
    respawn=False,
    parameters=[config_yaml, {'use_sim_time': use_sim_time}]
  )

  #for test node
  test_node = Node(
    package="perception_3d",
    executable="perception_3d_multilayer_spinning_lidar_hokuyo2d_test_node",
    name="perception_3d_multilayer_spinning_lidar_hokuyo2d_test_node",
    output="screen",
    parameters = [config_yaml, {'use_sim_time': use_sim_time}]
  )  

  bag_path = "/root/dddmr_bags/cicdtest/" + test_name + "/" + test_name
  #correct_yaml_format_by_ros2_version(bag_path)

  bag_player = ExecuteProcess(
      cmd=[
          "ros2",
          "bag",
          "play",
          "-r",
          "1.0",
          bag_path,
      ],
      output="screen",
  )

  return LaunchDescription([
      use_sim_time_arg,
      b2s,
      b2s2,
      p3d,
      l2p,
      o2g,
      gpl,
      TimerAction(period=5.0, actions=[bag_player]),
      rviz,
      TimerAction(period=3.0, actions=[test_node]),
      launch_testing.actions.ReadyToTest()
  ]), {'test_node': test_node}

# These tests will run concurrently with the dut process.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestGoodProcess(unittest.TestCase):

    def test_mapping(self, proc_output):
        # This will match stdout from any process.  In this example there is only one process
        # running
        proc_output.assertWaitFor('Done', timeout=900, stream='stdout')


@launch_testing.post_shutdown_test()
class TestStdOutput(unittest.TestCase):
    def test_assertion_message(self, proc_output, test_node):
        assertInStdout(
            proc_output,
            "Success", 
            process=test_node
        )