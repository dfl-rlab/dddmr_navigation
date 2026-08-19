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
          arguments=['-d', os.path.join(get_package_share_directory('lego_loam_bor'), 'rviz', 'lego_loam.rviz')], # <-- FIXED CLOSING BRACKET HERE
          condition=IfCondition(enable_rviz_config)
  )  

  ### Change test name and TF only
  test_name = 'mapping_mid360_t180'
  s2b = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="screen" ,
    arguments=["0.0", "0.0", "1.0", "3.1415926535", "3.1415926535", "0.0", "base_link", "livox_frame"]
  )

  the_yaml = os.path.join(
    get_package_share_directory('lego_loam_bor'),
    'test', 'config',
    test_name+'.yaml'
  )

  bag_path = "/root/dddmr_bags/cicdtest/" + test_name
  correct_yaml_format_by_ros2_version(bag_path)

  lego_loam_bag_node = Node(
    package="lego_loam_bor",
    executable="lego_loam_bag",
    output="screen",
    parameters = [the_yaml]
  )  

  shutdown_on_crash = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=lego_loam_bag_node,
          on_exit=lambda event, context: EmitEvent(event=Shutdown(reason='lego_loam_bag node crashed')) if event.returncode != 0 else None
      )
  )
  
  #for test node
  test_node = Node(
    package="lego_loam_bor",
    executable="mapping_test_node",
    name=test_name,
    output="screen"
  )  

  return LaunchDescription([
      s2b,
      lego_loam_bag_node,
      shutdown_on_crash,
      rviz,
      TimerAction(period=5.0, actions=[test_node]),
      launch_testing.actions.ReadyToTest()
  ]), {'test_node': test_node, 'lego_loam_bag_node': lego_loam_bag_node}

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

    def test_lego_loam_bag_node_exit_code(self, proc_info, lego_loam_bag_node):
        # Trigger a failure if lego_loam_bag node crashes (exits with non-zero code)
        launch_testing.asserts.assertExitCodes(proc_info, process=lego_loam_bag_node)