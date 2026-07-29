import os
import sys
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_testing.asserts import assertInStdout

import pytest

@pytest.mark.launch_test
def generate_test_description():

  # Declare the launch argument for simulation time
  use_sim_time_arg = DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true'
  )
  
  # Reference the launch argument value
  use_sim_time = LaunchConfiguration('use_sim_time')

  ### Change test name and TF only
  test_name = 'mcl_3dl'
  s2b = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="screen" ,
    arguments=["0.3", "0.0", "0.38", "3.1415926535", "0.0", "0.0", "base_link", "laser_link"]
  )

  mcl_3dl_yaml = os.path.join(
    get_package_share_directory('mcl_3dl'),
    'test', 'config',
    test_name+'.yaml'
  )

  dddmr_pg_map_server = Node(
    package="dddmr_pg_map_server",
    executable="dddmr_pg_map_server_node",
    output="screen",
    parameters = [mcl_3dl_yaml, {'use_sim_time': use_sim_time}]
  )  

  mcl_3dl_feature_node = Node(
    package="lego_loam_bor",
    executable="mcl_feature",
    output="screen",
    parameters = [mcl_3dl_yaml, {'use_sim_time': use_sim_time}]
  )  

  mcl_3dl_node = Node(
    package="mcl_3dl",
    executable="mcl_3dl",
    output="screen",
    parameters = [mcl_3dl_yaml, {'use_sim_time': use_sim_time}]
  )  

  #for test node
  test_node = Node(
    package="mcl_3dl",
    executable="mcl_3dl_test_node",
    name=test_name,
    output="screen"
  )  

  bag_player = ExecuteProcess(
      cmd=[
          "ros2",
          "bag",
          "play",
          "-r",
          "2.0",
          "/root/dddmr_bags/cicdtest/mcl_3dl_c16",
      ],
      output="screen",
  )

  # for debug
  rviz = Node(
          package="rviz2",
          executable="rviz2",
          output="screen",
          arguments=['-d', os.path.join(get_package_share_directory('mcl_3dl'), 'rviz', 'mcl_3dl_demo.rviz')]
  )  

  return LaunchDescription([
      use_sim_time_arg,
      s2b,
      dddmr_pg_map_server,
      mcl_3dl_feature_node,
      mcl_3dl_node,
      bag_player,
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