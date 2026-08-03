import os
import sys
import unittest

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
  test_name = 'mapping_c16_t0'
  s2b = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="screen" ,
    arguments=["0.3", "0.0", "0.5", "-3.1415926535", "0.0", "0.0", "base_link", "laser_link"]
  )

  the_yaml = os.path.join(
    get_package_share_directory('lego_loam_bor'),
    'test', 'config',
    test_name+'.yaml'
  )

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