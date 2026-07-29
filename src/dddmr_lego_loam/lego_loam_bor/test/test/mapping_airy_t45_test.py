import os
import sys
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_testing.asserts import assertInStdout

import pytest

@pytest.mark.launch_test
def generate_test_description():

  ### Change test name and TF only
  test_name = 'mapping_airy_t45'
  s2b = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    output="screen" ,
    arguments=["0.3", "0.0", "0.5", "0.0", "0.78539815", "0.0", "base_link", "airy"]
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
  
  #for test node
  test_node = Node(
    package="lego_loam_bor",
    executable="mapping_test_node",
    name=test_name,
    output="screen"
  )  
  
  # for debug
  rviz = Node(
          package="rviz2",
          executable="rviz2",
          output="screen",
          arguments=['-d', os.path.join(get_package_share_directory('lego_loam_bor'), 'rviz', 'lego_loam.rviz')]
  )  

  return LaunchDescription([
      s2b,
      lego_loam_bag_node,
      rviz,
      TimerAction(period=5.0, actions=[test_node]),
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