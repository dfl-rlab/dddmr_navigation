"""Three-goal navigation test. Start Gazebo separately before launch_test."""
import os
import sys
import unittest
from pathlib import Path

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('p2p_move_base')
    # Resolve next to this test so source-tree edits also work with launch_test.
    config = str(Path(__file__).resolve().parents[1] / 'config' / 'nav_ackermann_p2p_test.yaml')
    with open(config, encoding='utf-8') as stream:
        settings = yaml.safe_load(stream)
    test_params = settings['nav_ackermann_p2p']['ros__parameters']
    map_dir = Path(settings['map1']['ros__parameters']['pose_graph_dir'])
    for name in ('poses.pcd', 'map.pcd', 'ground.pcd', 'pcd'):
        if not (map_dir / name).exists():
            raise RuntimeError(f'Missing test map asset: {map_dir / name}')
    # Fixed cleanup allowance after the single navigation deadline.
    timeout = float(test_params['total_timeout_sec']) + 20.0
    if timeout <= 0:
        raise ValueError('Test timeout must be positive')

    def node(package, executable, **kwargs):
        return Node(package=package, executable=executable, output='screen',
                    parameters=[config, {'use_sim_time': True}], **kwargs)

    nodes = [
        node('lego_loam_bor', 'mcl_feature', remappings=[('/lslidar_point_cloud', '/cloud')]),
        node('dddmr_pg_map_server', 'dddmr_pg_map_server_node'),
        node('mcl_3dl', 'mcl_3dl'),
        node('global_planner', 'global_planner_node'),
        node('p2p_move_base', 'p2p_move_base_node'),
        node('local_planner', 'ackermann_drive_to_cmd_vel_node'),
        node('tf2_ros', 'static_transform_publisher', name='gazebo_base_link_alias',
             arguments=['0', '0', '0', '0', '0', '0', 'saye/base_link', 'base_link']),
        node('tf2_ros', 'static_transform_publisher', name='sensor2baselink',
             arguments=['0', '0', '-0.219', '0', '0', '0', 'base_link', 'base_footprint']),
    ]
    runner = Path(__file__).resolve().parents[1] / 'test_node' / 'nav_ackermann_p2p_test_node.py'
    test_node = ExecuteProcess(
        cmd=[sys.executable, '-u', str(runner), '--ros-args', '--params-file', config,
             '-p', 'use_sim_time:=true'], output='screen')
    # The test owns goal submission; do not start the interactive clicked2goal client.
    rviz = node('rviz2', 'rviz2', condition=IfCondition(LaunchConfiguration('enable_rviz')),
                arguments=['-d', os.path.join(share, 'rviz', 'p2p_move_base_localization.rviz')])
    guards = [RegisterEventHandler(OnProcessExit(
        target_action=n,
        on_exit=[EmitEvent(event=Shutdown(reason='Required navigation process exited'))]
    )) for n in nodes]
    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        *guards, *nodes, rviz, test_node,
        TimerAction(period=timeout, actions=[EmitEvent(event=Shutdown(reason='Navigation test watchdog timeout'))]),
        launch_testing.actions.ReadyToTest(),
    ]), {'test_node': test_node, 'test_timeout': timeout}


class TestNavigation(unittest.TestCase):
    def test_goals(self, proc_output, test_node, test_timeout):
        proc_output.assertWaitFor('Done', process=test_node,
                                 timeout=test_timeout, stream='stdout')


@launch_testing.post_shutdown_test()
class TestNavigationExit(unittest.TestCase):
    def test_assertion_message(self, proc_output, test_node):
        launch_testing.asserts.assertInStdout(proc_output, 'Success', process=test_node)
