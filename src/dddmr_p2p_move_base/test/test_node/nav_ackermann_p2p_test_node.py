#!/usr/bin/env python3
"""Sequential navigation action test; arrival tolerances belong to the planner."""
import csv
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from dddmr_sys_core.action import PToPMoveBase
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass
class GoalRecord:
    name: str
    pose: list
    verdict: str = 'SKIPPED'
    action: str = 'NOT_SENT'
    reason: str = 'not_executed'
    elapsed: float = 0.0
    x: float = math.nan
    y: float = math.nan
    z: float = math.nan
    heading: float = math.nan
    xy_error: float = math.nan
    z_error: float = math.nan
    yaw_error: float = math.nan

    @property
    def target_yaw(self):
        return yaw(*self.pose[3:])

    def row(self):
        return [self.name, self.verdict, self.action, self.reason, self.elapsed,
                *self.pose[:3], self.target_yaw, self.x, self.y, self.z,
                self.heading, self.xy_error, self.z_error, self.yaw_error]


class NavigationTest(Node):
    def __init__(self):
        super().__init__('nav_ackermann_p2p')
        self.started = time.monotonic()
        self.total = self.declare_parameter('total_timeout_sec', 960.0).value
        self.delay = self.declare_parameter('startup_delay_sec', 15.0).value
        if not math.isfinite(self.total) or self.total <= 0:
            raise ValueError('total_timeout_sec must be finite and positive')
        if not math.isfinite(self.delay) or self.delay < 0:
            raise ValueError('startup_delay_sec must be finite and nonnegative')
        self.report = self.declare_parameter(
            'result_file', '/tmp/nav_ackermann_p2p_results.csv').value
        names = self.declare_parameter('goal_names', Parameter.Type.STRING_ARRAY).value
        if not names or len(set(names)) != len(names):
            raise ValueError('goal_names must be nonempty and unique')
        self.goals = []
        for name in names:
            pose = list(self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY).value)
            if len(pose) != 7 or not all(math.isfinite(v) for v in pose):
                raise ValueError(f'{name} must contain seven finite values: x,y,z,qx,qy,qz,qw')
            norm = math.sqrt(sum(v * v for v in pose[3:]))
            if norm < 1e-6:
                raise ValueError(f'{name} has zero quaternion')
            pose[3:] = [v / norm for v in pose[3:]]
            self.goals.append(GoalRecord(name, pose))
        self.client = ActionClient(self, PToPMoveBase, '/p2p_move_base')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.ready = dict(map=False, ground=False, planner_graph=False)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_subs = []
        # weighted_ground is published after the global planner initializes A*.
        for key, topic in [('map', '/map1/mapcloud'), ('ground', '/map1/mapground'),
                           ('planner_graph', '/weighted_ground')]:
            self.cloud_subs.append(self.create_subscription(
                PointCloud2, topic,
                lambda msg, key=key: self.ready.update(
                    {key: bool(msg.width and msg.height and msg.data)}), qos))
        self.pending_goal = self.active_goal = self.active_result = None
        self.current = None
        self.goal_started = None

    def elapsed(self):
        return time.monotonic() - self.started

    def expired(self):
        return self.elapsed() >= self.total

    def pump(self):
        rclpy.spin_once(self, timeout_sec=0.01)

    def wait(self, future, deadline):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self.pump()
        return rclpy.ok() and future.done() and time.monotonic() < deadline

    def current_pose(self):
        try:
            transform = self.buffer.lookup_transform('map', 'base_footprint', Time())
            p, q = transform.transform.translation, transform.transform.rotation
            if all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                return p.x, p.y, p.z, yaw(q.x, q.y, q.z, q.w)
        except TransformException:
            pass
        return None

    def snapshot(self, goal):
        pose = self.current_pose()
        if pose is None:
            return False
        goal.x, goal.y, goal.z, goal.heading = pose
        goal.xy_error = math.hypot(goal.x - goal.pose[0], goal.y - goal.pose[1])
        goal.z_error = goal.z - goal.pose[2]
        delta = goal.heading - goal.target_yaw
        goal.yaw_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
        return True

    def cancel(self):
        """Bound cleanup, including an acceptance response arriving after timeout."""
        if not rclpy.ok():
            return
        deadline = time.monotonic() + 5.0
        if self.active_goal is None and self.pending_goal is not None:
            if self.wait(self.pending_goal, deadline):
                handle = self.pending_goal.result()
                if handle is not None and handle.accepted:
                    self.active_goal = handle
                    self.active_result = handle.get_result_async()
        if self.active_goal is not None:
            self.wait(self.active_goal.cancel_goal_async(), deadline)
            if self.active_result is not None:
                self.wait(self.active_result, deadline)
            self.get_logger().warning('Cancellation requested; navigation shuts down after the test result.')

    def run(self):
        self.get_logger().info(f'Minimum startup delay: {self.delay:.1f}s (included in total timeout)')
        self.get_logger().info('Waiting for action server, map/ground, planner graph and map -> base_footprint TF')
        last_log = time.monotonic()
        while rclpy.ok():
            self.pump()
            if self.expired():
                return self.finish(False, 'total_timeout_during_startup')
            if (self.elapsed() >= self.delay and self.client.server_is_ready()
                    and all(self.ready.values()) and self.current_pose() is not None):
                break
            if time.monotonic() - last_log >= 5.0:
                self.get_logger().info(
                    f'Startup: action={self.client.server_is_ready()} {self.ready} '
                    f'pose={self.current_pose() is not None} elapsed={self.elapsed():.1f}s')
                last_log = time.monotonic()
        if not rclpy.ok():
            return self.finish(False, 'interrupted')
        for goal in self.goals:
            if self.expired():
                return self.finish(False, 'total_timeout')
            self.current, self.goal_started = goal, time.monotonic()
            goal.verdict, goal.action, goal.reason = 'FAIL', 'WAITING_ACCEPTANCE', 'pending'
            request = PToPMoveBase.Goal()
            request.target_pose.header.frame_id = 'map'
            request.target_pose.header.stamp = self.get_clock().now().to_msg()
            p, q = request.target_pose.pose.position, request.target_pose.pose.orientation
            p.x, p.y, p.z, q.x, q.y, q.z, q.w = goal.pose
            self.get_logger().info(
                f'Sending {goal.name}: target={goal.pose[:3]}, yaw={goal.target_yaw:.6f}, '
                f'remaining_total={self.total - self.elapsed():.1f}s')
            self.pending_goal = self.client.send_goal_async(request)
            if not self.wait(self.pending_goal, self.started + self.total):
                return self.fail_current('total_timeout' if rclpy.ok() else 'interrupted')
            handle = self.pending_goal.result()
            self.pending_goal = None
            if handle is None or not handle.accepted:
                goal.action = 'REJECTED'
                return self.fail_current('goal_rejected')
            self.active_goal = handle
            goal.action = 'EXECUTING'
            self.active_result = handle.get_result_async()
            if not self.wait(self.active_result, self.started + self.total):
                return self.fail_current('total_timeout' if rclpy.ok() else 'interrupted')
            result = self.active_result.result()
            self.active_goal = self.active_result = None
            goal.elapsed = time.monotonic() - self.goal_started
            goal.action = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                           GoalStatus.STATUS_ABORTED: 'ABORTED',
                           GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(result.status, 'UNKNOWN')
            if not self.snapshot(goal):
                self.get_logger().warning(f'{goal.name}: pose unavailable; recording NaN')
            # Action protocol status is authoritative; pose errors are diagnostics only.
            if goal.action != 'SUCCEEDED':
                goal.reason = 'action_failed'
            elif self.expired():
                goal.reason = 'total_timeout'
            else:
                goal.verdict, goal.reason = 'PASS', 'action_succeeded'
            self.print_goal(goal)
            if goal.verdict != 'PASS':
                return self.finish(False, goal.reason)
            self.current = None
        return self.finish(True, 'all_goals_succeeded')

    def fail_current(self, reason):
        if self.current is not None:
            self.current.verdict, self.current.reason = 'FAIL', reason
            self.current.elapsed = time.monotonic() - self.goal_started
            self.snapshot(self.current)
        self.cancel()
        return self.finish(False, reason)

    @staticmethod
    def print_goal(goal):
        print(f'{goal.name} {goal.verdict} action={goal.action} reason={goal.reason} '
              f'elapsed={goal.elapsed:.3f}s target=({goal.pose[:3]},yaw={goal.target_yaw:.6f}) '
              f'measured=({goal.x:.6f},{goal.y:.6f},{goal.z:.6f},yaw={goal.heading:.6f}) '
              f'error=(xy={goal.xy_error:.6f}m,z={goal.z_error:.6f}m,yaw={goal.yaw_error:.6f}rad)', flush=True)

    def finish(self, passed, reason):
        print(f'=== Navigation summary: {"PASS" if passed else "FAIL"}; '
              f'reason={reason}; total={self.elapsed():.3f}s ===', flush=True)
        for goal in self.goals:
            self.print_goal(goal)
        try:
            with open(self.report, 'w', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                writer.writerow(['goal', 'verdict', 'action', 'reason', 'elapsed_sec',
                                 'target_x', 'target_y', 'target_z', 'target_yaw',
                                 'x', 'y', 'z', 'yaw', 'xy_error', 'z_error', 'yaw_error'])
                writer.writerows(goal.row() for goal in self.goals)
            print(f'Result file: {self.report}', flush=True)
        except OSError as error:
            print(f'Could not write result file: {error}', file=sys.stderr, flush=True)
            passed = False
        # Match LeGO-LOAM: completed runs return 0; launch_testing grades stdout.
        print('DoneSuccess' if passed else 'DoneFailed', flush=True)
        return 0


def main(args=None):
    node = None
    code = 1
    try:
        rclpy.init(args=args)
        node = NavigationTest()
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            code = node.fail_current('interrupted')
        else:
            print('DoneFailed', flush=True)
    except Exception as error:
        print(f'Navigation test error: {error}', file=sys.stderr, flush=True)
        if node is not None:
            code = node.fail_current(f'exception: {error}')
        else:
            print('DoneFailed', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
