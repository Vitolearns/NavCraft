"""Check measured speed independently of commands in both tracking controllers."""
import time
import unittest

from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry, Path
from navcraft_interfaces.msg import TrackingError
import rclpy


CONTROLLERS = ('pure_pursuit', 'stanley')


def generate_test_description():
    nodes = []
    for controller in CONTROLLERS:
        prefix = '/test_tracking_speed/' + controller
        nodes.append(Node(
            package='navcraft_tracking', executable=controller + '_controller_node',
            name='test_speed_' + controller, parameters=[{'target_speed': 0.35}],
            remappings=[('/odom', prefix + '/odom'),
                        ('/reference_path', prefix + '/path'),
                        ('/tracking_cmd_vel', prefix + '/command'),
                        ('/tracking_error', prefix + '/error')]))
    return LaunchDescription(nodes + [ReadyToTest()])


class TestMeasuredSpeed(unittest.TestCase):
    def test_stationary_forward_reverse_and_goal(self):
        rclpy.init()
        node = rclpy.create_node('check_tracking_measured_speed')
        try:
            for controller in CONTROLLERS:
                prefix = '/test_tracking_speed/' + controller
                odom_pub = node.create_publisher(Odometry, prefix + '/odom', 10)
                path_pub = node.create_publisher(Path, prefix + '/path', 10)
                errors, commands = [], []
                error_sub = node.create_subscription(
                    TrackingError, prefix + '/error', errors.append, 10)
                command_sub = node.create_subscription(
                    Twist, prefix + '/command', commands.append, 10)
                path = Path()
                path.header.frame_id = 'odom'
                for x in (0.0, 1.0, 2.0):
                    point = PoseStamped()
                    point.header.frame_id = 'odom'
                    point.pose.position.x = x
                    point.pose.orientation.w = 1.0
                    path.poses.append(point)
                odom = Odometry()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_footprint'
                odom.pose.pose.orientation.w = 1.0
                # Goal arrival commands zero even while measured motion persists.
                for actual_speed, x, command_speed, index in (
                        (0.0, 0.0, 0.35, 0),
                        (0.14, 0.0, 0.35, 0),
                        (-0.08, 0.0, 0.35, 0),
                        (0.12, 2.0, 0.0, 2)):
                    with self.subTest(controller=controller, speed=actual_speed, x=x):
                        odom.twist.twist.linear.x = actual_speed
                        odom.pose.pose.position.x = x
                        errors.clear()
                        commands.clear()
                        deadline = time.monotonic() + 8.0
                        while time.monotonic() < deadline:
                            stamp = node.get_clock().now().to_msg()
                            odom.header.stamp = path.header.stamp = stamp
                            odom_pub.publish(odom)
                            path_pub.publish(path)
                            rclpy.spin_once(node, timeout_sec=0.05)
                            if (errors and commands and
                                    errors[-1].controller_name == controller and
                                    errors[-1].nearest_path_index == index and
                                    abs(errors[-1].actual_speed - actual_speed) < 1e-9 and
                                    abs(commands[-1].linear.x - command_speed) < 1e-9):
                                self.assertAlmostEqual(errors[-1].target_speed, 0.35)
                                break
                        else:
                            self.fail(f'{controller}: expected measured speed {actual_speed}, '
                                      f'command {command_speed}; errors={errors[-1:]}')
                node.destroy_subscription(error_sub)
                node.destroy_subscription(command_sub)
                node.destroy_publisher(odom_pub)
                node.destroy_publisher(path_pub)
        finally:
            node.destroy_node()
            rclpy.shutdown()
