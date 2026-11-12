"""Drive the production EKF config with independent wheel and gyro inputs."""
import time
import unittest

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry
import rclpy
from sensor_msgs.msg import Imu


def generate_test_description():
    config = PathJoinSubstitution([
        FindPackageShare('navcraft_simulation'), 'config', 'navigation_ekf.yaml'])
    ekf = Node(package='robot_localization', executable='ekf_node', name='navigation_ekf',
               parameters=[config, {'publish_tf': False}],
               remappings=[('/wheel/odom', '/test_navigation_ekf/wheel'),
                           ('/imu/data', '/test_navigation_ekf/imu'),
                           ('odometry/filtered', '/test_navigation_ekf/filtered')])
    return LaunchDescription([ekf, ReadyToTest()])


class TestWheelImuFusion(unittest.TestCase):
    def test_gyro_changes_filtered_yaw_rate(self):
        rclpy.init()
        node = rclpy.create_node('check_navigation_ekf')
        try:
            wheels = node.create_publisher(Odometry, '/test_navigation_ekf/wheel', 10)
            gyro = node.create_publisher(Imu, '/test_navigation_ekf/imu', 10)
            estimates = []
            sub = node.create_subscription(Odometry, '/test_navigation_ekf/filtered',
                                           estimates.append, 10)
            odom = Odometry()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = 0.2
            odom.twist.covariance[0] = odom.twist.covariance[7] = 0.0025
            odom.twist.covariance[35] = 0.0225
            imu = Imu()
            imu.header.frame_id = 'base_footprint'
            imu.angular_velocity_covariance[8] = 0.0004

            for sign in (1, -1):
                imu.angular_velocity.z = sign * 0.45
                end = time.monotonic() + 10
                while time.monotonic() < end:
                    stamp = node.get_clock().now().to_msg()
                    imu.header.stamp = odom.header.stamp = stamp
                    wheels.publish(odom)
                    gyro.publish(imu)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if estimates:
                        twist = estimates[-1].twist.twist
                        if abs(twist.linear.x - 0.2) < 0.04 and sign * twist.angular.z > 0.30:
                            break
                    time.sleep(0.02)
                else:
                    self.fail(f'No fused estimate followed gyro sign {sign} and wheel speed')
            node.destroy_subscription(sub)
        finally:
            node.destroy_node()
            rclpy.shutdown()
