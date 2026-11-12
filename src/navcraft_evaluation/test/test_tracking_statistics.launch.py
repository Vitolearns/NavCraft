"""Verify live tracking metrics and their CSV/JSON exports with known samples."""
import csv
import json
from pathlib import Path
import shutil
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from navcraft_interfaces.msg import TrackingError, TrackingStatistics
import rclpy


def generate_test_description():
    output_dir = tempfile.mkdtemp(prefix='navcraft-statistics-test-')
    evaluator = Node(
        package='navcraft_evaluation', executable='experiment_runner_node',
        name='test_tracking_statistics',
        parameters=[{'controller_name': 'configured_controller',
                     'output_csv': str(Path(output_dir) / 'samples.csv'),
                     'output_json': str(Path(output_dir) / 'summary.json')}],
        remappings=[('/tracking_error', '/test_tracking_statistics/input'),
                    ('/experiment/statistics', '/test_tracking_statistics/output')])
    return LaunchDescription([evaluator, ReadyToTest()]), {'output_dir': output_dir}


class TestLiveStatistics(unittest.TestCase):
    def test_sample_counts_and_error_units(self):
        rclpy.init()
        node = rclpy.create_node('check_tracking_statistics')
        try:
            publisher = node.create_publisher(
                TrackingError, '/test_tracking_statistics/input', 10)
            received = []
            node.create_subscription(
                TrackingStatistics, '/test_tracking_statistics/output', received.append, 10)

            def wait_for(predicate):
                deadline = time.monotonic() + 8.0
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if predicate():
                        return
                self.fail('tracking statistics did not match the published samples')

            wait_for(lambda: publisher.get_subscription_count() == 1 and received)
            initial = received[-1]
            self.assertEqual(initial.samples, 0)
            self.assertEqual(initial.controller_name, 'configured_controller')
            self.assertEqual(initial.last_nearest_path_index, -1)
            self.assertEqual(initial.mean_abs_lateral_error_m, 0.0)
            self.assertEqual(initial.mean_abs_heading_error_rad, 0.0)
            for index, (lateral, heading, speed) in enumerate(
                    ((0.4, 0.3, 0.0), (-0.2, -0.1, 0.14), (0.0, 0.2, -0.08))):
                sample = TrackingError()
                sample.header.stamp = node.get_clock().now().to_msg()
                sample.header.frame_id = 'odom'
                sample.controller_name = 'measured_controller'
                sample.lateral_error = lateral
                sample.heading_error = heading
                sample.target_speed = 0.35
                sample.actual_speed = speed
                sample.nearest_path_index = index
                publisher.publish(sample)
                wait_for(lambda: received[-1].samples >= index + 1)
                self.assertEqual(received[-1].samples, index + 1)
            summary = received[-1]
            self.assertEqual(summary.controller_name, 'measured_controller')
            self.assertEqual(summary.last_nearest_path_index, 2)
            self.assertAlmostEqual(summary.mean_abs_lateral_error_m, 0.2)
            self.assertAlmostEqual(summary.mean_abs_heading_error_rad, 0.2)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestExportedStatistics(unittest.TestCase):
    def test_csv_and_json_match_samples(self, output_dir):
        try:
            with (Path(output_dir) / 'samples.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3)
            self.assertEqual([float(row['actual_speed']) for row in rows], [0.0, 0.14, -0.08])
            report = json.loads((Path(output_dir) / 'summary.json').read_text())
            self.assertEqual(report['samples'], 3)
            self.assertEqual(report['controller_name'], 'measured_controller')
            self.assertEqual(report['last_nearest_path_index'], 2)
            self.assertAlmostEqual(report['mean_abs_lateral_error'], 0.2)
            self.assertAlmostEqual(report['mean_abs_heading_error'], 0.2)
            self.assertAlmostEqual(report['sum_abs_lateral_error'], 0.6)
            self.assertAlmostEqual(report['sum_abs_heading_error'], 0.6)
        finally:
            shutil.rmtree(output_dir)
