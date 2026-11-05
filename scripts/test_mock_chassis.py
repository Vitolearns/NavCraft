#!/usr/bin/env python3
"""Verify ROS commands and wheel feedback on a private mock chassis."""
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from navcraft_interfaces.msg import ChassisState


def main():
    # Launch and test share a private domain, separate from the demo.
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('WAYPOINT_MOCK_DOMAIN', '182')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    output = Path('artifacts/single-robot')
    output.mkdir(parents=True, exist_ok=True)
    with (output / 'mock-launch.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'navcraft_hardware', 'hardware.launch.py', 'backend:=mock'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('mock_chassis_acceptance')
        latest = {}
        received = {}
        counts = {}
        def receive(key, msg):
            latest[key] = msg
            received[key] = time.monotonic()
            counts[key] = counts.get(key, 0) + 1
        for topic, kind in [('/wheel/odom', Odometry), ('/chassis/state', ChassisState)]:
            node.create_subscription(kind, topic, lambda msg, key=topic: receive(key, msg), 10)
        publisher = node.create_publisher(Twist, '/cmd_vel', 10)

        def until(predicate, command=None, timeout=15):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert process.poll() is None, 'mock launch exited'
                if command is not None:
                    publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            raise AssertionError('timeout waiting for mock chassis feedback')
        try:
            until(lambda: len(latest)==2)
            state = latest['/chassis/state']
            assert state.backend == 'mock' and state.connected
            assert publisher.get_subscription_count() == 1, 'expected exactly one mock drive'
            start = latest['/wheel/odom'].pose.pose.position
            command = Twist()
            command.linear.x = 0.2
            until(lambda: math.dist((start.x,start.y), (latest['/wheel/odom'].pose.pose.position.x,latest['/wheel/odom'].pose.pose.position.y)) > 0.10, command)
            checks = ['mock connected', 'travelled > 0.10 m']
            def speed():
                t = latest['/wheel/odom'].twist.twist
                return t.linear.x, t.linear.y, t.angular.z

            def stopped():
                return all(math.isfinite(v) and abs(v) < 0.001 for v in speed())

            # Require feedback received after the trigger, never a cached startup zero.
            at = time.monotonic()
            until(lambda: received['/wheel/odom'] > at and stopped(), Twist())
            checks.append('explicit zero stops fresh wheel feedback')
            command.linear.x = -0.2
            command.angular.z = 0.3
            until(lambda: speed()[0] < -0.1 and speed()[2] > 0.2, command)
            checks.append('reverse and turning feedback')
            at = time.monotonic()
            until(lambda: received['/wheel/odom'] > at and stopped(), timeout=1.5)
            stop_latency = time.monotonic() - at
            checks.append('command silence stops within 1.5 seconds')
            pose = latest['/wheel/odom'].pose.pose.position
            start_count = counts['/wheel/odom']
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                p = latest['/wheel/odom'].pose.pose.position
                assert stopped() and math.dist((pose.x, pose.y), (p.x, p.y)) < 0.01
            assert counts['/wheel/odom'] > start_count + 5
            checks.append('fresh stationary feedback throughout silence window')
            command.linear.x = 0.2
            command.angular.z = 0.0
            until(lambda: speed()[0] > 0.1, command)
            checks.append('fresh command resumes after timeout')
            invalid = Twist()
            invalid.linear.x = float('nan')
            at = time.monotonic()
            until(lambda: received['/wheel/odom'] > at and stopped(), invalid, timeout=1.5)
            checks.append('nonfinite command stops without corrupting odometry')
            p = latest['/wheel/odom'].pose.pose.position
            assert all(math.isfinite(v) for v in (p.x, p.y, p.z))
            report = {'passed': checks, 'count': len(checks),
                      'silence_stop_latency_s': stop_latency,
                      'ros_domain_id': os.environ['ROS_DOMAIN_ID'], 'backend': 'mock'}
            (output / 'mock-result.json').write_text(json.dumps(report, indent=2) + '\n')
            for check in checks:
                print('PASS', check, flush=True)
            print('SILENCE_STOP_LATENCY_S', stop_latency, flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == '__main__':
    main()
