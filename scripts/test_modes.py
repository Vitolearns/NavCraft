#!/usr/bin/env python3
"""Runtime checks for NavCraft modes; mock and Gazebo are labeled."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import SetBool, Trigger
from navcraft_interfaces.action import NavigateSequence
from navcraft_interfaces.srv import ListMaps

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['pure_pursuit', 'stanley', 'gazebo_stanley', 'gazebo_pure_pursuit', 'slam_toolbox', 'cartographer', 'navigation', 'ros2_control'])
parser.add_argument('--output', default='artifacts/navcraft-modes/runtime')
args = parser.parse_args()
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('navcraft_mode_check')
seen = {}
subscriptions = []
checks = []
processes = []
streams = []

def check(name, value):
    assert value, name
    checks.append(name)
    print('PASS', name, flush=True)

def wait(predicate, seconds=30):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False

def future(f, seconds=30):
    assert wait(f.done, seconds), 'future timed out'
    return f.result()

def sub(topic, kind, transient=False):
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    if transient:
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    subscriptions.append(node.create_subscription(kind, topic, lambda m: seen.__setitem__(topic, m), qos))

def start(command, name):
    stream = (out / (name + '.log')).open('w')
    streams.append(stream)
    processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))

def service(kind, topic, request):
    client = node.create_client(kind, topic)
    assert client.wait_for_service(timeout_sec=30), topic
    return future(client.call_async(request))

try:
    sub('/odom', Odometry)
    sub('/cmd_vel', Twist)
    if args.mode == 'ros2_control':
        from geometry_msgs.msg import TwistStamped
        from controller_manager_msgs.srv import ListControllers
        sub('/diff_drive_controller/odom', Odometry)
        start(['ros2', 'launch', 'navcraft_hardware',
               'ros2_control_hardware.launch.py', 'backend:=mock'], 'ros2_control')
        check('controller odometry received', wait(lambda: '/diff_drive_controller/odom' in seen, 40))
        active = False
        until = time.monotonic() + 30
        while time.monotonic() < until:
            controllers = service(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
            active = {'joint_state_broadcaster', 'diff_drive_controller'} <= {c.name for c in controllers.controller if c.state == 'active'}
            if active: break
        check('both controllers active', active)
        topics = dict(node.get_topic_names_and_types())
        topic = '/diff_drive_controller/cmd_vel'
        stamped = 'geometry_msgs/msg/TwistStamped' in topics.get(topic, [])
        if not stamped:
            topic = '/diff_drive_controller/cmd_vel_unstamped'
        pub = node.create_publisher(TwistStamped if stamped else Twist, topic, 10)
        check('controller command subscriber connected', wait(lambda: pub.get_subscription_count() > 0, 10))
        p0 = seen['/diff_drive_controller/odom'].pose.pose.position
        until = time.monotonic() + 3
        while time.monotonic() < until:
            msg = TwistStamped() if stamped else Twist()
            if stamped:
                msg.header.stamp = node.get_clock().now().to_msg(); msg.twist.linear.x = 0.2
            else:
                msg.linear.x = 0.2
            pub.publish(msg); rclpy.spin_once(node, timeout_sec=0.05)
        p1 = seen['/diff_drive_controller/odom'].pose.pose.position
        check('ros2_control mock moved over 0.2 m', math.hypot(p1.x - p0.x, p1.y - p0.y) > 0.2)
        check('controller command timeout stops wheels', wait(lambda:
            abs(seen['/diff_drive_controller/odom'].twist.twist.linear.x) < 0.001, 2))
    elif args.mode in ('pure_pursuit', 'stanley', 'gazebo_stanley', 'gazebo_pure_pursuit'):
        gazebo = args.mode.startswith('gazebo_')
        backend = 'Gazebo' if gazebo else 'mock'
        launch_args = ['navcraft.launch.py', 'mode:=tracking', 'gui:=false', 'controller:=' + args.mode.removeprefix('gazebo_')] if gazebo else [
            'tracking.launch.py', 'use_mock_chassis:=true', 'controller:=' + args.mode]
        start(['ros2', 'launch', 'navcraft_bringup'] + launch_args + [
               'output_csv:=' + str(out / (args.mode + '.csv')),
               'output_json:=' + str(out / (args.mode + '.json'))], args.mode)
        check(backend + ' odometry received', wait(lambda: '/odom' in seen, 45))
        p0 = seen['/odom'].pose.pose.position
        check(backend + ' tracking moved over 0.5 m', wait(lambda: math.hypot(
            seen['/odom'].pose.pose.position.x - p0.x,
            seen['/odom'].pose.pose.position.y - p0.y) > 0.5, 25))
        check('one final velocity publisher', node.count_publishers('/cmd_vel') == 1)
        q = SetBool.Request(); q.data = True
        check('emergency stop accepted', service(SetBool, '/enable_emergency_stop', q).success)
        check('emergency stop zero command', wait(lambda: '/cmd_vel' in seen and
            abs(seen['/cmd_vel'].linear.x) + abs(seen['/cmd_vel'].angular.z) < 1e-8, 3))
        p1 = seen['/odom'].pose.pose.position
        end = time.monotonic() + 1
        wait(lambda: time.monotonic() >= end, 2)
        check(backend + ' chassis held after stop', math.hypot(
            seen['/odom'].pose.pose.position.x - p1.x,
            seen['/odom'].pose.pose.position.y - p1.y) < 0.08)
    elif args.mode in ('slam_toolbox', 'cartographer'):
        sub('/map', OccupancyGrid, True)
        start(['ros2', 'launch', 'navcraft_bringup', 'navcraft.launch.py',
               'mode:=slam', 'gui:=false', 'slam_mode:=' + args.mode], args.mode)
        check('Gazebo odometry received', wait(lambda: '/odom' in seen, 45))
        check('SLAM occupancy map received', wait(lambda: '/map' in seen and
            len(seen['/map'].data) > 100 and any(x >= 0 for x in seen['/map'].data), 60))
        check('one final velocity publisher', node.count_publishers('/cmd_vel') == 1)
        metadata = seen['/map'].info
        print('MAP', metadata.width, metadata.height, metadata.resolution, flush=True)
    else:
        start(['ros2', 'launch', 'navcraft_bringup', 'navcraft.launch.py',
               'mode:=navigation', 'gui:=false'], 'navigation')
        check('Gazebo odometry received', wait(lambda: '/odom' in seen, 45))
        maps = service(ListMaps, '/list_maps', ListMaps.Request())
        check('map catalog contains indoor_room', maps.success and 'indoor_room' in maps.names)
        client = ActionClient(node, NavigateSequence, '/navigate_sequence')
        check('waypoint action available', client.wait_for_server(timeout_sec=30))
        # Allow Nav2 lifecycle activation, then require genuine navigation success.
        from lifecycle_msgs.srv import GetState
        lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
        check('Nav2 lifecycle available', lifecycle.wait_for_service(timeout_sec=45))
        active = False
        until = time.monotonic() + 45
        while time.monotonic() < until:
            if future(lifecycle.call_async(GetState.Request()), 5).current_state.id == 3:
                active = True; break
        check('Nav2 active', active)
        p0 = seen['/odom'].pose.pose.position
        goal = NavigateSequence.Goal()
        for x in (0.6, 0.0):
            p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x = x
            p.pose.orientation.w = 1.0; goal.goals.append(p)
        handle = future(client.send_goal_async(goal))
        check('sequence accepted', handle.accepted)
        result = future(handle.get_result_async(), 100)
        check('Gazebo sequence succeeded', result.status == 4 and result.result.success and result.result.succeeded_goals == 2)
        motion_goal = NavigateSequence.Goal()
        target = PoseStamped(); target.header.frame_id = 'map'
        target.pose.position.x = 1.3; target.pose.orientation.w = 1.0
        motion_goal.goals.append(target)
        moving = future(client.send_goal_async(motion_goal))
        check('control sequence accepted', moving.accepted)
        moving_result = moving.get_result_async()
        check('control sequence moving', wait(lambda: abs(seen['/cmd_vel'].linear.x) > 0.03, 30))
        check('waypoint pause accepted', service(Trigger, '/pause_mission', Trigger.Request()).success)
        check('waypoint pause stops command', wait(lambda: abs(seen['/cmd_vel'].linear.x) + abs(seen['/cmd_vel'].angular.z) < 0.001, 10))
        held = seen['/odom'].pose.pose.position
        end = time.monotonic() + 1
        wait(lambda: time.monotonic() >= end, 2)
        check('waypoint pause holds odometry', math.hypot(
            seen['/odom'].pose.pose.position.x - held.x, seen['/odom'].pose.pose.position.y - held.y) < 0.08)
        check('waypoint resume accepted', service(Trigger, '/resume_mission', Trigger.Request()).success)
        check('waypoint resumes motion', wait(lambda: abs(seen['/cmd_vel'].linear.x) > 0.03, 30))
        check('waypoint cancel accepted', bool(future(moving.cancel_goal_async()).goals_canceling))
        check('waypoint cancel terminal', future(moving_result, 20).status == 5)
        check('waypoint cancel stops command', wait(lambda: abs(seen['/cmd_vel'].linear.x) + abs(seen['/cmd_vel'].angular.z) < 0.001, 10))
        check('one final velocity publisher', node.count_publishers('/cmd_vel') == 1)
finally:
    for p in processes:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    for p in processes:
        try: p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait()
    for stream in streams: stream.close()
    (out / (args.mode + '-checks.json')).write_text(json.dumps({'mode': args.mode, 'passed': checks}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
