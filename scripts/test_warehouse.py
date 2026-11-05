#!/usr/bin/env python3
"""Gazebo acceptance: online obstacle insertion, replanning and six warehouse goals."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import GetState
from navcraft_interfaces.action import NavigateSequence
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from test_simulation import wait, result
from warehouse_obstacle import move_obstacle
from warehouse_route import STATIONS


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '186')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['GZ_PARTITION'] = 'navcraft-' + os.environ['ROS_DOMAIN_ID']
    out = Path('artifacts/navcraft-warehouse')
    out.mkdir(parents=True, exist_ok=True)
    report_path = out/'result.json'
    report_path.unlink(missing_ok=True)
    log = (out/'launch.log').open('w')
    launch = subprocess.Popen(['bash','scripts/demo.sh','up','gui:=false','use_rviz:=false'],
                              start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    ground_truth = subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/model/mobile_robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '--ros-args', '-r', '/model/mobile_robot/pose:=/warehouse_ground_truth',
    ], start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
    node = Node('warehouse_acceptance')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    positions, plans, velocities = [], [], []
    odometry, pose_frames = [], set()
    truth_received = [0.0]
    def record_truth(message):
        for transform in message.transforms:
            pose_frames.add(transform.child_frame_id)
            if transform.child_frame_id == 'mobile_robot':
                assert transform.header.frame_id == 'indoor_room', transform.header.frame_id
                t=transform.transform.translation
                positions.append((t.x,t.y))
                truth_received[0] = time.monotonic()
    subscriptions = [
        node.create_subscription(Odometry, '/odom', lambda m: odometry.append((m.pose.pose.position.x,m.pose.pose.position.y)),10),
        node.create_subscription(TFMessage, '/warehouse_ground_truth', record_truth,10),
        node.create_subscription(NavPath, '/plan', lambda m: plans.append((time.monotonic(),[(p.pose.position.x,p.pose.position.y) for p in m.poses])),10),
        node.create_subscription(Twist, '/cmd_vel', lambda m: velocities.append((m.linear.x,m.angular.z)),10),
    ]
    client = ActionClient(node, NavigateSequence, '/navigate_sequence')
    checks, endpoints = [], []
    blocked = False
    def check(name, ok):
        assert ok, name
        checks.append(name)
        print('PASS', name, flush=True)
    def submit(point):
        goal = NavigateSequence.Goal()
        p = PoseStamped(); p.header.frame_id='map'
        p.pose.position.x,p.pose.position.y=point; p.pose.orientation.w=1.0
        goal.goals.append(p)
        handle=result(client.send_goal_async(goal))
        assert handle.accepted, 'waypoint rejected'
        return handle
    costmap = node.create_client(GetCostmap, '/local_costmap/get_costmap')
    def endpoint(name, point):
        wait(lambda: time.monotonic()-truth_received[0] < .5, 5)
        tf=buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
        xy=(tf.transform.translation.x,tf.transform.translation.y)
        endpoints.append({'station':name,'goal':point,'map_pose':xy,
                          'world_pose':positions[-1], 'odom_pose':odometry[-1]})
        check(name+' endpoint within 0.35m',math.dist(xy,point)<.35)
        check(name+' Gazebo world pose within 0.35m',math.dist(positions[-1],point)<.35)
    def obstacle_marked():
        # Query a fresh full snapshot: /costmap normally publishes incremental updates.
        m=result(costmap.call_async(GetCostmap.Request())).map
        assert m.header.frame_id == 'odom', m.header.frame_id
        info=m.metadata
        # Lidar marks the visible box surface, not necessarily its hidden centre.
        for row in range(info.size_y):
            y=info.origin.position.y+(row+.5)*info.resolution
            if abs(y)>.32: continue
            for col in range(info.size_x):
                x=info.origin.position.x+(col+.5)*info.resolution
                if abs(x+1.5)<.32 and int(m.data[row*info.size_x+col])==254:
                    return True
        time.sleep(.25)
        return False
    try:
        assert client.wait_for_server(timeout_sec=30)
        for name in ['amcl','controller_server','planner_server','bt_navigator']:
            srv=node.create_client(GetState,'/'+name+'/get_state')
            assert srv.wait_for_service(timeout_sec=60), name
            deadline=time.monotonic()+90
            while time.monotonic()<deadline:
                if result(srv.call_async(GetState.Request())).current_state.id==3:break
                assert launch.poll() is None, 'launch exited'
                time.sleep(.5)
            else: raise AssertionError(name+' not active')
        wait(lambda:len(positions)>10,30)
        check('one final velocity publisher', node.count_publishers('/cmd_vel')==1)
        move_obstacle(False)
        assert costmap.wait_for_service(timeout_sec=10)
        check('insertion site initially clear in live costmap', not obstacle_marked())
        handle=submit(STATIONS[0][1])
        wait(lambda:bool(plans),30)
        baseline=plans[-1][1]
        check('initial path traverses obstacle insertion site', min(math.hypot(x+1.5,y) for x,y in baseline)<.25)
        wait(lambda:math.hypot(*positions[-1])>.10,30)
        check('obstacle inserted with robot over 0.9m away', math.dist(positions[-1],(-1.5,0))>.9)
        blocked=True
        move_obstacle(True)
        inserted=time.monotonic()
        wait(obstacle_marked,30)
        check('lidar obstacle appears in local costmap',True)
        def detour():
            return any(t>inserted and len(p)>5 and min(math.hypot(x+1.5,y) for x,y in p)>.40
                       and any(abs(x+1.5)<.35 and abs(y)>.40 for x,y in p) for t,p in plans)
        wait(detour,60)
        check('global path replans around inserted obstacle',True)
        received=result(handle.get_result_async(),180)
        check('receiving reached with obstacle present',received.status==4 and received.result.success)
        endpoint(*STATIONS[0])
        clearance=min(math.dist(p,(-1.5,0)) for p in positions)
        check('physical trajectory clears obstacle centre by over 0.40m',clearance>.40)
        move_obstacle(False);blocked=False
        for name, point in STATIONS[1:]:
            handle=submit(point)
            received=result(handle.get_result_async(),180)
            check(name+' navigation succeeds',received.status==4 and received.result.success)
            endpoint(name, point)
        wait(lambda:bool(velocities) and abs(velocities[-1][0])+abs(velocities[-1][1])<1e-6,10)
        check('velocity zero at completion',True)
        report_path.write_text(json.dumps({'passed':checks,'count':len(checks),'endpoints':endpoints,
            'minimum_obstacle_centre_distance_m':clearance,'distance_m':sum(math.dist(a,b) for a,b in zip(positions,positions[1:])),
            'ros_domain_id':os.environ['ROS_DOMAIN_ID']},indent=2)+'\n')
    finally:
        (out/'observations.json').write_text(json.dumps({'passed':checks,'positions':positions,'odometry':odometry,'pose_frames':sorted(pose_frames),'plans':plans,'endpoints':endpoints},indent=2)+'\n')
        if blocked:
            try:move_obstacle(False)
            except Exception as error: print('Obstacle cleanup:',error,flush=True)
        if launch.poll() is None:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=15)
        if ground_truth.poll() is None:
            os.killpg(ground_truth.pid,signal.SIGINT)
            try:ground_truth.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(ground_truth.pid,signal.SIGTERM);ground_truth.wait(timeout=5)
        log.close();executor.shutdown();node.destroy_node();rclpy.shutdown();thread.join(timeout=2)


if __name__=='__main__': main()
