#!/usr/bin/env python3
"""Launch the real Gazebo/Nav2 stack and verify motion, pause/resume and cancel."""
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from navcraft_interfaces.action import NavigateSequence
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def wait(predicate, seconds=10):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        if predicate(): return
        time.sleep(0.02)
    raise AssertionError('simulation condition timed out')


def result(future, seconds=10):
    wait(future.done, seconds)
    return future.result()


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '175')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    out = Path('artifacts/navcraft-simulation')
    out.mkdir(parents=True, exist_ok=True)
    log = open(out/'launch.log', 'w')
    launch = subprocess.Popen(['bash','scripts/demo.sh','up','gui:=false','use_rviz:=false'],
                              start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node('simulation_acceptance')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    states, velocities, positions = [], [], []
    samples_by_topic = {}
    subscriptions = [
        node.create_subscription(String, '/execution_state', lambda m: states.append(m.data), 10),
        node.create_subscription(Twist, '/cmd_vel', lambda m: velocities.append((time.monotonic(),m.linear.x,m.angular.z)), 10),
        node.create_subscription(Odometry, '/odom', lambda m: positions.append((m.pose.pose.position.x,m.pose.pose.position.y)), 10),
    ]
    for topic, kind in [('/scan', LaserScan), ('/imu/data', Imu), ('/sim/camera/image', Image)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, key=topic: samples_by_topic.update({key: msg}), qos_profile_sensor_data))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = ActionClient(node, NavigateSequence, '/navigate_sequence')
    controls = {n: node.create_client(Trigger, '/'+n+'_mission') for n in ['pause','resume']}
    checks = []
    def check(name, ok):
        assert ok, name
        checks.append(name)
        print('PASS',name,flush=True)
    def submit(points):
        g = NavigateSequence.Goal()
        for x,y in points:
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.0
            g.goals.append(p)
        h=result(client.send_goal_async(g))
        assert h.accepted
        return h
    try:
        assert client.wait_for_server(timeout_sec=30), 'executor startup'
        for name in ['amcl','controller_server','planner_server','bt_navigator']:
            service = node.create_client(GetState, '/'+name+'/get_state')
            assert service.wait_for_service(timeout_sec=60), name+' discovery'
            end=time.monotonic()+90
            while time.monotonic()<end:
                if result(service.call_async(GetState.Request())).current_state.id==3: break
                assert launch.poll() is None, 'launch exited'
                time.sleep(0.5)
            else:raise AssertionError(name+' not active')
        wait(lambda: len(positions)>10, 30)
        check('real Nav2 lifecycle active and odometry received',True)
        wait(lambda: len(samples_by_topic) == 3, 30)
        scan, imu, image = [samples_by_topic[k] for k in ('/scan', '/imu/data', '/sim/camera/image')]
        check('lidar supplies finite ranges', any(math.isfinite(r) and scan.range_min <= r <= scan.range_max for r in scan.ranges))
        check('IMU has a normalized orientation in imu_link', imu.header.frame_id == 'imu_link' and abs(sum(v*v for v in (imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w))-1)<0.01)
        check('camera supplies complete 640x480 frames', (image.width,image.height)==(640,480) and len(image.data)==image.step*image.height and image.step>0)
        for msg in (scan, imu, image):
            check('TF from base_link to '+msg.header.frame_id, bool(msg.header.frame_id) and buffer.can_transform('base_link',msg.header.frame_id,rclpy.time.Time()))
        start=positions[-1]
        h=submit([(-1.5,0.0),(0.0,0.0)])
        r=result(h.get_result_async(),180)
        check('real two-waypoint navigation succeeds',r.status==4 and r.result.succeeded_goals==2)
        distance=sum(math.dist(a,b) for a,b in zip(positions,positions[1:]))
        check('robot physically travelled over 2 metres',distance>2.0)
        transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        endpoint = (transform.transform.translation.x, transform.transform.translation.y)
        print('ENDPOINT map=', endpoint, 'odom=', positions[-1], flush=True)
        check('return endpoint in map within 0.3m', math.hypot(*endpoint)<0.3)
        h=submit([(-3.0,0.0)])
        wait(lambda: states[-1]=='RUNNING')
        before=positions[-1]
        wait(lambda: math.dist(before,positions[-1])>0.25,30)
        check('pause request accepted',result(controls['pause'].call_async(Trigger.Request())).success)
        wait(lambda: states[-1]=='PAUSED')
        paused=positions[-1];odom_count=len(positions);at=time.monotonic()
        time.sleep(0.5)
        samples = [(x,z) for t,x,z in velocities.copy() if t > at+0.1]
        check('paused velocity is zero', bool(samples) and all(abs(x)+abs(z)<1e-6 for x,z in samples))
        check('paused odometry stays within 0.08m',len(positions)>odom_count and all(math.dist(paused, pos)<0.08 for pos in positions[odom_count:]))
        check('resume request accepted',result(controls['resume'].call_async(Trigger.Request())).success)
        r=result(h.get_result_async(),150)
        check('resumed navigation succeeds',r.status==4 and r.result.success)
        h=submit([(0.0,0.0)])
        wait(lambda: states[-1]=='RUNNING')
        time.sleep(0.5)
        cancel_at = time.monotonic()
        result(h.cancel_goal_async())
        r=result(h.get_result_async())
        wait(lambda: states[-1]=='IDLE')
        time.sleep(0.15)
        check('real cancellation completes and stops velocity',r.status==5 and velocities[-1][0] > cancel_at and abs(velocities[-1][1])+abs(velocities[-1][2])<1e-6)
        report={'passed':checks,'count':len(checks),'first_sequence_distance_m':distance,
                'start_odom':start,'first_sequence_endpoint_map':endpoint,'final_odom':positions[-1],'ros_domain_id':os.environ['ROS_DOMAIN_ID']}
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    finally:
        (out/'observations.json').write_text(json.dumps({'passed': checks, 'last_odom': positions[-1] if positions else None, 'last_state': states[-1] if states else None}, indent=2)+'\n')
        if launch.poll() is None:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGTERM)
                launch.wait(timeout=15)
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2)


if __name__=='__main__':main()
