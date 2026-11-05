#!/usr/bin/env python3
"""ROS 2 action fault injection against the compiled C++ executor; no Gazebo."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from navcraft_interfaces.action import NavigateSequence
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


def wait(predicate, seconds=6):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('condition timed out')


def result(future, seconds=6):
    wait(future.done, seconds)
    return future.result()


class Harness(Node):
    def __init__(self):
        super().__init__('executor_regression')
        self.group = ReentrantCallbackGroup()
        self.state = None
        self.states = []
        self.mode = 'success'
        self.attempts = self.active = self.max_active = 0
        self.subscription = self.create_subscription(String, '/execution_state', self.on_state, 10)
        self.client = ActionClient(self, NavigateSequence, '/navigate_sequence', callback_group=self.group)
        self.controls = {n: self.create_client(Trigger, f'/{n}_mission', callback_group=self.group)
                         for n in ['pause', 'resume', 'cancel']}
        self.server = None

    def on_state(self, msg):
        self.state = msg.data
        self.states.append(msg.data)

    def start_mock(self):
        self.server = ActionServer(self, NavigateToPose, '/navigate_to_pose', self.execute,
                                   callback_group=self.group, goal_callback=self.goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT)

    def goal(self, _):
        if self.mode == 'late_accept':
            time.sleep(1.2)
        if self.mode == 'delayed_accept':
            time.sleep(0.4)
        return GoalResponse.REJECT if self.mode == 'reject' else GoalResponse.ACCEPT

    def execute(self, handle):
        mode = self.mode
        self.attempts += 1
        attempt = self.attempts
        self.active += 1
        self.max_active = max(self.max_active, self.active)
        try:
            start = time.monotonic()
            while time.monotonic() - start < 8:
                if handle.is_cancel_requested and mode != 'ignore_cancel':
                    time.sleep(0.25)  # Cancellation ACK is not terminal completion.
                    handle.canceled()
                    return NavigateToPose.Result()
                if mode in ['success', 'abort_once', 'abort_always'] and time.monotonic()-start > 0.2:
                    if mode == 'abort_always' or (mode == 'abort_once' and attempt == 1):
                        handle.abort()
                    else:
                        handle.succeed()
                    return NavigateToPose.Result()
                time.sleep(0.01)
            handle.abort()
            return NavigateToPose.Result()
        finally:
            self.active -= 1

    def goal_message(self, count=1):
        g = NavigateSequence.Goal()
        for _ in range(count):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = -1.0
            p.pose.orientation.w = 1.0
            g.goals.append(p)
        return g

    def submit(self, goal=None):
        return result(self.client.send_goal_async(goal or self.goal_message()))

    def service(self, name):
        return result(self.controls[name].call_async(Trigger.Request()))


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    lease = open(f'/tmp/navcraft-domain-{domain}.lock', 'w')
    fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    out = Path('artifacts/navcraft-regression')
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Harness()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    processes = []
    checks = []

    def check(name, condition):
        assert condition, name
        checks.append(name)
        print('PASS', name, flush=True)

    def launch(package, executable, args=()):
        log = open(out / f'{executable}-{len(processes)}.log', 'w')
        proc = subprocess.Popen(['ros2', 'run', package, executable, *args], stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append((proc, log))
        return proc

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=3)

    def start_executor():
        node.state = None
        p = launch('navcraft_mission', 'navigate_sequence_server_node', ['--ros-args',
                   '-p', 'server_timeout_s:=0.8', '-p', 'response_timeout_s:=0.6',
                   '-p', 'cancel_timeout_s:=0.7', '-p', 'goal_timeout_s:=2.0'])
        assert node.client.wait_for_server(timeout_sec=10)
        for client in node.controls.values():
            assert client.wait_for_service(timeout_sec=5)
        wait(lambda: node.state == 'IDLE')
        return p

    try:
        proc = start_executor()
        h = node.submit()
        r = result(h.get_result_async())
        check('missing Nav2 aborts; never simulates success', r.status == 6 and not r.result.success)
        node.start_mock()
        time.sleep(0.5)
        for invalid in ['empty', 'loop', 'frame', 'nan', 'quaternion', 'too_many']:
            g = node.goal_message(101 if invalid == 'too_many' else 1)
            if invalid == 'empty': g.goals = []
            if invalid == 'loop': g.loop = True
            if invalid == 'frame': g.goals[0].header.frame_id = 'odom'
            if invalid == 'nan': g.goals[0].pose.position.x = float('nan')
            if invalid == 'quaternion': g.goals[0].pose.orientation.w = 0.0
            check('reject invalid '+invalid, not node.submit(g).accepted)
        node.mode = 'success'
        h = node.submit(node.goal_message(2))
        r = result(h.get_result_async())
        check('two real action results complete sequence', r.status == 4 and r.result.succeeded_goals == 2)
        node.mode = 'delayed_accept'
        h = node.submit()
        wait(lambda: node.state == 'SENDING')
        check('pause while acceptance pending', node.service('pause').message == 'PAUSING')
        wait(lambda: node.state == 'PAUSED')
        check('late accepted goal stopped before PAUSED', node.active == 0)
        result(h.cancel_goal_async())
        check('cancel while paused completes', result(h.get_result_async()).status == 5)
        node.mode = 'hold'
        node.attempts = 0
        h = node.submit()
        wait(lambda: node.state == 'RUNNING')
        check('busy submission rejected', not node.submit().accepted)
        check('pause acknowledged as PAUSING', node.service('pause').message == 'PAUSING')
        check('resume during cancellation rejected', not node.service('resume').success)
        wait(lambda: node.state == 'PAUSED')
        check('paused only after Nav2 terminal result', node.active == 0)
        check('duplicate pause is idempotent', node.service('pause').success)
        check('resume accepted', node.service('resume').success)
        wait(lambda: node.state == 'RUNNING')
        result(h.cancel_goal_async())
        r = result(h.get_result_async())
        check('outer action cancellation terminal is CANCELED', r.status == 5 and not r.result.success)
        check('no overlapping Nav2 goals', node.max_active == 1)
        check('resume at IDLE rejected', not node.service('resume').success)
        for mode, success, attempts in [('reject', False, 0), ('abort_once', True, 2), ('abort_always', False, 2)]:
            node.mode, node.attempts = mode, 0
            h = node.submit()
            r = result(h.get_result_async())
            check(mode, r.result.success == success and node.attempts == attempts)
        node.mode = 'hold'
        h = node.submit()
        r = result(h.get_result_async(), 6)
        check('execution timeout cancels underlying action', r.status == 6 and node.active == 0)
        node.mode = 'success'
        h = node.submit()
        check('new task after clean failure succeeds', result(h.get_result_async()).result.success)
        node.mode = 'late_accept'
        previous_attempts = node.attempts
        h = node.submit()
        r = result(h.get_result_async())
        wait(lambda: node.state == 'FAULT')
        check('acceptance timeout faults closed', not r.result.success and not node.submit().accepted)
        wait(lambda: node.active == 0 and node.attempts > previous_attempts, 4)
        time.sleep(0.4)
        check('late acceptance cancelled without clearing fault', node.active == 0 and node.state == 'FAULT')
        stop(proc)
        time.sleep(0.3)
        proc = start_executor()
        node.mode = 'ignore_cancel'
        h = node.submit()
        wait(lambda: node.state == 'RUNNING')
        node.service('cancel')
        r = result(h.get_result_async())
        wait(lambda: node.state == 'FAULT')
        check('cancel timeout faults and rejects new work', not r.result.success and not node.submit().accepted)
        stop(proc)

        # Test the velocity gate independently, using controlled heartbeat and velocity publishers.
        gate = launch('navcraft_safety', 'cmd_vel_safety_gate_node')
        velocity = []
        subscription = node.create_subscription(Twist, '/cmd_vel', lambda m: velocity.append(m.linear.x), 10)
        state_pub = node.create_publisher(String, '/execution_state', 10)
        cmd_pub = node.create_publisher(Twist, '/nav2_cmd_vel', 10)
        estop = node.create_client(SetBool, '/emergency_stop', callback_group=node.group)
        assert estop.wait_for_service(timeout_sec=5)
        wait(lambda: len(velocity) > 3)
        check('gate starts stopped', velocity[-1] == 0)
        def drive(state='RUNNING', x=0.3):
            for _ in range(12):
                state_pub.publish(String(data=state))
                m = Twist(); m.linear.x = x; cmd_pub.publish(m)
                time.sleep(0.03)
        drive()
        check('gate forwards fresh running command', abs(velocity[-1]-0.3)<1e-6)
        time.sleep(0.45)
        check('heartbeat and command loss stops output', velocity[-1] == 0)
        drive()
        for _ in range(16):
            m = Twist(); m.linear.x = 0.3; cmd_pub.publish(m); time.sleep(0.03)
        check('state heartbeat loss alone stops output', velocity[-1] == 0)
        drive()
        for _ in range(16):
            state_pub.publish(String(data='RUNNING')); time.sleep(0.03)
        check('command loss alone stops output', velocity[-1] == 0)
        drive('PAUSING')
        check('PAUSING stops output', velocity[-1] == 0)
        result(estop.call_async(SetBool.Request(data=True)))
        drive()
        check('latched emergency stop blocks output', velocity[-1] == 0)
        result(estop.call_async(SetBool.Request(data=False)))
        drive(x=float('nan'))
        check('non-finite command blocked', velocity[-1] == 0)
        drive(x=3.0)
        check('velocity bounded', abs(velocity[-1]-0.5)<1e-6)
        stop(gate)
        (out / 'result.json').write_text(json.dumps({'passed': checks, 'count': len(checks)}, indent=2)+'\n')
    finally:
        for proc, log in reversed(processes):
            stop(proc)
            log.close()
        executor.shutdown(timeout_sec=12)
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2)


if __name__ == '__main__':
    main()
