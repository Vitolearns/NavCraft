#!/usr/bin/env python3
"""Verify managed launch exclusivity, shutdown, and same-domain restart."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '190')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    output = Path('artifacts/navcraft-lifecycle')
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('navcraft_lifecycle_check')
    samples = []
    subscription = node.create_subscription(Twist, '/cmd_vel', lambda msg: samples.append(msg), 10)
    checks = []
    processes = []
    streams = []

    def check(name, value):
        assert value, name
        checks.append(name)
        print('PASS', name, flush=True)

    def wait(predicate, seconds=30):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def launch(name):
        stream = (output / (name + '.log')).open('w')
        streams.append(stream)
        process = subprocess.Popen(['bash', 'scripts/demo.sh', 'up',
                                    'gui:=false', 'use_rviz:=false'],
                                   stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
        process.wait(timeout=35)

    def wait_active():
        client = node.create_client(GetState, '/bt_navigator/get_state')
        try:
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=0.5):
                    continue
                future = client.call_async(GetState.Request())
                if wait(future.done, 2) and future.result() is not None:
                    if future.result().current_state.id == 3:
                        return True
            return False
        finally:
            node.destroy_client(client)

    try:
        first = launch('first')
        check('first navigation stack active', wait_active())
        check('first managed launch publishes velocity', wait(lambda: len(samples) > 5))
        check('one final velocity publisher', node.count_publishers('/cmd_vel') == 1)
        duplicate = launch('duplicate')
        check('duplicate launch rejected', duplicate.wait(timeout=10) != 0)
        check('duplicate reports active domain lease',
              'already has a managed demo' in (output / 'duplicate.log').read_text())
        stop(first)
        check('shutdown removes final velocity publisher', wait(lambda: node.count_publishers('/cmd_vel') == 0))
        with Path(f'/tmp/navcraft-domain-{os.environ["ROS_DOMAIN_ID"]}.lock').open('a') as lease:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            check('domain lease released after shutdown', True)
        # Recreate the volatile subscription to discard queued pre-shutdown data.
        node.destroy_subscription(subscription)
        samples.clear()
        subscription = node.create_subscription(Twist, '/cmd_vel', lambda msg: samples.append(msg), 10)
        restarted = launch('restart')
        check('restarted navigation stack active', wait_active())
        samples.clear()
        check('same-domain restart produces fresh velocity', wait(lambda: len(samples) > 5))
        check('restart has exactly one final velocity publisher', node.count_publishers('/cmd_vel') == 1)
        stop(restarted)
        check('restart shutdown removes final velocity publisher', wait(lambda: node.count_publishers('/cmd_vel') == 0))
    finally:
        for process in processes:
            if process.poll() is None:
                stop(process)
        for stream in streams:
            stream.close()
        (output / 'checks.json').write_text(json.dumps(checks, indent=2) + '\n')
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
