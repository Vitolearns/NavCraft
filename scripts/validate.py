#!/usr/bin/env python3
"""Run the v0.1 acceptance matrix serially in isolated ROS domains."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='artifacts/v0.1')
    parser.add_argument('--domain-base', type=int, default=170)
    args = parser.parse_args()
    if not 1 <= args.domain_base <= 210:
        parser.error('--domain-base must be between 1 and 210')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    checks = [
        ('unit-and-executor', ['bash', 'scripts/test.sh']),
        ('mock-chassis', [sys.executable, 'scripts/test_mock_chassis.py']),
        ('udp-simulator', ['bash', 'scripts/udp_sim_acceptance.sh']),
        ('gazebo-navigation', [sys.executable, 'scripts/test_simulation.py']),
    ]
    for mode in ('ros2_control', 'pure_pursuit', 'stanley', 'gazebo_pure_pursuit',
                 'gazebo_stanley', 'slam_toolbox', 'cartographer', 'navigation'):
        checks.append((mode, [sys.executable, 'scripts/test_modes.py', mode,
                             '--output', str(output / mode)]))
    checks.append(('lifecycle', [sys.executable, 'scripts/test_lifecycle.py']))
    results = []
    for index, (name, command) in enumerate(checks):
        domain = args.domain_base + index
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                   GZ_PARTITION=f'navcraft-acceptance-{domain}',
                   KEEP_ROBOT_LOGS='1')
        print(f'RUN {name} domain={domain}', flush=True)
        started = time.monotonic()
        with (output / (name + '.log')).open('w') as stream:
            result = subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT)
        results.append({'name': name, 'domain': domain, 'command': command,
                        'exit_code': result.returncode,
                        'seconds': round(time.monotonic() - started, 2)})
        (output / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
        print(('PASS ' if result.returncode == 0 else 'FAIL ') + name, flush=True)
        if result.returncode:
            print(f'Inspect {output / (name + ".log")} before resuming.', flush=True)
            return result.returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
