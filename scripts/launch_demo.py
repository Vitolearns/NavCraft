#!/usr/bin/env python3
"""Hold the domain lease until the launch process group has stopped."""
import fcntl
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time


def signal_group(group, sig):
    try:
        os.killpg(group, sig)
        return True
    except ProcessLookupError:
        return False


def supervise(exit_pipe, lease, arguments):
    # The parent owns the only write end. Even SIGKILL closes it.
    process = subprocess.Popen(
        ['ros2', 'launch', 'navcraft_bringup', 'navcraft.launch.py', *arguments],
        start_new_session=True,
    )
    try:
        while process.poll() is None:
            if select.select([exit_pipe], [], [], 0.25)[0]:
                break
    finally:
        for sig, grace in [(signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)]:
            if not signal_group(process.pid, sig):
                break
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                process.poll()
                if not signal_group(process.pid, 0):
                    break
                time.sleep(0.1)
        process.wait()
        os.close(exit_pipe)
        lease.close()
    return process.returncode


def main():
    domain = int(os.environ['ROS_DOMAIN_ID'])
    lease = Path(f'/tmp/navcraft-domain-{domain}.lock').open('a')
    try:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit(f'ROS domain {domain} already has a managed demo; stop it before restarting.')

    read_fd, write_fd = os.pipe()
    # Block until the child has inherited the descriptors and the parent
    # has installed its shutdown handlers.
    signals = {signal.SIGINT, signal.SIGTERM, signal.SIGHUP}
    previous = signal.pthread_sigmask(signal.SIG_BLOCK, signals)
    pid = os.fork()
    if pid == 0:
        os.close(write_fd)
        os.setsid()
        signal.pthread_sigmask(signal.SIG_SETMASK, previous)
        return supervise(read_fd, lease, sys.argv[1:])

    os.close(read_fd)
    lease.close()
    writer = os.fdopen(write_fd, 'wb')
    def stop(_sig, _frame):
        writer.close()
    for sig in signals:
        signal.signal(sig, stop)
    signal.pthread_sigmask(signal.SIG_SETMASK, previous)
    try:
        _, status = os.waitpid(pid, 0)
    finally:
        writer.close()
    return os.waitstatus_to_exitcode(status)


if __name__ == '__main__':
    sys.exit(main())
