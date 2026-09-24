#!/usr/bin/env python3
"""Own the virtual display lifetime and preserve evidence before cleanup."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import time


def snapshot(directory, label):
    commands = {
        'processes': ['ps', '-eo', 'pid,ppid,pgid,sid,lstart,args'],
        'kernel': ['dmesg', '--ctime'],
        'memory': ['free', '-b'],
    }
    for name, command in commands.items():
        with (directory / f'{label}-{name}.log').open('w') as stream:
            try:
                subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=3)
            except (OSError, subprocess.TimeoutExpired) as error:
                stream.write(str(error))
    paths = [Path('/sys/fs/cgroup')]
    try:
        for line in Path('/proc/self/cgroup').read_text().splitlines():
            if line.startswith('0::'):
                paths.append(Path('/sys/fs/cgroup') / line[3:].lstrip('/'))
    except OSError:
        pass
    values = {}
    for root in paths:
        for name in ('memory.events', 'memory.current', 'memory.max', 'memory.peak'):
            try:
                values[str(root / name)] = (root / name).read_text()
            except OSError as error:
                values[str(root / name)] = str(error)
    (directory / f'{label}-cgroup.json').write_text(json.dumps(values, indent=2))


def stop(process):
    if process is None:
        return
    # Terminate only the group created for this child, even if its leader exited.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', required=True)
    parser.add_argument('--xvfb', default='Xvfb')
    parser.add_argument('--display-number', type=int, default=199)
    parser.add_argument('--no-strace', action='store_true', help='Local tests only')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('command is required after --')
    if args.display_number < 1:
        parser.error('display number must be positive')
    logs = Path(args.log_dir)
    logs.mkdir(parents=True, exist_ok=True)
    diagnostic = logs / 'display-diagnostics'
    diagnostic.mkdir(exist_ok=True)
    result = {'passed': False, 'command': command, 'failure': None}
    server = child = None
    display_lock = None
    reader, writer = os.pipe()
    started = time.monotonic()

    def event(kind, **details):
        record = dict(event=kind, elapsed=time.monotonic() - started,
                      utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), **details)
        with (logs / 'xvfb-events.jsonl').open('a') as stream:
            stream.write(json.dumps(record) + '\n')
        print('DISPLAY ' + json.dumps(record), flush=True)

    def interrupted(number, frame):
        raise InterruptedError(f'Supervisor received signal {number}')

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    code = 1
    try:
        snapshot(diagnostic, 'before')
        display_lock = open(f'/tmp/ci-xvfb-{os.getuid()}-{args.display_number}.lock', 'a')
        fcntl.flock(display_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        server_command = [args.xvfb, ':' + str(args.display_number), '-displayfd', str(writer), '-screen', '0',
                          '1280x720x24', '-nolisten', 'tcp', '-noreset']
        if not args.no_strace:
            if not shutil.which('strace'):
                raise RuntimeError('strace is required; install it in the workflow setup step')
            # Trace signal sender PIDs and Xvfb's actual exit. The owned process
            # is strace, whose exit status mirrors the traced command.
            server_command = ['strace', '-f', '-ttt', '-e', 'trace=process,signal',
                              '-o', str(logs / 'xvfb-strace.log'), *server_command]
        with (logs / 'xvfb.log').open('w') as stream:
            server = subprocess.Popen(server_command, stdout=stream, stderr=stream,
                                      pass_fds=(writer,), start_new_session=True)
        os.close(writer)
        writer = None
        event('server_started', supervisor_child_pid=server.pid, command=server_command)
        deadline = time.monotonic() + 10
        display = b''
        while b'\n' not in display:
            if server.poll() is not None:
                raise RuntimeError(f'Xvfb startup failed: status={server.returncode}')
            if time.monotonic() > deadline:
                raise RuntimeError('Xvfb display readiness timed out')
            if select.select([reader], [], [], .1)[0]:
                chunk = os.read(reader, 64)
                if not chunk:
                    raise RuntimeError('Xvfb closed readiness pipe')
                display += chunk
        number = display.decode().strip()
        if not number.isdigit():
            raise RuntimeError(f'Invalid Xvfb display number: {number!r}')
        env = dict(os.environ, DISPLAY=':' + number)
        env.pop('XAUTHORITY', None)
        result['display'] = env['DISPLAY']
        child = subprocess.Popen(command, env=env, start_new_session=True)
        event('command_started', pid=child.pid, display=env['DISPLAY'])
        next_snapshot = time.monotonic()
        while True:
            server_status = server.poll()
            command_status = child.poll()
            if server_status is not None:
                result['xvfb_exit_before_cleanup'] = server_status
                result['command_exit_before_cleanup'] = command_status
                event('server_exited_before_cleanup', status=server_status,
                      command_status=command_status)
                raise RuntimeError(f'Xvfb exited before cleanup: status={server_status}; see xvfb-strace.log')
            if command_status is not None:
                result['command_exit_before_cleanup'] = command_status
                result['xvfb_exit_before_cleanup'] = None
                event('command_exited', status=command_status, xvfb_alive=True)
                code = command_status if command_status >= 0 else 128 - command_status
                result['passed'] = code == 0
                break
            if time.monotonic() >= next_snapshot:
                snapshot(diagnostic, 'latest')
                next_snapshot = time.monotonic() + 2
            time.sleep(.05)
    except (Exception, KeyboardInterrupt) as error:
        result['failure'] = str(error) or type(error).__name__
        print('::error title=Virtual display supervisor::' + result['failure'], flush=True)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        event('cleanup_started')
        snapshot(diagnostic, 'after')
        stop(child)
        stop(server)
        result['server_status_after_cleanup'] = server.returncode if server else None
        (logs / 'xvfb-result.json').write_text(json.dumps(result, indent=2))
        os.close(reader)
        if writer is not None:
            os.close(writer)
        if display_lock is not None:
            display_lock.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
