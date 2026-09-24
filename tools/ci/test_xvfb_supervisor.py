import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class DisplaySupervisorTests(unittest.TestCase):
    def run_case(self, command, expected):
        with tempfile.TemporaryDirectory() as directory:
            script = Path(__file__).with_name('xvfb_supervisor.py')
            env = dict(os.environ, TEST_DISPLAY_LOG=directory)
            run = subprocess.run([sys.executable, str(script), '--log-dir', directory,
                                  '--', sys.executable, '-c', command],
                                 env=env, capture_output=True, text=True, timeout=35)
            self.assertEqual(run.returncode, expected, run.stdout + run.stderr)
            root = Path(directory)
            report = json.loads((root / 'xvfb-result.json').read_text())
            events = [json.loads(line) for line in (root / 'xvfb-events.jsonl').read_text().splitlines()]
            self.assertTrue((root / 'display-diagnostics/after-cgroup.json').exists())
            self.assertTrue((root / 'display-diagnostics/latest-processes.log').exists())
            return report, events, (root / 'xvfb-strace.log').read_text()

    def test_success_keeps_server_alive_until_cleanup(self):
        report, events, trace = self.run_case('import time; time.sleep(.3)', 0)
        self.assertTrue(report['passed'])
        self.assertIsNone(report['xvfb_exit_before_cleanup'])
        self.assertEqual(report['command_exit_before_cleanup'], 0)

    def test_command_failure_is_not_a_display_failure(self):
        report, events, trace = self.run_case('import time; time.sleep(.3); raise SystemExit(42)', 42)
        self.assertFalse(report['passed'])
        self.assertIsNone(report['xvfb_exit_before_cleanup'])
        self.assertEqual(report['command_exit_before_cleanup'], 42)

    def test_external_display_signal_is_captured_before_cleanup(self):
        command = '''
import json, os, pathlib, signal, time
root = pathlib.Path(os.environ['TEST_DISPLAY_LOG'])
events = [json.loads(line) for line in (root/'xvfb-events.jsonl').read_text().splitlines()]
tracer = next(e['supervisor_child_pid'] for e in events if e['event'] == 'server_started')
children = pathlib.Path(f'/proc/{tracer}/task/{tracer}/children').read_text().split()
assert len(children) == 1, children
os.kill(int(children[0]), signal.SIGTERM)
time.sleep(20)
'''
        report, events, trace = self.run_case(command, 1)
        self.assertIn('Xvfb exited before cleanup', report['failure'])
        self.assertIsNotNone(report['xvfb_exit_before_cleanup'])
        kinds = [e['event'] for e in events]
        self.assertLess(kinds.index('server_exited_before_cleanup'), kinds.index('cleanup_started'))
        self.assertIn('SIGTERM', trace)
        self.assertIn('si_pid=', trace)


if __name__ == '__main__':
    unittest.main()
