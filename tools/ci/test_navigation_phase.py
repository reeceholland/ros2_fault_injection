import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from navigation_phase import completed


class PhaseTests(unittest.TestCase):
    def report(self, root, passed=True):
        (root/'results.json').write_text(json.dumps(dict(passed=passed, goals=[{'passed': passed}],
                                                        observer={'passed': passed})))

    def test_only_complete_current_report_ends_active_phase(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root/'run-started').touch()
            self.assertFalse(completed(root))
            (root/'results.json').write_text('{')
            self.assertFalse(completed(root))
            self.report(root)
            self.assertTrue(completed(root))
            os.utime(root/'results.json', ns=(1, 1))
            self.assertFalse(completed(root))
            self.report(root, passed=False)
            self.assertTrue(completed(root))  # completed is not synonymous with passed

    def watchdog(self, root):
        source = Path(__file__).with_name('run_navigation_workflow.sh').read_text()
        function = source[source.index('start_fault_service_watchdog() {'):source.index('# Capture diagnostics')]
        bindir = root/'bin'
        bindir.mkdir()
        fake = bindir/'ros2'
        fake.write_text('#!/bin/sh\ntouch "$CI_LOG_DIR/probe-started"\n'
                        'while [ ! -f "$CI_LOG_DIR/allow-loss" ]; do sleep .02; done\nexit 1\n')
        fake.chmod(0o755)
        env = dict(os.environ, CI_LOG_DIR=str(root),
                   NAVIGATION_PHASE_HELPER=str(Path(__file__).with_name('navigation_phase.py')),
                   PATH=str(bindir)+':'+os.environ['PATH'])
        script = ('logger_pids=()\nregister_pid() { :; }\nFAULT_SERVICE=/fault_injection/set_fault_state\n'
                  + function + '\nstart_fault_service_watchdog\nwait "$!"\n')
        return subprocess.Popen(['bash', '-c', script], env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)

    def test_service_loss_after_report_does_not_fail(self):
        self.case(after_report=True)

    def test_outer_supervisor_preserves_success_and_failure(self):
        source = Path(__file__).with_name('run_navigation_workflow.sh').read_text()
        loop = source[source.index('# ROS availability is required only'):]
        for command_status, passed, expected in ((0, True, 0), (42, True, 42), (0, False, 1)):
            with self.subTest(command_status=command_status, report_passed=passed):
                with tempfile.TemporaryDirectory() as d:
                    root = Path(d)
                    (root/'run-started').touch()
                    self.report(root, passed)
                    env = dict(os.environ, CI_LOG_DIR=d,
                        NAVIGATION_PHASE_HELPER=str(Path(__file__).with_name('navigation_phase.py')))
                    script = ('set -eo pipefail\n'
                        'test_completed() { python3 "$NAVIGATION_PHASE_HELPER" "$CI_LOG_DIR"; }\n'
                        'sleep 20 &\nbag_pid=$!\n'
                        'trap \'kill "$bag_pid" 2>/dev/null || true; wait "$bag_pid" 2>/dev/null || true\' EXIT\n'
                        f'(sleep .2; exit {command_status}) &\nnavigation_pid=$!\n'
                        'logger_pids=(99999999)\n' + loop)
                    result = subprocess.run(['bash', '-c', script], env=env, capture_output=True,
                                            text=True, timeout=5)
                    self.assertEqual(result.returncode, expected, result.stdout+result.stderr)

    def test_early_service_loss_remains_latched_after_later_pass(self):
        self.case(after_report=False)

    def case(self, after_report):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root/'telemetry').mkdir()
            (root/'run-started').touch()
            process = self.watchdog(root)
            deadline = time.monotonic()+5
            while not (root/'probe-started').exists() and time.monotonic()<deadline:
                time.sleep(.02)
            self.assertTrue((root/'probe-started').exists())
            if after_report:
                self.report(root)
            (root/'allow-loss').touch()
            output, _ = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0 if after_report else 1, output)
            self.report(root)
            self.assertEqual((root/'active-watchdog-failure.txt').exists(), not after_report)


if __name__ == '__main__':
    unittest.main()
