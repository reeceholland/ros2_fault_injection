"""ROS integration regression: a topic may appear after its probe starts."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


class ReadinessProbeTests(unittest.TestCase):
    def check_probe(self, publish):
        source = Path(__file__).with_name('run_navigation_workflow.sh').read_text()
        function = source[source.index('check_topic_ready() {'):source.index('check_action_ready() {')]
        env = dict(os.environ, ROS_DOMAIN_ID='192', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        publisher = None
        with tempfile.TemporaryDirectory() as directory:
            env['CI_LOG_DIR'] = directory
            Path(directory, 'readiness').mkdir()
            script = ('set -eo pipefail\nREADINESS_TIMEOUT=8\n'
                      'fail_if_core_process_exited() { :; }\n'
                      'register_pid() { :; }\n' + function +
                      '\ncheck_topic_ready clock /ci_probe_clock rosgraph_msgs/msg/Clock\n')
            probe = subprocess.Popen(['bash', '-c', script], env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                time.sleep(2)
                self.assertIsNone(probe.poll(), 'Probe failed before a publisher could appear')
                if publish:
                    publisher = subprocess.Popen(['ros2', 'topic', 'pub', '-r', '5',
                        '/ci_probe_clock', 'rosgraph_msgs/msg/Clock', '{clock: {sec: 1}}'],
                        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        start_new_session=True)
                output, _ = probe.communicate(timeout=15)
                self.assertEqual(probe.returncode, 0 if publish else 1, output)
                self.assertIn('PASS' if publish else 'FAIL', output)
            finally:
                if probe.poll() is None:
                    probe.kill()
                    probe.wait()
                if publisher:
                    os.killpg(publisher.pid, signal.SIGTERM)
                    publisher.wait(timeout=5)

    def test_publisher_can_start_later(self):
        self.check_probe(True)

    def test_missing_publisher_still_fails_at_deadline(self):
        self.check_probe(False)


if __name__ == '__main__':
    unittest.main()
