# Copyright 2026 Reece Holland
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import json
import os
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch.actions.TimerAction(period=50.0, actions=[]),
        launch_testing.actions.ReadyToTest()])


class TestOutputConflicts(unittest.TestCase):
    def test_shared_and_separate_outputs(self):
        executable = os.path.join(get_package_prefix('ros2_fault_injection'),
                                  'lib', 'ros2_fault_injection', 'fault_injector_node')
        processes = []
        with tempfile.TemporaryDirectory() as directory:
            def start(name, output):
                config = os.path.join(directory, name + '.yaml')
                with open(config, 'w') as handle:
                    json.dump({'injectors': [{'id': 'scan', 'type': 'scan', 'topic': {
                        'input_topic': 'raw', 'output_topic': output}}], 'faults': []}, handle)
                logfile = os.path.join(directory, name + '.log')
                with open(logfile, 'w') as handle:
                    process = subprocess.Popen([
                        executable, '--ros-args', '-r', '__node:=' + name,
                        '-r', '__ns:=/conflict_integration', '-p', 'scenario_file:=' + config
                    ], stdout=handle, stderr=subprocess.STDOUT)
                processes.append(process)
                return process, logfile

            def read(path):
                with open(path) as handle:
                    return handle.read()

            def wait_text(path, text):
                deadline = time.monotonic() + 10
                while text not in read(path):
                    self.assertLess(time.monotonic(), deadline, read(path))
                    self.assertTrue(all(p.poll() is None for p in processes))
                    time.sleep(0.1)

            try:
                first, first_log = start('first', 'shared')
                _, separate_log = start('separate', 'independent')
                wait_text(first_log, 'Fault injector running')
                wait_text(separate_log, 'Fault injector running')
                time.sleep(2.2)
                self.assertNotIn('Output topic conflict on', read(first_log))
                self.assertNotIn('Output topic conflict on', read(separate_log))
                second, second_log = start('second', 'shared')
                warning = 'Output topic conflict on /conflict_integration/shared'
                wait_text(first_log, warning)
                wait_text(second_log, warning)
                time.sleep(2.2)
                for path in (first_log, second_log):
                    self.assertEqual(read(path).count(warning), 1)
                    self.assertIn('/conflict_integration/first', read(path))
                    self.assertIn('/conflict_integration/second', read(path))
                self.assertNotIn('Output topic conflict on', read(separate_log))
                second.send_signal(signal.SIGINT)
                second.wait(timeout=5)
                processes.remove(second)
                wait_text(first_log, 'Output topic conflict cleared')
                self.assertIsNone(first.poll())
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                for process in processes:
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
