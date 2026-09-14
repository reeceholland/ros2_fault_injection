# Copyright 2026 Reece Holland
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState
from ros2_fault_injection.srv import GetFaultConfig, SetFaultConfig, SetFaultState


@pytest.mark.launch_test
def generate_test_description():
    scenario = os.path.join(os.environ['ROS2_FAULT_INJECTION_SOURCE_DIR'],
                            'test', 'config', 'forwarding_integration_faults.yaml')
    injector = launch_ros.actions.Node(
        package='ros2_fault_injection', executable='fault_injector_node',
        name='forwarding_integration_injector',
        parameters=[{'scenario_file': scenario}],
        remappings=[('/fault_injection/set_fault_state',
                     '/forwarding_integration/set_fault_state'),
                    ('/fault_injection/set_fault_config',
                     '/forwarding_integration/set_fault_config'),
                    ('/fault_injection/get_fault_config',
                     '/forwarding_integration/get_fault_config')],
        output='screen')
    return launch.LaunchDescription([injector, launch_testing.actions.ReadyToTest()])


class TestServiceForwarding(unittest.TestCase):
    def test_dropout_recovery_and_isolation(self):
        rclpy.init()
        node = rclpy.create_node('forwarding_integration_client')
        received = {'affected': [], 'unaffected': []}
        publishers = {}
        subscriptions = []
        client = node.create_client(SetFaultState, '/forwarding_integration/set_fault_state')

        config_client = node.create_client(
            SetFaultConfig, '/forwarding_integration/set_fault_config')
        get_client = node.create_client(
            GetFaultConfig, '/forwarding_integration/get_fault_config')

        def call(service, request):
            self.assertTrue(service.wait_for_service(timeout_sec=5.0))
            future = service.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            self.assertTrue(future.done(), 'Service request timed out')
            return future.result()

        def get_config():
            response = call(get_client, GetFaultConfig.Request(fault_id='command_dropout'))
            self.assertTrue(response.success, response.message)
            return dict(zip(response.keys, response.values))

        def spin_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)

        def set_active(active):
            request = SetFaultState.Request(fault_id='command_dropout', active=active)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            self.assertTrue(future.done(), 'Fault-state service timed out')
            response = future.result()
            self.assertTrue(response.success, response.message)

        def publish_pair(sequence):
            expected = {}
            for index, name in enumerate(received):
                message = JointState()
                message.header.frame_id = name
                message.header.stamp.sec = sequence
                message.name = ['left', 'right']
                message.position = [float(sequence), float(index)]
                message.velocity = [0.2, -0.2]
                message.effort = [1.0, 2.0]
                expected[name] = message
                publishers[name].publish(message)
            return expected

        def wait_counts(affected, unaffected):
            deadline = time.monotonic() + 3.0
            while (len(received['affected']) < affected or
                   len(received['unaffected']) < unaffected):
                self.assertLess(time.monotonic(), deadline, 'Forwarding timed out')
                rclpy.spin_once(node, timeout_sec=0.01)

        try:
            for name in received:
                publishers[name] = node.create_publisher(
                    JointState, '/forwarding_integration/' + name + '_raw', 10)
                subscriptions.append(node.create_subscription(
                    JointState, '/forwarding_integration/' + name,
                    lambda msg, key=name: received[key].append(msg), 10))
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))
            deadline = time.monotonic() + 10.0
            while not (all(p.get_subscription_count() for p in publishers.values()) and
                       all(s.get_publisher_count() for s in subscriptions)):
                self.assertLess(time.monotonic(), deadline, 'DDS discovery timed out')
                rclpy.spin_once(node, timeout_sec=0.05)

            baseline = publish_pair(1)
            wait_counts(1, 1)
            for name in received:
                self.assertEqual(received[name], [baseline[name]])

            # Reject bad requests while a real fault is active. Neither its
            # configuration nor its output behaviour may change on rejection.
            set_active(True)
            original_config = get_config()
            invalid_requests = [
                (client, SetFaultState.Request(fault_id='missing_fault', active=True)),
                (client, SetFaultState.Request(fault_id='missing_fault', active=False)),
                (config_client, SetFaultConfig.Request(
                    fault_id='missing_fault', key='drop_probability', value='0.0')),
            ]
            for value in ['-0.1', '1.1', 'not-a-number']:
                invalid_requests.append((config_client, SetFaultConfig.Request(
                    fault_id='command_dropout', key='drop_probability', value=value)))
            for index, (service, request) in enumerate(invalid_requests):
                with self.subTest(request=str(request)):
                    response = call(service, request)
                    self.assertFalse(response.success)
                    self.assertTrue(response.message.strip(), 'Missing rejection explanation')
                    self.assertEqual(get_config(), original_config)
                    affected_count = len(received['affected'])
                    unaffected_count = len(received['unaffected'])
                    expected = publish_pair(100 + index)
                    wait_counts(affected_count, unaffected_count + 1)
                    spin_for(0.2)
                    self.assertEqual(len(received['affected']), affected_count)
                    self.assertEqual(received['unaffected'][-1], expected['unaffected'])

            # A valid configuration update must still work after the failures.
            response = call(config_client, SetFaultConfig.Request(
                fault_id='command_dropout', key='drop_probability', value='0.0'))
            self.assertTrue(response.success, response.message)
            self.assertEqual(float(get_config()['drop_probability']), 0.0)
            affected_count = len(received['affected'])
            unaffected_count = len(received['unaffected'])
            expected = publish_pair(200)
            wait_counts(affected_count + 1, unaffected_count + 1)
            for name in received:
                self.assertEqual(received[name][-1], expected[name])
            response = call(config_client, SetFaultConfig.Request(
                fault_id='command_dropout', key='drop_probability', value='1.0'))
            self.assertTrue(response.success, response.message)
            set_active(False)

            for cycle in range(3):
                set_active(True)
                affected_count = len(received['affected'])
                for offset in range(3):
                    expected = publish_pair(10 + cycle * 10 + offset)
                    target = len(received['unaffected']) + 1
                    wait_counts(affected_count, target)
                    self.assertEqual(received['unaffected'][-1], expected['unaffected'])
                spin_for(0.2)
                self.assertEqual(len(received['affected']), affected_count)

                set_active(False)
                spin_for(0.2)
                self.assertEqual(len(received['affected']), affected_count,
                                 'Deactivation replayed dropped messages')
                unaffected_count = len(received['unaffected'])
                recovery = publish_pair(15 + cycle * 10)
                wait_counts(affected_count + 1, unaffected_count + 1)
                spin_for(0.2)
                self.assertEqual(len(received['affected']), affected_count + 1)
                self.assertEqual(len(received['unaffected']), unaffected_count + 1)
                for name in received:
                    self.assertEqual(received[name][-1], recovery[name])
        finally:
            node.destroy_node()
            rclpy.shutdown()
