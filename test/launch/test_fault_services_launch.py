# Copyright 2026 Reece Holland
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

from ament_index_python.packages import get_package_prefix

from ros2_fault_injection.srv import GetFaultConfig
from ros2_fault_injection.srv import SetFaultConfig


@pytest.mark.launch_test
def generate_test_description():
    package_prefix = get_package_prefix("ros2_fault_injection")
    node_path = os.path.join(
        package_prefix,
        "lib",
        "ros2_fault_injection",
        "fault_injector_node",
    )

    source_dir = os.environ["ROS2_FAULT_INJECTION_SOURCE_DIR"]
    scenario_file = os.path.join(
        source_dir,
        "test",
        "config",
        "service_integration_faults.yaml",
    )

    fault_injector = launch_ros.actions.Node(
        executable=node_path,
        name="fault_injector_node_test",
        parameters=[{"scenario_file": scenario_file}],
        output="screen",
    )

    return (
        launch.LaunchDescription(
            [
                fault_injector,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "fault_injector": fault_injector,
        },
    )


class TestFaultServices(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_fault_services_client")

    def tearDown(self):
        self.node.destroy_node()

    def call_service(self, client, request, timeout_sec=5.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout_sec))

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=timeout_sec,
        )

        self.assertTrue(future.done())
        return future.result()

    def test_get_and_set_fault_config(self):
        get_config_client = self.node.create_client(
            GetFaultConfig,
            "/fault_injection/get_fault_config",
        )
        set_config_client = self.node.create_client(
            SetFaultConfig,
            "/fault_injection/set_fault_config",
        )

        get_request = GetFaultConfig.Request()
        get_request.fault_id = "odom_bias"

        get_response = self.call_service(get_config_client, get_request)

        self.assertTrue(get_response.success)
        self.assertEqual(get_response.injector_id, "odom")
        self.assertEqual(get_response.injector_type, "odom")

        config = dict(zip(get_response.keys, get_response.values))
        self.assertEqual(config["x_bias"], "1.0")

        set_request = SetFaultConfig.Request()
        set_request.fault_id = "odom_bias"
        set_request.key = "x_bias"
        set_request.value = "2.0"

        set_response = self.call_service(set_config_client, set_request)

        self.assertTrue(set_response.success)

        get_response = self.call_service(get_config_client, get_request)
        config = dict(zip(get_response.keys, get_response.values))

        self.assertEqual(config["x_bias"], "2.0")