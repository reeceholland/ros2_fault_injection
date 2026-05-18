#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/fault_config.hpp"
#include "ros2_fault_injection/odom_fault_injector.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("odom_fault_injector");

    ros2_fault_injection::InjectorConfig injector_config;
    injector_config.id = "odom";
    injector_config.type = "odom";
    injector_config.input_topic = "/odom_raw";
    injector_config.output_topic = "/odom";
    injector_config.qos_depth = 10;

    auto injector =
        std::make_shared<ros2_fault_injection::OdomFaultInjector>(
            *node,
            injector_config);

    RCLCPP_INFO(
        node->get_logger(),
        "Odom fault injector running: %s -> %s",
        injector_config.input_topic.c_str(),
        injector_config.output_topic.c_str());

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
