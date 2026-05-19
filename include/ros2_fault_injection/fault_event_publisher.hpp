#ifndef ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>

namespace ros2_fault_injection {

class FaultEventPublisher {
public:
  explicit FaultEventPublisher(rclcpp::Node& node);

  void publish(const std::string& fault_id, const std::string& state);

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_