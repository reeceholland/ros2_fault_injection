#include "ros2_fault_injection/fault_event_publisher.hpp"

namespace ros2_fault_injection {

FaultEventPublisher::FaultEventPublisher(rclcpp::Node& node) {
  pub_ = node.create_publisher<std_msgs::msg::String>("fault_injection/events", 10);
}

void FaultEventPublisher::publish(const std::string& fault_id, const std::string& state) {
  std_msgs::msg::String event_msg;
  event_msg.data = fault_id + " " + state;
  pub_->publish(event_msg);
}

}  // namespace ros2_fault_injection