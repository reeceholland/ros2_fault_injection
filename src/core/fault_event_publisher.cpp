#include "ros2_fault_injection/core/fault_event_publisher.hpp"

#include "ros2_fault_injection/msg/fault_event.hpp"

namespace ros2_fault_injection {

FaultEventPublisher::FaultEventPublisher(rclcpp::Node& node) : node_(&node) {
  pub_ = node.create_publisher<msg::FaultEvent>("fault_injection/events", 10);
}

void FaultEventPublisher::publish(const FaultEvent& event) {
  msg::FaultEvent event_msg;
  event_msg.stamp = node_->now();
  event_msg.fault_id = event.fault_id;
  event_msg.injector_id = event.injector_id;
  event_msg.state = event.state;
  event_msg.source = event.source;
  event_msg.details = event.details;
  pub_->publish(event_msg);
}

}  // namespace ros2_fault_injection