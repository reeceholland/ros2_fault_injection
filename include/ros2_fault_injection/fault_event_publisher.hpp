#ifndef ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_

#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include "ros2_fault_injection/msg/fault_event.hpp"

namespace ros2_fault_injection {

struct FaultEvent {
  std::string fault_id;
  std::string injector_id;
  std::string state;
  std::string source;
  std::string details;
};

class FaultEventPublisher {
public:
  explicit FaultEventPublisher(rclcpp::Node& node);

  void publish(const FaultEvent& event);

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<msg::FaultEvent>::SharedPtr pub_;
};

}  // namespace ros2_fault_injection

#endif  // ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_