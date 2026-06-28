// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_
#define ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_

#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include "ros2_fault_injection/msg/fault_event.hpp"

namespace ros2_fault_injection::core
{

/**
 * @brief Internal representation of a fault lifecycle event.
 *
 * The publisher adds the timestamp when converting this struct to
 * `ros2_fault_injection/msg/FaultEvent`.
 */
struct FaultEvent
{
  /// Fault identifier associated with the event.
  std::string fault_id;

  /// Injector identifier that owns the fault.
  std::string injector_id;

  /// Fault state, for example `active`, `inactive`, or `config_updated`.
  std::string state;

  /// Source of the event, for example `startup`, `scheduled`, or `manual`.
  std::string source;

  /// Human-readable event details.
  std::string details;
};

/**
 * @brief Publishes structured fault events on `fault_injection/events`.
 */
class FaultEventPublisher {
public:
  /**
   * @brief Create the event publisher.
   *
   * @param node Node used for publisher creation and event timestamps.
   */
  explicit FaultEventPublisher(rclcpp::Node & node);

  /**
   * @brief Publish a fault event with the node's current time.
   *
   * @param event Event payload to publish.
   */
  void publish(const FaultEvent & event);

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<msg::FaultEvent>::SharedPtr pub_;
};

}  // namespace ros2_fault_injection::core

namespace ros2_fault_injection
{
using core::FaultEvent;
using core::FaultEventPublisher;
}  // namespace ros2_fault_injection
#endif  // ROS2_FAULT_INJECTION__FAULT_EVENT_PUBLISHER_HPP_
