// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__ASSERTIONS__FAULT_EVENT_ASSERTION_HPP_
#define ROS2_FAULT_INJECTION__ASSERTIONS__FAULT_EVENT_ASSERTION_HPP_

#include <optional>
#include <string>

#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/msg/fault_event.hpp"

namespace ros2_fault_injection
{

  /**
   * @brief Assertion that passes when an expected fault event is observed.
   *
   * A fault event assertion checks `fault_id` and `state` from
   * `/fault_injection/events`. It is useful for verifying that scheduled,
   * startup, or manual fault transitions actually occurred.
   */
class FaultEventAssertion
{
public:
    /**
     * @brief Create an assertion from parsed scenario config.
     *
     * @param config Assertion configuration. Expected fields are `id`,
     * `fault_id`, `state`, and optionally `within`.
     */
  explicit FaultEventAssertion(const AssertionConfig & config);

    /**
     * @brief Process a fault event.
     *
     * If the event matches the configured `fault_id` and `state`, the assertion
     * is marked as passed.
     *
     * @param event Fault event message published by the framework.
     */
  void observe(const msg::FaultEvent & event);

    /**
     * @brief Check whether the assertion deadline has expired.
     *
     * @param elapsed_seconds Seconds since assertion monitoring started.
     */
  void update(double elapsed_seconds);

    /**
     * @brief Current assertion result.
     *
     * @return Assertion state and human-readable message.
     */
  AssertionResult result() const;

private:
  AssertionConfig config_;
  AssertionResult result_;
};

} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__ASSERTIONS__FAULT_EVENT_ASSERTION_HPP_
