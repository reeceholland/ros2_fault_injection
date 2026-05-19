#ifndef ROS2_FAULT_INJECTION__FAULT_INJECTOR_HPP_
#define ROS2_FAULT_INJECTION__FAULT_INJECTOR_HPP_

#include <string>

#include "ros2_fault_injection/fault_config.hpp"

namespace ros2_fault_injection
{
  class FaultInjector
  {
  public:
    virtual ~FaultInjector() = default;

    virtual std::string id() const = 0;
    virtual void add_fault(const FaultConfig &fault_config) = 0;
    virtual void activate_fault(const std::string &fault_id) = 0;
    virtual void deactivate_fault(const std::string &fault_id) = 0;
    virtual bool has_fault(const std::string &fault_id) const = 0;
   };
} // namespace ros2_fault_injection

#endif // ROS2_FAULT_INJECTION__FAULT_INJECTOR_HPP_