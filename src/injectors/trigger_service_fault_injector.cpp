#include "ros2_fault_injection/injectors/trigger_service_fault_injector.hpp"

namespace ros2_fault_injection {
TriggerServiceFaultInjector::TriggerServiceFaultInjector(rclcpp::Node& node,
                                                         const InjectorConfig& config)
    : FaultInjectorBase(node, config) {
  service_callback_group_ =
      node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_ = node_.create_service<std_srvs::srv::Trigger>(
      config.trigger_service->proxy_service,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        on_request(request, response);
      },
      rclcpp::ServicesQoS(), service_callback_group_);
  client_ = node_.create_client<std_srvs::srv::Trigger>(
      config.trigger_service->target_service, rclcpp::ServicesQoS(), client_callback_group_);

  RCLCPP_INFO(
      node_.get_logger(),
      "Trigger service fault injector initialized with proxy service '%s' and target service '%s'",
      config.trigger_service->proxy_service.c_str(),
      config.trigger_service->target_service.c_str());
}

void TriggerServiceFaultInjector::on_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const auto delay = active_delay();
  if (delay.count() > 0) {
    std::this_thread::sleep_for(delay);
  }

  if (active_bool("force_failure", false)) {
    response->success = false;
    response->message = active_string("failure_message", "Injected service failure");
    return;
  }

  if (!client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_WARN(node_.get_logger(), "Trigger service '%s' is not available",
                config_.trigger_service->target_service.c_str());
    response->success = false;
    response->message = "Trigger service unavailable";
    return;
  }

  auto future = client_->async_send_request(request);

  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_WARN(node_.get_logger(), "Trigger service '%s' did not respond in time",
                config_.trigger_service->target_service.c_str());
    response->success = false;
    response->message = "Trigger service timeout";
    return;
  }

  try {
    auto trigger_response = future.get();
    response->success = trigger_response->success;
    response->message = trigger_response->message;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_.get_logger(), "Error calling trigger service '%s': %s",
                 config_.trigger_service->target_service.c_str(), e.what());
    response->success = false;
    response->message = "Error calling trigger service";
  }
}
}  // namespace ros2_fault_injection