#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2_fault_injection/fault_injector_base.hpp"

namespace ros2_fault_injection {

class TestFaultInjector : public FaultInjectorBase {
public:
  TestFaultInjector(rclcpp::Node& node, const InjectorConfig& config)
      : FaultInjectorBase(node, config) {}
};

TEST(FaultInjectorBase, ReturnsStoredFaultConfig) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);

  const auto result = injector.get_fault_config("odom_bias");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "odom_bias");
  EXPECT_EQ(result->injector_id, "odom");
  EXPECT_EQ(result->config.at("x_bias"), "1.0");

  EXPECT_FALSE(injector.get_fault_config("missing_fault").has_value());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, StartsWithNoFaults) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  EXPECT_TRUE(injector.fault_ids().empty());
  EXPECT_TRUE(injector.active_fault_ids().empty());
  EXPECT_FALSE(injector.has_fault("missing"));

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, AddFaultRegistersItInactive) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);

  EXPECT_TRUE(injector.has_fault("odom_bias"));
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, ActivateKnownFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault("odom_bias");

  EXPECT_TRUE(injector.has_fault("odom_bias"));
  EXPECT_EQ(injector.active_fault_ids(), std::vector<std::string>{"odom_bias"});

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, DeactivateKnownFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  FaultConfig fault;
  fault.id = "odom_bias";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault("odom_bias");
  injector.deactivate_fault("odom_bias");

  EXPECT_TRUE(injector.has_fault("odom_bias"));
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, UnknownActivateDoesNotCreateFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  injector.activate_fault("unknown_fault");

  EXPECT_FALSE(injector.has_fault("unknown_fault"));
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, SameIdReplacesConfigAndResetsInactive) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  FaultConfig fault;
  fault.id = "fault1";
  fault.injector_id = "odom";
  fault.config["x_bias"] = "1.0";

  injector.add_fault(fault);
  injector.activate_fault("fault1");

  // Add a new fault with the same ID but different config
  FaultConfig new_fault;
  new_fault.id = "fault1";
  new_fault.injector_id = "odom";
  new_fault.config["x_bias"] = "2.0";

  injector.add_fault(new_fault);

  // The old config should be replaced and the fault should be inactive
  const auto result = injector.get_fault_config("fault1");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->config.at("x_bias"), "2.0");
  EXPECT_TRUE(injector.active_fault_ids().empty());

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, SetFaultConfigValueUpdatesExistingFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  FaultConfig fault;
  fault.id = "odom_yaw_bias";
  fault.injector_id = "odom";
  fault.config["yaw_bias_deg"] = "10.0";

  injector.add_fault(fault);

  const bool updated = injector.set_fault_config_value("odom_yaw_bias", "yaw_bias_deg", "20.0");

  EXPECT_TRUE(updated);

  const auto result = injector.get_fault_config("odom_yaw_bias");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->config.at("yaw_bias_deg"), "20.0");

  rclcpp::shutdown();
}

TEST(FaultInjectorBase, SetFaultConfigValueRejectsUnknownFault) {
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_fault_injector_base");

  InjectorConfig injector_config;
  injector_config.id = "odom";
  injector_config.type = "odom";
  injector_config.input_topic = "/odom_raw";
  injector_config.output_topic = "/odom";

  TestFaultInjector injector(*node, injector_config);

  const bool updated = injector.set_fault_config_value("missing_fault", "yaw_bias_deg", "20.0");

  EXPECT_FALSE(updated);
  EXPECT_FALSE(injector.has_fault("missing_fault"));

  rclcpp::shutdown();
}

}  // namespace ros2_fault_injection