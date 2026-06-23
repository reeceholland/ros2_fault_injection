// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "gtest/gtest.h"

#include "ros2_fault_injection/core/report_creator.hpp"
#include "ros2_fault_injection/core/fault_injector.hpp"

namespace rfi = ros2_fault_injection;
namespace rfi_assertions = ros2_fault_injection::assertions;
namespace rfi_core = ros2_fault_injection::core;

namespace
{

class FakeFaultInjector : public rfi::FaultInjector
{
public:
  std::string id() const override {return "injector1";}
  std::string type() const override {return "fake";}

  void add_fault(const rfi::FaultConfig &) override {}
  void clear_faults() override {}

  bool has_fault(const std::string &) const override {return false;}
  void activate_fault(const std::string &) override {}
  void deactivate_fault(const std::string &) override {}

  std::vector<std::string> fault_ids() const override
  {
    return {"fault1"};
  }

  std::vector<std::string> active_fault_ids() const override
  {
    return {};
  }

  std::optional<rfi::FaultConfig> get_fault_config(const std::string &) const override
  {
    return std::nullopt;
  }

  bool set_fault_config_value(
    const std::string &, const std::string &, const std::string &) override
  {
    return false;
  }

  std::vector<rfi::FaultConfigField> config_schema() const override
  {
    return {};
  }
};

} // namespace

TEST(ReportCreatorTest, CreatesFailedReportWhenAnyAssertionFails)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_report_creator_failed");
  rfi_core::ReportCreator report_creator(*node);

  rfi_assertions::AssertionResult passed;
  passed.id = "odom_hz";
  passed.type = "topic_hz";
  passed.state = rfi_assertions::AssertionState::Passed;
  passed.message = "Observed /odom above minimum rate";

  rfi_assertions::AssertionResult failed;
  failed.id = "scan_hz";
  failed.type = "topic_hz";
  failed.state = rfi_assertions::AssertionState::Failed;
  failed.message = "Timed out waiting for /scan";

  rfi::InjectorMap injectors;

  const auto report = report_creator.create_report("/tmp/scenario.yaml", injectors,
      {passed, failed});

  EXPECT_EQ(report.scenario_file, "/tmp/scenario.yaml");
  EXPECT_EQ(report.final_result, "failed");
  EXPECT_EQ(report.assertion_results.size(), 2);

  rclcpp::shutdown();
}

TEST(ReportCreatorTest, CreatesPassedReportWhenAllAssertionsPassed)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_report_creator_passed");
  rfi_core::ReportCreator report_creator(*node);

  rfi_assertions::AssertionResult passed1;
  passed1.id = "odom_hz";
  passed1.type = "topic_hz";
  passed1.state = rfi_assertions::AssertionState::Passed;
  passed1.message = "Observed /odom above minimum rate";

  rfi_assertions::AssertionResult passed2;
  passed2.id = "scan_hz";
  passed2.type = "topic_hz";
  passed2.state = rfi_assertions::AssertionState::Passed;
  passed2.message = "Observed /scan above minimum rate";

  rfi::InjectorMap injectors;

  const auto report = report_creator.create_report("/tmp/scenario.yaml", injectors,
      {passed1, passed2});

  EXPECT_EQ(report.scenario_file, "/tmp/scenario.yaml");
  EXPECT_EQ(report.final_result, "passed");
  EXPECT_EQ(report.assertion_results.size(), 2);

  rclcpp::shutdown();
}

TEST(ReportCreatorTest, CreatesRunningReportWhenAnyAssertionPending)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_report_creator_pending");
  rfi_core::ReportCreator report_creator(*node);

  rfi_assertions::AssertionResult pending;
  pending.id = "odom_hz";
  pending.type = "topic_hz";
  pending.state = rfi_assertions::AssertionState::Pending;
  pending.message = "Waiting for /odom to be observed";

  rfi_assertions::AssertionResult passed;
  passed.id = "scan_hz";
  passed.type = "topic_hz";
  passed.state = rfi_assertions::AssertionState::Passed;
  passed.message = "Observed /scan above minimum rate";

  rfi::InjectorMap injectors;

  const auto report = report_creator.create_report("/tmp/scenario.yaml", injectors,
      {pending, passed});

  EXPECT_EQ(report.scenario_file, "/tmp/scenario.yaml");
  EXPECT_EQ(report.final_result, "pending");
  EXPECT_EQ(report.assertion_results.size(), 2);

  rclcpp::shutdown();
}

TEST(ReportCreatorTest, MarkdownIncludesScenarioFileInjectorsFaultsAndAssertions)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_report_creator_markdown");
  rfi_core::ReportCreator report_creator(*node);

  rfi_assertions::AssertionResult passed;
  passed.id = "odom_hz";
  passed.type = "topic_hz";
  passed.state = rfi_assertions::AssertionState::Passed;
  passed.message = "Observed /odom above minimum rate";

  rfi::InjectorMap injectors;
  injectors["injector1"] = std::make_shared<FakeFaultInjector>();

  const auto report = report_creator.create_report("/tmp/scenario.yaml", injectors, {passed});
  const auto markdown = report_creator.to_markdown(report);

  EXPECT_NE(markdown.find("/tmp/scenario.yaml"), std::string::npos);
  EXPECT_NE(markdown.find("injector1"), std::string::npos);
  EXPECT_NE(markdown.find("odom_hz"), std::string::npos);
  EXPECT_NE(markdown.find("topic_hz"), std::string::npos);
  EXPECT_NE(markdown.find("passed"), std::string::npos);
  EXPECT_NE(markdown.find("Observed /odom above minimum rate"), std::string::npos);

  rclcpp::shutdown();
}

TEST(ReportCreatorTest, MarkdownHandlesEmptyAssertions)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<rclcpp::Node>("test_report_creator_markdown_empty_assertions");
  rfi_core::ReportCreator report_creator(*node);

  rfi::InjectorMap injectors;
  injectors["injector1"] = std::make_shared<FakeFaultInjector>();

  const auto report = report_creator.create_report("/tmp/scenario.yaml", injectors, {});
  const auto markdown = report_creator.to_markdown(report);

  EXPECT_NE(markdown.find("/tmp/scenario.yaml"), std::string::npos);
  EXPECT_NE(markdown.find("injector1"), std::string::npos);
  EXPECT_NE(markdown.find("No assertions configured."), std::string::npos);

  rclcpp::shutdown();
}
