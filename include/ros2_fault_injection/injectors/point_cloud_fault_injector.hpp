// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <deque>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ros2_fault_injection/core/fault_injector_base.hpp"

namespace ros2_fault_injection::injectors
{

  class PointCloudFaultInjector : public FaultInjectorBase
  {
  public:
    /**
     * @brief Create the point cloud fault injector.
     *
     * @param node Node used to create publishers, subscriptions, and timers.
     * @param config Injector topic and QoS configuration.
     */
    explicit PointCloudFaultInjector(rclcpp::Node &node, const InjectorConfig &config);

    static std::vector<FaultConfigField> static_config_schema();

    std::vector<FaultConfigField> config_schema() const override;

  private:
    struct DelayedPointCloud
    {
      sensor_msgs::msg::PointCloud2 msg;
      rclcpp::Time release_time;
    };

    void on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void flush_delayed();

    void apply_point_dropout(sensor_msgs::msg::PointCloud2 &msg);
    void apply_range_noise(sensor_msgs::msg::PointCloud2 &msg);
    void apply_dust_returns(sensor_msgs::msg::PointCloud2 &msg);
    void apply_intensity_scale(sensor_msgs::msg::PointCloud2 &msg);

    bool has_field(const sensor_msgs::msg::PointCloud2 &msg, const std::string &field_name) const;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<DelayedPointCloud> delayed_;
  };

} // namespace ros2_fault_injection::injectors