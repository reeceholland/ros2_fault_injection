// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__CORE__DUST_INTENSITY_MODEL_HPP
#define ROS2_FAULT_INJECTION__CORE__DUST_INTENSITY_MODEL_HPP
#include <random>

namespace ros2_fault_injection::core
{
/**
 * @brief Range-dependent lognormal intensity for a replacement dust return.
 *
 * The median is clamp(intercept + slope * range, minimum, maximum).
 * Bounds constrain the median, not sampled intensities. This phenomenological
 * model requires sensor-specific calibration; it does not model saturation.
 */
class DustIntensityModel
{
public:
  /// Construct with finite parameters, positive ordered bounds and non-negative log sigma.
  /// @throws std::invalid_argument if parameters violate these constraints.
  DustIntensityModel(
    double median_intercept,
    double median_slope_per_m,
    double minimum_median,
    double maximum_median,
    double log_sigma
  );
  /// Return the bounded median at a finite, non-negative range in metres.
  /// @throws std::invalid_argument for an invalid range.
  double median_intensity(double range_m) const;
  /// Sample using the caller's generator; zero log sigma returns the median without a draw.
  /// Samples are not clamped to the median bounds. Invalid ranges throw std::invalid_argument.
  double sample_intensity(double range_m, std::mt19937 & rng) const;

private:
  double median_intercept;
  double median_slope_per_m;
  double minimum_median;
  double maximum_median;
  double log_sigma;
  void validate() const;
};
} // namespace ros2_fault_injection::core

#endif // ROS2_FAULT_INJECTION__CORE__DUST_INTENSITY_MODEL_HPP
