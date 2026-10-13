// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/core/dust_intensity_model.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace ros2_fault_injection::core
{
DustIntensityModel::DustIntensityModel(
  double median_intercept,
  double median_slope_per_m,
  double minimum_median,
  double maximum_median,
  double log_sigma
)
: median_intercept(median_intercept),
  median_slope_per_m(median_slope_per_m),
  minimum_median(minimum_median),
  maximum_median(maximum_median),
  log_sigma(log_sigma)
{
  validate();
}

double DustIntensityModel::median_intensity(double range_m) const
{
  if(!std::isfinite(range_m) || range_m < 0) {
    throw std::invalid_argument(
        "DustIntensityModel median_intensity received non-finite or negative range_m");
  }
  return std::clamp(median_intercept + median_slope_per_m * range_m, minimum_median,
      maximum_median);
}

double DustIntensityModel::sample_intensity(double range_m, std::mt19937 & rng) const
{
  const double median = median_intensity(range_m);
  if(log_sigma == 0.0) {
    return median;
  }
  std::lognormal_distribution<double> distribution(std::log(median), log_sigma);
  return distribution(rng);
}

void DustIntensityModel::validate() const
{
  if(!std::isfinite(median_intercept) || !std::isfinite(median_slope_per_m) ||
    !std::isfinite(minimum_median) || !std::isfinite(maximum_median) ||
    !std::isfinite(log_sigma))
  {
    throw std::invalid_argument("DustIntensityModel contains non-finite values");
  }
  if (!(minimum_median > 0)) {
    throw std::invalid_argument("DustIntensityModel minimum_median must be greater than 0");
  }
  if (!(minimum_median <= maximum_median)) {
    throw std::invalid_argument(
        "DustIntensityModel minimum_median must be less than or equal to maximum_median");
  }
  if (!(log_sigma >= 0)) {
    throw std::invalid_argument("DustIntensityModel log_sigma must be greater than or equal to 0");
  }
}
}
