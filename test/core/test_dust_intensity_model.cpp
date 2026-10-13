// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "ros2_fault_injection/core/dust_intensity_model.hpp"

using namespace ros2_fault_injection::core;

TEST(DustIntensityModelTest, MedianIntensityClampsToMinimum)
{
  DustIntensityModel model(
    30.0, // median_intercept
    -2.0, // median_slope_per_m
    10.0,  // minimum_median
    25.0, // maximum_median
    0.0   // log_sigma
  );
  EXPECT_DOUBLE_EQ(model.median_intensity(100.0), 10.0);
}

TEST(DustIntensityModelTest, MedianIntensityClampsToMaximum)
{
  DustIntensityModel model(
    30.0, // median_intercept
    -2.0, // median_slope_per_m
    10.0,  // minimum_median
    25.0, // maximum_median
    0.0   // log_sigma
  );
  EXPECT_DOUBLE_EQ(model.median_intensity(0.0), 25.0);
}

TEST(DustIntensityModelTest, MedianIntensityWithinBounds)
{
  DustIntensityModel model(
    30.0, // median_intercept
    -2.0, // median_slope_per_m
    10.0,  // minimum_median
    25.0, // maximum_median
    0.0   // log_sigma
  );
  EXPECT_DOUBLE_EQ(model.median_intensity(10.0), 30.0 - 2.0 * 10.0);
}
