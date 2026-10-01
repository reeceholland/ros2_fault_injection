// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>

#include "ros2_fault_injection/core/dust_plume.hpp"

using namespace ros2_fault_injection::core;

TEST(DustPlumeTest, DensityAtCenterEqualsOne)
{
  DustPlume plume;
  EXPECT_DOUBLE_EQ(plume.relative_density(plume.center_x, plume.center_y, plume.center_z), 1.0);
}

TEST(DustPlumeTest, DensityAtOneSigma)
{
  DustPlume plume;
  EXPECT_NEAR(plume.relative_density(plume.center_x + plume.sigma_x, plume.center_y,
    plume.center_z), std::exp(-0.5), 1e-12);
}

TEST(DustPlumeTest, UseSeperateWidthsForEachAxis)
{
  DustPlume plume;

  plume.sigma_x = 2.0;
  plume.sigma_y = 3.0;
  plume.sigma_z = 4.0;

  const double expected = std::exp(-0.5);

  EXPECT_NEAR(plume.relative_density(2.0, 0.0, 0.0), expected, 1e-12);
  EXPECT_NEAR(plume.relative_density(0.0, 3.0, 0.0), expected, 1e-12);
  EXPECT_NEAR(plume.relative_density(0.0, 0.0, 4.0), expected, 1e-12);
}

TEST(DustPlumeTest, OpticalDepthUsesMidpointDensity)
{
  DustPlume plume;

  const double depth = plume.optical_depth(
    2.0, 0.0, 0.0, // Surface point: ray length is 2m
    0.5, // Interaction coefficent: 0.5 per meter
    2.0 // One segment covering the whole array
  );

  EXPECT_NEAR(depth, std::exp(-0.5), 1e-12);
}

TEST(DustPlumeTest, UnitOpticalDepthGivesExpectedProbability)
{
  DustPlume plume;
  plume.center_x = 1.0;

  const double probability = plume.interaction_probability(
    2.0, 0.0, 0.0,  // Ray endpoint.
    0.5,             // Interaction coefficient.
    2.0);            // One integration segment.

  EXPECT_NEAR(probability, 1.0 - std::exp(-1.0), 1e-12);
}

TEST(DustPlumeTest, FindsInteractionWithinSegment)
{
  DustPlume plume;
  plume.center_x = 1.0;

  const auto distance = plume.first_interaction_distance(
    2.0, 0.0, 0.0,
    0.5,                  // Interaction coefficient.
    2.0,                  // One segment.
    -std::expm1(-0.5));    // Sample giving target depth 0.5.

  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(distance.value(), 1.0, 1e-12);
}

TEST(DustPlumeTest, ReturnsNoInteractionWhenThresholdExceedsRayDepth)
{
  DustPlume plume;
  plume.center_x = 1.0;

  const auto distance = plume.first_interaction_distance(
    2.0, 0.0, 0.0,
    0.5,                 // Interaction coefficient.
    2.0,                 // One segment.
    -std::expm1(-2.0));   // Sample giving target depth 2.0.

  EXPECT_FALSE(distance.has_value());
}
