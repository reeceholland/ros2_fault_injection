// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

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
    0.0, 0.0, 0.0, // Ray origin.
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
    0.0, 0.0, 0.0, // Ray origin.
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
    0.0, 0.0, 0.0, // Ray origin.
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
    0.0, 0.0, 0.0, // Ray origin.
    2.0, 0.0, 0.0,
    0.5,                 // Interaction coefficient.
    2.0,                 // One segment.
    -std::expm1(-2.0));   // Sample giving target depth 2.0.

  EXPECT_FALSE(distance.has_value());
}

TEST(DustPlumeTest, TranslatingPlumeAndRayPreservesInteractions)
{
  DustPlume plume;
  plume.center_x = 2.0;
  plume.center_y = 1.0;
  plume.center_z = -0.5;
  plume.sigma_y = 0.7;
  plume.sigma_z = 0.4;
  auto shifted = plume;
  shifted.center_x += 10.0;
  shifted.center_y -= 4.0;
  shifted.center_z += 3.0;

  const double depth = plume.optical_depth(0, 0, 0, 4, 2, -1, 2.0, 0.1);
  ASSERT_GT(depth, 0.0);
  EXPECT_NEAR(depth,
    shifted.optical_depth(10, -4, 3, 14, -2, 2, 2.0, 0.1), 1e-12);
  EXPECT_NEAR(plume.interaction_probability(0, 0, 0, 4, 2, -1, 2.0, 0.1),
    shifted.interaction_probability(10, -4, 3, 14, -2, 2, 2.0, 0.1), 1e-12);

  const auto distance = plume.first_interaction_distance(0, 0, 0, 4, 2, -1, 2.0, 0.1, 0.2);
  const auto translated = shifted.first_interaction_distance(
    10, -4, 3, 14, -2, 2, 2.0, 0.1, 0.2);
  ASSERT_TRUE(distance.has_value());
  ASSERT_TRUE(translated.has_value());
  EXPECT_NEAR(*distance, *translated, 1e-12);
}

TEST(DustPlumeTest, CoincidentNonzeroEndpointsGiveNoInteraction)
{
  DustPlume plume;
  EXPECT_DOUBLE_EQ(plume.optical_depth(10, -4, 3, 10, -4, 3, 2.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(plume.interaction_probability(10, -4, 3, 10, -4, 3, 2.0, 0.1), 0.0);
  EXPECT_FALSE(plume.first_interaction_distance(10, -4, 3, 10, -4, 3, 2.0, 0.1, 0.2));
}

TEST(DustPlumeTest, RejectsInvalidOriginsAndOverflowingDisplacement)
{
  DustPlume plume;
  const double inf = std::numeric_limits<double>::infinity();
  const double largest = std::numeric_limits<double>::max();
  for (const double coefficient : {0.0, 1.0}) {
    EXPECT_THROW(plume.optical_depth(inf, 0, 0, 1, 0, 0, coefficient, 0.1), std::invalid_argument);
    EXPECT_THROW(plume.first_interaction_distance(inf, 0, 0, 1, 0, 0, coefficient, 0.1, 0.2),
      std::invalid_argument);
    EXPECT_THROW(plume.optical_depth(-largest, 0, 0, largest, 0, 0, coefficient, 0.1),
      std::invalid_argument);
    EXPECT_THROW(plume.interaction_probability(-largest, 0, 0, largest, 0, 0, coefficient, 0.1),
      std::invalid_argument);
    EXPECT_THROW(plume.first_interaction_distance(
      -largest, 0, 0, largest, 0, 0, coefficient, 0.1, 0.2), std::invalid_argument);
  }
}
