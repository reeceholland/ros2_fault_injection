// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROS2_FAULT_INJECTION__CORE__DUST_PLUME_HPP
#define ROS2_FAULT_INJECTION__CORE__DUST_PLUME_HPP

#include <optional>

namespace ros2_fault_injection::core
{

/**
 * @brief Stationary, axis-aligned Gaussian plume with unit peak relative density.
 * Widths are standard deviations in metres, not hard boundaries. The Gaussian
 * shape is a modelling assumption, not a measured concentration law.
 * Ray methods take explicit origins and endpoints. Both and the plume centre
 * must use the same frame; this class applies no TF transforms or motion.
 * Models interactions only, not detection, intensity, or multiple returns.
 * No RNG is owned: callers supply samples. Public parameters remain mutable.
 */
class DustPlume
{
public:
  /// Centre X coordinate in metres; must be finite.
  double center_x{0.0};
  /// Centre Y coordinate in metres; must be finite.
  double center_y{0.0};
  /// Centre Z coordinate in metres; must be finite.
  double center_z{0.0};

  /// Gaussian width along X in metres; must be finite and positive.
  double sigma_x{1.0};
  /// Gaussian width along Y in metres; must be finite and positive.
  double sigma_y{1.0};
  /// Gaussian width along Z in metres; must be finite and positive.
  double sigma_z{1.0};

  /**
   * @brief Evaluate relative Gaussian density at a position.
   * @param x Position X in metres in the plume frame.
   * @param y Position Y in metres in the plume frame.
   * @param z Position Z in metres in the plume frame.
   * @pre Query coordinates must be finite; only plume parameters are checked here.
   * @return Dimensionless density, one at the centre and tending to zero with distance.
   * @throws std::invalid_argument If plume parameters are invalid.
   */
  double relative_density(double x, double y, double z) const;

  /**
   * @brief Approximate integrated interaction rate using the midpoint rule.
   * @param origin_x Ray origin X in metres in the plume frame.
   * @param endpoint_x Ray endpoint X in metres in the plume frame.
   * @param origin_y Ray origin Y in metres in the plume frame.
   * @param endpoint_y Ray endpoint Y in metres in the plume frame.
   * @param origin_z Ray origin Z in metres in the plume frame.
   * @param endpoint_z Ray endpoint Z in metres in the plume frame.
   * @param interaction_coefficient Finite non-negative peak interaction rate in inverse metres.
   * @param step_size Finite positive maximum integration segment length in metres.
   * @return Dimensionless optical depth; zero for zero length or zero coefficient.
   * @throws std::invalid_argument If plume or ray inputs are invalid, or integration
   * requires an invalid segment count or more than 10000 segments.
   * @note Zero-length and zero-coefficient cases skip segment-count calculation.
   * Step size controls resolution; it is not a guaranteed integration error bound.
   */
  double optical_depth(
    double origin_x, double origin_y, double origin_z,
    double endpoint_x, double endpoint_y, double endpoint_z,
    double interaction_coefficient,
    double step_size) const;

  /**
   * @brief Compute the probability of at least one interaction before the endpoint.
   * Uses 1 - exp(-optical_depth), not a calibrated detected-return probability.
   * @param origin_x Ray origin X in metres in the plume frame.
   * @param endpoint_x Ray endpoint X in metres in the plume frame.
   * @param origin_y Ray origin Y in metres in the plume frame.
   * @param endpoint_y Ray endpoint Y in metres in the plume frame.
   * @param origin_z Ray origin Z in metres in the plume frame.
   * @param endpoint_z Ray endpoint Z in metres in the plume frame.
   * @param interaction_coefficient Finite non-negative peak interaction rate in inverse metres.
   * @param step_size Finite positive maximum integration segment length in metres.
   * @return Probability in [0, 1] for a valid numerical optical depth.
   * @throws std::invalid_argument Under the same conditions as optical_depth().
   */
  double interaction_probability(
    double origin_x, double origin_y, double origin_z,
    double endpoint_x, double endpoint_y, double endpoint_z,
    double interaction_coefficient,
    double step_size) const;

  /**
   * @brief Sample the first interaction using accumulated optical depth.
   * Uses a constant midpoint interaction rate within each integration segment.
   * Do not precede this sampling with a separate probability coin flip.
   * @param origin_x Ray origin X in metres in the plume frame.
   * @param endpoint_x Ray endpoint X in metres in the plume frame.
   * @param origin_y Ray origin Y in metres in the plume frame.
   * @param endpoint_y Ray endpoint Y in metres in the plume frame.
   * @param origin_z Ray origin Z in metres in the plume frame.
   * @param endpoint_z Ray endpoint Z in metres in the plume frame.
   * @param interaction_coefficient Finite non-negative peak interaction rate in inverse metres.
   * @param step_size Finite positive maximum integration segment length in metres.
   * @param uniform_sample Finite uniform sample in [0, 1), supplied by the caller.
   * @return Distance from the supplied ray origin in metres, or std::nullopt when the threshold is not crossed
   * before the endpoint, the ray has zero length, or the coefficient is zero.
   * A zero sample may produce a zero-distance interaction.
   * @throws std::invalid_argument If plume or ray inputs, the sample, or the segment
   * count are invalid (maximum 10000 segments).
   * @note Zero-length and zero-coefficient cases skip segment-count calculation.
   */
  std::optional<double> first_interaction_distance(
    double origin_x, double origin_y, double origin_z,
    double endpoint_x, double endpoint_y, double endpoint_z,
    double interaction_coefficient,
    double step_size,
    double uniform_sample) const;

  /**
   * @brief Check finite centres and finite, strictly positive widths.
   * @throws std::invalid_argument If any plume parameter violates these conditions.
   * @note Does not validate query coordinates or integration settings.
   */
  void validate() const;
};

} // namespace ros2_fault_injection::core

#endif // ROS2_FAULT_INJECTION__CORE__DUST_PLUME_HPP
