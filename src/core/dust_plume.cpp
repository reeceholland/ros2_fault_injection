/**
 * @file dust_plume.cpp
 * @brief Gaussian plume density and numerical ray-interaction model.
 * Assumes a nonhomogeneous Poisson interaction process with survival exp(-tau).
 * This approximates interactions; it does not simulate particles or sensor detection.
 */
#include <cmath>
#include <stdexcept>
#include <cstddef>
#include <string>

#include "ros2_fault_injection/core/dust_plume.hpp"

namespace ros2_fault_injection::core
{

/**
 * @details Normalised squared distance is:
 * @code
 * q_squared = ((x - center_x) / sigma_x)^2
 *           + ((y - center_y) / sigma_y)^2
 *           + ((z - center_z) / sigma_z)^2
 * rho = exp(-0.5 * q_squared)
 * @endcode
 * Here ^2 denotes a mathematical square, not C++ XOR.
 * The field has unit peak density; its spatial integral is not normalised to one.
 */
double DustPlume::relative_density(double x, double y, double z) const
{
  validate();

  const double dx = (x - center_x) / sigma_x;
  const double dy = (y - center_y) / sigma_y;
  const double dz = (z - center_z) / sigma_z;
  return std::exp(-0.5 * (dx * dx + dy * dy + dz * dz));
}

/**
 * @details Optical depth is k times the integral of relative density along the ray.
 * With N = ceil(L / step_size), sample each segment at its midpoint:
 * @code
 * tau = k * segment_length * sum(midpoint_densities)
 * @endcode
 * Density times distance has units of metres; k [inverse metres] makes tau dimensionless.
 */
double DustPlume::optical_depth(
  double x, double y, double z, double interaction_coefficient,
  double step_size) const
{
  validate();

  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    throw std::invalid_argument("DustPlume: x, y, and z must be finite.");
  }

  if (!std::isfinite(interaction_coefficient) || interaction_coefficient < 0.0) {
    throw std::invalid_argument(
        "DustPlume: interaction_coefficient must be non-negative and finite.");
  }

  if (!std::isfinite(step_size) || step_size <= 0.0) {
    throw std::invalid_argument("DustPlume: step_size must be positive and finite.");
  }

  if(interaction_coefficient == 0.0) {
    return 0.0;
  }

  const double length = std::hypot(x, y, z);

  if(length == 0.0) {
    return 0.0;
  }

  constexpr std::size_t max_segments = 10000; // Limit computation per ray.

  const double count = std::ceil(length / step_size);

  if(!std::isfinite(count) || count < 1.0 || count > static_cast<double>(max_segments)) {
    throw std::invalid_argument("DustPlume: ray integration requires between 1 and " +
        std::to_string(max_segments) + ".");
  }

  const auto segment_count = static_cast<std::size_t>(count);

  const double segment_length = length / static_cast<double>(segment_count);

  double density_sum = 0.0;

  for(std::size_t i = 0; i < segment_count; ++i) {
    const double fraction = (static_cast<double>(i) + 0.5) / static_cast<double>(segment_count);

    density_sum += relative_density(x * fraction, y * fraction, z * fraction);
  }

  return interaction_coefficient * segment_length * density_sum;
}

/**
 * @details -expm1(-tau) equals 1 - exp(-tau), but avoids cancellation
 * when subtracting nearly equal floating-point numbers at small optical depths.
 */
double DustPlume::interaction_probability(
  double x, double y, double z,
  double interaction_coefficient, double step_size) const
{
  const double depth = optical_depth(x, y, z, interaction_coefficient, step_size);
  return -std::expm1(-depth);
}

/**
 * @details Transform the supplied sample u to threshold -ln(1-u).
 * Accumulate optical depth until a segment crosses the threshold, then interpolate
 * within that segment using the fraction of optical depth still needed.
 * The strict comparison excludes the final endpoint in exact arithmetic.
 * If the threshold is not crossed, no interaction occurs before the surface.
 */
std::optional<double> DustPlume::first_interaction_distance(
  double x, double y, double z,
  double interaction_coefficient,
  double step_size,
  double uniform_sample) const
{
  validate();

  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    throw std::invalid_argument("DustPlume: x, y, and z must be finite.");
  }

  if (!std::isfinite(interaction_coefficient) || interaction_coefficient < 0.0) {
    throw std::invalid_argument(
        "DustPlume: interaction_coefficient must be non-negative and finite.");
  }

  if (!std::isfinite(step_size) || step_size <= 0.0) {
    throw std::invalid_argument("DustPlume: step_size must be positive and finite.");
  }

  if (!std::isfinite(uniform_sample) || uniform_sample < 0.0 || uniform_sample >= 1.0) {
    throw std::invalid_argument(
        "DustPlume: uniform_sample must be finite and be in the range [0, 1).");
  }

  const double target_depth = -std::log1p(-uniform_sample);

  const double length = std::hypot(x, y, z);

  if (length == 0.0 || interaction_coefficient == 0.0) {
    return std::nullopt;
  }

  constexpr std::size_t max_segments = 10000;
  const double count = std::ceil(length / step_size);

  if (!std::isfinite(count) ||
    count < 1.0 ||
    count > static_cast<double>(max_segments))
  {
    throw std::invalid_argument(
      "DustPlume: ray integration requires between 1 and " + std::to_string(max_segments) +
        " segments.");
  }

  const auto segment_count = static_cast<std::size_t>(count);
  const double segment_length = length / static_cast<double>(segment_count);

  double accumulated_depth = 0.0;

  for (std::size_t i = 0; i < segment_count; ++i) {
    const double fraction =
      (static_cast<double>(i) + 0.5) /
      static_cast<double>(segment_count);

    const double density = relative_density(
    fraction * x, fraction * y, fraction * z);

    const double segment_depth =
      interaction_coefficient * density * segment_length;

    const double remaining_depth = target_depth - accumulated_depth;

    if (segment_depth > 0.0 && remaining_depth < segment_depth) {
      const double fraction_into_segment = remaining_depth / segment_depth;
      const double segment_start =
        static_cast<double>(i) * segment_length;

      return segment_start + fraction_into_segment * segment_length;
    }

    accumulated_depth += segment_depth;
  }

  return std::nullopt;

}

/**
 * @details Negative centre coordinates are valid positions. Widths must be
 * strictly positive because relative_density() divides coordinate offsets by them.
 */
void DustPlume::validate() const
{
  if (!std::isfinite(center_x) || !std::isfinite(center_y) || !std::isfinite(center_z)) {
    throw std::invalid_argument("DustPlume: center coordinates must be finite.");
  }

  if (!std::isfinite(sigma_x) || sigma_x <= 0.0) {
    throw std::invalid_argument("DustPlume: sigma_x must be positive and finite.");
  }

  if (!std::isfinite(sigma_y) || sigma_y <= 0.0) {
    throw std::invalid_argument("DustPlume: sigma_y must be positive and finite.");
  }

  if (!std::isfinite(sigma_z) || sigma_z <= 0.0) {
    throw std::invalid_argument("DustPlume: sigma_z must be positive and finite.");
  }
}

} // namespace ros2_fault_injection::core
