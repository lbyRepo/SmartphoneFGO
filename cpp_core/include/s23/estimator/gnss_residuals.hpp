#pragma once

#include <cmath>
#include <vector>

#include "s23/estimator/synthetic_gnss.hpp"
#include "s23/math/linear_algebra.hpp"

namespace s23 {

struct GnssKinematicState {
  Vec3 position_ecef_m;
  Vec3 velocity_ecef_mps;
  double clock_bias_m = 0.0;
  double clock_drift_mps = 0.0;
};

inline double PseudorangeResidual(
    const GnssKinematicState& state,
    const SyntheticGnssMeasurement& measurement) {
  const double geometric_range = math::Distance(state.position_ecef_m, measurement.satellite_position_ecef_m);
  return geometric_range + state.clock_bias_m - measurement.pseudorange_m;
}

inline std::vector<double> PseudorangeJacobian(
    const GnssKinematicState& state,
    const SyntheticGnssMeasurement& measurement) {
  const Vec3 receiver_minus_sat = math::Subtract(state.position_ecef_m, measurement.satellite_position_ecef_m);
  const Vec3 unit = math::Normalize(receiver_minus_sat);
  return {unit.value[0], unit.value[1], unit.value[2], 0.0, 0.0, 0.0, 1.0, 0.0};
}

inline double DopplerResidual(
    const GnssKinematicState& state,
    const SyntheticGnssMeasurement& measurement) {
  const Vec3 receiver_minus_sat = math::Subtract(state.position_ecef_m, measurement.satellite_position_ecef_m);
  const Vec3 line_of_sight_sat_to_rx = math::Normalize(receiver_minus_sat);
  const double predicted_rate =
      math::Dot(line_of_sight_sat_to_rx, math::Subtract(state.velocity_ecef_mps, measurement.satellite_velocity_ecef_mps)) +
      state.clock_drift_mps;
  return predicted_rate - measurement.pseudorange_rate_mps;
}

inline std::vector<double> NumericJacobian(
    const GnssKinematicState& state,
    const SyntheticGnssMeasurement& measurement,
    double (*residual)(const GnssKinematicState&, const SyntheticGnssMeasurement&),
    double step = 1e-3) {
  std::vector<double> jacobian(8, 0.0);
  for (int i = 0; i < 8; ++i) {
    GnssKinematicState plus = state;
    GnssKinematicState minus = state;
    auto perturb = [i](GnssKinematicState& target, double delta) {
      if (i < 3) {
        target.position_ecef_m.value[i] += delta;
      } else if (i < 6) {
        target.velocity_ecef_mps.value[i - 3] += delta;
      } else if (i == 6) {
        target.clock_bias_m += delta;
      } else {
        target.clock_drift_mps += delta;
      }
    };
    perturb(plus, step);
    perturb(minus, -step);
    jacobian[i] = (residual(plus, measurement) - residual(minus, measurement)) / (2.0 * step);
  }
  return jacobian;
}

inline double MaxAbsDifference(const std::vector<double>& a, const std::vector<double>& b) {
  double max_abs = 0.0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
    max_abs = std::max(max_abs, std::abs(a[i] - b[i]));
  }
  return max_abs;
}

}  // namespace s23
