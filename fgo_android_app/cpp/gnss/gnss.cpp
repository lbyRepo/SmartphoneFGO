#include "navgraph/gnss.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace navgraph {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kEarthRotationRadps = 7.2921151467e-5;
constexpr int kStateSize = 4;

double dot(const Vec3& left, const Vec3& right) {
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

double norm(const Vec3& value) { return std::sqrt(dot(value, value)); }

Vec3 subtract(const Vec3& left, const Vec3& right) {
  return {left[0] - right[0], left[1] - right[1], left[2] - right[2]};
}

bool finite(const Vec3& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

Vec3 rotate_satellite(const Vec3& position, double transit_seconds) {
  const double angle = kEarthRotationRadps * transit_seconds;
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return {cosine * position[0] + sine * position[1],
          -sine * position[0] + cosine * position[1], position[2]};
}

double huber_weight(double standardized_residual, double cutoff = 2.5) {
  const double magnitude = std::abs(standardized_residual);
  return magnitude <= cutoff ? 1.0 : cutoff / magnitude;
}

class NormalEquations {
 public:
  void Add(const std::array<double, kStateSize>& row, double residual,
           double scale) {
    for (int i = 0; i < kStateSize; ++i) {
      const double weighted_i = row[i] * scale;
      if (weighted_i == 0.0) continue;
      gradient_[i] += weighted_i * residual * scale;
      for (int j = i; j < kStateSize; ++j) {
        const double value = weighted_i * row[j] * scale;
        if (value == 0.0) continue;
        hessian_[i][j] += value;
        if (i != j) hessian_[j][i] += value;
      }
    }
  }

  bool Solve(double damping, std::array<double, kStateSize>* solution) const {
    auto matrix = hessian_;
    std::array<double, kStateSize> right{};
    for (int index = 0; index < kStateSize; ++index) {
      matrix[index][index] += damping;
      right[index] = -gradient_[index];
    }
    for (int column = 0; column < kStateSize; ++column) {
      int pivot = column;
      for (int row = column + 1; row < kStateSize; ++row) {
        if (std::abs(matrix[row][column]) >
            std::abs(matrix[pivot][column])) {
          pivot = row;
        }
      }
      if (!std::isfinite(matrix[pivot][column]) ||
          std::abs(matrix[pivot][column]) < 1.0e-14) {
        return false;
      }
      if (pivot != column) {
        std::swap(matrix[column], matrix[pivot]);
        std::swap(right[column], right[pivot]);
      }
      const double divisor = matrix[column][column];
      for (int row = column + 1; row < kStateSize; ++row) {
        const double factor = matrix[row][column] / divisor;
        if (factor == 0.0) continue;
        matrix[row][column] = 0.0;
        for (int entry = column + 1; entry < kStateSize; ++entry) {
          matrix[row][entry] -= factor * matrix[column][entry];
        }
        right[row] -= factor * right[column];
      }
    }
    for (int row = kStateSize - 1; row >= 0; --row) {
      double value = right[row];
      for (int column = row + 1; column < kStateSize; ++column) {
        value -= matrix[row][column] * (*solution)[column];
      }
      (*solution)[row] = value / matrix[row][row];
      if (!std::isfinite((*solution)[row])) return false;
    }
    return true;
  }

 private:
  std::array<std::array<double, kStateSize>, kStateSize> hessian_{};
  std::array<double, kStateSize> gradient_{};
};

ReceiverState perturb(const ReceiverState& state, int component, double amount) {
  ReceiverState result = state;
  if (component < 3) {
    result.position_ecef_m[component] += amount;
  } else if (component < 6) {
    result.velocity_ecef_mps[component - 3] += amount;
  } else if (component == 6) {
    result.clock_bias_m += amount;
  } else {
    result.clock_drift_mps += amount;
  }
  return result;
}

}  // namespace

GnssPrediction predict(const ReceiverState& receiver,
                       const GnssObservation& observation,
                       bool apply_earth_rotation) {
  const double transit_seconds =
      (observation.corrected_pseudorange_m - receiver.clock_bias_m) /
      kSpeedOfLightMps;
  const Vec3 satellite_position =
      apply_earth_rotation
          ? rotate_satellite(observation.satellite_position_m, transit_seconds)
          : observation.satellite_position_m;
  const Vec3 delta = subtract(receiver.position_ecef_m, satellite_position);
  const double distance = norm(delta);
  GnssPrediction result;
  if (!std::isfinite(distance) || distance <= 1.0) {
    result.pseudorange_m = std::numeric_limits<double>::quiet_NaN();
    result.range_rate_mps = std::numeric_limits<double>::quiet_NaN();
    result.range_derivative_ecef.fill(
        std::numeric_limits<double>::quiet_NaN());
    result.range_rate_position_derivative_ecef.fill(
        std::numeric_limits<double>::quiet_NaN());
    return result;
  }
  for (int axis = 0; axis < 3; ++axis) {
    result.range_derivative_ecef[axis] = delta[axis] / distance;
  }
  Vec3 relative_velocity{};
  for (int axis = 0; axis < 3; ++axis) {
    const double satellite_velocity =
        std::isfinite(observation.satellite_velocity_mps[axis])
            ? observation.satellite_velocity_mps[axis]
            : 0.0;
    relative_velocity[axis] =
        receiver.velocity_ecef_mps[axis] - satellite_velocity;
  }
  const double radial_velocity =
      dot(result.range_derivative_ecef, relative_velocity);
  for (int axis = 0; axis < 3; ++axis) {
    result.range_rate_position_derivative_ecef[axis] =
        (relative_velocity[axis] -
         result.range_derivative_ecef[axis] * radial_velocity) /
        distance;
  }
  result.pseudorange_m = distance + receiver.clock_bias_m;
  result.range_rate_mps = radial_velocity + receiver.clock_drift_mps;
  return result;
}

WlsSolution solve_wls(const GnssEpoch& epoch, const ReceiverState& initial,
                      bool apply_earth_rotation) {
  WlsSolution solution;
  solution.timestamp_millis = epoch.timestamp_millis;
  solution.state = initial;
  solution.observation_count = static_cast<int>(epoch.observations.size());
  if (epoch.observations.size() < 4 || !finite(initial.position_ecef_m) ||
      !finite(initial.velocity_ecef_mps) ||
      !std::isfinite(initial.clock_bias_m) ||
      !std::isfinite(initial.clock_drift_mps)) {
    return solution;
  }

  for (int iteration = 1; iteration <= 20; ++iteration) {
    solution.iterations = iteration;
    NormalEquations equations;
    int factors = 0;
    for (const GnssObservation& observation : epoch.observations) {
      if (!std::isfinite(observation.corrected_pseudorange_m) ||
          !std::isfinite(observation.pseudorange_sigma_m) ||
          observation.pseudorange_sigma_m <= 0.0) {
        continue;
      }
      const GnssPrediction prediction =
          predict(solution.state, observation, apply_earth_rotation);
      if (!std::isfinite(prediction.pseudorange_m)) continue;
      const double residual =
          prediction.pseudorange_m - observation.corrected_pseudorange_m;
      const double sigma = observation.pseudorange_sigma_m;
      const double scale = std::sqrt(huber_weight(residual / sigma)) / sigma;
      equations.Add({prediction.range_derivative_ecef[0],
                     prediction.range_derivative_ecef[1],
                     prediction.range_derivative_ecef[2], 1.0},
                    residual, scale);
      ++factors;
    }
    std::array<double, kStateSize> step{};
    if (factors < 4 || !equations.Solve(1.0e-9, &step)) break;
    for (int axis = 0; axis < 3; ++axis) {
      solution.state.position_ecef_m[axis] += step[axis];
    }
    solution.state.clock_bias_m += step[3];
    const double position_step =
        std::sqrt(step[0] * step[0] + step[1] * step[1] +
                  step[2] * step[2]);
    if (position_step < 1.0e-3 && std::abs(step[3]) < 1.0e-3) {
      solution.converged = true;
      break;
    }
  }

  for (int pass = 0; pass < 3; ++pass) {
    NormalEquations equations;
    int factors = 0;
    for (const GnssObservation& observation : epoch.observations) {
      if (!std::isfinite(observation.corrected_range_rate_mps) ||
          !std::isfinite(observation.range_rate_sigma_mps) ||
          observation.range_rate_sigma_mps <= 0.0) {
        continue;
      }
      const GnssPrediction prediction =
          predict(solution.state, observation, apply_earth_rotation);
      if (!std::isfinite(prediction.range_rate_mps)) continue;
      const double residual =
          prediction.range_rate_mps - observation.corrected_range_rate_mps;
      const double sigma = std::max(observation.range_rate_sigma_mps, 0.15);
      const double scale = std::sqrt(huber_weight(residual / sigma)) / sigma;
      equations.Add({prediction.range_derivative_ecef[0],
                     prediction.range_derivative_ecef[1],
                     prediction.range_derivative_ecef[2], 1.0},
                    residual, scale);
      ++factors;
    }
    std::array<double, kStateSize> step{};
    if (factors < 4 || !equations.Solve(1.0e-9, &step)) break;
    for (int axis = 0; axis < 3; ++axis) {
      solution.state.velocity_ecef_mps[axis] += step[axis];
    }
    solution.state.clock_drift_mps += step[3];
    const double step_norm = std::sqrt(step[0] * step[0] + step[1] * step[1] +
                                       step[2] * step[2] + step[3] * step[3]);
    if (step_norm < 1.0e-5) break;
  }

  for (const GnssObservation& observation : epoch.observations) {
    if (!std::isfinite(observation.pseudorange_sigma_m) ||
        observation.pseudorange_sigma_m <= 0.0) {
      ++solution.rejected_count;
      continue;
    }
    const GnssPrediction prediction =
        predict(solution.state, observation, apply_earth_rotation);
    if (!std::isfinite(prediction.pseudorange_m) ||
        std::abs(prediction.pseudorange_m -
                 observation.corrected_pseudorange_m) /
                observation.pseudorange_sigma_m >
            8.0) {
      ++solution.rejected_count;
    }
  }
  solution.numerically_valid =
      finite(solution.state.position_ecef_m) &&
      finite(solution.state.velocity_ecef_mps) &&
      std::isfinite(solution.state.clock_bias_m) &&
      std::isfinite(solution.state.clock_drift_mps);
  return solution;
}

double max_abs_pseudorange_jacobian_error(
    const ReceiverState& receiver, const GnssObservation& observation,
    bool apply_earth_rotation) {
  const GnssPrediction prediction =
      predict(receiver, observation, apply_earth_rotation);
  std::array<double, 8> analytic{};
  for (int axis = 0; axis < 3; ++axis) {
    analytic[axis] = prediction.range_derivative_ecef[axis];
  }
  analytic[6] = 1.0;
  double maximum = 0.0;
  for (int component = 0; component < 8; ++component) {
    const double epsilon = component < 7 ? 1.0e-3 : 1.0e-4;
    const double plus = predict(perturb(receiver, component, epsilon), observation,
                                apply_earth_rotation)
                            .pseudorange_m;
    const double minus =
        predict(perturb(receiver, component, -epsilon), observation,
                apply_earth_rotation)
            .pseudorange_m;
    const double numerical = (plus - minus) / (2.0 * epsilon);
    maximum = std::max(maximum, std::abs(analytic[component] - numerical));
  }
  return maximum;
}

double max_abs_doppler_jacobian_error(const ReceiverState& receiver,
                                      const GnssObservation& observation,
                                      bool apply_earth_rotation) {
  const GnssPrediction prediction =
      predict(receiver, observation, apply_earth_rotation);
  std::array<double, 8> analytic{};
  for (int axis = 0; axis < 3; ++axis) {
    analytic[axis] = prediction.range_rate_position_derivative_ecef[axis];
    analytic[axis + 3] = prediction.range_derivative_ecef[axis];
  }
  analytic[7] = 1.0;
  double maximum = 0.0;
  for (int component = 0; component < 8; ++component) {
    const double epsilon = component < 7 ? 1.0e-3 : 1.0e-4;
    const double plus = predict(perturb(receiver, component, epsilon), observation,
                                apply_earth_rotation)
                            .range_rate_mps;
    const double minus =
        predict(perturb(receiver, component, -epsilon), observation,
                apply_earth_rotation)
            .range_rate_mps;
    const double numerical = (plus - minus) / (2.0 * epsilon);
    maximum = std::max(maximum, std::abs(analytic[component] - numerical));
  }
  return maximum;
}

}  // namespace navgraph
