#include "navgraph/fgo.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace navgraph {
namespace {

constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kAccelBiasIndex = 6;
constexpr int kClockBiasIndex = 9;
constexpr int kClockDriftIndex = 10;

bool finite(const Vec3& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

bool finite(const ReceiverState& value) {
  return finite(value.position_ecef_m) && finite(value.velocity_ecef_mps) &&
         std::isfinite(value.clock_bias_m) &&
         std::isfinite(value.clock_drift_mps);
}

bool finite(const FgoState& value) {
  return std::all_of(value.begin(), value.end(),
                     [](double entry) { return std::isfinite(entry); });
}

bool solve_linear_system(std::vector<double>* matrix,
                         std::vector<double>* right, int dimension) {
  for (int column = 0; column < dimension; ++column) {
    int pivot = column;
    for (int row = column + 1; row < dimension; ++row) {
      if (std::abs((*matrix)[row * dimension + column]) >
          std::abs((*matrix)[pivot * dimension + column])) {
        pivot = row;
      }
    }
    const double pivot_value = (*matrix)[pivot * dimension + column];
    if (!std::isfinite(pivot_value) || std::abs(pivot_value) < 1.0e-14) {
      return false;
    }
    if (pivot != column) {
      for (int entry = 0; entry < dimension; ++entry) {
        std::swap((*matrix)[column * dimension + entry],
                  (*matrix)[pivot * dimension + entry]);
      }
      std::swap((*right)[column], (*right)[pivot]);
    }
    const double divisor = (*matrix)[column * dimension + column];
    for (int row = column + 1; row < dimension; ++row) {
      const double factor = (*matrix)[row * dimension + column] / divisor;
      if (factor == 0.0) continue;
      (*matrix)[row * dimension + column] = 0.0;
      for (int entry = column + 1; entry < dimension; ++entry) {
        (*matrix)[row * dimension + entry] -=
            factor * (*matrix)[column * dimension + entry];
      }
      (*right)[row] -= factor * (*right)[column];
    }
  }
  for (int row = dimension - 1; row >= 0; --row) {
    double value = (*right)[row];
    for (int column = row + 1; column < dimension; ++column) {
      value -= (*matrix)[row * dimension + column] * (*right)[column];
    }
    (*right)[row] = value / (*matrix)[row * dimension + row];
    if (!std::isfinite((*right)[row])) return false;
  }
  return true;
}

class NormalEquations {
 public:
  explicit NormalEquations(int dimension)
      : dimension_(dimension),
        hessian_(static_cast<std::size_t>(dimension * dimension)),
        gradient_(static_cast<std::size_t>(dimension)) {}

  bool Add(const std::vector<double>& row, double residual, double scale) {
    if (static_cast<int>(row.size()) != dimension_ ||
        !std::isfinite(residual) || !std::isfinite(scale) || scale <= 0.0) {
      return false;
    }
    if (!std::all_of(row.begin(), row.end(),
                     [](double value) { return std::isfinite(value); })) {
      return false;
    }
    for (int i = 0; i < dimension_; ++i) {
      const double weighted_i = row[i] * scale;
      if (!std::isfinite(weighted_i)) return false;
      if (weighted_i == 0.0) continue;
      gradient_[i] += weighted_i * residual * scale;
      for (int j = i; j < dimension_; ++j) {
        const double value = weighted_i * row[j] * scale;
        if (!std::isfinite(value)) return false;
        if (value == 0.0) continue;
        hessian_[i * dimension_ + j] += value;
        if (i != j) hessian_[j * dimension_ + i] += value;
      }
    }
    return true;
  }

  bool Solve(double damping, std::vector<double>* solution) const {
    std::vector<double> matrix = hessian_;
    std::vector<double> right(static_cast<std::size_t>(dimension_));
    for (int entry = 0; entry < dimension_; ++entry) {
      matrix[entry * dimension_ + entry] += damping;
      right[entry] = -gradient_[entry];
    }
    if (!solve_linear_system(&matrix, &right, dimension_)) return false;
    *solution = std::move(right);
    return true;
  }

  bool InverseDiagonal(const std::array<int, 8>& requested, double damping,
                       std::array<double, 8>* output) const {
    for (std::size_t output_index = 0; output_index < requested.size();
         ++output_index) {
      const int requested_index = requested[output_index];
      if (requested_index < 0 || requested_index >= dimension_) return false;
      std::vector<double> matrix = hessian_;
      for (int entry = 0; entry < dimension_; ++entry) {
        matrix[entry * dimension_ + entry] += damping;
      }
      std::vector<double> right(static_cast<std::size_t>(dimension_));
      right[requested_index] = 1.0;
      if (!solve_linear_system(&matrix, &right, dimension_)) return false;
      const double variance = right[requested_index];
      if (!std::isfinite(variance) || variance <= 0.0) return false;
      (*output)[output_index] = variance;
    }
    return true;
  }

 private:
  int dimension_;
  std::vector<double> hessian_;
  std::vector<double> gradient_;
};

double huber_weight(double standardized_residual, double cutoff = 2.5) {
  const double magnitude = std::abs(standardized_residual);
  return magnitude <= cutoff ? 1.0 : cutoff / magnitude;
}

FgoState initial_state(const LocalFrame& frame, const ReceiverState& wls) {
  FgoState state{};
  const Vec3 position = ecef_position_to_enu(frame, wls.position_ecef_m);
  const Vec3 velocity = ecef_vector_to_enu(frame, wls.velocity_ecef_mps);
  for (int axis = 0; axis < 3; ++axis) {
    state[kPositionIndex + axis] = position[axis];
    state[kVelocityIndex + axis] = velocity[axis];
  }
  state[kClockBiasIndex] = wls.clock_bias_m;
  state[kClockDriftIndex] = wls.clock_drift_mps;
  return state;
}

ReceiverState receiver_state(const LocalFrame& frame, const FgoState& state) {
  return {enu_position_to_ecef(
              frame, {state[0], state[1], state[2]}),
          enu_vector_to_ecef(frame, {state[3], state[4], state[5]}),
          state[kClockBiasIndex], state[kClockDriftIndex]};
}

bool add_factor(NormalEquations* equations, const std::vector<double>& row,
                double residual, double sigma, bool robust) {
  if (!std::isfinite(sigma) || sigma <= 0.0 || !std::isfinite(residual)) {
    return false;
  }
  const double standardized = residual / sigma;
  const double weight = robust ? huber_weight(standardized) : 1.0;
  if (!std::isfinite(weight) || weight <= 0.0) return false;
  return equations->Add(row, residual, std::sqrt(weight) / sigma);
}

}  // namespace

FgoWindowSolution solve_fgo_window(
    const LocalFrame& frame, const std::vector<GnssEpoch>& epochs,
    const std::vector<ReceiverState>& wls,
    const std::vector<FgoPreintegration>& preintegrations,
    bool apply_earth_rotation) {
  FgoWindowSolution solution;
  if (epochs.empty() || epochs.size() != wls.size() ||
      preintegrations.size() + 1 != epochs.size()) {
    solution.status = FgoStatus::kInvalidInput;
    return solution;
  }
  if (!valid_local_frame(frame)) {
    solution.status = FgoStatus::kInvalidFrame;
    return solution;
  }
  for (const ReceiverState& seed : wls) {
    if (!finite(seed)) {
      solution.status = FgoStatus::kNonFiniteInput;
      return solution;
    }
    solution.states.push_back(initial_state(frame, seed));
  }
  for (std::size_t interval = 0; interval < preintegrations.size(); ++interval) {
    const FgoPreintegration& preintegration = preintegrations[interval];
    if (!std::isfinite(preintegration.delta_time_seconds) ||
        preintegration.delta_time_seconds <= 0.0 ||
        !finite(preintegration.delta_position_enu_m) ||
        !finite(preintegration.delta_velocity_enu_mps)) {
      solution.status = FgoStatus::kNonFiniteInput;
      return solution;
    }
    const std::int64_t start_nanos = epochs[interval].integration_timestamp_nanos;
    const std::int64_t end_nanos =
        epochs[interval + 1].integration_timestamp_nanos;
    if (end_nanos <= start_nanos) {
      solution.status = FgoStatus::kInvalidInput;
      return solution;
    }
    const long double delta_nanos = static_cast<long double>(end_nanos) -
                                    static_cast<long double>(start_nanos);
    const double expected_dt = static_cast<double>(delta_nanos * 1.0e-9L);
    const double tolerance = std::max(1.0e-6, expected_dt * 1.0e-6);
    if (!std::isfinite(expected_dt) ||
        std::abs(preintegration.delta_time_seconds - expected_dt) > tolerance) {
      solution.status = FgoStatus::kInvalidInput;
      return solution;
    }
  }

  const int state_count = static_cast<int>(epochs.size());
  const int dimension = state_count * kFgoStateSize;
  NormalEquations last_equations(dimension);
  bool have_equations = false;
  for (int iteration = 1; iteration <= 15; ++iteration) {
    solution.iterations = iteration;
    NormalEquations equations(dimension);

    const FgoState prior_state = initial_state(frame, wls.front());
    const std::array<double, kFgoStateSize> prior_sigmas = {
        8.0, 8.0, 12.0, 2.0, 2.0, 2.0,
        0.12, 0.12, 0.12, 100.0, 10.0};
    for (int component = 0; component < kFgoStateSize; ++component) {
      std::vector<double> row(static_cast<std::size_t>(dimension));
      row[component] = 1.0;
      if (!add_factor(&equations, row,
                      solution.states[0][component] - prior_state[component],
                      prior_sigmas[component], false)) {
        ++solution.skipped_factors;
      }
    }

    for (int epoch_index = 0; epoch_index < state_count; ++epoch_index) {
      const FgoState& state = solution.states[epoch_index];
      const ReceiverState receiver = receiver_state(frame, state);
      for (const GnssObservation& observation :
           epochs[epoch_index].observations) {
        const GnssPrediction prediction =
            predict(receiver, observation, apply_earth_rotation);
        const int offset = epoch_index * kFgoStateSize;
        std::vector<double> pseudorange_row(
            static_cast<std::size_t>(dimension));
        const Vec3 range_enu =
            ecef_vector_to_enu(frame, prediction.range_derivative_ecef);
        for (int axis = 0; axis < 3; ++axis) {
          pseudorange_row[offset + kPositionIndex + axis] = range_enu[axis];
        }
        pseudorange_row[offset + kClockBiasIndex] = 1.0;
        if (!add_factor(
                &equations, pseudorange_row,
                prediction.pseudorange_m -
                    observation.corrected_pseudorange_m,
                observation.pseudorange_sigma_m, true)) {
          ++solution.skipped_factors;
        }
        if (std::isfinite(observation.corrected_range_rate_mps) &&
            std::isfinite(observation.range_rate_sigma_mps)) {
          std::vector<double> doppler_row(
              static_cast<std::size_t>(dimension));
          const Vec3 position_derivative = ecef_vector_to_enu(
              frame, prediction.range_rate_position_derivative_ecef);
          for (int axis = 0; axis < 3; ++axis) {
            doppler_row[offset + kPositionIndex + axis] =
                position_derivative[axis];
            doppler_row[offset + kVelocityIndex + axis] = range_enu[axis];
          }
          doppler_row[offset + kClockDriftIndex] = 1.0;
          if (!add_factor(
                  &equations, doppler_row,
                  prediction.range_rate_mps -
                      observation.corrected_range_rate_mps,
                  observation.range_rate_sigma_mps, true)) {
            ++solution.skipped_factors;
          }
        }
      }
    }

    for (int interval_index = 0; interval_index < state_count - 1;
         ++interval_index) {
      const int first_offset = interval_index * kFgoStateSize;
      const int second_offset = (interval_index + 1) * kFgoStateSize;
      const FgoState& first = solution.states[interval_index];
      const FgoState& second = solution.states[interval_index + 1];
      const FgoPreintegration& preintegration =
          preintegrations[interval_index];
      const double dt = preintegration.delta_time_seconds;
      for (int axis = 0; axis < 3; ++axis) {
        std::vector<double> position_row(
            static_cast<std::size_t>(dimension));
        position_row[second_offset + kPositionIndex + axis] = 1.0;
        position_row[first_offset + kPositionIndex + axis] = -1.0;
        position_row[first_offset + kVelocityIndex + axis] = -dt;
        position_row[first_offset + kAccelBiasIndex + axis] =
            0.5 * dt * dt;
        const double position_residual =
            second[kPositionIndex + axis] - first[kPositionIndex + axis] -
            first[kVelocityIndex + axis] * dt -
            preintegration.delta_position_enu_m[axis] +
            0.5 * first[kAccelBiasIndex + axis] * dt * dt;
        if (!add_factor(&equations, position_row, position_residual, 0.20,
                        false)) {
          ++solution.skipped_factors;
        }

        std::vector<double> velocity_row(
            static_cast<std::size_t>(dimension));
        velocity_row[second_offset + kVelocityIndex + axis] = 1.0;
        velocity_row[first_offset + kVelocityIndex + axis] = -1.0;
        velocity_row[first_offset + kAccelBiasIndex + axis] = dt;
        const double velocity_residual =
            second[kVelocityIndex + axis] - first[kVelocityIndex + axis] -
            preintegration.delta_velocity_enu_mps[axis] +
            first[kAccelBiasIndex + axis] * dt;
        if (!add_factor(&equations, velocity_row, velocity_residual, 0.12,
                        false)) {
          ++solution.skipped_factors;
        }

        std::vector<double> bias_row(static_cast<std::size_t>(dimension));
        bias_row[second_offset + kAccelBiasIndex + axis] = 1.0;
        bias_row[first_offset + kAccelBiasIndex + axis] = -1.0;
        if (!add_factor(
                &equations, bias_row,
                second[kAccelBiasIndex + axis] -
                    first[kAccelBiasIndex + axis],
                0.02, false)) {
          ++solution.skipped_factors;
        }
      }

      std::vector<double> clock_row(static_cast<std::size_t>(dimension));
      clock_row[second_offset + kClockBiasIndex] = 1.0;
      clock_row[first_offset + kClockBiasIndex] = -1.0;
      clock_row[first_offset + kClockDriftIndex] = -dt;
      if (!add_factor(
              &equations, clock_row,
              second[kClockBiasIndex] - first[kClockBiasIndex] -
                  first[kClockDriftIndex] * dt,
              10.0, false)) {
        ++solution.skipped_factors;
      }
      std::vector<double> drift_row(static_cast<std::size_t>(dimension));
      drift_row[second_offset + kClockDriftIndex] = 1.0;
      drift_row[first_offset + kClockDriftIndex] = -1.0;
      if (!add_factor(
              &equations, drift_row,
              second[kClockDriftIndex] - first[kClockDriftIndex], 1.0,
              false)) {
        ++solution.skipped_factors;
      }
    }

    last_equations = equations;
    have_equations = true;
    std::vector<double> step;
    if (!equations.Solve(1.0e-6, &step)) {
      solution.status = FgoStatus::kLinearSolveFailure;
      break;
    }
    double maximum_position_step = 0.0;
    for (int state_index = 0; state_index < state_count; ++state_index) {
      const int offset = state_index * kFgoStateSize;
      double position_step_squared = 0.0;
      for (int component = 0; component < kFgoStateSize; ++component) {
        solution.states[state_index][component] += step[offset + component];
      }
      for (int axis = 0; axis < 3; ++axis) {
        position_step_squared += step[offset + kPositionIndex + axis] *
                                 step[offset + kPositionIndex + axis];
      }
      maximum_position_step =
          std::max(maximum_position_step, std::sqrt(position_step_squared));
      if (!finite(solution.states[state_index])) {
        solution.status = FgoStatus::kNonFiniteInput;
        return solution;
      }
    }
    if (maximum_position_step < 1.0e-3) {
      solution.converged = true;
      solution.status = FgoStatus::kOk;
      break;
    }
  }
  if (solution.status == FgoStatus::kInvalidInput) {
    solution.status = FgoStatus::kOk;
  }
  if (solution.converged && have_equations) {
    const int last_offset = (state_count - 1) * kFgoStateSize;
    const std::array<int, 8> requested = {
        last_offset + kPositionIndex,
        last_offset + kPositionIndex + 1,
        last_offset + kPositionIndex + 2,
        last_offset + kVelocityIndex,
        last_offset + kVelocityIndex + 1,
        last_offset + kVelocityIndex + 2,
        last_offset + kClockBiasIndex,
        last_offset + kClockDriftIndex};
    std::array<double, 8> variances{};
    if (last_equations.InverseDiagonal(requested, 1.0e-6, &variances)) {
      solution.uncertainty.valid = true;
      for (int axis = 0; axis < 3; ++axis) {
        solution.uncertainty.position_sigma_enu_m[axis] =
            std::sqrt(variances[axis]);
        solution.uncertainty.velocity_sigma_enu_mps[axis] =
            std::sqrt(variances[3 + axis]);
      }
      solution.uncertainty.clock_bias_sigma_m = std::sqrt(variances[6]);
      solution.uncertainty.clock_drift_sigma_mps = std::sqrt(variances[7]);
    }
  }
  return solution;
}

}  // namespace navgraph
