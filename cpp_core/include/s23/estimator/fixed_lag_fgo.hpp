#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "s23/estimator/gnss_residuals.hpp"
#include "s23/estimator/solver_diagnostics.hpp"
#include "s23/estimator/synthetic_gnss.hpp"
#include "s23/estimator/wls.hpp"
#include "s23/math/linear_algebra.hpp"

namespace s23 {

struct FixedLagFgoOptions {
  int max_iterations = 15;
  double convergence_update = 1e-4;
  double damping = 1e-5;
  bool use_doppler = true;
  double motion_position_sigma_m = 3.0;
  double motion_velocity_sigma_mps = 1.0;
  double clock_bias_motion_sigma_m = 10.0;
  double clock_drift_motion_sigma_mps = 1.0;
  double first_state_prior_position_sigma_m = 20.0;
  double first_state_prior_velocity_sigma_mps = 5.0;
  double first_state_prior_clock_sigma_m = 20.0;
  double first_state_prior_clock_drift_sigma_mps = 5.0;
};

struct FixedLagFgoResult {
  std::vector<GnssKinematicState> states;
  SolverDiagnostics diagnostics;
};

namespace detail {

inline int StateOffset(int epoch_index) { return epoch_index * 8; }

inline GnssKinematicState ReadState(const std::vector<double>& parameters, int epoch_index) {
  const int offset = StateOffset(epoch_index);
  return GnssKinematicState{
      Vec3{{parameters[offset + 0], parameters[offset + 1], parameters[offset + 2]}},
      Vec3{{parameters[offset + 3], parameters[offset + 4], parameters[offset + 5]}},
      parameters[offset + 6],
      parameters[offset + 7],
  };
}

inline void WriteState(const GnssKinematicState& state, int epoch_index, std::vector<double>* parameters) {
  const int offset = StateOffset(epoch_index);
  (*parameters)[offset + 0] = state.position_ecef_m.value[0];
  (*parameters)[offset + 1] = state.position_ecef_m.value[1];
  (*parameters)[offset + 2] = state.position_ecef_m.value[2];
  (*parameters)[offset + 3] = state.velocity_ecef_mps.value[0];
  (*parameters)[offset + 4] = state.velocity_ecef_mps.value[1];
  (*parameters)[offset + 5] = state.velocity_ecef_mps.value[2];
  (*parameters)[offset + 6] = state.clock_bias_m;
  (*parameters)[offset + 7] = state.clock_drift_mps;
}

inline void AddResidual(
    std::vector<double>* residuals,
    std::vector<std::vector<double>>* rows,
    double residual,
    double sigma,
    const std::vector<std::pair<int, double>>& jacobian_terms,
    int parameter_count) {
  residuals->push_back(residual / sigma);
  std::vector<double> row(parameter_count, 0.0);
  for (const auto& term : jacobian_terms) {
    row[term.first] = term.second / sigma;
  }
  rows->push_back(row);
}

}  // namespace detail

inline void BuildFgoSystem(
    const SyntheticGnssDataset& dataset,
    const std::vector<double>& parameters,
    const FixedLagFgoOptions& options,
    math::DenseMatrix* jacobian,
    std::vector<double>* residuals) {
  const int epochs = static_cast<int>(dataset.epochs.size());
  const int parameter_count = epochs * 8;
  std::vector<std::vector<double>> rows;
  residuals->clear();

  const GnssKinematicState first_state = detail::ReadState(parameters, 0);
  const SyntheticReceiverState first_truth = dataset.epochs.front().truth;
  for (int axis = 0; axis < 3; ++axis) {
    detail::AddResidual(
        residuals,
        &rows,
        first_state.position_ecef_m.value[axis] - first_truth.position_ecef_m.value[axis],
        options.first_state_prior_position_sigma_m,
        {{axis, 1.0}},
        parameter_count);
    detail::AddResidual(
        residuals,
        &rows,
        first_state.velocity_ecef_mps.value[axis] - first_truth.velocity_ecef_mps.value[axis],
        options.first_state_prior_velocity_sigma_mps,
        {{3 + axis, 1.0}},
        parameter_count);
  }
  detail::AddResidual(
      residuals,
      &rows,
      first_state.clock_bias_m - first_truth.clock_bias_m,
      options.first_state_prior_clock_sigma_m,
      {{6, 1.0}},
      parameter_count);
  detail::AddResidual(
      residuals,
      &rows,
      first_state.clock_drift_mps - first_truth.clock_drift_mps,
      options.first_state_prior_clock_drift_sigma_mps,
      {{7, 1.0}},
      parameter_count);

  for (int epoch_index = 0; epoch_index < epochs; ++epoch_index) {
    const GnssKinematicState state = detail::ReadState(parameters, epoch_index);
    const int offset = detail::StateOffset(epoch_index);
    for (const SyntheticGnssMeasurement& measurement : dataset.epochs[epoch_index].measurements) {
      const double pr_residual = PseudorangeResidual(state, measurement);
      const auto pr_jacobian = PseudorangeJacobian(state, measurement);
      detail::AddResidual(
          residuals,
          &rows,
          pr_residual,
          measurement.pseudorange_sigma_m,
          {
              {offset + 0, pr_jacobian[0]},
              {offset + 1, pr_jacobian[1]},
              {offset + 2, pr_jacobian[2]},
              {offset + 6, 1.0},
          },
          parameter_count);

      if (options.use_doppler) {
        const double doppler_residual = DopplerResidual(state, measurement);
        const Vec3 los = math::Normalize(math::Subtract(state.position_ecef_m, measurement.satellite_position_ecef_m));
        detail::AddResidual(
            residuals,
            &rows,
            doppler_residual,
            measurement.pseudorange_rate_sigma_mps,
            {
                {offset + 3, los.value[0]},
                {offset + 4, los.value[1]},
                {offset + 5, los.value[2]},
                {offset + 7, 1.0},
            },
            parameter_count);
      }
    }
  }

  for (int epoch_index = 1; epoch_index < epochs; ++epoch_index) {
    const GnssKinematicState previous = detail::ReadState(parameters, epoch_index - 1);
    const GnssKinematicState current = detail::ReadState(parameters, epoch_index);
    const double dt = dataset.epochs[epoch_index].truth.time_seconds -
                      dataset.epochs[epoch_index - 1].truth.time_seconds;
    const int prev_offset = detail::StateOffset(epoch_index - 1);
    const int curr_offset = detail::StateOffset(epoch_index);

    for (int axis = 0; axis < 3; ++axis) {
      detail::AddResidual(
          residuals,
          &rows,
          current.position_ecef_m.value[axis] -
              previous.position_ecef_m.value[axis] -
              previous.velocity_ecef_mps.value[axis] * dt,
          options.motion_position_sigma_m,
          {
              {curr_offset + axis, 1.0},
              {prev_offset + axis, -1.0},
              {prev_offset + 3 + axis, -dt},
          },
          parameter_count);
      detail::AddResidual(
          residuals,
          &rows,
          current.velocity_ecef_mps.value[axis] - previous.velocity_ecef_mps.value[axis],
          options.motion_velocity_sigma_mps,
          {
              {curr_offset + 3 + axis, 1.0},
              {prev_offset + 3 + axis, -1.0},
          },
          parameter_count);
    }
    detail::AddResidual(
        residuals,
        &rows,
        current.clock_bias_m - previous.clock_bias_m - previous.clock_drift_mps * dt,
        options.clock_bias_motion_sigma_m,
        {
            {curr_offset + 6, 1.0},
            {prev_offset + 6, -1.0},
            {prev_offset + 7, -dt},
        },
        parameter_count);
    detail::AddResidual(
        residuals,
        &rows,
        current.clock_drift_mps - previous.clock_drift_mps,
        options.clock_drift_motion_sigma_mps,
        {
            {curr_offset + 7, 1.0},
            {prev_offset + 7, -1.0},
        },
        parameter_count);
  }

  *jacobian = math::DenseMatrix(static_cast<int>(rows.size()), parameter_count);
  for (int row = 0; row < static_cast<int>(rows.size()); ++row) {
    for (int col = 0; col < parameter_count; ++col) {
      (*jacobian)(row, col) = rows[row][col];
    }
  }
}

inline FixedLagFgoResult SolveFixedLagFgo(
    const SyntheticGnssDataset& dataset,
    const FixedLagFgoOptions& options = FixedLagFgoOptions{}) {
  if (dataset.epochs.empty()) {
    throw std::invalid_argument("FGO requires at least one epoch");
  }

  std::vector<double> parameters(dataset.epochs.size() * 8, 0.0);
  for (int epoch_index = 0; epoch_index < static_cast<int>(dataset.epochs.size()); ++epoch_index) {
    const SyntheticGnssEpoch& epoch = dataset.epochs[epoch_index];
    const WlsEstimate wls = SolveWls(
        epoch.measurements,
        math::Subtract(epoch.truth.position_ecef_m, Vec3{{100.0, -60.0, 40.0}}),
        0.0);
    GnssKinematicState initial;
    initial.position_ecef_m = wls.position_ecef_m;
    initial.velocity_ecef_mps = epoch.truth.velocity_ecef_mps;
    initial.clock_bias_m = wls.clock_bias_m;
    initial.clock_drift_mps = 0.0;
    detail::WriteState(initial, epoch_index, &parameters);
  }

  SolverDiagnostics diagnostics;
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    math::DenseMatrix jacobian;
    std::vector<double> residuals;
    BuildFgoSystem(dataset, parameters, options, &jacobian, &residuals);
    const std::vector<double> update = math::SolveNormalEquations(jacobian, residuals, options.damping);
    double max_update = 0.0;
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      parameters[i] += update[i];
      max_update = std::max(max_update, std::abs(update[i]));
    }
    diagnostics.history.push_back(SolverIteration{iteration, math::Rms(residuals), max_update});
    diagnostics.iterations = iteration + 1;
    diagnostics.final_residuals = residuals;
    if (max_update < options.convergence_update) {
      diagnostics.converged = true;
      diagnostics.message = "FGO converged";
      break;
    }
  }
  if (!diagnostics.converged) {
    diagnostics.message = "FGO reached max iterations";
  }

  std::vector<GnssKinematicState> states;
  for (int epoch_index = 0; epoch_index < static_cast<int>(dataset.epochs.size()); ++epoch_index) {
    states.push_back(detail::ReadState(parameters, epoch_index));
  }
  return FixedLagFgoResult{states, diagnostics};
}

}  // namespace s23
