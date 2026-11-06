#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "s23/estimator/gnss_residuals.hpp"
#include "s23/estimator/solver_diagnostics.hpp"
#include "s23/math/linear_algebra.hpp"

namespace s23 {

struct WlsEstimate {
  Vec3 position_ecef_m;
  double clock_bias_m = 0.0;
  SolverDiagnostics diagnostics;
};

struct WlsOptions {
  int max_iterations = 12;
  double convergence_update_m = 1e-4;
  double damping = 1e-6;
};

inline WlsEstimate SolveWls(
    const std::vector<SyntheticGnssMeasurement>& measurements,
    const Vec3& initial_position_ecef_m,
    double initial_clock_bias_m = 0.0,
    const WlsOptions& options = WlsOptions{}) {
  if (measurements.size() < 4) {
    throw std::invalid_argument("WLS requires at least 4 pseudorange measurements");
  }

  Vec3 position = initial_position_ecef_m;
  double clock_bias = initial_clock_bias_m;
  SolverDiagnostics diagnostics;

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    math::DenseMatrix jacobian(static_cast<int>(measurements.size()), 4);
    std::vector<double> residuals(measurements.size(), 0.0);

    for (std::size_t row = 0; row < measurements.size(); ++row) {
      GnssKinematicState state;
      state.position_ecef_m = position;
      state.clock_bias_m = clock_bias;
      residuals[row] = PseudorangeResidual(state, measurements[row]) / measurements[row].pseudorange_sigma_m;
      const auto full_jacobian = PseudorangeJacobian(state, measurements[row]);
      jacobian(static_cast<int>(row), 0) = full_jacobian[0] / measurements[row].pseudorange_sigma_m;
      jacobian(static_cast<int>(row), 1) = full_jacobian[1] / measurements[row].pseudorange_sigma_m;
      jacobian(static_cast<int>(row), 2) = full_jacobian[2] / measurements[row].pseudorange_sigma_m;
      jacobian(static_cast<int>(row), 3) = full_jacobian[6] / measurements[row].pseudorange_sigma_m;
    }

    const std::vector<double> update = math::SolveNormalEquations(jacobian, residuals, options.damping);
    position.value[0] += update[0];
    position.value[1] += update[1];
    position.value[2] += update[2];
    clock_bias += update[3];

    const double max_update = std::max({
        std::abs(update[0]),
        std::abs(update[1]),
        std::abs(update[2]),
        std::abs(update[3]),
    });
    diagnostics.history.push_back(SolverIteration{iteration, math::Rms(residuals), max_update});
    diagnostics.iterations = iteration + 1;
    diagnostics.final_residuals = residuals;

    if (max_update < options.convergence_update_m) {
      diagnostics.converged = true;
      diagnostics.message = "WLS converged";
      break;
    }
  }

  if (!diagnostics.converged) {
    diagnostics.message = "WLS reached max iterations";
  }
  return WlsEstimate{position, clock_bias, diagnostics};
}

}  // namespace s23
