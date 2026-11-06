#include <iostream>

#include "s23/estimator/fixed_lag_fgo.hpp"
#include "s23/estimator/synthetic_gnss.hpp"
#include "s23/estimator/wls.hpp"
#include "s23/math/linear_algebra.hpp"

int main() {
  const s23::SyntheticGnssDataset dataset = s23::MakeSyntheticGnssDataset();
  const auto& first_epoch = dataset.epochs.front();
  const s23::WlsEstimate wls = s23::SolveWls(
      first_epoch.measurements,
      s23::math::Subtract(first_epoch.truth.position_ecef_m, s23::Vec3{{250.0, -180.0, 90.0}}),
      0.0);
  const s23::FixedLagFgoResult fgo = s23::SolveFixedLagFgo(dataset);

  std::cout << "WLS converged: " << wls.diagnostics.converged
            << " iterations: " << wls.diagnostics.iterations
            << " final_rms: " << s23::math::Rms(wls.diagnostics.final_residuals)
            << " position_error_m: " << s23::math::Distance(wls.position_ecef_m, first_epoch.truth.position_ecef_m)
            << "\n";
  for (const auto& iteration : wls.diagnostics.history) {
    std::cout << "  WLS iter " << iteration.iteration
              << " rms=" << iteration.rms_residual
              << " max_update=" << iteration.max_update << "\n";
  }

  std::cout << "FGO converged: " << fgo.diagnostics.converged
            << " iterations: " << fgo.diagnostics.iterations
            << " final_rms: " << s23::math::Rms(fgo.diagnostics.final_residuals)
            << "\n";
  for (const auto& iteration : fgo.diagnostics.history) {
    std::cout << "  FGO iter " << iteration.iteration
              << " rms=" << iteration.rms_residual
              << " max_update=" << iteration.max_update << "\n";
  }
  for (std::size_t i = 0; i < fgo.states.size(); ++i) {
    std::cout << "  epoch " << i
              << " fgo_position_error_m="
              << s23::math::Distance(fgo.states[i].position_ecef_m, dataset.epochs[i].truth.position_ecef_m)
              << " fgo_velocity_error_mps="
              << s23::math::Distance(fgo.states[i].velocity_ecef_mps, dataset.epochs[i].truth.velocity_ecef_mps)
              << "\n";
  }
  return 0;
}
