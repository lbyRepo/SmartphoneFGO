#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "s23/estimator/ekf_baseline.hpp"
#include "s23/estimator/fixed_lag_fgo.hpp"
#include "s23/estimator/gnss_residuals.hpp"
#include "s23/estimator/imu_propagator.hpp"
#include "s23/estimator/synthetic_gnss.hpp"
#include "s23/estimator/wls.hpp"
#include "s23/math/linear_algebra.hpp"

namespace {

void ExpectNear(double actual, double expected, double tolerance, const char* label) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << label << ": expected " << expected << " got " << actual << "\n";
    std::abort();
  }
}

void ExpectLess(double actual, double threshold, const char* label) {
  if (!(actual < threshold)) {
    std::cerr << label << ": expected < " << threshold << " got " << actual << "\n";
    std::abort();
  }
}

double PositionError(const s23::Vec3& a, const s23::Vec3& b) {
  return s23::math::Distance(a, b);
}

void TestWlsSyntheticConvergence() {
  const s23::SyntheticGnssDataset dataset = s23::MakeSyntheticGnssDataset();
  const s23::SyntheticGnssEpoch& epoch = dataset.epochs.front();
  const s23::Vec3 initial = s23::math::Subtract(epoch.truth.position_ecef_m, s23::Vec3{{250.0, -180.0, 90.0}});
  const s23::WlsEstimate estimate = s23::SolveWls(epoch.measurements, initial, 0.0);

  assert(estimate.diagnostics.converged);
  ExpectLess(PositionError(estimate.position_ecef_m, epoch.truth.position_ecef_m), 2.0, "WLS position error");
  ExpectNear(estimate.clock_bias_m, epoch.truth.clock_bias_m, 2.0, "WLS clock bias");
  assert(!estimate.diagnostics.history.empty());
  assert(!estimate.diagnostics.final_residuals.empty());
}

void TestFixedLagFgoConvergenceAndComparison() {
  const s23::SyntheticGnssDataset dataset = s23::MakeSyntheticGnssDataset();
  const s23::FixedLagFgoResult result = s23::SolveFixedLagFgo(dataset);
  assert(result.diagnostics.converged);
  assert(result.states.size() == dataset.epochs.size());
  assert(!result.diagnostics.history.empty());
  assert(!result.diagnostics.final_residuals.empty());

  double fgo_position_error_sum = 0.0;
  double wls_position_error_sum = 0.0;
  for (std::size_t i = 0; i < dataset.epochs.size(); ++i) {
    const auto& epoch = dataset.epochs[i];
    const s23::WlsEstimate wls = s23::SolveWls(
        epoch.measurements,
        s23::math::Subtract(epoch.truth.position_ecef_m, s23::Vec3{{250.0, -180.0, 90.0}}),
        0.0);
    fgo_position_error_sum += PositionError(result.states[i].position_ecef_m, epoch.truth.position_ecef_m);
    wls_position_error_sum += PositionError(wls.position_ecef_m, epoch.truth.position_ecef_m);
    ExpectLess(PositionError(result.states[i].position_ecef_m, epoch.truth.position_ecef_m), 3.0, "FGO position error");
    ExpectLess(s23::math::Distance(result.states[i].velocity_ecef_mps, epoch.truth.velocity_ecef_mps), 0.5, "FGO velocity error");
    ExpectNear(result.states[i].clock_drift_mps, epoch.truth.clock_drift_mps, 0.5, "FGO clock drift");
  }
  ExpectLess(fgo_position_error_sum / dataset.epochs.size(), 3.0, "FGO average position error");
  ExpectLess(wls_position_error_sum / dataset.epochs.size(), 3.0, "WLS average position error");
}

void TestPseudorangeJacobianMatchesNumerical() {
  const s23::SyntheticGnssDataset dataset = s23::MakeSyntheticGnssDataset();
  const auto& epoch = dataset.epochs.front();
  s23::GnssKinematicState state{
      s23::math::Add(epoch.truth.position_ecef_m, s23::Vec3{{10.0, -5.0, 3.0}}),
      epoch.truth.velocity_ecef_mps,
      epoch.truth.clock_bias_m + 1.0,
      epoch.truth.clock_drift_mps,
  };
  const std::vector<double> analytic = s23::PseudorangeJacobian(state, epoch.measurements.front());
  const std::vector<double> numeric = s23::NumericJacobian(state, epoch.measurements.front(), s23::PseudorangeResidual);
  ExpectLess(s23::MaxAbsDifference(analytic, numeric), 1e-5, "Pseudorange Jacobian max abs difference");
}

void TestDopplerFactorAffectsVelocity() {
  const s23::SyntheticGnssDataset dataset = s23::MakeSyntheticGnssDataset();
  const s23::FixedLagFgoResult with_doppler = s23::SolveFixedLagFgo(dataset);
  assert(with_doppler.diagnostics.converged);

  double doppler_velocity_error = 0.0;
  for (std::size_t i = 0; i < dataset.epochs.size(); ++i) {
    doppler_velocity_error += s23::math::Distance(
        with_doppler.states[i].velocity_ecef_mps,
        dataset.epochs[i].truth.velocity_ecef_mps);
    ExpectNear(with_doppler.states[i].clock_drift_mps, dataset.epochs[i].truth.clock_drift_mps, 0.5, "Doppler clock drift");
  }
  ExpectLess(doppler_velocity_error / dataset.epochs.size(), 0.2, "Doppler velocity average error");
}

void TestImuAndEkfBaselineInterfaces() {
  s23::NavState initial;
  initial.position_ecef_m = s23::Vec3{{1.0, 2.0, 3.0}};
  initial.velocity_ecef_mps = s23::Vec3{{2.0, 0.0, 0.0}};
  s23::ImuSample accel;
  accel.timestamp.elapsed_realtime_nanos = 1'000'000'000;
  accel.values = s23::Vec3{{1.0, 0.0, 0.0}};

  s23::ImuPropagator propagator;
  const s23::NavState propagated = propagator.PropagateConstantAttitude(initial, accel, 2.0);
  ExpectNear(propagated.position_ecef_m.value[0], 7.0, 1e-9, "IMU propagated position");
  ExpectNear(propagated.velocity_ecef_mps.value[0], 4.0, 1e-9, "IMU propagated velocity");

  s23::EkfGnssInsBaseline ekf;
  ekf.Initialize(initial);
  ekf.PropagateImu(accel, 1.0);
  ekf.CorrectWithGnssPosition(s23::Vec3{{10.0, 2.0, 3.0}});
  const auto state = ekf.LatestState();
  assert(state.has_value());
  assert(ekf.LatestStatus().accepted_measurements == 1);
}

}  // namespace

int main() {
  TestWlsSyntheticConvergence();
  TestFixedLagFgoConvergenceAndComparison();
  TestPseudorangeJacobianMatchesNumerical();
  TestDopplerFactorAffectsVelocity();
  TestImuAndEkfBaselineInterfaces();
  std::cout << "estimator_core_test passed\n";
  return 0;
}
