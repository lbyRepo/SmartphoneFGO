#include "navgraph/ekf.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL ekf_core_test: " << message << '\n';
    std::exit(1);
  }
}

double norm(const navgraph::Vec3& value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

navgraph::GnssEpoch make_epoch(const navgraph::EkfFrame& frame,
                               std::int64_t timestamp_nanos) {
  const navgraph::ReceiverState truth{
      frame.reference_ecef_m, {8.0, -3.0, 1.5}, 42.0, -0.8};
  const std::vector<navgraph::Vec3> satellite_positions = {
      {15600000.0, 7540000.0, 20140000.0},
      {18760000.0, 2750000.0, 18610000.0},
      {17610000.0, 14630000.0, 13480000.0},
      {19170000.0, 610000.0, 18390000.0},
      {17800000.0, -12800000.0, 15000000.0},
      {21100000.0, 8300000.0, -9200000.0},
      {20200000.0, -7400000.0, 11800000.0},
      {16400000.0, 12100000.0, -15100000.0},
  };
  const std::vector<navgraph::Vec3> satellite_velocities = {
      {-1100.0, 2400.0, 700.0},  {-900.0, -2100.0, 1300.0},
      {1700.0, -800.0, 900.0},   {-2100.0, 400.0, -500.0},
      {1300.0, 1600.0, -1100.0}, {-700.0, 900.0, 2200.0},
      {600.0, -1800.0, -1400.0}, {1900.0, 300.0, 1200.0},
  };

  navgraph::GnssEpoch epoch;
  epoch.timestamp_millis = 1234567890;
  epoch.integration_timestamp_nanos = timestamp_nanos;
  for (std::size_t index = 0; index < satellite_positions.size(); ++index) {
    navgraph::GnssObservation observation;
    observation.timestamp_millis = epoch.timestamp_millis;
    observation.svid = static_cast<int>(index + 1);
    observation.satellite_position_m = satellite_positions[index];
    observation.satellite_velocity_mps = satellite_velocities[index];
    // Seed once to establish a realistic transit time, then evaluate the final
    // model with that pseudorange as the Earth-rotation input.
    observation.corrected_pseudorange_m = 2.3e7;
    observation.pseudorange_sigma_m = 2.0;
    observation.corrected_range_rate_mps = 0.0;
    observation.range_rate_sigma_mps = 0.3;
    auto prediction = navgraph::predict(truth, observation, true);
    observation.corrected_pseudorange_m = prediction.pseudorange_m;
    prediction = navgraph::predict(truth, observation, true);
    observation.corrected_pseudorange_m = prediction.pseudorange_m;
    observation.corrected_range_rate_mps = prediction.range_rate_mps;
    epoch.observations.push_back(observation);
  }
  epoch.observations[6].corrected_pseudorange_m += 1000.0;
  epoch.observations[7].corrected_range_rate_mps += 100.0;
  return epoch;
}

}  // namespace

int main() {
  const navgraph::EkfCovariance initial_covariance =
      navgraph::default_ekf_covariance();
  expect(navgraph::ekf_covariance_is_finite_symmetric(initial_covariance),
         "default covariance is not finite and symmetric");

  navgraph::InsState stationary;
  stationary.timestamp_nanos = 1'000'000'000LL;
  stationary.clock_bias_m = 12.0;
  stationary.clock_drift_mps = 0.25;
  const navgraph::ImuSample previous{
      stationary.timestamp_nanos, {0.0, 0.0, navgraph::kStandardGravityMps2},
      {0.0, 0.0, 0.0}};
  const navgraph::ImuSample current{
      stationary.timestamp_nanos + 10'000'000LL,
      {0.0, 0.0, navgraph::kStandardGravityMps2}, {0.0, 0.0, 0.0}};
  const navgraph::EkfStepResult propagated = navgraph::propagate_ekf(
      stationary, initial_covariance, previous, current);
  expect(propagated.status == navgraph::EkfStatus::kOk,
         "stationary propagation status");
  expect(propagated.health.propagation_steps == 1,
         "propagation health count");
  expect(norm(propagated.state.position_enu_m) < 1.0e-12,
         "stationary position moved");
  expect(norm(propagated.state.velocity_enu_mps) < 1.0e-12,
         "stationary velocity moved");
  expect(std::abs(propagated.state.clock_bias_m - 12.0025) < 1.0e-12,
         "clock bias propagation");
  expect(navgraph::ekf_covariance_is_finite_symmetric(propagated.covariance),
         "propagated covariance symmetry");

  const navgraph::EkfStepResult duplicate = navgraph::propagate_ekf(
      stationary, initial_covariance, previous, previous);
  expect(duplicate.status == navgraph::EkfStatus::kNonIncreasingTimestamp,
         "duplicate timestamp status");
  navgraph::EkfCovariance invalid_covariance = initial_covariance;
  invalid_covariance[5] = std::numeric_limits<double>::quiet_NaN();
  const navgraph::EkfStepResult invalid = navgraph::propagate_ekf(
      stationary, invalid_covariance, previous, current);
  expect(invalid.status == navgraph::EkfStatus::kNonFiniteInput,
         "non-finite covariance status");

  const navgraph::EkfFrame frame{
      {6378137.0, 0.0, 0.0},
      {0.0, 1.0, 0.0,
       0.0, 0.0, 1.0,
       1.0, 0.0, 0.0}};
  navgraph::InsState estimate;
  estimate.timestamp_nanos = 2'000'000'000LL;
  estimate.position_enu_m = {18.0, -12.0, 7.0};
  // Convert an ECEF truth velocity into ENU at latitude/longitude zero.
  estimate.velocity_enu_mps = {-1.0, 0.5, 5.0};
  estimate.clock_bias_m = 25.0;
  estimate.clock_drift_mps = 0.5;
  const navgraph::GnssEpoch epoch = make_epoch(frame, estimate.timestamp_nanos);
  const double initial_position_error = norm(estimate.position_enu_m);
  const navgraph::EkfStepResult updated = navgraph::update_ekf(
      frame, estimate, initial_covariance, epoch, true);
  expect(updated.status == navgraph::EkfStatus::kOk, "GNSS update status");
  expect(updated.health.accepted_pseudorange == 7,
         "expected seven accepted pseudoranges");
  expect(updated.health.rejected_pseudorange == 1,
         "expected pseudorange outlier rejection");
  expect(updated.health.accepted_doppler == 7,
         "expected seven accepted Doppler observations");
  expect(updated.health.rejected_doppler == 1,
         "expected Doppler outlier rejection");
  expect(norm(updated.state.position_enu_m) < initial_position_error,
         "GNSS update did not reduce position error");
  expect(navgraph::ekf_covariance_is_finite_symmetric(updated.covariance),
         "updated covariance symmetry");

  navgraph::GnssEpoch nonfinite_epoch = epoch;
  nonfinite_epoch.observations.resize(1);
  nonfinite_epoch.observations[0].corrected_pseudorange_m =
      std::numeric_limits<double>::quiet_NaN();
  nonfinite_epoch.observations[0].corrected_range_rate_mps =
      std::numeric_limits<double>::quiet_NaN();
  const navgraph::EkfStepResult nonfinite_observation = navgraph::update_ekf(
      frame, estimate, initial_covariance, nonfinite_epoch, true);
  expect(nonfinite_observation.status == navgraph::EkfStatus::kOk,
         "invalid observation should not poison the epoch");
  expect(nonfinite_observation.health.rejected_pseudorange == 1,
         "invalid pseudorange rejection count");
  expect(navgraph::ekf_covariance_is_finite_symmetric(
             nonfinite_observation.covariance),
         "invalid observation poisoned covariance");

  navgraph::GnssEpoch mistimed = epoch;
  mistimed.integration_timestamp_nanos += 1'000'001LL;
  const navgraph::EkfStepResult timing = navgraph::update_ekf(
      frame, estimate, initial_covariance, mistimed, true);
  expect(timing.status == navgraph::EkfStatus::kGnssTimestampMismatch,
         "GNSS timestamp mismatch status");

  navgraph::EkfFrame nonorthogonal_frame = frame;
  nonorthogonal_frame.ecef_to_enu = {
      1.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, 1.0};
  const navgraph::EkfStepResult frame_failure = navgraph::update_ekf(
      nonorthogonal_frame, estimate, initial_covariance, epoch, true);
  expect(frame_failure.status == navgraph::EkfStatus::kInvalidFrame,
         "non-orthogonal frame status");

  std::cout << "PASS ekf_core_test position_error_before_m="
            << initial_position_error << " position_error_after_m="
            << norm(updated.state.position_enu_m)
            << " accepted_pr=" << updated.health.accepted_pseudorange
            << " accepted_doppler=" << updated.health.accepted_doppler << '\n';
  return 0;
}
