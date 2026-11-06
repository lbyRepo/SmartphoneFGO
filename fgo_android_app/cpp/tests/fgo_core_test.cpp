#include "navgraph/fgo.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL fgo_core_test: " << message << '\n';
    std::exit(1);
  }
}

double distance(const navgraph::Vec3& left, const navgraph::Vec3& right) {
  const double x = left[0] - right[0];
  const double y = left[1] - right[1];
  const double z = left[2] - right[2];
  return std::sqrt(x * x + y * y + z * z);
}

navgraph::GnssEpoch make_epoch(const navgraph::ReceiverState& truth,
                               std::int64_t timestamp_millis,
                               std::int64_t timestamp_nanos) {
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
  navgraph::GnssEpoch epoch;
  epoch.timestamp_millis = timestamp_millis;
  epoch.integration_timestamp_nanos = timestamp_nanos;
  for (std::size_t index = 0; index < satellite_positions.size(); ++index) {
    navgraph::GnssObservation observation;
    observation.timestamp_millis = timestamp_millis;
    observation.svid = static_cast<int>(index + 1);
    observation.satellite_position_m = satellite_positions[index];
    observation.satellite_velocity_mps = {0.0, 0.0, 0.0};
    observation.corrected_pseudorange_m = 2.3e7;
    observation.pseudorange_sigma_m = 2.5;
    observation.corrected_range_rate_mps = 0.0;
    observation.range_rate_sigma_mps = 0.15;
    const navgraph::GnssPrediction prediction =
        navgraph::predict(truth, observation, false);
    observation.corrected_pseudorange_m = prediction.pseudorange_m;
    observation.corrected_range_rate_mps = prediction.range_rate_mps;
    epoch.observations.push_back(observation);
  }
  return epoch;
}

}  // namespace

int main() {
  const navgraph::LocalFrame frame{
      {6378137.0, 0.0, 0.0},
      {0.0, 1.0, 0.0,
       0.0, 0.0, 1.0,
       1.0, 0.0, 0.0}};
  std::vector<navgraph::ReceiverState> truth;
  std::vector<navgraph::GnssEpoch> epochs;
  std::vector<navgraph::ReceiverState> seeds;
  for (int index = 0; index < 3; ++index) {
    const navgraph::Vec3 position_enu = {static_cast<double>(index), 0.0, 0.0};
    navgraph::ReceiverState state;
    state.position_ecef_m = navgraph::enu_position_to_ecef(frame, position_enu);
    state.velocity_ecef_mps =
        navgraph::enu_vector_to_ecef(frame, {1.0, 0.0, 0.0});
    state.clock_bias_m = 50.0 + 0.2 * index;
    state.clock_drift_mps = 0.2;
    truth.push_back(state);
    epochs.push_back(make_epoch(state, 1000 + index * 1000,
                                10'000'000'000LL + index * 1'000'000'000LL));
    navgraph::ReceiverState seed = state;
    seed.position_ecef_m = navgraph::enu_position_to_ecef(
        frame, {position_enu[0] + 5.0, -3.0, 2.0});
    seed.velocity_ecef_mps =
        navgraph::enu_vector_to_ecef(frame, {1.4, -0.3, 0.2});
    seed.clock_bias_m -= 12.0;
    seed.clock_drift_mps += 0.4;
    seeds.push_back(seed);
  }
  epochs[1].observations[2].corrected_pseudorange_m += 35.0;
  const std::vector<navgraph::FgoPreintegration> preintegrations = {
      {1.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      {1.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  };
  const double initial_error = distance(seeds.back().position_ecef_m,
                                        truth.back().position_ecef_m);
  const navgraph::FgoWindowSolution solution = navgraph::solve_fgo_window(
      frame, epochs, seeds, preintegrations, false);
  expect(solution.status == navgraph::FgoStatus::kOk, "solver status");
  expect(solution.converged, "solver did not converge");
  expect(solution.states.size() == 3, "state count");
  const navgraph::Vec3 final_position = navgraph::enu_position_to_ecef(
      frame, {solution.states.back()[0], solution.states.back()[1],
              solution.states.back()[2]});
  const double final_error =
      distance(final_position, truth.back().position_ecef_m);
  expect(final_error < initial_error, "FGO did not reduce final position error");
  expect(final_error < 2.0, "FGO final position error is unexpectedly large");
  expect(solution.uncertainty.valid, "missing marginal uncertainty");
  expect(solution.uncertainty.position_sigma_enu_m[0] > 0.0,
         "invalid position uncertainty");

  navgraph::LocalFrame invalid_frame = frame;
  invalid_frame.ecef_to_enu[3] = invalid_frame.ecef_to_enu[0];
  invalid_frame.ecef_to_enu[4] = invalid_frame.ecef_to_enu[1];
  invalid_frame.ecef_to_enu[5] = invalid_frame.ecef_to_enu[2];
  expect(navgraph::solve_fgo_window(invalid_frame, epochs, seeds,
                                    preintegrations, false)
             .status == navgraph::FgoStatus::kInvalidFrame,
         "invalid frame status");

  auto invalid_preintegrations = preintegrations;
  invalid_preintegrations[0].delta_time_seconds = 0.0;
  expect(navgraph::solve_fgo_window(frame, epochs, seeds,
                                    invalid_preintegrations, false)
             .status == navgraph::FgoStatus::kNonFiniteInput,
         "invalid preintegration status");

  auto mistimed_epochs = epochs;
  mistimed_epochs[1].integration_timestamp_nanos += 10'000'000LL;
  expect(navgraph::solve_fgo_window(frame, mistimed_epochs, seeds,
                                    preintegrations, false)
             .status == navgraph::FgoStatus::kInvalidInput,
         "preintegration/epoch timestamp mismatch status");

  std::cout << "PASS fgo_core_test iterations=" << solution.iterations
            << " position_error_before_m=" << initial_error
            << " position_error_after_m=" << final_error
            << " sigma_e_m="
            << solution.uncertainty.position_sigma_enu_m[0] << '\n';
  return 0;
}
