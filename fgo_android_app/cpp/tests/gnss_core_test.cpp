#include "navgraph/gnss.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using navgraph::GnssEpoch;
using navgraph::GnssObservation;
using navgraph::ReceiverState;
using navgraph::Vec3;

double norm(const Vec3& value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

Vec3 subtract(const Vec3& left, const Vec3& right) {
  return {left[0] - right[0], left[1] - right[1], left[2] - right[2]};
}

double dot(const Vec3& left, const Vec3& right) {
  return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

GnssEpoch synthetic_epoch(const ReceiverState& truth) {
  const std::vector<Vec3> satellites = {
      {15600000.0, 7540000.0, 20140000.0},
      {18760000.0, 2750000.0, 18610000.0},
      {17610000.0, 14630000.0, 13480000.0},
      {19170000.0, 610000.0, 18390000.0},
      {17800000.0, -13200000.0, 14800000.0},
      {21100000.0, 9000000.0, -12000000.0},
      {-15000000.0, 18000000.0, 14000000.0},
      {-17000000.0, -13000000.0, 16000000.0},
  };
  const std::vector<Vec3> velocities = {
      {-900.0, 2100.0, -400.0},   {-1200.0, 1700.0, 600.0},
      {-1500.0, -900.0, 1800.0},  {-700.0, 2300.0, -300.0},
      {1400.0, 1200.0, -1000.0},  {-1800.0, 500.0, 1100.0},
      {900.0, -1500.0, 1300.0},   {1200.0, 800.0, -1700.0},
  };
  GnssEpoch epoch;
  epoch.timestamp_millis = 12345;
  epoch.integration_timestamp_nanos = 12345000000LL;
  for (std::size_t index = 0; index < satellites.size(); ++index) {
    const Vec3 delta = subtract(truth.position_ecef_m, satellites[index]);
    const double distance = norm(delta);
    const Vec3 line_of_sight = {delta[0] / distance, delta[1] / distance,
                                delta[2] / distance};
    const Vec3 relative_velocity = {
        truth.velocity_ecef_mps[0] - velocities[index][0],
        truth.velocity_ecef_mps[1] - velocities[index][1],
        truth.velocity_ecef_mps[2] - velocities[index][2],
    };
    GnssObservation observation;
    observation.timestamp_millis = epoch.timestamp_millis;
    observation.constellation = 1;
    observation.svid = static_cast<int>(index + 1);
    observation.signal_type = "SYNTHETIC_L1";
    observation.satellite_position_m = satellites[index];
    observation.satellite_velocity_mps = velocities[index];
    observation.corrected_pseudorange_m = distance + truth.clock_bias_m;
    observation.pseudorange_sigma_m = 2.5;
    observation.corrected_range_rate_mps =
        dot(line_of_sight, relative_velocity) + truth.clock_drift_mps;
    observation.range_rate_sigma_mps = 0.15;
    observation.cn0_dbhz = 40.0;
    observation.elevation_deg = 35.0;
    epoch.observations.push_back(observation);
  }
  return epoch;
}

}  // namespace

int main() {
  const ReceiverState truth{{-1500000.0, 6190000.0, 150000.0},
                            {12.0, -3.0, 0.8}, 87.5, 0.22};
  const GnssEpoch epoch = synthetic_epoch(truth);
  const navgraph::WlsSolution solution =
      navgraph::solve_wls(epoch, ReceiverState{}, false);
  require(solution.numerically_valid, "solution must remain finite");
  require(solution.converged, "position WLS must converge");
  require(norm(subtract(solution.state.position_ecef_m,
                        truth.position_ecef_m)) < 1.0e-3,
          "position must match deterministic truth below 1 mm");
  require(norm(subtract(solution.state.velocity_ecef_mps,
                        truth.velocity_ecef_mps)) < 1.0e-4,
          "velocity must match deterministic truth below 0.1 mm/s");
  require(std::abs(solution.state.clock_bias_m - truth.clock_bias_m) < 1.0e-3,
          "clock bias must match truth");
  require(std::abs(solution.state.clock_drift_mps - truth.clock_drift_mps) <
              1.0e-4,
          "clock drift must match truth");
  require(solution.rejected_count == 0, "perfect observations are not outliers");

  const double pseudorange_error =
      navgraph::max_abs_pseudorange_jacobian_error(truth,
                                                   epoch.observations.front(), false);
  const double doppler_error = navgraph::max_abs_doppler_jacobian_error(
      truth, epoch.observations.front(), false);
  require(pseudorange_error < 2.0e-5,
          "pseudorange Jacobian must pass central differences");
  require(doppler_error < 2.0e-5,
          "Doppler Jacobian must pass central differences");

  GnssEpoch rank_deficient = epoch;
  rank_deficient.observations.resize(3);
  const navgraph::WlsSolution rejected =
      navgraph::solve_wls(rank_deficient, ReceiverState{}, false);
  require(!rejected.converged && !rejected.numerically_valid,
          "fewer than four observations must fail explicitly");

  GnssEpoch invalid_sigma = epoch;
  for (auto& observation : invalid_sigma.observations) {
    observation.pseudorange_sigma_m =
        std::numeric_limits<double>::quiet_NaN();
  }
  const navgraph::WlsSolution invalid =
      navgraph::solve_wls(invalid_sigma, ReceiverState{}, false);
  require(!invalid.converged,
          "non-finite observation weights must not enter normal equations");

  std::cout << "PASS gnss_core_test position_error_m="
            << norm(subtract(solution.state.position_ecef_m,
                             truth.position_ecef_m))
            << " velocity_error_mps="
            << norm(subtract(solution.state.velocity_ecef_mps,
                             truth.velocity_ecef_mps))
            << " pr_jacobian=" << pseudorange_error
            << " doppler_jacobian=" << doppler_error << '\n';
  return 0;
}
