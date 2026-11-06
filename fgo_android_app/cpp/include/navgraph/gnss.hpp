#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace navgraph {

using Vec3 = std::array<double, 3>;

struct GnssObservation {
  std::int64_t timestamp_millis = 0;
  int constellation = 0;
  int svid = 0;
  std::string signal_type;
  Vec3 satellite_position_m{};
  Vec3 satellite_velocity_mps{};
  double corrected_pseudorange_m = 0.0;
  double pseudorange_sigma_m = 1.0;
  double corrected_range_rate_mps = 0.0;
  double range_rate_sigma_mps = 0.0;
  double cn0_dbhz = 0.0;
  double elevation_deg = 0.0;
};

struct GnssEpoch {
  std::int64_t timestamp_millis = 0;
  std::int64_t integration_timestamp_nanos = 0;
  std::vector<GnssObservation> observations;
};

struct ReceiverState {
  Vec3 position_ecef_m{};
  Vec3 velocity_ecef_mps{};
  double clock_bias_m = 0.0;
  double clock_drift_mps = 0.0;
};

struct GnssPrediction {
  double pseudorange_m = 0.0;
  double range_rate_mps = 0.0;
  Vec3 range_derivative_ecef{};
  Vec3 range_rate_position_derivative_ecef{};
};

struct WlsSolution {
  std::int64_t timestamp_millis = 0;
  ReceiverState state;
  int iterations = 0;
  int observation_count = 0;
  int rejected_count = 0;
  bool converged = false;
  bool numerically_valid = false;
};

GnssPrediction predict(const ReceiverState& receiver,
                       const GnssObservation& observation,
                       bool apply_earth_rotation = true);

WlsSolution solve_wls(const GnssEpoch& epoch,
                      const ReceiverState& initial = ReceiverState{},
                      bool apply_earth_rotation = true);

double max_abs_pseudorange_jacobian_error(
    const ReceiverState& receiver,
    const GnssObservation& observation,
    bool apply_earth_rotation = true);

double max_abs_doppler_jacobian_error(
    const ReceiverState& receiver,
    const GnssObservation& observation,
    bool apply_earth_rotation = true);

}  // namespace navgraph
