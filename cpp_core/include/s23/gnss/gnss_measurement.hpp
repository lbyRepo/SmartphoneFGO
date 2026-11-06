#pragma once

#include <optional>
#include <string>
#include <vector>

#include "s23/core/types.hpp"

namespace s23 {

struct GnssClockSample {
  Timestamp callback_timestamp;
  int64_t time_nanos = 0;
  int hardware_clock_discontinuity_count = 0;
  std::optional<int> leap_second;
  std::optional<double> time_uncertainty_nanos;
  std::optional<int64_t> full_bias_nanos;
  std::optional<double> bias_nanos;
  std::optional<double> bias_uncertainty_nanos;
  std::optional<double> drift_nanos_per_second;
  std::optional<double> drift_uncertainty_nanos_per_second;
  std::optional<int64_t> elapsed_realtime_nanos_from_clock;
  std::optional<double> elapsed_realtime_uncertainty_nanos;
};

struct GnssRawMeasurement {
  Timestamp callback_timestamp;
  int constellation_type = 0;
  int svid = 0;
  int state = 0;
  int64_t received_sv_time_nanos = 0;
  int64_t received_sv_time_uncertainty_nanos = 0;
  double cn0_dbhz = 0.0;
  double pseudorange_rate_mps = 0.0;
  double pseudorange_rate_uncertainty_mps = 0.0;
  int accumulated_delta_range_state = 0;
  double accumulated_delta_range_m = 0.0;
  double accumulated_delta_range_uncertainty_m = 0.0;
  int multipath_indicator = 0;
  std::optional<double> carrier_frequency_hz;
  std::optional<std::string> code_type;
  std::optional<double> baseband_cn0_dbhz;
  std::optional<double> automatic_gain_control_db;
  std::optional<double> satellite_inter_signal_bias_nanos;
  std::optional<double> satellite_inter_signal_bias_uncertainty_nanos;
  std::optional<double> full_inter_signal_bias_nanos;
  std::optional<double> full_inter_signal_bias_uncertainty_nanos;
};

struct GnssEpoch {
  GnssClockSample clock;
  std::vector<GnssRawMeasurement> measurements;
};

}  // namespace s23
