#pragma once

#include <optional>
#include <string>
#include <vector>

#include "s23/core/types.hpp"

namespace s23 {

enum class MeasurementRejectReason {
  kNone,
  kMissingSatelliteId,
  kConstellationRejected,
  kCn0BelowThreshold,
  kFrequencyRejected,
  kInvalidTimeState,
  kMissingClockBias,
  kPseudorangeUnavailable,
  kPseudorangeOutOfRange,
  kPseudorangeUncertaintyTooLarge,
  kPseudorangeRateInvalid,
  kPseudorangeRateUncertaintyTooLarge,
};

struct GnssObservation {
  Timestamp epoch_timestamp;
  int constellation_type = 0;
  std::string constellation_name;
  int svid = 0;
  double pseudorange_m = 0.0;
  double pseudorange_uncertainty_m = 0.0;
  double pseudorange_rate_mps = 0.0;
  double pseudorange_rate_uncertainty_mps = 0.0;
  double cn0_dbhz = 0.0;
  std::optional<double> carrier_frequency_hz;
  int64_t received_sv_time_nanos = 0;
  int raw_state = 0;
};

struct RejectedGnssMeasurement {
  Timestamp epoch_timestamp;
  std::optional<int> constellation_type;
  std::optional<int> svid;
  MeasurementRejectReason reason = MeasurementRejectReason::kNone;
  std::string detail;
};

struct GnssObservationEpoch {
  Timestamp epoch_timestamp;
  std::vector<GnssObservation> observations;
  std::vector<RejectedGnssMeasurement> rejected_measurements;
};

}  // namespace s23
