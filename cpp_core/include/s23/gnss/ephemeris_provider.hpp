#pragma once

#include <optional>

#include "s23/coordinates/coordinates.hpp"
#include "s23/gnss/observation.hpp"

namespace s23 {

struct SatelliteCorrection {
  Ecef satellite_position_ecef_m;
  Vec3 satellite_velocity_ecef_mps;
  double satellite_clock_bias_m = 0.0;
  double satellite_clock_drift_mps = 0.0;
  double ionosphere_delay_m = 0.0;
  double troposphere_delay_m = 0.0;
  double relativistic_correction_m = 0.0;
  double group_delay_m = 0.0;
};

class IEphemerisCorrectionProvider {
 public:
  virtual ~IEphemerisCorrectionProvider() = default;

  virtual std::optional<SatelliteCorrection> CorrectionFor(
      const GnssObservation& observation,
      const Timestamp& receiver_timestamp) const = 0;
};

}  // namespace s23
