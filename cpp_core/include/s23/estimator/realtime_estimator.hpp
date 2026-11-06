#pragma once

#include <optional>
#include <string>

#include "s23/core/types.hpp"
#include "s23/gnss/fused_location_sample.hpp"
#include "s23/gnss/gnss_measurement.hpp"
#include "s23/ins/imu_sample.hpp"

namespace s23 {

struct EstimatorConfig {
  double output_rate_hz = 100.0;
  double fgo_window_seconds = 2.0;
  bool enable_fgo_feedback = false;
  bool enable_tdcp = false;
};

class IRealtimeEstimator {
 public:
  virtual ~IRealtimeEstimator() = default;

  virtual void Initialize(const EstimatorConfig& config) = 0;
  virtual void PushImu(const ImuSample& sample) = 0;
  virtual void PushGnssEpoch(const GnssEpoch& epoch) = 0;
  virtual void PushFusedLocation(const FusedLocationSample& sample) = 0;
  virtual std::optional<NavState> LatestState() const = 0;
  virtual EstimatorStatus LatestStatus() const = 0;
  virtual void Reset(const std::string& reason) = 0;
};

}  // namespace s23
