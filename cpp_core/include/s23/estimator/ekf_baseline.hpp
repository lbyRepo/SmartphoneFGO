#pragma once

#include <optional>
#include <string>

#include "s23/core/types.hpp"
#include "s23/estimator/imu_propagator.hpp"

namespace s23 {

struct EkfBaselineConfig {
  ImuPropagationConfig imu;
  double gnss_position_gain = 0.5;
};

class EkfGnssInsBaseline {
 public:
  explicit EkfGnssInsBaseline(EkfBaselineConfig config = EkfBaselineConfig{}) : config_(config), propagator_(config.imu) {}

  void Initialize(const NavState& initial_state) {
    state_ = initial_state;
    status_.initialized = true;
    status_.healthy = true;
    status_.message = "EKF baseline initialized";
  }

  void PropagateImu(const ImuSample& accelerometer_sample, double dt_seconds) {
    if (!state_) {
      status_.healthy = false;
      status_.message = "EKF propagation requested before initialization";
      return;
    }
    state_ = propagator_.PropagateConstantAttitude(*state_, accelerometer_sample, dt_seconds);
  }

  void CorrectWithGnssPosition(const Vec3& position_ecef_m) {
    if (!state_) {
      status_.healthy = false;
      status_.message = "EKF correction requested before initialization";
      return;
    }
    for (int axis = 0; axis < 3; ++axis) {
      state_->position_ecef_m.value[axis] =
          (1.0 - config_.gnss_position_gain) * state_->position_ecef_m.value[axis] +
          config_.gnss_position_gain * position_ecef_m.value[axis];
    }
    status_.accepted_measurements += 1;
    status_.message = "EKF baseline GNSS position correction applied";
  }

  std::optional<NavState> LatestState() const { return state_; }
  EstimatorStatus LatestStatus() const { return status_; }

 private:
  EkfBaselineConfig config_;
  ImuPropagator propagator_;
  std::optional<NavState> state_;
  EstimatorStatus status_;
};

}  // namespace s23
