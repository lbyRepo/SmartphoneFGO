#pragma once

#include "s23/core/types.hpp"
#include "s23/ins/imu_sample.hpp"
#include "s23/math/linear_algebra.hpp"

namespace s23 {

struct ImuPropagationConfig {
  Vec3 gravity_ecef_mps2{{0.0, 0.0, 0.0}};
};

class ImuPropagator {
 public:
  explicit ImuPropagator(ImuPropagationConfig config = ImuPropagationConfig{}) : config_(config) {}

  NavState PropagateConstantAttitude(
      const NavState& state,
      const ImuSample& accelerometer_sample,
      double dt_seconds) const {
    NavState propagated = state;
    propagated.timestamp = accelerometer_sample.timestamp;
    const Vec3 unbiased_accel = math::Subtract(accelerometer_sample.values, state.accel_bias_mps2);
    const Vec3 accel_ecef = math::Add(unbiased_accel, config_.gravity_ecef_mps2);
    for (int axis = 0; axis < 3; ++axis) {
      propagated.position_ecef_m.value[axis] +=
          state.velocity_ecef_mps.value[axis] * dt_seconds +
          0.5 * accel_ecef.value[axis] * dt_seconds * dt_seconds;
      propagated.velocity_ecef_mps.value[axis] += accel_ecef.value[axis] * dt_seconds;
    }
    return propagated;
  }

 private:
  ImuPropagationConfig config_;
};

}  // namespace s23
