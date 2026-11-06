#pragma once

#include <array>
#include <cstdint>

#include "navgraph/gnss.hpp"

namespace navgraph {

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ImuSample {
  std::int64_t timestamp_nanos = 0;
  Vec3 specific_force_body_mps2{};
  Vec3 angular_rate_body_radps{};
};

struct InsState {
  std::int64_t timestamp_nanos = 0;
  Vec3 position_enu_m{};
  Vec3 velocity_enu_mps{};
  Quaternion attitude_body_to_enu{};
  Vec3 accelerometer_bias_body_mps2{};
  Vec3 gyro_bias_body_radps{};
  double clock_bias_m = 0.0;
  double clock_drift_mps = 0.0;
};

enum class InsStatus {
  kOk = 0,
  kNonIncreasingTimestamp = 1,
  kStateTimestampMismatch = 2,
  kNonFiniteInput = 3,
  kInvalidQuaternion = 4,
};

struct InsPropagationResult {
  InsStatus status = InsStatus::kNonFiniteInput;
  InsState state;
};

Quaternion normalized(const Quaternion& quaternion);
Quaternion multiply(const Quaternion& left, const Quaternion& right);
Quaternion from_rotation_vector(const Vec3& rotation_radians);
Vec3 rotate(const Quaternion& quaternion, const Vec3& vector);

InsPropagationResult propagate_ins(const InsState& state,
                                   const ImuSample& previous,
                                   const ImuSample& current);

constexpr double kStandardGravityMps2 = 9.80665;

}  // namespace navgraph
