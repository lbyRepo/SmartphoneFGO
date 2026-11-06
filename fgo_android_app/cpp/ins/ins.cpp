#include "navgraph/ins.hpp"

#include <cmath>

namespace navgraph {
namespace {

double quaternion_norm(const Quaternion& value) {
  return std::sqrt(value.w * value.w + value.x * value.x +
                   value.y * value.y + value.z * value.z);
}

double vector_norm(const Vec3& value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

bool finite(const Vec3& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

bool finite(const Quaternion& value) {
  return std::isfinite(value.w) && std::isfinite(value.x) &&
         std::isfinite(value.y) && std::isfinite(value.z);
}

bool finite(const InsState& value) {
  return finite(value.position_enu_m) && finite(value.velocity_enu_mps) &&
         finite(value.attitude_body_to_enu) &&
         finite(value.accelerometer_bias_body_mps2) &&
         finite(value.gyro_bias_body_radps) &&
         std::isfinite(value.clock_bias_m) &&
         std::isfinite(value.clock_drift_mps);
}

bool finite(const ImuSample& value) {
  return finite(value.specific_force_body_mps2) &&
         finite(value.angular_rate_body_radps);
}

}  // namespace

Quaternion normalized(const Quaternion& quaternion) {
  const double magnitude = quaternion_norm(quaternion);
  if (!std::isfinite(magnitude) || magnitude <= 1.0e-15) return {};
  return {quaternion.w / magnitude, quaternion.x / magnitude,
          quaternion.y / magnitude, quaternion.z / magnitude};
}

Quaternion multiply(const Quaternion& left, const Quaternion& right) {
  return normalized({
      left.w * right.w - left.x * right.x - left.y * right.y -
          left.z * right.z,
      left.w * right.x + left.x * right.w + left.y * right.z -
          left.z * right.y,
      left.w * right.y - left.x * right.z + left.y * right.w +
          left.z * right.x,
      left.w * right.z + left.x * right.y - left.y * right.x +
          left.z * right.w,
  });
}

Quaternion from_rotation_vector(const Vec3& rotation_radians) {
  const double angle = vector_norm(rotation_radians);
  if (angle < 1.0e-12) {
    return normalized({1.0, 0.5 * rotation_radians[0],
                       0.5 * rotation_radians[1],
                       0.5 * rotation_radians[2]});
  }
  const double half_angle = 0.5 * angle;
  const double scale = std::sin(half_angle) / angle;
  return {std::cos(half_angle), rotation_radians[0] * scale,
          rotation_radians[1] * scale, rotation_radians[2] * scale};
}

Vec3 rotate(const Quaternion& quaternion, const Vec3& vector) {
  const Quaternion q = normalized(quaternion);
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  return {
      (1.0 - 2.0 * (yy + zz)) * vector[0] +
          2.0 * (xy - wz) * vector[1] + 2.0 * (xz + wy) * vector[2],
      2.0 * (xy + wz) * vector[0] +
          (1.0 - 2.0 * (xx + zz)) * vector[1] +
          2.0 * (yz - wx) * vector[2],
      2.0 * (xz - wy) * vector[0] + 2.0 * (yz + wx) * vector[1] +
          (1.0 - 2.0 * (xx + yy)) * vector[2],
  };
}

InsPropagationResult propagate_ins(const InsState& state,
                                   const ImuSample& previous,
                                   const ImuSample& current) {
  InsPropagationResult result;
  result.state = state;
  if (current.timestamp_nanos <= previous.timestamp_nanos) {
    result.status = InsStatus::kNonIncreasingTimestamp;
    return result;
  }
  if (state.timestamp_nanos != previous.timestamp_nanos) {
    result.status = InsStatus::kStateTimestampMismatch;
    return result;
  }
  if (!finite(state) || !finite(previous) || !finite(current)) {
    result.status = InsStatus::kNonFiniteInput;
    return result;
  }
  const double attitude_norm = quaternion_norm(state.attitude_body_to_enu);
  if (!std::isfinite(attitude_norm) || attitude_norm <= 1.0e-15) {
    result.status = InsStatus::kInvalidQuaternion;
    return result;
  }
  const long double delta_nanos =
      static_cast<long double>(current.timestamp_nanos) -
      static_cast<long double>(previous.timestamp_nanos);
  const double dt = static_cast<double>(delta_nanos * 1.0e-9L);
  if (!std::isfinite(dt) || dt <= 0.0) {
    result.status = InsStatus::kNonIncreasingTimestamp;
    return result;
  }

  Vec3 angular_rate{};
  Vec3 specific_force{};
  for (int axis = 0; axis < 3; ++axis) {
    angular_rate[axis] =
        0.5 * (previous.angular_rate_body_radps[axis] +
               current.angular_rate_body_radps[axis]) -
        state.gyro_bias_body_radps[axis];
    specific_force[axis] =
        0.5 * (previous.specific_force_body_mps2[axis] +
               current.specific_force_body_mps2[axis]) -
        state.accelerometer_bias_body_mps2[axis];
  }
  const Quaternion half_step_attitude = multiply(
      state.attitude_body_to_enu,
      from_rotation_vector({0.5 * angular_rate[0] * dt,
                            0.5 * angular_rate[1] * dt,
                            0.5 * angular_rate[2] * dt}));
  Vec3 acceleration_enu = rotate(half_step_attitude, specific_force);
  acceleration_enu[2] -= kStandardGravityMps2;

  result.state.timestamp_nanos = current.timestamp_nanos;
  for (int axis = 0; axis < 3; ++axis) {
    result.state.position_enu_m[axis] =
        state.position_enu_m[axis] + state.velocity_enu_mps[axis] * dt +
        0.5 * acceleration_enu[axis] * dt * dt;
    result.state.velocity_enu_mps[axis] =
        state.velocity_enu_mps[axis] + acceleration_enu[axis] * dt;
  }
  result.state.attitude_body_to_enu = multiply(
      state.attitude_body_to_enu,
      from_rotation_vector({angular_rate[0] * dt, angular_rate[1] * dt,
                            angular_rate[2] * dt}));
  result.state.clock_bias_m =
      state.clock_bias_m + state.clock_drift_mps * dt;
  result.status = finite(result.state) ? InsStatus::kOk
                                       : InsStatus::kNonFiniteInput;
  return result;
}

}  // namespace navgraph
