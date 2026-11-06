#include "navgraph/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace navgraph {
namespace {

constexpr int kPositionIndex = 0;
constexpr int kVelocityIndex = 3;
constexpr int kAttitudeIndex = 6;
constexpr int kAccelBiasIndex = 9;
constexpr int kGyroBiasIndex = 12;
constexpr int kClockBiasIndex = 15;
constexpr int kClockDriftIndex = 16;
constexpr double kPseudorangeGateSquared = 25.0;
constexpr double kDopplerGateSquared = 25.0;

int index(int row, int column) {
  return row * kEkfErrorStateSize + column;
}

bool finite(const Vec3& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

bool finite(const Quaternion& value) {
  return std::isfinite(value.w) && std::isfinite(value.x) &&
         std::isfinite(value.y) && std::isfinite(value.z);
}

double quaternion_norm(const Quaternion& value) {
  return std::sqrt(value.w * value.w + value.x * value.x +
                   value.y * value.y + value.z * value.z);
}

bool finite(const InsState& value) {
  return finite(value.position_enu_m) && finite(value.velocity_enu_mps) &&
         finite(value.attitude_body_to_enu) &&
         finite(value.accelerometer_bias_body_mps2) &&
         finite(value.gyro_bias_body_radps) &&
         std::isfinite(value.clock_bias_m) &&
         std::isfinite(value.clock_drift_mps);
}

Matrix3 rotation_matrix(const Quaternion& quaternion) {
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
      1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),
      2.0 * (xz + wy),       2.0 * (xy + wz),
      1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
      2.0 * (xz - wy),       2.0 * (yz + wx),
      1.0 - 2.0 * (xx + yy),
  };
}

ReceiverState receiver_state(const EkfFrame& frame, const InsState& state) {
  return {enu_position_to_ecef(frame, state.position_enu_m),
          enu_vector_to_ecef(frame, state.velocity_enu_mps),
          state.clock_bias_m, state.clock_drift_mps};
}

void symmetrize(EkfCovariance* covariance) {
  for (int row = 0; row < kEkfErrorStateSize; ++row) {
    for (int column = row + 1; column < kEkfErrorStateSize; ++column) {
      const double value =
          0.5 * ((*covariance)[index(row, column)] +
                 (*covariance)[index(column, row)]);
      (*covariance)[index(row, column)] = value;
      (*covariance)[index(column, row)] = value;
    }
  }
}

EkfStatus map_ins_status(InsStatus status) {
  switch (status) {
    case InsStatus::kOk:
      return EkfStatus::kOk;
    case InsStatus::kNonIncreasingTimestamp:
      return EkfStatus::kNonIncreasingTimestamp;
    case InsStatus::kStateTimestampMismatch:
      return EkfStatus::kStateTimestampMismatch;
    case InsStatus::kNonFiniteInput:
      return EkfStatus::kNonFiniteInput;
    case InsStatus::kInvalidQuaternion:
      return EkfStatus::kInvalidQuaternion;
  }
  return EkfStatus::kNonFiniteInput;
}

struct TransitionEntry {
  int row;
  int column;
  double value;
};

EkfCovariance propagate_covariance(const EkfCovariance& covariance, double dt,
                                   const Matrix3& body_to_enu,
                                   const Vec3& specific_force_body) {
  std::vector<TransitionEntry> entries;
  entries.reserve(43);
  for (int axis = 0; axis < 3; ++axis) {
    entries.push_back({kPositionIndex + axis, kVelocityIndex + axis, dt});
  }

  const Matrix3 skew = {
      0.0,                    -specific_force_body[2], specific_force_body[1],
      specific_force_body[2], 0.0,                     -specific_force_body[0],
      -specific_force_body[1], specific_force_body[0], 0.0,
  };
  Matrix3 acceleration_attitude{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int inner = 0; inner < 3; ++inner) {
        acceleration_attitude[row * 3 + column] +=
            body_to_enu[row * 3 + inner] * skew[inner * 3 + column];
      }
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      const double attitude_derivative =
          -acceleration_attitude[row * 3 + column];
      entries.push_back({kPositionIndex + row, kAttitudeIndex + column,
                         0.5 * attitude_derivative * dt * dt});
      entries.push_back({kVelocityIndex + row, kAttitudeIndex + column,
                         attitude_derivative * dt});
      entries.push_back({kPositionIndex + row, kAccelBiasIndex + column,
                         -0.5 * body_to_enu[row * 3 + column] * dt * dt});
      entries.push_back({kVelocityIndex + row, kAccelBiasIndex + column,
                         -body_to_enu[row * 3 + column] * dt});
    }
    entries.push_back(
        {kAttitudeIndex + row, kGyroBiasIndex + row, -dt});
  }
  entries.push_back({kClockBiasIndex, kClockDriftIndex, dt});

  EkfCovariance left = covariance;
  for (const TransitionEntry& entry : entries) {
    for (int column = 0; column < kEkfErrorStateSize; ++column) {
      left[index(entry.row, column)] +=
          entry.value * covariance[index(entry.column, column)];
    }
  }
  EkfCovariance propagated = left;
  for (const TransitionEntry& entry : entries) {
    for (int row = 0; row < kEkfErrorStateSize; ++row) {
      propagated[index(row, entry.row)] +=
          left[index(row, entry.column)] * entry.value;
    }
  }

  constexpr double kAccelerationNoise = 0.08;
  constexpr double kGyroNoise = 0.004;
  constexpr double kAccelerometerBiasWalk = 0.002;
  constexpr double kGyroBiasWalk = 0.0002;
  for (int axis = 0; axis < 3; ++axis) {
    propagated[index(kPositionIndex + axis, kPositionIndex + axis)] +=
        0.25 * kAccelerationNoise * kAccelerationNoise * dt * dt * dt * dt;
    propagated[index(kVelocityIndex + axis, kVelocityIndex + axis)] +=
        kAccelerationNoise * kAccelerationNoise * dt * dt;
    propagated[index(kAttitudeIndex + axis, kAttitudeIndex + axis)] +=
        kGyroNoise * kGyroNoise * dt;
    propagated[index(kAccelBiasIndex + axis, kAccelBiasIndex + axis)] +=
        kAccelerometerBiasWalk * kAccelerometerBiasWalk * dt;
    propagated[index(kGyroBiasIndex + axis, kGyroBiasIndex + axis)] +=
        kGyroBiasWalk * kGyroBiasWalk * dt;
  }
  propagated[index(kClockBiasIndex, kClockBiasIndex)] += 4.0 * dt;
  propagated[index(kClockDriftIndex, kClockDriftIndex)] += 0.04 * dt;
  symmetrize(&propagated);
  return propagated;
}

bool apply_correction(const std::array<double, kEkfErrorStateSize>& error,
                      InsState* state) {
  InsState corrected = *state;
  for (int axis = 0; axis < 3; ++axis) {
    corrected.position_enu_m[axis] += error[kPositionIndex + axis];
    corrected.velocity_enu_mps[axis] += error[kVelocityIndex + axis];
    corrected.accelerometer_bias_body_mps2[axis] +=
        error[kAccelBiasIndex + axis];
    corrected.gyro_bias_body_radps[axis] += error[kGyroBiasIndex + axis];
  }
  corrected.attitude_body_to_enu = multiply(
      from_rotation_vector({error[kAttitudeIndex], error[kAttitudeIndex + 1],
                            error[kAttitudeIndex + 2]}),
      corrected.attitude_body_to_enu);
  corrected.clock_bias_m += error[kClockBiasIndex];
  corrected.clock_drift_mps += error[kClockDriftIndex];
  if (!finite(corrected)) return false;
  *state = corrected;
  return true;
}

bool scalar_update(double innovation,
                   const std::array<double, kEkfErrorStateSize>& row,
                   double sigma, double gate_squared, InsState* state,
                   EkfCovariance* covariance) {
  if (!std::isfinite(innovation) || !std::isfinite(sigma) || sigma <= 0.0 ||
      !std::all_of(row.begin(), row.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }
  std::array<double, kEkfErrorStateSize> projected{};
  for (int matrix_row = 0; matrix_row < kEkfErrorStateSize; ++matrix_row) {
    for (int column = 0; column < kEkfErrorStateSize; ++column) {
      projected[matrix_row] +=
          (*covariance)[index(matrix_row, column)] * row[column];
    }
  }
  double innovation_variance = sigma * sigma;
  for (int entry = 0; entry < kEkfErrorStateSize; ++entry) {
    innovation_variance += row[entry] * projected[entry];
  }
  if (!std::isfinite(innovation_variance) || innovation_variance <= 1.0e-12 ||
      innovation * innovation / innovation_variance > gate_squared) {
    return false;
  }

  std::array<double, kEkfErrorStateSize> gain{};
  std::array<double, kEkfErrorStateSize> correction{};
  for (int entry = 0; entry < kEkfErrorStateSize; ++entry) {
    gain[entry] = projected[entry] / innovation_variance;
    correction[entry] = gain[entry] * innovation;
  }
  InsState corrected_state = *state;
  if (!apply_correction(correction, &corrected_state)) return false;

  EkfCovariance left{};
  for (int matrix_row = 0; matrix_row < kEkfErrorStateSize; ++matrix_row) {
    for (int column = 0; column < kEkfErrorStateSize; ++column) {
      const double identity = matrix_row == column ? 1.0 : 0.0;
      left[index(matrix_row, column)] =
          identity - gain[matrix_row] * row[column];
    }
  }
  EkfCovariance left_times_covariance{};
  for (int matrix_row = 0; matrix_row < kEkfErrorStateSize; ++matrix_row) {
    for (int column = 0; column < kEkfErrorStateSize; ++column) {
      for (int inner = 0; inner < kEkfErrorStateSize; ++inner) {
        left_times_covariance[index(matrix_row, column)] +=
            left[index(matrix_row, inner)] *
            (*covariance)[index(inner, column)];
      }
    }
  }
  EkfCovariance updated{};
  const double variance = sigma * sigma;
  for (int matrix_row = 0; matrix_row < kEkfErrorStateSize; ++matrix_row) {
    for (int column = 0; column < kEkfErrorStateSize; ++column) {
      for (int inner = 0; inner < kEkfErrorStateSize; ++inner) {
        updated[index(matrix_row, column)] +=
            left_times_covariance[index(matrix_row, inner)] *
            left[index(column, inner)];
      }
      updated[index(matrix_row, column)] +=
          gain[matrix_row] * variance * gain[column];
    }
  }
  symmetrize(&updated);
  if (!ekf_covariance_is_finite_symmetric(updated, 1.0e-8)) return false;
  *state = corrected_state;
  *covariance = updated;
  return true;
}

}  // namespace

EkfCovariance default_ekf_covariance() {
  EkfCovariance covariance{};
  const std::array<double, kEkfErrorStateSize> standard_deviations = {
      8.0,  8.0,  12.0, 2.0,  2.0,  2.0,
      10.0 * 3.14159265358979323846 / 180.0,
      10.0 * 3.14159265358979323846 / 180.0,
      20.0 * 3.14159265358979323846 / 180.0,
      0.15, 0.15, 0.15, 0.02, 0.02, 0.02, 100.0, 10.0,
  };
  for (int entry = 0; entry < kEkfErrorStateSize; ++entry) {
    covariance[index(entry, entry)] =
        standard_deviations[entry] * standard_deviations[entry];
  }
  return covariance;
}

EkfStepResult propagate_ekf(const InsState& state,
                            const EkfCovariance& covariance,
                            const ImuSample& previous,
                            const ImuSample& current) {
  EkfStepResult result;
  result.state = state;
  result.covariance = covariance;
  if (!ekf_covariance_is_finite_symmetric(covariance, 1.0e-8)) {
    result.status = EkfStatus::kNonFiniteInput;
    return result;
  }
  const InsPropagationResult nominal = propagate_ins(state, previous, current);
  result.status = map_ins_status(nominal.status);
  if (nominal.status != InsStatus::kOk) return result;

  const long double delta_nanos =
      static_cast<long double>(current.timestamp_nanos) -
      static_cast<long double>(previous.timestamp_nanos);
  const double dt = static_cast<double>(delta_nanos * 1.0e-9L);
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
  const Quaternion midpoint = multiply(
      state.attitude_body_to_enu,
      from_rotation_vector({0.5 * angular_rate[0] * dt,
                            0.5 * angular_rate[1] * dt,
                            0.5 * angular_rate[2] * dt}));
  result.covariance = propagate_covariance(
      covariance, dt, rotation_matrix(midpoint), specific_force);
  if (!ekf_covariance_is_finite_symmetric(result.covariance, 1.0e-8)) {
    result.state = state;
    result.covariance = covariance;
    result.status = EkfStatus::kNonFiniteInput;
    return result;
  }
  result.state = nominal.state;
  result.health.propagation_steps = 1;
  result.status = EkfStatus::kOk;
  return result;
}

EkfStepResult update_ekf(const EkfFrame& frame, const InsState& state,
                         const EkfCovariance& covariance,
                         const GnssEpoch& epoch, bool apply_earth_rotation) {
  EkfStepResult result;
  result.state = state;
  result.covariance = covariance;
  if (!finite(state) || !ekf_covariance_is_finite_symmetric(covariance, 1.0e-8)) {
    result.status = EkfStatus::kNonFiniteInput;
    return result;
  }
  if (quaternion_norm(state.attitude_body_to_enu) <= 1.0e-15) {
    result.status = EkfStatus::kInvalidQuaternion;
    return result;
  }
  if (!valid_local_frame(frame)) {
    result.status = EkfStatus::kInvalidFrame;
    return result;
  }
  const long double timestamp_delta =
      static_cast<long double>(epoch.integration_timestamp_nanos) -
      static_cast<long double>(state.timestamp_nanos);
  if (std::abs(timestamp_delta) > 1000000.0L) {
    result.status = EkfStatus::kGnssTimestampMismatch;
    return result;
  }

  for (const GnssObservation& observation : epoch.observations) {
    ReceiverState receiver = receiver_state(frame, result.state);
    GnssPrediction prediction =
        predict(receiver, observation, apply_earth_rotation);
    std::array<double, kEkfErrorStateSize> pseudorange_row{};
    const Vec3 range_enu =
        ecef_vector_to_enu(frame, prediction.range_derivative_ecef);
    for (int axis = 0; axis < 3; ++axis) {
      pseudorange_row[kPositionIndex + axis] = range_enu[axis];
    }
    pseudorange_row[kClockBiasIndex] = 1.0;
    const bool pseudorange_accepted = scalar_update(
        observation.corrected_pseudorange_m - prediction.pseudorange_m,
        pseudorange_row, std::max(observation.pseudorange_sigma_m, 1.0),
        kPseudorangeGateSquared, &result.state, &result.covariance);
    if (pseudorange_accepted) {
      ++result.health.accepted_pseudorange;
    } else {
      ++result.health.rejected_pseudorange;
    }

    if (std::isfinite(observation.corrected_range_rate_mps) &&
        std::isfinite(observation.range_rate_sigma_mps)) {
      receiver = receiver_state(frame, result.state);
      prediction = predict(receiver, observation, apply_earth_rotation);
      std::array<double, kEkfErrorStateSize> doppler_row{};
      const Vec3 range_rate_position_enu = ecef_vector_to_enu(
          frame, prediction.range_rate_position_derivative_ecef);
      const Vec3 range_velocity_enu =
          ecef_vector_to_enu(frame, prediction.range_derivative_ecef);
      for (int axis = 0; axis < 3; ++axis) {
        doppler_row[kPositionIndex + axis] = range_rate_position_enu[axis];
        doppler_row[kVelocityIndex + axis] = range_velocity_enu[axis];
      }
      doppler_row[kClockDriftIndex] = 1.0;
      const bool doppler_accepted = scalar_update(
          observation.corrected_range_rate_mps - prediction.range_rate_mps,
          doppler_row,
          std::max(observation.range_rate_sigma_mps, 0.15),
          kDopplerGateSquared, &result.state, &result.covariance);
      if (doppler_accepted) {
        ++result.health.accepted_doppler;
      } else {
        ++result.health.rejected_doppler;
      }
    }
  }
  result.status = EkfStatus::kOk;
  return result;
}

bool ekf_covariance_is_finite_symmetric(const EkfCovariance& covariance,
                                        double tolerance) {
  if (!std::isfinite(tolerance) || tolerance < 0.0) return false;
  for (int row = 0; row < kEkfErrorStateSize; ++row) {
    if (!std::isfinite(covariance[index(row, row)]) ||
        covariance[index(row, row)] < -tolerance) {
      return false;
    }
    for (int column = 0; column < kEkfErrorStateSize; ++column) {
      if (!std::isfinite(covariance[index(row, column)]) ||
          std::abs(covariance[index(row, column)] -
                   covariance[index(column, row)]) > tolerance) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace navgraph
