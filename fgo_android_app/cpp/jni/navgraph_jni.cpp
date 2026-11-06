#include <jni.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "navgraph/ekf.hpp"
#include "navgraph/fgo.hpp"
#include "navgraph/gnss.hpp"
#include "navgraph/ins.hpp"

namespace {

constexpr int kObservationStride = 10;
constexpr int kInitialStateSize = 8;
constexpr int kResultSize = 13;
constexpr int kInsStateSize = 18;
constexpr int kImuSampleSize = 6;
constexpr int kInsResultSize = 19;
constexpr int kEkfFrameSize = 12;
constexpr int kEkfResultSize = kInsStateSize + navgraph::kEkfCovarianceSize + 6;
constexpr int kFgoWlsStride = 8;
constexpr int kFgoPreintegrationStride = 7;
constexpr int kFgoResultTailSize = 13;

void throw_illegal_argument(JNIEnv* env, const char* message) {
  jclass type = env->FindClass("java/lang/IllegalArgumentException");
  if (type != nullptr) env->ThrowNew(type, message);
}

navgraph::Vec3 vec3_at(const double* values, int offset) {
  return {values[offset], values[offset + 1], values[offset + 2]};
}

navgraph::InsState unpack_ins_state(std::int64_t timestamp_nanos,
                                    const double* values) {
  navgraph::InsState state;
  state.timestamp_nanos = timestamp_nanos;
  state.position_enu_m = vec3_at(values, 0);
  state.velocity_enu_mps = vec3_at(values, 3);
  state.attitude_body_to_enu = {values[6], values[7], values[8], values[9]};
  state.accelerometer_bias_body_mps2 = vec3_at(values, 10);
  state.gyro_bias_body_radps = vec3_at(values, 13);
  state.clock_bias_m = values[16];
  state.clock_drift_mps = values[17];
  return state;
}

navgraph::ImuSample unpack_imu_sample(std::int64_t timestamp_nanos,
                                      const double* values) {
  return {timestamp_nanos, vec3_at(values, 0), vec3_at(values, 3)};
}

void pack_ins_state(const navgraph::InsState& state, double* values) {
  values[0] = state.position_enu_m[0];
  values[1] = state.position_enu_m[1];
  values[2] = state.position_enu_m[2];
  values[3] = state.velocity_enu_mps[0];
  values[4] = state.velocity_enu_mps[1];
  values[5] = state.velocity_enu_mps[2];
  values[6] = state.attitude_body_to_enu.w;
  values[7] = state.attitude_body_to_enu.x;
  values[8] = state.attitude_body_to_enu.y;
  values[9] = state.attitude_body_to_enu.z;
  values[10] = state.accelerometer_bias_body_mps2[0];
  values[11] = state.accelerometer_bias_body_mps2[1];
  values[12] = state.accelerometer_bias_body_mps2[2];
  values[13] = state.gyro_bias_body_radps[0];
  values[14] = state.gyro_bias_body_radps[1];
  values[15] = state.gyro_bias_body_radps[2];
  values[16] = state.clock_bias_m;
  values[17] = state.clock_drift_mps;
}

navgraph::GnssEpoch unpack_epoch(std::int64_t timestamp_millis,
                                 std::int64_t integration_timestamp_nanos,
                                 const double* values, int packed_size) {
  navgraph::GnssEpoch epoch;
  epoch.timestamp_millis = timestamp_millis;
  epoch.integration_timestamp_nanos = integration_timestamp_nanos;
  const int observation_count = packed_size / kObservationStride;
  epoch.observations.reserve(static_cast<std::size_t>(observation_count));
  for (int observation_index = 0; observation_index < observation_count;
       ++observation_index) {
    const int offset = observation_index * kObservationStride;
    navgraph::GnssObservation observation;
    observation.timestamp_millis = timestamp_millis;
    observation.svid = observation_index + 1;
    observation.satellite_position_m = vec3_at(values, offset);
    observation.satellite_velocity_mps = vec3_at(values, offset + 3);
    observation.corrected_pseudorange_m = values[offset + 6];
    observation.pseudorange_sigma_m = values[offset + 7];
    observation.corrected_range_rate_mps = values[offset + 8];
    observation.range_rate_sigma_mps = values[offset + 9];
    epoch.observations.push_back(observation);
  }
  return epoch;
}

jdoubleArray pack_ekf_result(JNIEnv* env,
                             const navgraph::EkfStepResult& result) {
  std::array<double, kEkfResultSize> values{};
  pack_ins_state(result.state, values.data());
  std::copy(result.covariance.begin(), result.covariance.end(),
            values.begin() + kInsStateSize);
  int offset = kInsStateSize + navgraph::kEkfCovarianceSize;
  values[offset++] = static_cast<double>(result.status);
  values[offset++] = static_cast<double>(result.health.propagation_steps);
  values[offset++] = static_cast<double>(result.health.accepted_pseudorange);
  values[offset++] = static_cast<double>(result.health.rejected_pseudorange);
  values[offset++] = static_cast<double>(result.health.accepted_doppler);
  values[offset] = static_cast<double>(result.health.rejected_doppler);
  jdoubleArray output = env->NewDoubleArray(kEkfResultSize);
  if (output == nullptr) return nullptr;
  env->SetDoubleArrayRegion(output, 0, kEkfResultSize, values.data());
  return output;
}

navgraph::LocalFrame unpack_frame(const double* values) {
  navgraph::LocalFrame frame;
  frame.reference_ecef_m = vec3_at(values, 0);
  std::copy(values + 3, values + kEkfFrameSize,
            frame.ecef_to_enu.begin());
  return frame;
}

}  // namespace

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_example_navgraph_estimator_NativeGnssWls_nativeSolve(
    JNIEnv* env, jobject /* receiver */, jlong timestamp_millis,
    jlong integration_timestamp_nanos, jdoubleArray packed_observations,
    jdoubleArray packed_initial_state, jboolean apply_earth_rotation) {
  if (packed_observations == nullptr || packed_initial_state == nullptr) {
    throw_illegal_argument(env, "Native WLS arrays must not be null");
    return nullptr;
  }
  const jsize packed_size = env->GetArrayLength(packed_observations);
  if (packed_size < 4 * kObservationStride ||
      packed_size % kObservationStride != 0) {
    throw_illegal_argument(
        env, "Packed observations need at least four stride-10 records");
    return nullptr;
  }
  if (env->GetArrayLength(packed_initial_state) != kInitialStateSize) {
    throw_illegal_argument(env, "Initial receiver state must have eight values");
    return nullptr;
  }
  std::vector<double> values(static_cast<std::size_t>(packed_size));
  std::array<double, kInitialStateSize> initial{};
  env->GetDoubleArrayRegion(packed_observations, 0, packed_size,
                            values.data());
  env->GetDoubleArrayRegion(packed_initial_state, 0, kInitialStateSize,
                            initial.data());
  if (env->ExceptionCheck()) return nullptr;

  navgraph::GnssEpoch epoch;
  epoch.timestamp_millis = static_cast<std::int64_t>(timestamp_millis);
  epoch.integration_timestamp_nanos =
      static_cast<std::int64_t>(integration_timestamp_nanos);
  const int observation_count = packed_size / kObservationStride;
  epoch.observations.reserve(static_cast<std::size_t>(observation_count));
  for (int index = 0; index < observation_count; ++index) {
    const int offset = index * kObservationStride;
    navgraph::GnssObservation observation;
    observation.timestamp_millis = epoch.timestamp_millis;
    observation.constellation = 0;
    observation.svid = index + 1;
    observation.satellite_position_m = {values[offset], values[offset + 1],
                                        values[offset + 2]};
    observation.satellite_velocity_mps = {
        values[offset + 3], values[offset + 4], values[offset + 5]};
    observation.corrected_pseudorange_m = values[offset + 6];
    observation.pseudorange_sigma_m = values[offset + 7];
    observation.corrected_range_rate_mps = values[offset + 8];
    observation.range_rate_sigma_mps = values[offset + 9];
    epoch.observations.push_back(observation);
  }
  navgraph::ReceiverState seed;
  seed.position_ecef_m = {initial[0], initial[1], initial[2]};
  seed.velocity_ecef_mps = {initial[3], initial[4], initial[5]};
  seed.clock_bias_m = initial[6];
  seed.clock_drift_mps = initial[7];
  const navgraph::WlsSolution solution = navgraph::solve_wls(
      epoch, seed, apply_earth_rotation == JNI_TRUE);
  const std::array<double, kResultSize> result = {
      solution.state.position_ecef_m[0],
      solution.state.position_ecef_m[1],
      solution.state.position_ecef_m[2],
      solution.state.velocity_ecef_mps[0],
      solution.state.velocity_ecef_mps[1],
      solution.state.velocity_ecef_mps[2],
      solution.state.clock_bias_m,
      solution.state.clock_drift_mps,
      static_cast<double>(solution.iterations),
      static_cast<double>(solution.observation_count),
      static_cast<double>(solution.rejected_count),
      solution.converged ? 1.0 : 0.0,
      solution.numerically_valid ? 1.0 : 0.0,
  };
  jdoubleArray output = env->NewDoubleArray(kResultSize);
  if (output == nullptr) return nullptr;
  env->SetDoubleArrayRegion(output, 0, kResultSize, result.data());
  return output;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_example_navgraph_estimator_NativeInsMechanization_nativePropagate(
    JNIEnv* env, jobject /* receiver */, jlong state_timestamp_nanos,
    jdoubleArray packed_state, jlong previous_timestamp_nanos,
    jdoubleArray packed_previous, jlong current_timestamp_nanos,
    jdoubleArray packed_current) {
  if (packed_state == nullptr || packed_previous == nullptr ||
      packed_current == nullptr) {
    throw_illegal_argument(env, "Native INS arrays must not be null");
    return nullptr;
  }
  if (env->GetArrayLength(packed_state) != kInsStateSize ||
      env->GetArrayLength(packed_previous) != kImuSampleSize ||
      env->GetArrayLength(packed_current) != kImuSampleSize) {
    throw_illegal_argument(
        env, "INS state needs 18 values and each IMU sample needs six");
    return nullptr;
  }
  std::array<double, kInsStateSize> state_values{};
  std::array<double, kImuSampleSize> previous_values{};
  std::array<double, kImuSampleSize> current_values{};
  env->GetDoubleArrayRegion(packed_state, 0, kInsStateSize,
                            state_values.data());
  env->GetDoubleArrayRegion(packed_previous, 0, kImuSampleSize,
                            previous_values.data());
  env->GetDoubleArrayRegion(packed_current, 0, kImuSampleSize,
                            current_values.data());
  if (env->ExceptionCheck()) return nullptr;

  const navgraph::InsState state = unpack_ins_state(
      static_cast<std::int64_t>(state_timestamp_nanos), state_values.data());
  const navgraph::ImuSample previous = unpack_imu_sample(
      static_cast<std::int64_t>(previous_timestamp_nanos),
      previous_values.data());
  const navgraph::ImuSample current = unpack_imu_sample(
      static_cast<std::int64_t>(current_timestamp_nanos), current_values.data());
  const navgraph::InsPropagationResult propagated =
      navgraph::propagate_ins(state, previous, current);
  const navgraph::InsState& output_state = propagated.state;
  const std::array<double, kInsResultSize> result = {
      output_state.position_enu_m[0],
      output_state.position_enu_m[1],
      output_state.position_enu_m[2],
      output_state.velocity_enu_mps[0],
      output_state.velocity_enu_mps[1],
      output_state.velocity_enu_mps[2],
      output_state.attitude_body_to_enu.w,
      output_state.attitude_body_to_enu.x,
      output_state.attitude_body_to_enu.y,
      output_state.attitude_body_to_enu.z,
      output_state.accelerometer_bias_body_mps2[0],
      output_state.accelerometer_bias_body_mps2[1],
      output_state.accelerometer_bias_body_mps2[2],
      output_state.gyro_bias_body_radps[0],
      output_state.gyro_bias_body_radps[1],
      output_state.gyro_bias_body_radps[2],
      output_state.clock_bias_m,
      output_state.clock_drift_mps,
      static_cast<double>(propagated.status),
  };
  jdoubleArray output = env->NewDoubleArray(kInsResultSize);
  if (output == nullptr) return nullptr;
  env->SetDoubleArrayRegion(output, 0, kInsResultSize, result.data());
  return output;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_example_navgraph_estimator_NativeGnssInsEkf_nativePropagate(
    JNIEnv* env, jobject /* receiver */, jlong state_timestamp_nanos,
    jdoubleArray packed_state, jdoubleArray packed_covariance,
    jlong previous_timestamp_nanos, jdoubleArray packed_previous,
    jlong current_timestamp_nanos, jdoubleArray packed_current) {
  if (packed_state == nullptr || packed_covariance == nullptr ||
      packed_previous == nullptr || packed_current == nullptr) {
    throw_illegal_argument(env, "Native EKF propagation arrays must not be null");
    return nullptr;
  }
  if (env->GetArrayLength(packed_state) != kInsStateSize ||
      env->GetArrayLength(packed_covariance) != navgraph::kEkfCovarianceSize ||
      env->GetArrayLength(packed_previous) != kImuSampleSize ||
      env->GetArrayLength(packed_current) != kImuSampleSize) {
    throw_illegal_argument(
        env, "EKF propagation needs state 18, covariance 289, and IMU 6");
    return nullptr;
  }
  std::array<double, kInsStateSize> state_values{};
  navgraph::EkfCovariance covariance{};
  std::array<double, kImuSampleSize> previous_values{};
  std::array<double, kImuSampleSize> current_values{};
  env->GetDoubleArrayRegion(packed_state, 0, kInsStateSize,
                            state_values.data());
  env->GetDoubleArrayRegion(packed_covariance, 0,
                            navgraph::kEkfCovarianceSize,
                            covariance.data());
  env->GetDoubleArrayRegion(packed_previous, 0, kImuSampleSize,
                            previous_values.data());
  env->GetDoubleArrayRegion(packed_current, 0, kImuSampleSize,
                            current_values.data());
  if (env->ExceptionCheck()) return nullptr;
  const navgraph::EkfStepResult result = navgraph::propagate_ekf(
      unpack_ins_state(static_cast<std::int64_t>(state_timestamp_nanos),
                       state_values.data()),
      covariance,
      unpack_imu_sample(static_cast<std::int64_t>(previous_timestamp_nanos),
                        previous_values.data()),
      unpack_imu_sample(static_cast<std::int64_t>(current_timestamp_nanos),
                        current_values.data()));
  return pack_ekf_result(env, result);
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_example_navgraph_estimator_NativeGnssInsEkf_nativeUpdate(
    JNIEnv* env, jobject /* receiver */, jdoubleArray packed_frame,
    jlong state_timestamp_nanos, jdoubleArray packed_state,
    jdoubleArray packed_covariance, jlong timestamp_millis,
    jlong integration_timestamp_nanos, jdoubleArray packed_observations,
    jboolean apply_earth_rotation) {
  if (packed_frame == nullptr || packed_state == nullptr ||
      packed_covariance == nullptr || packed_observations == nullptr) {
    throw_illegal_argument(env, "Native EKF update arrays must not be null");
    return nullptr;
  }
  const jsize observation_size = env->GetArrayLength(packed_observations);
  if (env->GetArrayLength(packed_frame) != kEkfFrameSize ||
      env->GetArrayLength(packed_state) != kInsStateSize ||
      env->GetArrayLength(packed_covariance) != navgraph::kEkfCovarianceSize ||
      observation_size % kObservationStride != 0) {
    throw_illegal_argument(
        env, "EKF update needs frame 12, state 18, covariance 289, and stride-10 observations");
    return nullptr;
  }
  std::array<double, kEkfFrameSize> frame_values{};
  std::array<double, kInsStateSize> state_values{};
  navgraph::EkfCovariance covariance{};
  std::vector<double> observation_values(
      static_cast<std::size_t>(observation_size));
  env->GetDoubleArrayRegion(packed_frame, 0, kEkfFrameSize,
                            frame_values.data());
  env->GetDoubleArrayRegion(packed_state, 0, kInsStateSize,
                            state_values.data());
  env->GetDoubleArrayRegion(packed_covariance, 0,
                            navgraph::kEkfCovarianceSize,
                            covariance.data());
  if (observation_size > 0) {
    env->GetDoubleArrayRegion(packed_observations, 0, observation_size,
                              observation_values.data());
  }
  if (env->ExceptionCheck()) return nullptr;
  const navgraph::EkfFrame frame = unpack_frame(frame_values.data());
  const navgraph::GnssEpoch epoch = unpack_epoch(
      static_cast<std::int64_t>(timestamp_millis),
      static_cast<std::int64_t>(integration_timestamp_nanos),
      observation_values.data(), observation_size);
  const navgraph::EkfStepResult result = navgraph::update_ekf(
      frame,
      unpack_ins_state(static_cast<std::int64_t>(state_timestamp_nanos),
                       state_values.data()),
      covariance, epoch, apply_earth_rotation == JNI_TRUE);
  return pack_ekf_result(env, result);
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_example_navgraph_estimator_NativeFixedLagSmoother_nativeSolveWindow(
    JNIEnv* env, jobject /* receiver */, jdoubleArray packed_frame,
    jlongArray epoch_timestamps_millis,
    jlongArray epoch_integration_timestamps_nanos,
    jintArray observation_offsets, jdoubleArray packed_observations,
    jdoubleArray packed_wls, jdoubleArray packed_preintegrations,
    jboolean apply_earth_rotation) {
  if (packed_frame == nullptr || epoch_timestamps_millis == nullptr ||
      epoch_integration_timestamps_nanos == nullptr ||
      observation_offsets == nullptr || packed_observations == nullptr ||
      packed_wls == nullptr || packed_preintegrations == nullptr) {
    throw_illegal_argument(env, "Native FGO arrays must not be null");
    return nullptr;
  }
  const jsize epoch_count = env->GetArrayLength(epoch_timestamps_millis);
  const jsize observation_size = env->GetArrayLength(packed_observations);
  if (epoch_count < 1 ||
      env->GetArrayLength(epoch_integration_timestamps_nanos) != epoch_count ||
      env->GetArrayLength(observation_offsets) != epoch_count + 1 ||
      env->GetArrayLength(packed_frame) != kEkfFrameSize ||
      observation_size % kObservationStride != 0 ||
      env->GetArrayLength(packed_wls) != epoch_count * kFgoWlsStride ||
      env->GetArrayLength(packed_preintegrations) !=
          (epoch_count - 1) * kFgoPreintegrationStride) {
    throw_illegal_argument(env, "Native FGO input dimensions are inconsistent");
    return nullptr;
  }

  std::array<double, kEkfFrameSize> frame_values{};
  std::vector<jlong> timestamps_millis(static_cast<std::size_t>(epoch_count));
  std::vector<jlong> timestamps_nanos(static_cast<std::size_t>(epoch_count));
  std::vector<jint> offsets(static_cast<std::size_t>(epoch_count + 1));
  std::vector<double> observations(static_cast<std::size_t>(observation_size));
  std::vector<double> wls(
      static_cast<std::size_t>(epoch_count * kFgoWlsStride));
  std::vector<double> preintegrations(static_cast<std::size_t>(
      (epoch_count - 1) * kFgoPreintegrationStride));
  env->GetDoubleArrayRegion(packed_frame, 0, kEkfFrameSize,
                            frame_values.data());
  env->GetLongArrayRegion(epoch_timestamps_millis, 0, epoch_count,
                          timestamps_millis.data());
  env->GetLongArrayRegion(epoch_integration_timestamps_nanos, 0, epoch_count,
                          timestamps_nanos.data());
  env->GetIntArrayRegion(observation_offsets, 0, epoch_count + 1,
                         offsets.data());
  if (observation_size > 0) {
    env->GetDoubleArrayRegion(packed_observations, 0, observation_size,
                              observations.data());
  }
  env->GetDoubleArrayRegion(packed_wls, 0,
                            epoch_count * kFgoWlsStride, wls.data());
  if (epoch_count > 1) {
    env->GetDoubleArrayRegion(
        packed_preintegrations, 0,
        (epoch_count - 1) * kFgoPreintegrationStride,
        preintegrations.data());
  }
  if (env->ExceptionCheck()) return nullptr;

  const int observation_count = observation_size / kObservationStride;
  if (offsets.front() != 0 || offsets.back() != observation_count) {
    throw_illegal_argument(env, "Native FGO observation offsets are invalid");
    return nullptr;
  }
  std::vector<navgraph::GnssEpoch> epochs;
  std::vector<navgraph::ReceiverState> seeds;
  epochs.reserve(static_cast<std::size_t>(epoch_count));
  seeds.reserve(static_cast<std::size_t>(epoch_count));
  for (int epoch_index = 0; epoch_index < epoch_count; ++epoch_index) {
    if (offsets[epoch_index] < 0 ||
        offsets[epoch_index] > offsets[epoch_index + 1]) {
      throw_illegal_argument(env, "Native FGO observation offsets must increase");
      return nullptr;
    }
    const int first_observation = offsets[epoch_index];
    const int count = offsets[epoch_index + 1] - first_observation;
    const double* epoch_observations = count > 0
        ? observations.data() + first_observation * kObservationStride
        : nullptr;
    epochs.push_back(unpack_epoch(
        static_cast<std::int64_t>(timestamps_millis[epoch_index]),
        static_cast<std::int64_t>(timestamps_nanos[epoch_index]),
        epoch_observations, count * kObservationStride));
    const int offset = epoch_index * kFgoWlsStride;
    navgraph::ReceiverState seed;
    seed.position_ecef_m = vec3_at(wls.data(), offset);
    seed.velocity_ecef_mps = vec3_at(wls.data(), offset + 3);
    seed.clock_bias_m = wls[offset + 6];
    seed.clock_drift_mps = wls[offset + 7];
    seeds.push_back(seed);
  }
  std::vector<navgraph::FgoPreintegration> factors;
  factors.reserve(static_cast<std::size_t>(std::max(epoch_count - 1, 0)));
  for (int interval = 0; interval < epoch_count - 1; ++interval) {
    const int offset = interval * kFgoPreintegrationStride;
    factors.push_back({preintegrations[offset],
                       vec3_at(preintegrations.data(), offset + 1),
                       vec3_at(preintegrations.data(), offset + 4)});
  }
  const navgraph::FgoWindowSolution solution = navgraph::solve_fgo_window(
      unpack_frame(frame_values.data()), epochs, seeds, factors,
      apply_earth_rotation == JNI_TRUE);
  const int result_size =
      static_cast<int>(solution.states.size()) * navgraph::kFgoStateSize +
      kFgoResultTailSize;
  std::vector<double> result(static_cast<std::size_t>(result_size));
  int result_offset = 0;
  for (const navgraph::FgoState& state : solution.states) {
    std::copy(state.begin(), state.end(), result.begin() + result_offset);
    result_offset += navgraph::kFgoStateSize;
  }
  result[result_offset++] = static_cast<double>(solution.iterations);
  result[result_offset++] = solution.converged ? 1.0 : 0.0;
  result[result_offset++] = static_cast<double>(solution.status);
  result[result_offset++] = static_cast<double>(solution.skipped_factors);
  result[result_offset++] = solution.uncertainty.valid ? 1.0 : 0.0;
  for (double sigma : solution.uncertainty.position_sigma_enu_m) {
    result[result_offset++] = sigma;
  }
  for (double sigma : solution.uncertainty.velocity_sigma_enu_mps) {
    result[result_offset++] = sigma;
  }
  result[result_offset++] = solution.uncertainty.clock_bias_sigma_m;
  result[result_offset] = solution.uncertainty.clock_drift_sigma_mps;
  jdoubleArray output = env->NewDoubleArray(result_size);
  if (output == nullptr) return nullptr;
  env->SetDoubleArrayRegion(output, 0, result_size, result.data());
  return output;
}
