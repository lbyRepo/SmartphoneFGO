#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace s23 {

struct Timestamp {
  int64_t elapsed_realtime_nanos = 0;
  std::optional<int64_t> wall_time_millis;
};

struct Vec3 {
  std::array<double, 3> value{0.0, 0.0, 0.0};
};

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct NavState {
  Timestamp timestamp;
  Vec3 position_ecef_m;
  Vec3 velocity_ecef_mps;
  Quaternion attitude_body_to_ecef;
  Vec3 gyro_bias_radps;
  Vec3 accel_bias_mps2;
  double receiver_clock_bias_m = 0.0;
  double receiver_clock_drift_mps = 0.0;
};

struct EstimatorStatus {
  bool initialized = false;
  bool healthy = false;
  uint32_t accepted_measurements = 0;
  uint32_t rejected_measurements = 0;
  double output_latency_ms = 0.0;
  std::string message;
};

}  // namespace s23
