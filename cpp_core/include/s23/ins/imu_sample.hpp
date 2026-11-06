#pragma once

#include <string>

#include "s23/core/types.hpp"

namespace s23 {

enum class ImuSensorType {
  kAccelerometer,
  kGyroscope,
};

struct ImuSample {
  Timestamp timestamp;
  ImuSensorType sensor_type = ImuSensorType::kAccelerometer;
  std::string sensor_name;
  int accuracy = 0;
  Vec3 values;
};

}  // namespace s23
