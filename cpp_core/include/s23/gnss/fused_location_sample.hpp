#pragma once

#include <optional>
#include <string>

#include "s23/core/types.hpp"

namespace s23 {

struct FusedLocationSample {
  Timestamp callback_timestamp;
  int64_t location_elapsed_realtime_nanos = 0;
  std::string provider;
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  int64_t time_millis = 0;
  std::optional<double> altitude_m;
  std::optional<double> speed_mps;
  std::optional<double> bearing_deg;
  std::optional<double> horizontal_accuracy_m;
  std::optional<double> vertical_accuracy_m;
  std::optional<double> speed_accuracy_mps;
  std::optional<double> bearing_accuracy_deg;
  bool mock = false;
};

}  // namespace s23
