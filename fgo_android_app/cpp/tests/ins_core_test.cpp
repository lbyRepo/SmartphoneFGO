#include "navgraph/ins.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

double norm(const navgraph::Vec3& value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

}  // namespace

int main() {
  navgraph::InsState state;
  state.timestamp_nanos = 1'000'000'000LL;
  state.clock_bias_m = 10.0;
  state.clock_drift_mps = 2.0;
  navgraph::ImuSample previous{
      state.timestamp_nanos, {0.0, 0.0, navgraph::kStandardGravityMps2},
      {0.0, 0.0, 0.0}};
  navgraph::ImuSample current = previous;
  current.timestamp_nanos += 1'000'000'000LL;
  const auto stationary = navgraph::propagate_ins(state, previous, current);
  require(stationary.status == navgraph::InsStatus::kOk,
          "stationary propagation must succeed");
  require(norm(stationary.state.position_enu_m) < 1.0e-12,
          "specific force must cancel ENU gravity");
  require(norm(stationary.state.velocity_enu_mps) < 1.0e-12,
          "stationary velocity must remain zero");
  require(std::abs(stationary.state.clock_bias_m - 12.0) < 1.0e-12,
          "receiver clock bias must propagate with drift");

  previous.specific_force_body_mps2[0] = 2.0;
  current.specific_force_body_mps2[0] = 2.0;
  const auto accelerated = navgraph::propagate_ins(state, previous, current);
  require(std::abs(accelerated.state.position_enu_m[0] - 1.0) < 1.0e-12,
          "constant acceleration position must match analytic result");
  require(std::abs(accelerated.state.velocity_enu_mps[0] - 2.0) < 1.0e-12,
          "constant acceleration velocity must match analytic result");

  previous.angular_rate_body_radps[2] = M_PI_2;
  current = previous;
  current.timestamp_nanos += 1'000'000'000LL;
  const auto rotated = navgraph::propagate_ins(state, previous, current);
  require(std::abs(rotated.state.attitude_body_to_enu.w - std::sqrt(0.5)) <
              1.0e-12 &&
              std::abs(rotated.state.attitude_body_to_enu.z - std::sqrt(0.5)) <
                  1.0e-12,
          "constant yaw rate must produce a 90 degree quaternion");

  state.accelerometer_bias_body_mps2 = {0.3, -0.2, 0.1};
  state.gyro_bias_body_radps = {0.01, -0.02, 0.03};
  previous.specific_force_body_mps2 = {
      state.accelerometer_bias_body_mps2[0],
      state.accelerometer_bias_body_mps2[1],
      navgraph::kStandardGravityMps2 +
          state.accelerometer_bias_body_mps2[2]};
  previous.angular_rate_body_radps = state.gyro_bias_body_radps;
  current = previous;
  current.timestamp_nanos += 10'000'000LL;
  const auto bias_cancelled = navgraph::propagate_ins(state, previous, current);
  require(norm(bias_cancelled.state.position_enu_m) < 1.0e-12 &&
              norm(bias_cancelled.state.velocity_enu_mps) < 1.0e-12,
          "configured accelerometer bias must cancel measured bias");
  require(std::abs(bias_cancelled.state.attitude_body_to_enu.w - 1.0) <
              1.0e-12,
          "configured gyro bias must cancel measured bias");

  current.timestamp_nanos = previous.timestamp_nanos;
  require(navgraph::propagate_ins(state, previous, current).status ==
              navgraph::InsStatus::kNonIncreasingTimestamp,
          "duplicate timestamp must fail explicitly");
  current.timestamp_nanos = previous.timestamp_nanos + 1;
  state.timestamp_nanos = previous.timestamp_nanos - 1;
  require(navgraph::propagate_ins(state, previous, current).status ==
              navgraph::InsStatus::kStateTimestampMismatch,
          "state/sample timestamp mismatch must fail explicitly");
  state.timestamp_nanos = previous.timestamp_nanos;
  current.specific_force_body_mps2[0] =
      std::numeric_limits<double>::quiet_NaN();
  require(navgraph::propagate_ins(state, previous, current).status ==
              navgraph::InsStatus::kNonFiniteInput,
          "non-finite IMU input must fail explicitly");

  std::cout << "PASS ins_core_test stationary_error_m="
            << norm(stationary.state.position_enu_m)
            << " yaw_quaternion_z=" << rotated.state.attitude_body_to_enu.z
            << '\n';
  return 0;
}
