#pragma once

#include <array>
#include <vector>

#include "navgraph/frame.hpp"
#include "navgraph/gnss.hpp"

namespace navgraph {

constexpr int kFgoStateSize = 11;
using FgoState = std::array<double, kFgoStateSize>;

struct FgoPreintegration {
  double delta_time_seconds = 0.0;
  Vec3 delta_position_enu_m{};
  Vec3 delta_velocity_enu_mps{};
};

struct FgoStateUncertainty {
  bool valid = false;
  Vec3 position_sigma_enu_m{};
  Vec3 velocity_sigma_enu_mps{};
  double clock_bias_sigma_m = 0.0;
  double clock_drift_sigma_mps = 0.0;
};

enum class FgoStatus {
  kOk = 0,
  kInvalidInput = 1,
  kNonFiniteInput = 2,
  kInvalidFrame = 3,
  kLinearSolveFailure = 4,
};

struct FgoWindowSolution {
  FgoStatus status = FgoStatus::kInvalidInput;
  std::vector<FgoState> states;
  int iterations = 0;
  bool converged = false;
  int skipped_factors = 0;
  FgoStateUncertainty uncertainty;
};

FgoWindowSolution solve_fgo_window(
    const LocalFrame& frame, const std::vector<GnssEpoch>& epochs,
    const std::vector<ReceiverState>& wls,
    const std::vector<FgoPreintegration>& preintegrations,
    bool apply_earth_rotation = true);

}  // namespace navgraph
