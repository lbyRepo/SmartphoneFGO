#pragma once

#include <array>

#include "navgraph/frame.hpp"
#include "navgraph/gnss.hpp"
#include "navgraph/ins.hpp"

namespace navgraph {

constexpr int kEkfErrorStateSize = 17;
constexpr int kEkfCovarianceSize =
    kEkfErrorStateSize * kEkfErrorStateSize;

using EkfCovariance = std::array<double, kEkfCovarianceSize>;

// ecef_to_enu is row-major. Passing it across the platform boundary keeps the
// native and reference implementations on exactly the same local tangent frame.
using EkfFrame = LocalFrame;

enum class EkfStatus {
  kOk = 0,
  kNonIncreasingTimestamp = 1,
  kStateTimestampMismatch = 2,
  kNonFiniteInput = 3,
  kInvalidQuaternion = 4,
  kGnssTimestampMismatch = 5,
  kInvalidFrame = 6,
};

struct EkfHealthDelta {
  int propagation_steps = 0;
  int accepted_pseudorange = 0;
  int rejected_pseudorange = 0;
  int accepted_doppler = 0;
  int rejected_doppler = 0;
};

struct EkfStepResult {
  EkfStatus status = EkfStatus::kNonFiniteInput;
  InsState state;
  EkfCovariance covariance{};
  EkfHealthDelta health;
};

EkfCovariance default_ekf_covariance();

EkfStepResult propagate_ekf(const InsState& state,
                            const EkfCovariance& covariance,
                            const ImuSample& previous,
                            const ImuSample& current);

EkfStepResult update_ekf(const EkfFrame& frame, const InsState& state,
                         const EkfCovariance& covariance,
                         const GnssEpoch& epoch,
                         bool apply_earth_rotation = true);

bool ekf_covariance_is_finite_symmetric(const EkfCovariance& covariance,
                                        double tolerance = 1.0e-10);

}  // namespace navgraph
