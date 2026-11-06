#pragma once

#include <vector>

#include "s23/core/types.hpp"
#include "s23/math/linear_algebra.hpp"

namespace s23 {

struct SyntheticSatellite {
  int svid = 0;
  Vec3 position_ecef_m;
  Vec3 velocity_ecef_mps;
};

struct SyntheticGnssMeasurement {
  int svid = 0;
  Vec3 satellite_position_ecef_m;
  Vec3 satellite_velocity_ecef_mps;
  double pseudorange_m = 0.0;
  double pseudorange_sigma_m = 1.0;
  double pseudorange_rate_mps = 0.0;
  double pseudorange_rate_sigma_mps = 0.2;
};

struct SyntheticReceiverState {
  double time_seconds = 0.0;
  Vec3 position_ecef_m;
  Vec3 velocity_ecef_mps;
  double clock_bias_m = 0.0;
  double clock_drift_mps = 0.0;
};

struct SyntheticGnssEpoch {
  SyntheticReceiverState truth;
  std::vector<SyntheticGnssMeasurement> measurements;
};

struct SyntheticGnssDataset {
  std::vector<SyntheticGnssEpoch> epochs;
};

inline SyntheticGnssMeasurement MakeMeasurement(
    int svid,
    const Vec3& sat_position,
    const Vec3& sat_velocity,
    const SyntheticReceiverState& receiver,
    double pseudorange_noise_m,
    double range_rate_noise_mps) {
  const Vec3 sat_to_receiver = math::Subtract(receiver.position_ecef_m, sat_position);
  const double geometric_range = math::Norm(sat_to_receiver);
  const Vec3 line_of_sight_sat_to_rx = math::Scale(sat_to_receiver, 1.0 / geometric_range);
  const double range_rate = math::Dot(line_of_sight_sat_to_rx, math::Subtract(receiver.velocity_ecef_mps, sat_velocity));
  return SyntheticGnssMeasurement{
      svid,
      sat_position,
      sat_velocity,
      geometric_range + receiver.clock_bias_m + pseudorange_noise_m,
      1.0,
      range_rate + receiver.clock_drift_mps + range_rate_noise_mps,
      0.2,
  };
}

inline SyntheticGnssDataset MakeSyntheticGnssDataset() {
  const std::vector<SyntheticSatellite> satellites{
      {1, Vec3{{15600000.0, 21700000.0, 20100000.0}}, Vec3{{-820.0, 510.0, 40.0}}},
      {2, Vec3{{-18700000.0, 13400000.0, 21400000.0}}, Vec3{{420.0, -760.0, 110.0}}},
      {3, Vec3{{21100000.0, -14300000.0, 17800000.0}}, Vec3{{120.0, 690.0, -150.0}}},
      {4, Vec3{{17600000.0, 18600000.0, -19100000.0}}, Vec3{{-500.0, -210.0, 620.0}}},
      {5, Vec3{{-15200000.0, -21300000.0, 19800000.0}}, Vec3{{730.0, 130.0, -90.0}}},
      {6, Vec3{{23200000.0, 7300000.0, 20400000.0}}, Vec3{{-240.0, 430.0, -550.0}}},
      {7, Vec3{{-9300000.0, 23800000.0, -18400000.0}}, Vec3{{360.0, -290.0, 480.0}}},
      {8, Vec3{{12500000.0, -22800000.0, -19600000.0}}, Vec3{{-180.0, 350.0, 530.0}}},
  };

  SyntheticGnssDataset dataset;
  for (int epoch = 0; epoch < 4; ++epoch) {
    const double t = static_cast<double>(epoch);
    SyntheticReceiverState receiver{
        t,
        Vec3{{-2700400.0 + 12.0 * t, 4285150.0 - 4.0 * t, 3850120.0 + 2.5 * t}},
        Vec3{{12.0, -4.0, 2.5}},
        48.0 + 0.15 * t,
        0.15,
    };
    SyntheticGnssEpoch gnss_epoch;
    gnss_epoch.truth = receiver;
    for (std::size_t i = 0; i < satellites.size(); ++i) {
      const double pr_noise = (static_cast<int>(i) % 3 - 1) * 0.35 + 0.05 * epoch;
      const double rr_noise = (static_cast<int>(i) % 4 - 1.5) * 0.015;
      gnss_epoch.measurements.push_back(MakeMeasurement(
          satellites[i].svid,
          satellites[i].position_ecef_m,
          satellites[i].velocity_ecef_mps,
          receiver,
          pr_noise,
          rr_noise));
    }
    dataset.epochs.push_back(gnss_epoch);
  }
  return dataset;
}

}  // namespace s23
