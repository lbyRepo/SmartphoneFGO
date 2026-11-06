#pragma once

#include <array>

#include "navgraph/gnss.hpp"

namespace navgraph {

using Matrix3 = std::array<double, 9>;

struct LocalFrame {
  Vec3 reference_ecef_m{};
  Matrix3 ecef_to_enu{};
};

bool valid_local_frame(const LocalFrame& frame, double tolerance = 1.0e-6);
Vec3 ecef_position_to_enu(const LocalFrame& frame,
                          const Vec3& position_ecef_m);
Vec3 enu_position_to_ecef(const LocalFrame& frame,
                          const Vec3& position_enu_m);
Vec3 ecef_vector_to_enu(const LocalFrame& frame, const Vec3& vector_ecef);
Vec3 enu_vector_to_ecef(const LocalFrame& frame, const Vec3& vector_enu);

}  // namespace navgraph
