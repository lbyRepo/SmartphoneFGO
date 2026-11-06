#include "navgraph/frame.hpp"

#include <cmath>

namespace navgraph {
namespace {

bool finite(const Vec3& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

bool finite(const Matrix3& value) {
  for (double entry : value) {
    if (!std::isfinite(entry)) return false;
  }
  return true;
}

Vec3 multiply(const Matrix3& matrix, const Vec3& vector) {
  Vec3 result{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result[row] += matrix[row * 3 + column] * vector[column];
    }
  }
  return result;
}

Vec3 multiply_transpose(const Matrix3& matrix, const Vec3& vector) {
  Vec3 result{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result[row] += matrix[column * 3 + row] * vector[column];
    }
  }
  return result;
}

}  // namespace

bool valid_local_frame(const LocalFrame& frame, double tolerance) {
  if (!finite(frame.reference_ecef_m) || !finite(frame.ecef_to_enu) ||
      !std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  for (int row = 0; row < 3; ++row) {
    double row_norm = 0.0;
    for (int column = 0; column < 3; ++column) {
      row_norm += frame.ecef_to_enu[row * 3 + column] *
                  frame.ecef_to_enu[row * 3 + column];
    }
    if (std::abs(row_norm - 1.0) > tolerance) return false;
    for (int other = 0; other < row; ++other) {
      double row_dot = 0.0;
      for (int column = 0; column < 3; ++column) {
        row_dot += frame.ecef_to_enu[row * 3 + column] *
                   frame.ecef_to_enu[other * 3 + column];
      }
      if (std::abs(row_dot) > tolerance) return false;
    }
  }
  const double determinant =
      frame.ecef_to_enu[0] *
          (frame.ecef_to_enu[4] * frame.ecef_to_enu[8] -
           frame.ecef_to_enu[5] * frame.ecef_to_enu[7]) -
      frame.ecef_to_enu[1] *
          (frame.ecef_to_enu[3] * frame.ecef_to_enu[8] -
           frame.ecef_to_enu[5] * frame.ecef_to_enu[6]) +
      frame.ecef_to_enu[2] *
          (frame.ecef_to_enu[3] * frame.ecef_to_enu[7] -
           frame.ecef_to_enu[4] * frame.ecef_to_enu[6]);
  return std::abs(determinant - 1.0) <= tolerance;
}

Vec3 ecef_position_to_enu(const LocalFrame& frame,
                          const Vec3& position_ecef_m) {
  return multiply(frame.ecef_to_enu,
                  {position_ecef_m[0] - frame.reference_ecef_m[0],
                   position_ecef_m[1] - frame.reference_ecef_m[1],
                   position_ecef_m[2] - frame.reference_ecef_m[2]});
}

Vec3 enu_position_to_ecef(const LocalFrame& frame,
                          const Vec3& position_enu_m) {
  const Vec3 delta = multiply_transpose(frame.ecef_to_enu, position_enu_m);
  return {frame.reference_ecef_m[0] + delta[0],
          frame.reference_ecef_m[1] + delta[1],
          frame.reference_ecef_m[2] + delta[2]};
}

Vec3 ecef_vector_to_enu(const LocalFrame& frame, const Vec3& vector_ecef) {
  return multiply(frame.ecef_to_enu, vector_ecef);
}

Vec3 enu_vector_to_ecef(const LocalFrame& frame, const Vec3& vector_enu) {
  return multiply_transpose(frame.ecef_to_enu, vector_enu);
}

}  // namespace navgraph
