#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "s23/core/types.hpp"

namespace s23::math {

inline Vec3 Add(const Vec3& a, const Vec3& b) {
  return Vec3{{a.value[0] + b.value[0], a.value[1] + b.value[1], a.value[2] + b.value[2]}};
}

inline Vec3 Subtract(const Vec3& a, const Vec3& b) {
  return Vec3{{a.value[0] - b.value[0], a.value[1] - b.value[1], a.value[2] - b.value[2]}};
}

inline Vec3 Scale(const Vec3& v, double scale) {
  return Vec3{{scale * v.value[0], scale * v.value[1], scale * v.value[2]}};
}

inline double Dot(const Vec3& a, const Vec3& b) {
  return a.value[0] * b.value[0] + a.value[1] * b.value[1] + a.value[2] * b.value[2];
}

inline double Norm(const Vec3& v) { return std::sqrt(Dot(v, v)); }

inline Vec3 Normalize(const Vec3& v) {
  const double norm = Norm(v);
  if (norm <= 0.0) {
    throw std::invalid_argument("Cannot normalize zero vector");
  }
  return Scale(v, 1.0 / norm);
}

inline double Distance(const Vec3& a, const Vec3& b) { return Norm(Subtract(a, b)); }

class DenseMatrix {
 public:
  DenseMatrix() = default;
  DenseMatrix(int rows, int cols) : rows_(rows), cols_(cols), values_(rows * cols, 0.0) {}

  int rows() const { return rows_; }
  int cols() const { return cols_; }

  double& operator()(int row, int col) { return values_[row * cols_ + col]; }
  double operator()(int row, int col) const { return values_[row * cols_ + col]; }

 private:
  int rows_ = 0;
  int cols_ = 0;
  std::vector<double> values_;
};

inline std::vector<double> SolveLinearSystem(DenseMatrix a, std::vector<double> b) {
  const int n = a.rows();
  if (a.cols() != n || static_cast<int>(b.size()) != n) {
    throw std::invalid_argument("SolveLinearSystem requires a square matrix");
  }

  for (int pivot = 0; pivot < n; ++pivot) {
    int best = pivot;
    double best_abs = std::abs(a(pivot, pivot));
    for (int row = pivot + 1; row < n; ++row) {
      const double candidate = std::abs(a(row, pivot));
      if (candidate > best_abs) {
        best = row;
        best_abs = candidate;
      }
    }
    if (best_abs < 1e-18) {
      throw std::runtime_error("Singular linear system");
    }
    if (best != pivot) {
      for (int col = pivot; col < n; ++col) {
        std::swap(a(pivot, col), a(best, col));
      }
      std::swap(b[pivot], b[best]);
    }

    const double diagonal = a(pivot, pivot);
    for (int col = pivot; col < n; ++col) {
      a(pivot, col) /= diagonal;
    }
    b[pivot] /= diagonal;

    for (int row = 0; row < n; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = a(row, pivot);
      if (factor == 0.0) {
        continue;
      }
      for (int col = pivot; col < n; ++col) {
        a(row, col) -= factor * a(pivot, col);
      }
      b[row] -= factor * b[pivot];
    }
  }
  return b;
}

inline std::vector<double> SolveNormalEquations(
    const DenseMatrix& jacobian,
    const std::vector<double>& residuals,
    double damping = 1e-6) {
  const int rows = jacobian.rows();
  const int cols = jacobian.cols();
  if (static_cast<int>(residuals.size()) != rows) {
    throw std::invalid_argument("Residual size does not match Jacobian rows");
  }

  DenseMatrix normal(cols, cols);
  std::vector<double> rhs(cols, 0.0);
  for (int row = 0; row < rows; ++row) {
    for (int i = 0; i < cols; ++i) {
      rhs[i] -= jacobian(row, i) * residuals[row];
      for (int j = 0; j < cols; ++j) {
        normal(i, j) += jacobian(row, i) * jacobian(row, j);
      }
    }
  }
  for (int i = 0; i < cols; ++i) {
    normal(i, i) += damping;
  }
  return SolveLinearSystem(normal, rhs);
}

inline double Rms(const std::vector<double>& residuals) {
  if (residuals.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const double residual : residuals) {
    sum += residual * residual;
  }
  return std::sqrt(sum / static_cast<double>(residuals.size()));
}

}  // namespace s23::math
