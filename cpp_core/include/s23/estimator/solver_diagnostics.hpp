#pragma once

#include <string>
#include <vector>

namespace s23 {

struct SolverIteration {
  int iteration = 0;
  double rms_residual = 0.0;
  double max_update = 0.0;
};

struct SolverDiagnostics {
  bool converged = false;
  int iterations = 0;
  std::vector<SolverIteration> history;
  std::vector<double> final_residuals;
  std::string message;
};

}  // namespace s23
