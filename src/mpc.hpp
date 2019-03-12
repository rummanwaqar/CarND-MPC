#pragma once

#include <vector>
#include "Eigen/Core"

class MPC {
 public:
  const double Lf = 2.67;

  MPC() {};

  virtual ~MPC() = default;

  // Solve the model given an initial state and poly trajactory
  // Return the next state and actuations as a vector.
  std::vector<double> solve(const Eigen::VectorXd &state,
                            const Eigen::VectorXd &coeffs);
};
