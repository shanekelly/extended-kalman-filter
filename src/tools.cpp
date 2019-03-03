#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4); // initialize 4 element vector for root mean squared error
  rmse << 0, 0, 0, 0; // x position, y position, x velocity, y velocity

  // Check that `estimations` is not of length zero and that `estimations` and `ground_truth` have
  //   the same number of entries.
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // Accumulate squared residuals.
  for (std::size_t i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i]; // element-wise difference
    residual = residual.array() * residual.array(); // square each difference element-wise
    rmse += residual;
  }

  // Calculate the mean.
  rmse = rmse/estimations.size();

  // Calculate the squared root.
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
