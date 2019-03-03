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
  MatrixXd Hj(3,4);

  float px = x_state(0); // x position
  float py = x_state(1); // y position
  float vx = x_state(2); // x velocity
  float vy = x_state(3); // y velocity

  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // Check for division by zero.
  if (fabs(c1) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  // Compute the Jacobian matrix
  Hj << px / c2,                        py / c2,                       0,     0,
        -py / c1,                       px / c1,                       0,     0,
        py * ( vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px/c2, py/c2;

  return Hj;
}
