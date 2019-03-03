#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initialize matrices.
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix for lidar.
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  // Measurement covariance matrix for radar.
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1; 

  // The initial transition matrix F_.
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // State covariance matrix P.
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0,    0,
             0, 1, 0,    0,
             0, 0, 1000, 0,
             0, 0, 0,    1000;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /* Initialize */

  if (!is_initialized_) {
    // Initialize state with first measurement.
    std::cout << "EKF: " << std::endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout <<"radar"<< std::endl;
      // Read radar measurements.
      float ro = measurement_pack.raw_measurements_(0); // range to target
      float phi = measurement_pack.raw_measurements_(1); // angle to target
      float ro_dot = measurement_pack.raw_measurements_(2); // change in range to target
      // Convert from polar to cartesian.
      ekf_.x_(0) = ro * cos(phi); // x position
      ekf_.x_(1) = ro * sin(phi); // y position
      ekf_.x_(2) = ro_dot * cos(phi); // x velocity
      ekf_.x_(3) = ro_dot * sin(phi); // y velocity
      std::cout << "x_ = " << ekf_.x_ << std::endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "lidar" << std::endl;
      // Read lidar measurements.
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // Done initializing, no need to predict or update.
    is_initialized_ = true;
    return;
  }

  /* Predict */

  // Compute the time elapsed between the current and previous measurements, in seconds.
  float dt_1 = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt_1 * dt_1;
  float dt_3 = dt_2 * dt_1;
  float dt_4 = dt_3 * dt_1;

  // Modify the F matrix so that the time is integrated.
  ekf_.F_(0, 2) = dt_1;
  ekf_.F_(1, 3) = dt_1;

  // Set the acceleration noise components.
  float noise_ax = 9;
  float noise_ay = 9;

  // Set the process covariance matrix, Q.
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0,                   dt_3 / 2 * noise_ax, 0,
             0,                   dt_4 / 4 * noise_ay, 0,                   dt_3 / 2 * noise_ay,
             dt_3 / 2 * noise_ax, 0,                   dt_2*noise_ax,       0,
             0,                   dt_3 / 2 * noise_ay, 0,                   dt_2 * noise_ay;

  ekf_.Predict();
  std::cout << "x_ = " << ekf_.x_ << std::endl;

  /* Update */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) { // ridar update
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else { // lidar update
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Print the output.
  std::cout << "x_ = " << ekf_.x_ << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
