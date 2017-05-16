#include "FusionEKF.h"

#include <iostream>
#include <stdexcept>

#include "Eigen/Dense"

Eigen::MatrixXd GetLaserRMatrix() {
  Eigen::MatrixXd result(2, 2);
  result <<
      0.0225, 0,
      0, 0.0225;
  return result;
}

Eigen::MatrixXd GetLaserHMatrix() {
  Eigen::MatrixXd result(2, 4);
  result <<
      1, 0, 0, 0,
      0, 1, 0, 0;
  return result;
}

Eigen::MatrixXd GetRadarRMatrix() {
  Eigen::MatrixXd result(3, 3);
  result <<
      0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
  return result;
}

Eigen::MatrixXd GetStateTransitionMatrix() {
  Eigen::MatrixXd result(4, 4);
  result << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  return result;
}

Eigen::MatrixXd GetInitialPMatrix() {
  // Copied from Lesson 3
  // TODO(dukexar): why such numbers?
  Eigen::MatrixXd result(4, 4);
  result << 1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1000, 0,
  0, 0, 0, 1000;
  return result;
}

FusionEKF::FusionEKF()
    : is_initialized_(false), previous_timestamp_(0),
      laser_updater_(GetLaserHMatrix(), GetLaserRMatrix()),
      radar_updater_(GetRadarRMatrix()),
      kf_(9, 9, GetStateTransitionMatrix(), GetInitialPMatrix()) {

}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    previous_timestamp_ = measurement_pack.timestamp_;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout << "Got radar initial" << std::endl;
      kf_.Update(radar_updater_.First(measurement_pack.raw_measurements_, kf_.state()));
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "Got laser initial" << std::endl;
      kf_.Update(laser_updater_.First(measurement_pack.raw_measurements_, kf_.state()));
    } else {
      throw std::runtime_error("Invalid input");
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  kf_.Predict((measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0);

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    std::cout << "Got radar update" << std::endl;
    kf_.Update(radar_updater_.Next(measurement_pack.raw_measurements_, kf_.state()));
  } else {
    std::cout << "Got laser update" << std::endl;
    kf_.Update(laser_updater_.Next(measurement_pack.raw_measurements_, kf_.state()));
  }

  // print the output
  State state = kf_.state();
  std::cout << "Final state ----------------------" << std::endl;
  std::cout << "x_ = " << state.x.transpose() << std::endl;
  std::cout << "P_ = " << state.p << std::endl;
  std::cout << "-----------------------" << std::endl;
}
