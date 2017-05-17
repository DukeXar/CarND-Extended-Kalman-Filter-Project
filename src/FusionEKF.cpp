#include "FusionEKF.h"

#include <stdexcept>

#include "Eigen/Dense"

namespace {

Eigen::MatrixXd GetLaserRMatrix() {
  Eigen::MatrixXd result(2, 2);
  result <<
         0.0225, 0,
      0, 0.0225;
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

Eigen::MatrixXd GetInitialPMatrix() {
  Eigen::MatrixXd result(4, 4);
  result <<
         1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;
  return result;
}

} // namespace

FusionEKF::FusionEKF()
    : is_initialized_(false),
      previous_timestamp_(0),
      laser_updater_(GetLaserRMatrix()),
      radar_updater_(GetRadarRMatrix()),
      kf_(9, 9, GetInitialPMatrix()) {

}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      radar_updater_.First(measurement_pack.raw_measurements_, kf_);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      laser_updater_.First(measurement_pack.raw_measurements_, kf_);
    } else {
      throw std::runtime_error("Invalid input");
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  kf_.Predict((measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0);
  previous_timestamp_ = measurement_pack.timestamp_;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    radar_updater_.Next(measurement_pack.raw_measurements_, kf_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    laser_updater_.Next(measurement_pack.raw_measurements_, kf_);
  } else {
    throw std::runtime_error("Invalid input");
  }
}
