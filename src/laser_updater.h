#ifndef LASER_UPDATER_H__
#define LASER_UPDATER_H__

#include "Eigen/Dense"

#include "kalman_filter.h"

class LaserUpdater {
 public:
  explicit LaserUpdater(const Eigen::MatrixXd &r);

  /// Applies the first measurement to the Kalman Filter.
  void First(const Eigen::VectorXd &measurement, KalmanFilter &kf);
  /// Update the Kalman Filter with a measurement.
  void Next(const Eigen::VectorXd &measurement, KalmanFilter &kf);

 private:
  Eigen::MatrixXd h_;
  Eigen::MatrixXd r_;
};

#endif /* LASER_UPDATER_H__ */
