#ifndef RADAR_UPDATER_H__
#define RADAR_UPDATER_H__

#include "Eigen/Dense"

#include "kalman_filter.h"

class RadarUpdater {
 public:
  explicit RadarUpdater(const Eigen::MatrixXd &r);

  /// Applies the first measurement to the Kalman Filter.
  void First(const Eigen::VectorXd &measurement, KalmanFilter &kf);
  /// Update the Kalman Filter with a measurement.
  void Next(const Eigen::VectorXd &measurement, KalmanFilter &kf);

 private:
  Eigen::MatrixXd r_;
};

#endif /* RADAR_UPDATER_H__ */
