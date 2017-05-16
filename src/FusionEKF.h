#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "kalman_filter.h"

#include "laser_updater.h"
#include "radar_updater.h"


class FusionEKF {
 public:
  FusionEKF();
  
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  State state() const { return kf_.state(); }
  
 private:
  bool is_initialized_;
  long long previous_timestamp_;
  LaserUpdater laser_updater_;
  RadarUpdater radar_updater_;
  KalmanFilter kf_;
};

#endif /* FusionEKF_H_ */
