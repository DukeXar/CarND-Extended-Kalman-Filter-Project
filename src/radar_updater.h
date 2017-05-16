#ifndef RADAR_UPDATER_H__
#define RADAR_UPDATER_H__

#include "Eigen/Dense"

#include "kalman_filter.h"

class RadarUpdater {
public:
  explicit RadarUpdater(const Eigen::MatrixXd &r);
  
  State Next(const Eigen::VectorXd &measurement, const State &state);
  State First(const Eigen::VectorXd &measurement, const State &state);
  
private:
  Eigen::MatrixXd r_;
};

#endif /* RADAR_UPDATER_H__ */
