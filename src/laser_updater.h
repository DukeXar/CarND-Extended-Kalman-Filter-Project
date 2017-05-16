#ifndef LASER_UPDATER_H__
#define LASER_UPDATER_H__

#include "Eigen/Dense"

#include "kalman_filter.h"

class LaserUpdater {
public:
  LaserUpdater(const Eigen::MatrixXd &h, const Eigen::MatrixXd &r)
  : h_(h), ht_(h.transpose()), r_(r) {}
  
  State Next(const Eigen::VectorXd &measurement, const State &state);
  State First(const Eigen::VectorXd &measurement, const State &state);
  
private:
  Eigen::MatrixXd h_;
  Eigen::MatrixXd ht_;
  Eigen::MatrixXd r_;
};

#endif /* LASER_UPDATER_H__ */
