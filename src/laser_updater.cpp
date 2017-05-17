#include "laser_updater.h"

LaserUpdater::LaserUpdater(const Eigen::MatrixXd &r) : h_(2, 4), r_(r) {
  h_ <<
     1, 0, 0, 0,
      0, 1, 0, 0;
}

void LaserUpdater::First(const Eigen::VectorXd &measurement, KalmanFilter &kf) {
  Eigen::VectorXd x(4);
  x << measurement(0), measurement(1), 0, 0;
  kf.Set(x);
}

void LaserUpdater::Next(const Eigen::VectorXd &measurement, KalmanFilter &kf) {
  Eigen::VectorXd y = measurement - h_ * kf.state().x;

  kf.Update(h_, r_, y);
}
