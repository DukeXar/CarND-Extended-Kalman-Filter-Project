#include "laser_updater.h"

#include <iostream>

State LaserUpdater::Next(const Eigen::VectorXd &measurement, const State &state) {
  Eigen::VectorXd y = measurement - h_ * state.x;
  Eigen::MatrixXd s = h_ * state.p * ht_ + r_;
  Eigen::MatrixXd k = state.p * ht_ * s.inverse();

  auto I = Eigen::MatrixXd::Identity(h_.cols(), h_.cols());
  State result{state.x + k * y, (I - k * h_) * state.p};
  return result;
}

State LaserUpdater::First(const Eigen::VectorXd &measurement, const State &state) {
  Eigen::VectorXd x(4);
  x << measurement(0), measurement(1), 0, 0;
  return State{x, state.p};
}
