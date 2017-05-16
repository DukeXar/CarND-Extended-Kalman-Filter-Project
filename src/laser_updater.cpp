#include "laser_updater.h"

#include <iostream>

State LaserUpdater::Next(const Eigen::VectorXd &measurement, const State &state) {
  std::cout << "--------------------->>>>>>>>" << std::endl;
  //std::cout << "LaserUpdater::Next(" << measurement << ", state.x=" << state.x << ", state.p=" << state.p << std::endl;
  std::cout << "LaserUpdater::Next(" << measurement.transpose() << ")\n" << std::endl;
  Eigen::VectorXd y = measurement - h_ * state.x;
  Eigen::MatrixXd s = h_ * state.p * ht_ + r_;
  Eigen::MatrixXd k = state.p * ht_ * s.inverse();
  
  std::cout << "h_=\n" << h_ << "\n" << std::endl;
  std::cout << "y=" << y << "\n" << std::endl;
  std::cout << "state.p=" << state.p << "\n" << std::endl;
  //std::cout << "s=" << s << "\n" << std::endl;
  std::cout << "inv(s)=" << s.inverse() << "\n" << std::endl;
  std::cout << "k=" << k << "\n" << std::endl;
  //std::cout << "h_=" << h_ << "\n" << std::endl;
  
  //std::cout << "k*h=" << (k * h_) << "\n" << std::endl;
  
  auto I = Eigen::MatrixXd::Identity(h_.cols(), h_.cols());
  //std::cout << "I=" << I << std::endl;
  //std::cout << "---------------------" << std::endl;
  State result{state.x + k * y, (I - k * h_) * state.p};
  std::cout << "---------------------<<<<<<<<<<<" << std::endl;
  return result;
}

State LaserUpdater::First(const Eigen::VectorXd &measurement, const State &state) {
  Eigen::VectorXd x(4);
  x << measurement(0), measurement(1), 0, 0;
  return State{x, state.p};
}
