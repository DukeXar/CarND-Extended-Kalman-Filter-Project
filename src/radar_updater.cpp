#include "radar_updater.h"

#include <cmath>
#include <iostream>


namespace {

Eigen::VectorXd ToRadarMeasurement(const Eigen::VectorXd &x) {
  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);
  
  Eigen::VectorXd result(3);
  
  if (px == 0.0 && py == 0.0) {
    // TODO(dukexar): warn
    std::cout << "Warn: px==0 && py==0" << std::endl;
    return result;
  }
  
  double rho = sqrt(px * px + py * py);
  
  if (rho == 0.0) {
    // TODO(dukexar): warn
    std::cout << "Warn: rho==0" << std::endl;
    return result;
  }
  
  double ksi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;
  
  result << rho, ksi, rho_dot;
  return result;
}

Eigen::MatrixXd CalculateRadarJakobian(const Eigen::VectorXd &x) {
  auto px = x(0);
  auto py = x(1);
  auto vx = x(2);
  auto vy = x(3);
  
  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = c1 * c2;
  
  Eigen::MatrixXd result(3, 4);
  if (c1 == 0.0) {
    // TODO(dukexar): warning
    return result;
  }
  
  result << px / c2, py / c2, 0, 0,
  -py / c1, px / c1, 0, 0,
  py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;
  return result;
}

} // namespace

RadarUpdater::RadarUpdater(const Eigen::MatrixXd &r) : r_(r) {
}

State RadarUpdater::Next(const Eigen::VectorXd &measurement, const State &state) {
  Eigen::MatrixXd hj = CalculateRadarJakobian(state.x);
  Eigen::MatrixXd hjt = hj.transpose();
  
  Eigen::VectorXd correctedMeasurement = measurement;
  double ksi = measurement(1);

  if (ksi < -M_PI) {
    ksi += M_PI;
  } else if (ksi > M_PI) {
    ksi -= M_PI;
  }
  
  correctedMeasurement(1) = ksi;
  
  Eigen::VectorXd y = measurement - ToRadarMeasurement(state.x);
  Eigen::MatrixXd s = hj * state.p * hjt + r_;
  Eigen::MatrixXd k = state.p * hjt * s.inverse();
  
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.x.size(), state.x.size());
  return State{state.x + k * y, (I - k * hj) * state.p};
}

State RadarUpdater::First(const Eigen::VectorXd &measurement, const State &state) {
  double rho = measurement(0);
  double ksi = measurement(1);
  double px = rho * cos(ksi);
  double py = rho * sin(ksi);
  Eigen::VectorXd projected(4);
  projected << px, py, 0, 0;
  return State{projected, state.p};
}
