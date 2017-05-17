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

  if (std::abs(px) == 0.0) {
    std::cout << "Warn: px==0" << std::endl;
    result << 0, 0, 0;
    return result;
  }

  double rho = sqrt(px * px + py * py);

  if (rho == 0.0) {
    std::cout << "Warn: rho==0" << std::endl;
    result << 0, 0, 0;
    return result;
  }

  double phi = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;

  result << rho, phi, rho_dot;
  return result;
}

Eigen::MatrixXd CalculateRadarJakobian(const Eigen::VectorXd &x) {
  double px = x(0);
  double py = x(1);
  double vx = x(2);
  double vy = x(3);

  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = c1 * c2;

  Eigen::MatrixXd result(3, 4);
  if (c1 == 0.0) {
    std::cout << "Warn: c1==0" << std::endl;
    result << 0, 0, 0;
    return result;
  }

  result <<
         (px / c2), (py / c2), 0, 0,
      (-py / c1), (px / c1), 0, 0,
      (py * (vx * py - vy * px) / c3), (px * (px * vy - py * vx) / c3), (px / c2), (py / c2);
  return result;
}

double ClampPhi(double phi) {
  while (phi < -M_PI) {
    phi += 2 * M_PI;
  }
  while (phi > M_PI) {
    phi -= 2 * M_PI;
  }
  return phi;
}

} // namespace

RadarUpdater::RadarUpdater(const Eigen::MatrixXd &r) : r_(r) {
}

void RadarUpdater::First(const Eigen::VectorXd &measurement, KalmanFilter &kf) {
  double rho = measurement(0);
  double phi = measurement(1);

  Eigen::VectorXd projected(4);
  projected << rho * cos(phi), rho * sin(phi), 0, 0;
  kf.Set(projected);
}

void RadarUpdater::Next(const Eigen::VectorXd &measurement, KalmanFilter &kf) {
  State state = kf.state();

  Eigen::VectorXd y = measurement - ToRadarMeasurement(state.x);
  y(1) = ClampPhi(y(1));

  Eigen::MatrixXd hj = CalculateRadarJakobian(state.x);

  kf.Update(hj, r_, y);
}
