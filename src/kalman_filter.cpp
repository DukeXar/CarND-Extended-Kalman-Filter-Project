#include "kalman_filter.h"

#include <iostream>

KalmanFilter::KalmanFilter(double noise_ax, double noise_ay,
                           const Eigen::MatrixXd &p): state_(), f_(4, 4), Qv_(2, 2) {
  Qv_ << noise_ax, 0, 0, noise_ay;
  f_ << 1, 0, 1, 0,
  0, 1, 0, 1,
  0, 0, 1, 0,
  0, 0, 0, 1;
  state_.p = p;
}

void KalmanFilter::Predict(double dt) {
  f_(0, 2) = dt;
  f_(1, 3) = dt;

  double dt2 = dt * dt;

  Eigen::MatrixXd g(4, 2);
  g << dt2 / 2, 0,
      0, dt2 / 2,
      dt, 0,
      0, dt;

  Eigen::MatrixXd q = g * Qv_ * g.transpose();

  state_.x = f_ * state_.x; // + u
  state_.p = f_ * state_.p * f_.transpose() + q;
}

void KalmanFilter::Update(const State &state) {
  state_ = state;
}
