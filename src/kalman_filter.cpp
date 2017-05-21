#include "kalman_filter.h"

#include <iostream>

KalmanFilter::KalmanFilter(double noise_ax, double noise_ay,
                           const Eigen::MatrixXd &p) : is_set_(false), state_(), f_(4, 4), Qv_(2, 2) {
  Qv_ << noise_ax, 0, 0, noise_ay;
  f_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;
  state_.p = p;
}

void KalmanFilter::Predict(double dt) {
  double dt2 = dt * dt;

  Eigen::MatrixXd g(4, 2);
  g << dt2 / 2, 0,
      0, dt2 / 2,
      dt, 0,
      0, dt;

  Eigen::MatrixXd q = g * Qv_ * g.transpose();

  f_(0, 2) = dt;
  f_(1, 3) = dt;
  
  state_.x = f_ * state_.x; // + u
  state_.p = f_ * state_.p * f_.transpose() + q;
}

void KalmanFilter::Set(const Eigen::VectorXd &x) {
  state_.x = x;
  is_set_ = true;
}

void KalmanFilter::Update(const Eigen::MatrixXd &h, const Eigen::MatrixXd &r, const Eigen::VectorXd &y) {
  if (!is_set_) {
    throw std::runtime_error("Should call Set() with initial x");
  }

  Eigen::MatrixXd ht = h.transpose();
  Eigen::MatrixXd htp = state_.p * ht;
  Eigen::MatrixXd s = h * htp + r;
  Eigen::MatrixXd k = htp * s.inverse();

  auto I = Eigen::MatrixXd::Identity(h.cols(), h.cols());

  state_.x = state_.x + k * y;
  state_.p = (I - k * h) * state_.p;
}
