#include "kalman_filter.h"

#include <iostream>

KalmanFilter::KalmanFilter(double noise_ax, double noise_ay, const Eigen::MatrixXd &f,
                           const Eigen::MatrixXd &p): state_(), f_(f), Qv_(2, 2) {
  Qv_ << noise_ax, 0, 0, noise_ay;
  state_.p = p;
}

void KalmanFilter::Predict(double dt) {
  f_(0, 2) = dt;
  f_(1, 3) = dt;

  //std::cout << "--------------------->>>>>>>>" << std::endl;
  //std::cout << "Predict(" << dt << ", state.x=\n" << state_.x.transpose() << ",\nstate.p=\n" << state_.p << ")\n" << std::endl;
  //std::cout << "f_=\n" << f_ << "\n" << std::endl;
  
  double dt2 = dt * dt;

  Eigen::MatrixXd g(4, 2);
  g << dt2 / 2, 0,
      0, dt2 / 2,
      dt, 0,
      0, dt;

  Eigen::MatrixXd q = g * Qv_ * g.transpose();

  state_.x = f_ * state_.x; // + u
  state_.p = f_ * state_.p * f_.transpose() + q;
  
  //std::cout << "After prediction:\nstate.x=\n" << state_.x.transpose() << ",\nstate.p=\n" << state_.p << "\n" << std::endl;
  //std::cout << "---------------------<<<<<<" << std::endl;
}

void KalmanFilter::Update(const State &state) {
  state_ = state;
  //std::cout << "After update:\nstate.x=\n" << state_.x.transpose() << ",\nstate.p=\n" << state_.p << "\n" << std::endl;
}
