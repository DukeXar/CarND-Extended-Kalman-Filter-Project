#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

struct State {
  Eigen::VectorXd x;
  Eigen::MatrixXd p;
};

class KalmanFilter {
 public:
  /**
   * @param noise_ax Acceleration process noise over x
   * @param noise_ay Acceleration process noise over y
   * @param p Initial state covariance matrix
   */
  KalmanFilter(double noise_ax, double noise_ay, const Eigen::MatrixXd &p);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void Predict(double dt);

  /**
   * Sets initial state of the filter with position x
   */
  void Set(const Eigen::VectorXd &x);

  /**
   * Updates state of the filter with measurement difference y
   * @param h Jacobian or measurement function matrix H
   * @param r measurement noise covariance matrix
   * @param y = z - H * state.x
   */
  void Update(const Eigen::MatrixXd &h, const Eigen::MatrixXd &r, const Eigen::VectorXd &y);

  /// Returns the state of the filter.
  State state() const { return state_; }

 private:
  bool is_set_;
  State state_;
  Eigen::MatrixXd f_;
  Eigen::MatrixXd Qv_;
};

#endif /* KALMAN_FILTER_H_ */
