#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#define PI acos(-1)
//#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

//Tools tools_ekfUpdate;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float theta = atan2(x_(1), x_(0));
  float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;//double
  Eigen::VectorXd h = Eigen::VectorXd(3);
  h << rho, theta, rho_dot;
  Eigen::VectorXd y = z - h;
  
  //normalizing y in theta
  if(y(1)>PI){
    y(1)-=2*PI;
  }
  if(y(1)<-PI){
    y(1)+=2*PI;
  }
  //H_ = tools_ekfUpdate.CalculateJacobian(x_);
  CalculateJacobian();
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K =  P_ * Ht * Si;
  x_ = x_ + (K * y);
  int x_size = x_.size();
  Eigen::MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::CalculateJacobian() {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    H_ = Hj;
    return;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0.0, 0.0,
      -(py/c1), (px/c1), 0.0, 0.0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  H_ = Hj;
}