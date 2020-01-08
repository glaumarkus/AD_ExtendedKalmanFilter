#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  Update_from_y(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho = sqrt(px * px + py * py);
  if (rho < 0.00001){
    px += 0.001;
    py += 0.001;
	rho = sqrt(px * px + py * py);
  }
  
  float theta = atan2(py,px);
  float rho_dot = (px*vx+py*vy)/rho;
    
  VectorXd H(3);
  H << rho, theta, rho_dot;
  VectorXd y = z - H;
  
  Normalize_Angle(y);
  Update_from_y(y);
}

void KalmanFilter::Update_from_y(const VectorXd &y) {
  
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

void KalmanFilter::Normalize_Angle(VectorXd &y) {
  
  while ( y(1) > M_PI || y(1) < -M_PI){
    if (y(1) > M_PI){
      y(1) -= M_PI;
    }
    else{
      y(1) += M_PI;
    }
  }
}
