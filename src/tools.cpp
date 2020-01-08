#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  int n = estimations.size();
    
  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd d = estimations[i] - ground_truth[i];
    d = d.array() * d.array();
    rmse += d;
  }
  
  rmse = rmse / n;
  rmse = sqrt(rmse.array());
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
    
  float rho = px * px + py * py;

  if(fabs(rho) < .00001) {
    px += .001;
    py += .001;
    rho = px * px + py * py;
  }
  float theta = sqrt(rho);
  float rho_dot = rho * theta;
    
  MatrixXd Hj(3,4);
  Hj << px/theta, py/theta, 0, 0,
        -py/rho, px/rho, 0, 0,
        (py*(vx*py - vy*px))/rho_dot, (px*(vy*px - vx*py))/rho_dot, px/theta, py/theta;
  return Hj;
}