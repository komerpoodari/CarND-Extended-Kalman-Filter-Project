
#include <iostream>
#include <math.h>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */

  	x_ = F_ * x_;  // + u,  but u = 0; mean = 0
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //convert cartesian coordinates to polar
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
        
  float rho = sqrt(px*px + py*py);
  
  /**
  In C++, atan2() returns values between -pi and pi. When calculating phi in 
  y = z - h(x) for radar measurements, the resulting angle phi in the y vector 
  should be adjusted so that it is between -pi and pi. The Kalman filter is 
  expecting small angle values between the range -pi and pi. 
  HINT: when working in radians, you can add 2\pi2π or subtract 2\pi2π until 
  the angle is within the desired range.
  */
  
  float theta = atan2(py, px);
  
  const float pi = 3.1415927;
  
  // debug statement
  if ((theta < -pi) || (theta > pi)) {
      cout << "atan2 result: " << theta << endl;
  }

  float rho_dot = (px*vx + py+vy) / rho;
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;
        
  VectorXd y = z - z_pred;
  
   //cout << "y(1) theta: " << y(1) << endl;
  // in case absolute theta  > 360 degrees 
  if (abs(y(1)) > 2*pi) {
      cout << "y(1) before mod: " << y(1) << endl;
      y(1) = fmodf(y(1), 2*pi);
      cout << "y(1) after mod: " << y(1) << endl;
  }
  
  // in case absolute theta  > 180 degrees 
  if (y(1) > pi) {
      cout << "y(1) before sub: " << y(1) << endl;
      y(1) -= 2*pi;
      cout << "y(1) after sub: " << y(1) << endl;
  }
  else if (y(1) < -pi) {
      cout << "y(1) before add: " << y(1) << endl;
      y(1) += 2*pi;
      cout << "y(1) after add: " << y(1) << endl;
  }
  
        
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
