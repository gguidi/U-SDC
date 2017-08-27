#include "kalman_filter.h"
#include <iostream>
#include <math.h>


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
  /**
  TODO:
    * predict the state
    for both lidar and radar we use the F matrix and not Fj because the 
    prediction step we chose is linear.
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

// update is used for laser data as it is linear
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
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float ro = std::sqrt(px*px + py*py);
  float phi = 0;
  float ro_dot = 0;
  if (px!=0){
  	phi = atan2(py,px);  	
  }
  if (ro!=0){
  	ro_dot = (px*vx+py*vy)/ro;
  }
  //std::cout << "ro = " << ro  << std::endl;
  //std::cout << "phi = " << phi << "rad" << std::endl;
  //std::cout << "ro_dot = " << ro_dot << std::endl;
	
	VectorXd z_pred(3);
  z_pred << ro, phi, ro_dot;
	VectorXd y = z - z_pred;
	//std::cout << "y = " << y(1) << std::endl;
	// normalize phi to be in range -pi ; pi
	y(1) = fmod(y(1),3.1415);
  //std::cout << "y_norm = " << y(1) << std::endl;
  //std::cout << "y = " << y << std::endl;
	
	MatrixXd H_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_t + R_;
	
	MatrixXd Si = S.inverse();
	MatrixXd PHj = P_ * H_t;
	MatrixXd K = PHj * Si;

	// create a new estimate **************
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
