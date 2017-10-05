#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  first_calculation = true;

}

void PID::UpdateError(double cte) {
	if(first_calculation){
		p_error = cte;
	  first_calculation = false;
	}
	d_error = cte - p_error;
  p_error = cte;
  i_error += cte ;
}

double PID::TotalError() {
	double steering = -Kp_*p_error -Kd_*d_error -Ki_*i_error;
	cout << "steering angle = " << steering <<endl;
	if(steering > 1.){
		steering = 1.;
	}
	else if(steering < -1.0){
		steering = -1;
	}
	std::cout << "steering angle = " << steering << std::endl;
	return steering;
}

