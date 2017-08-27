#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // set the vector size for x_
  ekf_.x_ = VectorXd(4);
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  // initialize the state covariance matrix P_
  ekf_.P_ = MatrixXd(4, 4);
  /*
  ekf_.P_ << 20, 0, 0, 0,
             0, 20, 0, 0,
             0, 0, 40, 0,
             0, 0, 0, 40;
  */
  ekf_.P_ << 100, 0, 0, 0,
             0, 10, 0, 0,
             0, 0, 5, 0,
             0, 0, 0, 5;
  //the initial transition matrix Q_
  ekf_.Q_ = MatrixXd(4, 4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 5, 0.2;

    // initialize the time
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // initialize the x vector
      VectorXd radar_polar = measurement_pack.raw_measurements_;
      ekf_.x_(0)=radar_polar(0)*cos(radar_polar(1));
      ekf_.x_(1)=radar_polar(0)*sin(radar_polar(1));
      //cout<<"px_init r = "<<ekf_.x_(0)<<endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],5,0.2;
      //cout<<"px_init l = "<<ekf_.x_(0)<<endl;
      //cout<<"py_init l = "<<ekf_.x_(1)<<endl;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // compute the elapsed time, dt in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // prepare some variables to help in the matrix calculation
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // update the F matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  int noise_ax = 9;
  int noise_ay = 9;
  //update the process covariance matrix
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  //std::cout << ekf_.Q_ << std::endl;

  ekf_.Predict();
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
