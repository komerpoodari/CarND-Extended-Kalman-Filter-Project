#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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
  Hj_ = MatrixXd(3, 4); // need to be calculated based on state vector x

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

 
  H_laser_ << 1, 0, 0, 0,   // the essential logic is to drop velocity components 
               0, 1, 0, 0;   // so only position components are retained.
  
   

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
  
  // initialize in the constructor; useful particularly when LASER only / RADAR only processing; RMSEcalculation could be invoked initially, even before ProcessMeasurement is initialized.
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1.0, 1.0, 0.0, 0.0;  // komer - the last two elements need to be experimented for best RMSE

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
    cout << "EKF: " << endl;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);

      ekf_.x_(0) = rho * cos(theta); //px
      ekf_.x_(1) = rho * sin(theta); //py

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_[0]; //px
      ekf_.x_(1) = measurement_pack.raw_measurements_[1]; //py
    }
    
    if (fabs(ekf_.x_(0)) < 0.0001 and fabs(ekf_.x_(1)) < 0.0001) {
        ekf_.x_(0) = 0.0001;
        ekf_.x_(1) = 0.0001;
    }

    //the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 0, 0,
			   0, 1, 0, 0,
			   0, 0, 1, 0,
			   0, 0, 0, 1;
    
    //state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;  
    
    //the initial transition matrix Q_
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << 0, 0, 0, 0,
			   0, 0, 0, 0,
			   0, 0, 0, 0,
			   0, 0, 0, 0;
    
    // initalize timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Initialization complete" << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  	
  //compute the time elapsed between the current and previous measurements  - Lesson 5, section 11 & 12. 
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  
  // update timestamp
  previous_timestamp_ = measurement_pack.timestamp_;


 float dt_2 = dt * dt;
 float dt_3 = dt_2 * dt;
 float dt_4 = dt_3 * dt;

 //Modify the F matrix so that the time is integrated
 ekf_.F_(0, 2) = dt;
 ekf_.F_(1, 3) = dt;

 //set the process covariance matrix Q
 ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			 0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			 dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			 0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

 //3. Call the Kalman Filter predict() function 
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
    
    //compute Hj_
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    
    //Assign Hj to ekf_.H_
    ekf_.H_ = Hj_;
    
    //Assign R_radar_ to ekf_.R_
    ekf_.R_ = R_radar_;
    
    //call extended EKF Update
	ekf_.UpdateEKF(measurement_pack.raw_measurements_); 
    
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    
    //4.a Call the Kalman Filter update() function
	// with the most recent raw measurements_
	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
