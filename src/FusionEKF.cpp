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
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


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
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    
    ekf_.x_ = VectorXd(4);
   

	previous_timestamp_ = measurement_pack.timestamp_;
	
	
			  
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float range = measurement_pack.raw_measurements_[0];
      float bearing = measurement_pack.raw_measurements_[1];
      float range_rate = measurement_pack.raw_measurements_[2];
      ekf_.x_ << range * cos(bearing), range * sin(bearing), 0, 0;
      //ekf_.x_ << range * cos(bearing), range * sin(bearing), range_rate * cos(bearing), range_rate * sin(bearing);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

	
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
	
	cout << "EKF init: " << ekf_.x_ << endl;
	
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	
	float pn_xvar_ = 9.0;
	float pn_yvar_ = 9.0;
	  
	  // process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << pow(dt, 4)/4 * pn_xvar_, 0, pow(dt, 3)/2 * pn_xvar_, 0,
	        0, pow(dt, 4)/4 * pn_yvar_, 0, pow(dt, 3)/2 * pn_yvar_,
	        pow(dt, 3)/2 * pn_xvar_, 0, pow(dt, 2) * pn_xvar_, 0,
	        0, pow(dt, 3)/2 * pn_yvar_, 0, pow(dt, 2) * pn_yvar_;
                
    
			  
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, dt, 0,
			  0, 1, 0, dt,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

			  
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
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
