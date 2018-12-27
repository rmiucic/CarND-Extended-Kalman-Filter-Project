#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
			        0, 1, 0, 0;
  //in lesson 25/15 Radar Measurements
  //process noise is omega
  //z=h(x_prime) + omega
  //       ^         ^
  //       |         |
  //     Linear    Process
  //     Motion    Noise 

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */

  if (!is_initialized_) 
  {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    double px_=0.0;
    double py_=0.0;
    double vx_=0.0;
    double vy_=0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double ro_=measurement_pack.raw_measurements_(0);
      double theta_=measurement_pack.raw_measurements_(1);
      double ro_dot_=measurement_pack.raw_measurements_(2);
       
      px_=ro_*cos(theta_);
      py_=ro_*sin(theta_);
      vx_=ro_dot_*cos(theta_);
      vy_=ro_dot_*sin(theta_);
      //std::cout << "rmiucic RADAR initialized ekf_.x_" << std::endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      // TODO: Initialize state.
  
        px_=measurement_pack.raw_measurements_(0);
        py_=measurement_pack.raw_measurements_(1);
      //std::cout << "rmiucic LIDAR initialized ekf_.x_" << std::endl;
    }
    // small px_, py_
    if(fabs(px_) < 0.0001)
    {
        px_= 0.1;
    }

    if(fabs(py_) < 0.0001)
    {
        py_= 0.1;
    }
    ekf_.x_ << px_, py_,vx_,vy_;
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

    	//measurement matrix
    //ekf_.H_ = MatrixXd(2, 4);
    //ekf_.H_ << 1, 0, 0, 0,
    //           0, 1, 0, 0;

    //the initial transition matrix F_
	  ekf_.F_ = MatrixXd(4, 4);
	  ekf_.F_ << 1, 0, 1, 0,
			         0, 1, 0, 1,
			         0, 0, 1, 0,
			         0, 0, 0, 1;
      
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    //cout << "rmiucic ekf_.x initialized to  = " << ekf_.x_ << endl;

    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  //ekf_.F_=MatrixXd(4, 4);
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  double noise_ax = 9;
  double noise_ay = 9;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();
  //std::cout << "rmiucic after Predict ekf_.x_ = " << ekf_.x_ << endl;

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  bool enable_lidar=true;
  bool enable_radar=true;
  if (enable_radar && measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
    // TODO: Radar updates
    //std::cout << "rmiucic before Radar update ekf_.x_ = " << ekf_.x_ << endl;
    ekf_.R_=R_radar_;
    ekf_.H_=tools.CalculateJacobian(ekf_.x_);
    if (ekf_.H_.isZero(0))
    {
        cout << "rmiucic Jacobian is zero" << endl;
        return;
    }

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else if (enable_lidar && measurement_pack.sensor_type_ == MeasurementPackage::LASER)
  {
    // TODO: Laser updates
    //std::cout << "rmiucic before LIDAR update ekf_.x_ = " << ekf_.x_ << endl;
    ekf_.R_=R_laser_;
    ekf_.H_=H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
