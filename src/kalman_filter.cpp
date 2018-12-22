#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
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
  //std::cout << "rmiucic Predict x_=" << x_ << std::endl;
  x_ = F_ * x_;
  //std::cout << "rmiucic Predict F_=" << F_ << std::endl;
  //std::cout << "rmiucic Predict x_=" << x_ << std::endl;

	MatrixXd Ft = F_.transpose();
  //std::cout << "rmiucic Predict Ft=" << Ft << std::endl;
  //std::cout << "rmiucic Predict F_=" << F_ << std::endl;
  //std::cout << "rmiucic Predict Q_=" << Q_ << std::endl;

  P_ = F_ * P_ * Ft + Q_;
  //std::cout << "rmiucic Predict P_=" << P_ << std::endl;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  //std::cout << "rmiucic Update z=" << z << std::endl;
  //std::cout << "rmiucic Update H_=" << H_ << std::endl;
  //std::cout << "rmiucic Update x_=" << x_ << std::endl;
  //std::cout << "rmiucic Update y=" << y << std::endl;

	MatrixXd S = H_ * P_ * H_.transpose() + R_;
  //std::cout << "rmiucic Update P_=" << P_ << std::endl;
  //std::cout << "rmiucic Update R_=" << R_ << std::endl;
  //std::cout << "rmiucic Update S=" << S << std::endl;

	MatrixXd K = P_ * H_.transpose() * S.inverse();
  //std::cout << "rmiucic Update K=" << P_ << std::endl;
  //std::cout << "rmiucic Update Si=" << S.inverse() << std::endl;

	//new estimate
	x_ = x_ + (K * y);
  //std::cout << "rmiucic Update x_=" << x_ << std::endl;

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //std::cout << "rmiucic Update I=" << I << std::endl;

	P_ = (I - K * H_) * P_;
  //std::cout << "rmiucic Update P_=" << P_ << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // recover state parameters
  float px=x_(0);
  float py=x_(1);
  float vx=x_(2);
  float vy=x_(3);
  //Calculate h 
  MatrixXd hj(3,1);
  float rho = sqrt(px*px+py*py);
  float phi = 0;
  float rho_dot = 0;
  if (fabs(px) < 0.0001 or fabs(py) < 0.0001)
  {
    if(fabs(px) < 0.0001)
    {
      std::cout << "rmiucic UpdateEKF px is small" << std::endl;
      px = 0.0001;
    }
    if(fabs(py) < 0.0001)
    {
      std::cout << "rmiucic UpdateEKF py is small" << std::endl;
      py = 0.0001;
    }
    rho = sqrt(px*px+py*py);
    phi = 0;
    rho_dot = 0;
  }
  else
  {
    rho = sqrt(px*px+py*py);
    phi = atan2(py,px);
    rho_dot = (px*vx+py*vy)/rho;
  }
  hj << rho,
        phi,
        rho_dot;

  //std::cout << "rmiucic UpdateEKF phi=" << phi << std::endl;
  //std::cout << "rmiucic UpdateEKF z=" << z << std::endl;
  //std::cout << "rmiucic UpdateEKF hj=" << hj << std::endl;
  VectorXd y = z - hj;
  //std::cout << "rmiucic UpdateEKF phi=" << y(1) << std::endl;
  bool not_in_pi_bounds=true;
  while(not_in_pi_bounds)
  {
    if(y(1)>3.141592653)
    {
      y(1)=y(1)-2*3.141592653;
    }
    else if(y(1)<-3.141592653)
    {
      y(1)=y(1)+2*3.141592653;
    }
    else
    {
      not_in_pi_bounds=false;
    }
  }
  //std::cout << "rmiucic UpdateEKF y=" << y << std::endl;

  //std::cout << "rmiucic UpdateEKF H_=" << H_ << std::endl;
  //std::cout << "rmiucic UpdateEKF P_=" << P_ << std::endl;
  //std::cout << "rmiucic UpdateEKF H_.transpose()=" << H_.transpose() << std::endl;
  //std::cout << "rmiucic UpdateEKF R_=" << R_ << std::endl;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;

  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  //std::cout << "rmiucic UpdateEKF x_=" << x_ << std::endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
