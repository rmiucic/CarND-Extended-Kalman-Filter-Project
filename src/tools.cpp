#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  //std::cout << "rmiucic6" << std::endl;
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if(0==estimations.size())
  {
    std::cout << "estimations.size() iz ZERO!!!" << std::endl;
    return rmse;
  }
  
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size()!=ground_truth.size())
  {
    std::cout << "estimations.size() is defferent than ground_truth.size()!!!" << std::endl;
    return rmse;
  }
  // TODO: accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd acc_sq_res=estimations[i]-ground_truth[i];
    acc_sq_res = acc_sq_res.array()*acc_sq_res.array();
    rmse = rmse + acc_sq_res;
  }

  // TODO: calculate the mean
  rmse=rmse/estimations.size();
  // TODO: calculate the squared root
  rmse=rmse.array().sqrt();

  // return the result
  //std::cout << "rmiucic CalculateRMSE rmse=" << rmse << std::endl;
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  // recover state parameters
  float px=x_state(0);
  float py=x_state(1);
  float vx=x_state(2);
  float vy=x_state(3);
  //Calculate H Jacobian
  // check division by zero
  float px2py2=px*px+py*py;
  if(0==px2py2)
  {
   std::cout << "rmiucic ERROR computing Jacobian" << std::endl;
  }
  // compute the Jacobian matrix
  float H00=px/sqrt(px2py2);
  float H01=py/sqrt(px2py2);
  float H02=0;
  float H03=0;

  float H10=-py/px2py2;
  float H11=px/px2py2;
  float H12=0;
  float H13=0;

  float H20=py*(vx*py-vy*px)/pow(px2py2,3/2);
  float H21=px*(vy*px-vx*py)/pow(px2py2,3/2);
  float H22=px/sqrt(px2py2);
  float H23=py/sqrt(px2py2);
  
  MatrixXd Hj(3,4);
  Hj << H00, H01, H02, H03,
        H10, H11, H12, H13,
        H20, H21, H22, H23;
  return Hj;

}
