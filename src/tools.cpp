#include <iostream>
#include "tools.h"

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
    
  MatrixXd Hj(3,4);
    
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
    
  // Some re-used equations for matrix below
  float c1 = px * px + py * py;
  //check division by zero
  if(c1 < .00001) {
    px = .001;
    py = .001;
    c1 = px * px + py * py;
  }
  float c2 = sqrt(c1);
  float c3 = c1 * c2;
    
  //compute the Jacobian matrix
  Hj << px/c2, py/c2, 0, 0,
        -py/c1, px/c1, 0, 0,
        (py*(vx*py - vy*px))/c3, (px*(vy*px - vx*py))/c3, px/c2, py/c2;
    
  //return the matrix
  return Hj;
}
