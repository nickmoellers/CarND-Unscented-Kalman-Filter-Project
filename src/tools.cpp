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
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if( estimations.size() == 0 ) return rmse;
  //  * the estimation vector size should equal ground truth vector size
  if( estimations.size() != ground_truth.size() ) return rmse;

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd x_e=estimations[i];
    VectorXd x_gt=ground_truth[i];
    VectorXd residual = x_e-x_gt;
    VectorXd squared_residual = residual.array()*residual.array();
    rmse+=squared_residual;
  }

  //cout << "RTE = " << rmse << endl;

  //calculate the mean
  rmse = rmse/estimations.size();

  // << "RME = " << rmse << endl;s

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //cout << "RMSE = " << rmse << endl;

  //cout << "Calculated RMSE." << endl;


  //return the result
  return rmse;
}

