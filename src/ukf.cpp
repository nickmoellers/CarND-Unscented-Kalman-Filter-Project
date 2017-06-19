#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 0, 0, 0, 0, 0;



  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5; // TUNE

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5; // TUNE

  // Constants below from manufacturer
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

    // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
  /*P_ << std_laspx_, 0, 0, 0, 0,
        0, std_laspy_, 0, 0, 0,
        0, 0, std_radr_, 0, 0,
        0, 0, 0, std_radphi_, 0,
        0, 0, 0, 0, std_radrd_;*/

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */


  ///* time when the state is true, in us
  time_us_;

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //augmented sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  ///* Weights of sigma points
  //VectorXd weights_;
  weights_ = VectorXd(2*n_aug_+1);
  weights_[0] = lambda_/(lambda_+n_aug_);

  for( int i = 1; i < 2*n_aug_+1; i++) {
      weights_(i) = 0.5/(lambda_+n_aug_);
  }

  NIS_laser_ = 0;
  NIS_radar_ = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  //cout << "Processing Measurement." << endl;

  /*****************************************************************************
   *  Initialization
  ****************************************************************************/
  if (!is_initialized_ ) {

    float x = 0;
    float y = 0;
    float v = 0;
    float phi = 0;
    float phidot = 0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //cout << "Initializing x_ from radar data." << endl;

      float ro = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];
      float ro_dot = meas_package.raw_measurements_[2];

      x = ro * cos( theta );
      y = ro * sin( theta );
      int vx = ro_dot * cos( theta );
      int vy = ro_dot * sin( theta );
      v = sqrt( vx * vx + vy * vy );
      //phi
      //phidot;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //cout << "Initializing x_ from laser data." << endl;
      x = meas_package.raw_measurements_[0];
      y = meas_package.raw_measurements_[1];
  


    } else {
      //cout << "WARNING: Failed to initalize ukf_.x_ from data" << std::endl;
    }

    x_ << x, y, v, phi, phidot;

    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    //cout << "Initialized." << endl;

    return;
  }


  float dt = (meas_package.timestamp_ - time_us_)/1000000.0; //add one more zero?
  time_us_ = meas_package.timestamp_;

  if( dt > 0.0001 ) { //improvement suggestions by reviewer

    //cout << "Predicting." << endl;
    Prediction(dt);

    //cout << "Udating." << endl;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates 
      if(use_radar_) UpdateRadar(meas_package);
    } else {
      // Laser updates
      if(use_laser_) UpdateLidar(meas_package);
    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //cout << "Generating Sigma Points." << endl;
  //Generate Signma Points
  AugmentedSigmaPoints( );

  //cout << "Predicting Sigma Points." << endl;
  //Predict Sigma Points
  SigmaPointPrediction( delta_t );

  //cout << "Predicting Mean and covariance." << endl;
  //Predict Mean and Covariance
  PredictMeanAndCovariance();

  

}


/*******************************************************************************
* From Lesson Section 17:
*******************************************************************************/

void UKF::AugmentedSigmaPoints( ) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create augmented mean state
  x_aug.fill(0);
  x_aug.head(5) = x_;

  //create augmented covariance matrix
  P_aug.fill(0.0); 
  P_aug.topLeftCorner( 5, 5 ) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  float sqrtlambdaplusnx = sqrt(lambda_+n_aug_);
  Xsig_aug_.col(0) = x_aug;
  for( int i = 0 ; i < n_aug_ ; i++ ) {
    VectorXd B = A.col(i) * sqrtlambdaplusnx;
    Xsig_aug_.col(i+1) = x_aug + B;
    Xsig_aug_.col(i+n_aug_+1) = x_aug - B;
  }


  //print result
  //cout << "Xsig_aug_ = " << std::endl << Xsig_aug_ << std::endl;

}

/*******************************************************************************
* From Lesson Section 20:
*******************************************************************************/

void UKF::SigmaPointPrediction( double delta_t ) {

  //predict sigma points
  for( int i = 0; i < 2*n_aug_+1; i++) {
      double x = Xsig_aug_(0,i);
      double y = Xsig_aug_(1,i);
      double v = Xsig_aug_(2,i);
      double phi = Xsig_aug_(3,i);
      double phidot = Xsig_aug_(4,i);
      double nu = Xsig_aug_(5,i);
      double nudot = Xsig_aug_(6,i);
      
      double x_p = x;
      double y_p = y;
      double v_p = v;
      double phi_p = phi;
      double phidot_p = phidot;
      
      //avoid division by zero
      if(fabs(phidot) < 0.001) {
          x_p += v*cos(phi)*delta_t;
          y_p += v*sin(phi)*delta_t;
      } else {
          x_p += (v/phidot)*(sin(phi+phidot*delta_t)-sin(phi));
          y_p += (v/phidot)*(cos(phi)-cos(phi+phidot*delta_t));
      }
      
      x_p += 0.5*delta_t*delta_t*cos(phi)*nu;
      y_p += 0.5*delta_t*delta_t*sin(phi)*nu;
      
      v_p += delta_t*nu;
      phi_p += phidot*delta_t + 0.5*delta_t*delta_t*nudot;
      phidot_p += delta_t*nudot;
      
      //write predicted sigma points into right column
      Xsig_pred_(0,i) = x_p;
      Xsig_pred_(1,i) = y_p;
      Xsig_pred_(2,i) = v_p;
      Xsig_pred_(3,i) = phi_p;
      Xsig_pred_(4,i) = phidot_p;
      
  }
  
  //print result
  //cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;

}

/*******************************************************************************
* From Lesson Section 23:
*******************************************************************************/
void UKF::PredictMeanAndCovariance( ) {

  //predict state mean
  x_.fill(0.0);
  for( int i = 0; i < 2*n_aug_+1; i++) {
     x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for( int i = 0; i < 2*n_aug_+1; i++) {
    MatrixXd x_diff = Xsig_pred_.col(i)-x_;
    NormalizeAngle( x_diff(3) );
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

}



/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  //cout << "Lidar" << endl;

  n_z_ = 2;

  float x = meas_package.raw_measurements_[0];
  float y = meas_package.raw_measurements_[1];

  VectorXd z = VectorXd(n_z_);
  z << x, y;

  R_ = MatrixXd(n_z_, n_z_);
  R_ << std_laspx_, 0,
        0, std_laspy_;

  //create example matrix with sigma points in measurement space
  Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);
  
  //create example vector for mean predicted measurement
  z_pred_ = VectorXd(n_z_); //predicted rho, phi, rhodot

  //matrix for predicted measurement covariance
  S_ = MatrixXd(n_z_,n_z_);

  //vector for incoming radar measurement
  z_ = VectorXd(n_z_); // rho, phi, rho_dot

  //cout << "Predicting Lidar Measurement" << endl;
  PredictLidarMeasurement(z);

  //cout << "Updating State" << endl;
  UpdateState( z );

  NIS_laser_ = CaluclateNIS( z );

  //cout << "L\t" << NIS_laser_ << endl;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */


  //set measurement dimension, radar can measure r, phi, and r_dot
  

  //cout << "Radar" << endl;

  n_z_ = 3;

  float ro = meas_package.raw_measurements_[0];
  float theta = meas_package.raw_measurements_[1];
  float ro_dot = meas_package.raw_measurements_[2];

  VectorXd z = VectorXd(n_z_);
  z << ro, theta, ro_dot;

  R_ = MatrixXd(n_z_, n_z_);
  R_ << std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

  //create example matrix with sigma points in measurement space
  Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);
  
  //create example vector for mean predicted measurement
  z_pred_ = VectorXd(n_z_); //predicted rho, phi, rhodot

  //matrix for predicted measurement covariance
  S_ = MatrixXd(n_z_,n_z_);

  //vector for incoming radar measurement
  z_ = VectorXd(n_z_); // rho, phi, rho_dot

  //cout << "Predicting Radar Measurement" << endl;

  PredictRadarMeasurement(z);

  //cout << "Updating State" << endl;

  UpdateState(z );

  NIS_radar_ = CaluclateNIS( z );

  //cout << "R\t" << NIS_radar_ << endl;

}

//No idea what goes in here
void UKF::PredictLidarMeasurement(VectorXd z) {

//transform sigma points into measurement space
  for( int i = 0 ; i < 2*n_aug_+1 ; i++ ) {
      double x = Xsig_pred_(0,i);
      double y = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      
      Zsig_(0,i) = x;
      Zsig_(1,i) = y;

      //possible to add other dimensions???
  }
  
  //calculate mean predicted measurement
  z_pred_.fill(0.0);
  for( int i = 0; i< 2*n_aug_+1; i++) {
      z_pred_ += weights_(i) * Zsig_.col(i);
  }
  
  //calculate measurement covariance matrix S
  S_.fill(0.0);
  for( int i = 0 ; i < 2*n_aug_+1; i++ ) {
      VectorXd Zdiff = Zsig_.col(i)-z_pred_;
      NormalizeAngle(Zdiff(1));
      //Zdiff(1) = atan2(sin(Zdiff(1)), cos(Zdiff(1)));
      S_ += weights_(i) * Zdiff * Zdiff.transpose();
  }  
        
  S_+=R_;

  return;
}

/*******************************************************************************
* From Lesson Section 26:
*******************************************************************************/

void UKF::PredictRadarMeasurement(VectorXd z) {

  //transform sigma points into measurement space
  for( int i = 0 ; i < 2*n_aug_+1 ; i++ ) {

      double x = Xsig_pred_(0,i);
      double y = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      
      double vx = cos(yaw)*v;
      double vy = sin(yaw)*v;
      
      double r = sqrt(x*x+y*y);
      double phi = atan2(y, x);
      double rdot = (x * vx + y * vy)/r;
      
      Zsig_(0,i) = r;
      Zsig_(1,i) = phi;
      Zsig_(2,i) = rdot;
  }

  //calculate mean predicted measurement
  z_pred_.fill(0.0);
  for( int i = 0; i< 2*n_aug_+1; i++) {
      z_pred_ += weights_(i) * Zsig_.col(i);
  }

  //calculate measurement covariance matrix S
  S_.fill(0.0);
  for( int i = 0 ; i < 2*n_aug_+1; i++ ) {
      VectorXd Zdiff = Zsig_.col(i)-z_pred_;
      NormalizeAngle(Zdiff(1));
      //Zdiff(1) = atan2(sin(Zdiff(1)), cos(Zdiff(1)));
      S_ += weights_(i) * Zdiff * Zdiff.transpose();
  }  
 
  S_+=R_;

}


/*******************************************************************************
* From Lesson Section 29:
*******************************************************************************/

void UKF::UpdateState( VectorXd z ) {

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for( int i = 0; i< n_aug_*2+1 ; i ++ ) {
    VectorXd Zdiff = Zsig_.col(i) - z_pred_;
    NormalizeAngle(Zdiff(1));
    //Zdiff(1) = atan2(sin(Zdiff(1)),cos(Zdiff(1)));
    VectorXd Xdiff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(Xdiff(1));
    //Xdiff(1) = atan2(sin(Xdiff(1)),cos(Xdiff(1)));
    Tc+= weights_(i) * Xdiff * Zdiff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc*S_.inverse();
  
  //update state mean and covariance matrix
  VectorXd Zdiff = z-z_pred_;
  NormalizeAngle(Zdiff(1));

  x_ = x_ + K*Zdiff;
  P_ = P_ - K*S_*K.transpose();



}

void UKF::NormalizeAngle( double& phi ) {
  phi = atan2(sin(phi), cos(phi));
}

double UKF::CaluclateNIS( VectorXd z ) {
  double NIS = 0.0;
  MatrixXd Si = S_.inverse();
  NIS = z.transpose() * Si * z;
  return NIS;
}