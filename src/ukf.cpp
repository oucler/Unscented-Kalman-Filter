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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = 7;
  int n_sig = 2 * n_aug_ + 1;
  lambda_ = 3 - n_aug_;
  Xsig_pred_ = MatrixXd(n_x_, n_sig);
  // Set weights
  weights_ = VectorXd(n_sig);
  weights_(0) = (double)lambda_ / (lambda_ + n_aug_);
  double weights_val = 1.0 / (2.0 * (lambda_ + n_aug_));
  weights_.bottomRows(n_sig-1).fill(weights_val);
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
  // Initialization
  if (!is_initialized_) {
    x_ << 0.0,0.0,0.0,0,0.0;
    P_ << 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 20.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 13.15, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.1;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float rhodot = meas_package.raw_measurements_(2);
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
      x_(2) = rhodot;
      P_(2,2) = 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      if (fabs(x_(0)) < 0.001 && fabs(x_(1)) < 0.001) {
        x_(0) = 0.001;
        x_(1) = 0.001;
      }
    }
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
  }
  else {
    if ((!use_radar_ &&
         meas_package.sensor_type_ == MeasurementPackage::RADAR)
        ||
        (!use_laser_ &&
          meas_package.sensor_type_ == MeasurementPackage::LASER)) {
      return;
    }
    float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;
    Prediction(dt);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
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
/*******************************************************************************
 * Augmentation
 ******************************************************************************/
  int n_sig = 2 * n_aug_ + 1;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  MatrixXd B = sqrt(lambda_ + n_aug_) * A;

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.leftCols(n_aug_ + 1).rightCols(n_aug_) = x_aug.replicate(1, B.cols()) + B;
  Xsig_aug.rightCols(n_aug_) = x_aug.replicate(1, B.cols()) - B;

/*******************************************************************************
 * Prediction
 ******************************************************************************/

  //predict sigma points
  for (int i = 0; i< n_sig; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    double delta_t2 = delta_t * delta_t;
    double yawd_dt = yawd * delta_t;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        Xsig_pred_(0,i) = p_x + v/yawd * (sin(yaw + yawd_dt) - sin(yaw)) +
                          (0.5 * delta_t2 * cos(yawd) * nu_a);
        Xsig_pred_(1,i) = p_y + v/yawd * (-cos(yaw + yawd_dt) + cos(yaw)) +
                          (0.5 * delta_t2 * sin(yawd) * nu_a);
    }
    else {
        Xsig_pred_(0,i) = p_x + v*delta_t*cos(yaw) +
                          (0.5 * delta_t2 * cos(yawd) * nu_a);
        Xsig_pred_(1,i) = p_y + v*delta_t*sin(yaw) +
                          (0.5 * delta_t2 * sin(yawd) * nu_a);
    }

    Xsig_pred_(2,i) = v + nu_a * delta_t;
    Xsig_pred_(3,i) = yaw + yawd_dt + 0.5 * nu_yawdd * delta_t2;
    Xsig_pred_(4,i) = yawd + nu_yawdd * delta_t;
  }
/*******************************************************************************
 * Predicted Mean Covariance
 ******************************************************************************/

  // Predict state mean
  MatrixXd x_sum = Xsig_pred_.array().rowwise() * weights_.transpose().array();
  x_ = x_sum.rowwise().sum();

  //predict state covariance matrix
  MatrixXd X_abs = Xsig_pred_.array().colwise() - x_.array();

  //angle normalization
  for (int i=0; i < n_sig; i++) {
    X_abs(3,i) = NormalizeAngle(X_abs(3,i));
  }

  // Will be used in Update also
  X_abs_w_ = X_abs.array().rowwise() * weights_.transpose().array();

  P_ = X_abs_w_ * X_abs.transpose();
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


/*******************************************************************************
 * Predicted Lidar Measurement
 ******************************************************************************/

  int n_z = 2;
  int n_sig = 2 * n_aug_ + 1;
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  VectorXd z = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < n_sig; ++i) {
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);
  }

  //calculate mean predicted measurement
  MatrixXd z_sum = Zsig.array().rowwise() * weights_.transpose().array();
  z_pred = z_sum.rowwise().sum();

  //calculate measurement covariance matrix S
  MatrixXd Z_abs = Zsig.array().colwise() - z_pred.array();

  //angle normalization
  for (int i=0; i < n_sig; i++) {
    Z_abs(1,i) = NormalizeAngle(Z_abs(1,i));
  }
  MatrixXd Z_abs_w = Z_abs.array().rowwise() * weights_.transpose().array();
  S = Z_abs_w * Z_abs.transpose();

   //add measurement noise covariance matrix
  S(0,0) += std_laspx_ * std_laspx_;
  S(1,1) += std_laspy_ * std_laspy_;

/*******************************************************************************
 * UKF Radar Update
 ******************************************************************************/

  MatrixXd Tc = X_abs_w_ * Z_abs.transpose();

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  z = meas_package.raw_measurements_;
  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = NormalizeAngle(z_diff(1));


  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

  // prevent P from diverging
  if (NIS_laser_ > 100.0) {
    is_initialized_ = false;
  }
  else {
    //update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
  }
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
/*******************************************************************************
 * Predicted Radar Measurement
 ******************************************************************************/
  int n_z = 3;
  int n_sig = 2 * n_aug_ + 1;

  MatrixXd Zsig = MatrixXd(n_z, n_sig);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  VectorXd z = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for (int i = 0; i < n_sig; ++i) {
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double y = Xsig_pred_(3,i);

      Zsig(0,i) = sqrt((px * px) + (py * py));
      Zsig(1,i) = atan2(py, px);
      Zsig(2,i) = (((px * cos(y)) + (py * sin(y))) * v) / Zsig(0,i);
  }

  //calculate mean predicted measurement
  MatrixXd z_sum = Zsig.array().rowwise() * weights_.transpose().array();
  z_pred = z_sum.rowwise().sum();

  //calculate measurement covariance matrix S
  MatrixXd Z_abs = Zsig.array().colwise() - z_pred.array();

  //angle normalization
  for (int i=0; i < n_sig; i++) {
    Z_abs(1,i) = NormalizeAngle(Z_abs(1,i));
  }

  MatrixXd Z_abs_w = Z_abs.array().rowwise() * weights_.transpose().array();

  S = Z_abs_w * Z_abs.transpose();

  //add measurement noise covariance matrix
  S(0,0) += std_radr_ * std_radr_;
  S(1,1) += std_radphi_ * std_radphi_;
  S(2,2) += std_radrd_ * std_radrd_;


/*******************************************************************************
 * UKF Radar Update
 ******************************************************************************/

  MatrixXd Tc = X_abs_w_ * Z_abs.transpose();

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  z = meas_package.raw_measurements_;
  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = NormalizeAngle(z_diff(1));

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  // prevent P from diverging
  if (NIS_radar_ > 100.0) {
    is_initialized_ = false;
  }
  else {
    //update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S * K.transpose();
  }
}

/**
 * Normalizes the angle, yaw or phi. If angle is too big, subtracts the
 * part.
 * @param double angle
 * @return The normalized angle
 */
double UKF::NormalizeAngle(double angle) {
  //for very high yaw
  int big_yaw_pi = 100.0 * M_PI;
  double pi2 = 2.0 * M_PI;
  if (fabs(angle) > big_yaw_pi) {
    angle -= floor(angle / pi2) * (pi2);
  }

  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;

  return angle;
}