#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// constructor
UKF::UKF() {

    is_initialized_ = false;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(n_x_);

    // initial covariance matrix
    P_ = MatrixXd(n_x_, n_x_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 3;

    // set weights_
    // create vector for weights_
    weights_ = VectorXd(2 * n_aug_ + 1);

    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights_
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }

}

// destructor
UKF::~UKF() {}

// process measurement
void UKF::processMeasurement(MeasurementPackage meas_package)
{
    if(!is_initialized_){

        initializeStateVector(meas_package);

    } else {

        MatrixXd Xsig_aug = calculateSigmaPoints();
        MatrixXd Xsig_pred = passSigmaPointsToProcessFunc(meas_package, Xsig_aug);
        Predict(Xsig_pred);

        if(meas_package.sensor_type_ == MeasurementPackage::RADAR){

            updateRadar(meas_package, Xsig_pred);

        } else if(meas_package.sensor_type_ == MeasurementPackage::LASER) {

            updateLidar(meas_package, Xsig_pred);

        }
    }

}

// initialize the state vector
void UKF::initializeStateVector(MeasurementPackage meas_package)
{
    is_initialized_ = true;

    float px, py;

    // initialize the state vector and the covariance matrix
    if(meas_package.sensor_type_== MeasurementPackage::LASER){
        px = meas_package.raw_measurements_(0);
        py = meas_package.raw_measurements_(1);
        x_ <<   px,
                py,
                0,
                0,
                0;
    } else if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        px = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
        py = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
        x_ <<   px,
                py,
                0,
                0,
                0;
    }

    // initialize P_ as a 5x5 identity matrix
    P_ = MatrixXd::Identity(5, 5);

    previous_timestamp_= meas_package.timestamp_;
}

// calculate the co-ordinates of the sigma points
MatrixXd UKF::calculateSigmaPoints()
{
    // Comments: calculate the co-ordinates of the sigma points

    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

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
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i < n_aug_; i++){
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

    return Xsig_aug;
}

// pass the calculated sigma points through the process function
MatrixXd UKF::passSigmaPointsToProcessFunc(MeasurementPackage meas_package, MatrixXd Xsig_aug)
{
    // Comments: pass the sigma points through the process function

    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
    double delta_t = (meas_package.timestamp_ - previous_timestamp_)/1000000.0; //time diff in sec
    previous_timestamp_ = meas_package.timestamp_;

    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++){

        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred(0,i) = px_p;
        Xsig_pred(1,i) = py_p;
        Xsig_pred(2,i) = v_p;
        Xsig_pred(3,i) = yaw_p;
        Xsig_pred(4,i) = yawd_p;
    }

    return Xsig_pred;
}

// predict x' and P'
void UKF::Predict(MatrixXd Xsig_pred)
{
    // Description: Calculate x' and P'

    // predict state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x_;

        //angle normalization
        normalizeAngle(&x_diff(3));

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

// update x' and P' with incoming radar measurement
void UKF::updateRadar(MeasurementPackage meas_package, MatrixXd Xsig_pred)
{
    // Description: Transform sigma points to the measurement space
    int n_z = 3;
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // extract values for better readibility
        double p_x = Xsig_pred(0,i);
        double p_y = Xsig_pred(1,i);
        double v  = Xsig_pred(2,i);
        double yaw = Xsig_pred(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;

    S = S + R;

    // Description: Measurement update

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        normalizeAngle(&z_diff(1));

        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x_;

        //angle normalization
        normalizeAngle(&x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

    //angle normalization
    normalizeAngle(&z_diff(1));

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}

// update x' and P' with incoming lidar measurement
void UKF::updateLidar(MeasurementPackage meas_package, MatrixXd Xsig_pred)
{
    // use the normal Kalman Filter equations as opposed to the EKF or the UKF

    MatrixXd H_ = MatrixXd(2, 5);

    H_ <<   1, 0, 0, 0, 0,
            0, 1, 0, 0, 0;

    VectorXd z = VectorXd(2);
    z <<    meas_package.raw_measurements_(0),
            meas_package.raw_measurements_(1);

    MatrixXd y_ = MatrixXd(2, 1);
    y_ = z - H_ * x_;

    MatrixXd R = MatrixXd(2, 2);
    R <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

    MatrixXd S_ = MatrixXd(2, 2);
    S_ = H_ * P_ * H_.transpose() + R;

    MatrixXd K_ = MatrixXd(4, 2);
    K_ = P_ * H_.transpose() * S_.inverse();

    x_ = x_ + K_ * y_;

    MatrixXd I_ = MatrixXd::Identity(5, 5);

    P_ = (I_ - K_ * H_) * P_;

}

void UKF::normalizeAngle(double *angle)
{
    while (*angle > M_PI)
        *angle -= 2. * M_PI;

    while (*angle < -M_PI)
        *angle += 2. * M_PI;
}

