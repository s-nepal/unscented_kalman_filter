#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    // Laser measurement noise standard deviation position1 in m
    const double std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    const double std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    const double std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    const double std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    const double std_radrd_ = 0.3;


    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    // if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;


    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // predicted sigma points matrix
    MatrixXd Xsig_pred_;


    // time when the state is true, in us
    long long time_us_;

    // previous timestamp
    long long previous_timestamp_;


    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    // Weights of sigma points
    VectorXd weights_;

    // State dimension
    const int n_x_ = 5;

    // Augmented state dimension
    const int n_aug_ = 7;

    // Sigma point spreading parameter
    const double lambda_ = 3 - n_aug_;

    // the current NIS for radar
    double NIS_radar_;

    // the current NIS for laser
    double NIS_laser_;

    // constructor
    UKF();

    virtual ~UKF();

    // methods

    void processMeasurement(MeasurementPackage meas_package);

    void initializeStateVector(MeasurementPackage meas_package);

    MatrixXd calculateSigmaPoints();

    MatrixXd passSigmaPointsToProcessFunc(MeasurementPackage meas_package, MatrixXd Xsig_aug);

    void Predict(MatrixXd Xsig_pred);

    void updateRadar(MeasurementPackage meas_package, MatrixXd Xsig_pred);

    void updateLidar(MeasurementPackage meas_package, MatrixXd Xsig_pred);

    void normalizeAngle(double *angle);

};

#endif
