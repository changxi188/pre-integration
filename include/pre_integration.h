#pragma once
#include "common.h"

class PreIntegration
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    PreIntegration(const Vector3d& gyro_bias, const Vector3d& acc_bias, const ImuCalib& imu_calib);

private:
    Vector3d delta_P_;
    Vector3d delta_V_;
    Matrix3d delta_R_;

    Vector3d gyro_bias_;
    Vector3d acc_bias_;

    // imu noise
    Eigen::DiagonalMatrix<double, 6> noise_ga_;

    // imu bias noise
    Eigen::DiagonalMatrix<double, 6> noise_ga_walk_;

    // pre-integration covariance
    Eigen::Matrix<double, 9, 9> C_;

    // information matrix
    Eigen::Matrix<double, 9, 9> info_;
};
