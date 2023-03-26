#pragma once
#include <ostream>
#include <string>
#include "common.h"

class PreIntegration
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    PreIntegration(const Vector3d& gyro_bias, const Vector3d& acc_bias, const ImuCalib& imu_calib);

    // used to debug
    friend std::ostream& operator<<(std::ostream& os, const PreIntegration& pre_integration)
    {
        os << "pre integration info : "
           << "\ngyro_bias : " << pre_integration.gyro_bias_.transpose()
           << "\nacc_bias : " << pre_integration.acc_bias_.transpose()
           << "\nga_noise_cov : " << pre_integration.ga_noise_cov_.diagonal().transpose()
           << "\nga_walk_noise_cov : " << pre_integration.ga_walk_noise_cov_.diagonal().transpose()
           << "\ndelta P : " << pre_integration.delta_P_.transpose()
           << "\ndelta V : " << pre_integration.delta_V_.transpose() << "\ndelta R : \n"
           << pre_integration.delta_R_ << "\nintegration covariance : \n"
           << pre_integration.C_ << "\nintegration infomation matrix : \n"
           << pre_integration.info_ << std::endl;

        return os;
    }

private:

    Vector3d gyro_bias_;
    Vector3d acc_bias_;
    // imu noise
    Eigen::DiagonalMatrix<double, 6> ga_noise_cov_;
    // imu bias noise
    Eigen::DiagonalMatrix<double, 6> ga_walk_noise_cov_;

    // pre-integration result
    Vector3d delta_P_;
    Vector3d delta_V_;
    Matrix3d delta_R_;

    // pre-integration covariance
    Eigen::Matrix<double, 9, 9> C_;

    // pre-integration information matrix
    Eigen::Matrix<double, 9, 9> info_;
};
