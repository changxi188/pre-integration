#include "pre_integration.h"

PreIntegration::PreIntegration(const Vector3d& gyro_bias, const Vector3d& acc_bias, const ImuCalib& imu_calib)
{
    gyro_bias_         = gyro_bias;
    acc_bias_          = acc_bias;
    ga_noise_cov_      = imu_calib.ga_noise_cov;
    ga_walk_noise_cov_ = imu_calib.ga_walk_noise_cov;

    delta_P_.setZero();
    delta_V_.setZero();
    delta_R_.setIdentity();
    C_.setZero();
    info_.setZero();
}
