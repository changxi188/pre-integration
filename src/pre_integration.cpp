#include "pre_integration.h"
#include <eigen3/Eigen/src/Householder/BlockHouseholder.h>
#include <sophus/so3.hpp>

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

void PreIntegration::IntegrateNewMeasurement(const Vector3d& angular_velocity, const Vector3d acceleration,
                                             const double delta_t)
{

    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.
    imu_measurements_.emplace_back(ImuMeasurement(angular_velocity, acceleration, delta_t));

    // Matrixs to compute preintegration covariance
    Eigen::Matrix<double, 9, 9> A;
    Eigen::Matrix<double, 9, 6> B;
    A.setIdentity();
    B.setZero();

    Vector3d un_biased_ang_vel = angular_velocity - gyro_bias_;
    Vector3d un_biased_acc     = acceleration - acc_bias_;

    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    delta_P_ = delta_P_ + delta_V_ * delta_t + 0.5 * delta_R_ * un_biased_acc * delta_t * delta_t;
    delta_V_ = delta_V_ + delta_R_ * un_biased_acc * delta_t;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    Matrix3d un_biased_acc_hat = Sophus::SO3d::hat(un_biased_acc);
    A.block<3, 3>(3, 0)        = -delta_R_ * delta_t * un_biased_acc_hat;
    A.block<3, 3>(6, 0)        = -0.5 * delta_R_ * delta_t * delta_t * un_biased_acc_hat;
    A.block<3, 3>(6, 3)        = 0.5 * delta_R_ * delta_t * delta_t;
    B.block<3, 3>(3, 3)        = delta_R_ * delta_t;
    B.block<3, 3>(6, 3)        = 0.5 * delta_R_ * delta_t * delta_t;

    // Update delta rotation
    IntegrateRotation delta_R_i(un_biased_ang_vel, delta_t);
    delta_R_ = NormalizeRotation(delta_R_ * delta_R_i.dR);

    // Compute rotation parts of matrices A and B
    A.block<3, 3>(0, 0) = delta_R_i.dR.transpose();
    B.block<3, 3>(0, 0) = delta_R_i.right_J * delta_t;

    // Update covariance
    C_ = A * C_ * A.transpose() + B * ga_noise_cov_ * B.transpose();

    LOG(INFO) << "delta_t : " << delta_t;
    LOG(INFO) << "angular_velocity : " << angular_velocity.transpose();
    LOG(INFO) << "acceleration : " << acceleration.transpose();
    LOG(INFO) << "pre-integration : " << *this;
}
