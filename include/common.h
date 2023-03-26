#pragma once

#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <sophus/se3.hpp>

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

struct IMU
{
    double timestamp;

    // x_ang_vel, y_ang_vel, z_ang_vel
    Vector3d angular_velocity;

    // x_acc. y_acc, z_acc
    Vector3d acceleration;

    Matrix3d Rwb;
    Vector3d twb;
    Vector3d velocity;
    Vector3d acc_bias;
    Vector3d gyro_bias;
};

struct ImuCalib
{
    // gyro and acc measurement noise covariance
    Eigen::DiagonalMatrix<double, 6> ga_noise_cov;

    // gyro and acc bias random walk noise covariance
    Eigen::DiagonalMatrix<double, 6> ga_walk_noise_cov;

    // extrinsic, camera to body transformation
    Sophus::SE3d T_bc;

    friend std::ostream& operator<<(std::ostream& os, const ImuCalib& imu_calib)
    {
        os << "\ngyro and acc measurement covariance : " << imu_calib.ga_noise_cov.diagonal().transpose()
           << "\ngyro and acc bias covariance : " << imu_calib.ga_walk_noise_cov.diagonal().transpose() << std::endl;

        return os;
    }
};

Matrix3d NormalizeRotation(const Matrix3d& R);
