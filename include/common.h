#pragma once

#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

struct IMU
{
    double timestamp;
    // x acc. y acc, z acc
    Vector3d acceleration;
    Vector3d angular_velocity;

    Matrix3d Rwb;
    Vector3d twb;
    Vector3d imu_velocity;
    Vector3d acc_bias;
    Vector3d gyro_bias;
};
