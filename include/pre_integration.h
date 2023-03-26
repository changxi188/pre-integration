#pragma once
#include <cmath>
#include <ostream>
#include <sophus/so3.hpp>
#include <string>
#include <vector>
#include "common.h"

const double eps = 1e-4;

class PreIntegration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct ImuMeasurement
    {
        ImuMeasurement(const Vector3d& _angular_velocity, const Vector3d& _acceleration, const double _delta_t)
          : angular_velocity(_angular_velocity), acceleration(_acceleration), delta_t(_delta_t)
        {
        }
        Vector3d angular_velocity;
        Vector3d acceleration;
        double   delta_t;
    };

    // integrate of 1 gyro measurement
    class IntegrateRotation
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        IntegrateRotation(const Vector3d& angular_velocity, const double dt)
        {
            const Vector3d angular = angular_velocity * dt;

            const double x_ang = angular(0);
            const double y_ang = angular(1);
            const double z_ang = angular(2);

            const double d2 = x_ang * x_ang + y_ang * y_ang + z_ang * z_ang;
            const double d  = std::sqrt(d2);

            Vector3d v;
            v << x_ang, y_ang, z_ang;

            Matrix3d W = Sophus::SO3d::hat(v);

            if (d < eps)
            {
                dR      = Matrix3d::Identity() + W;
                right_J = Matrix3d::Identity();
            }
            else
            {
                dR = Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0 - cos(d)) / d2;
                right_J = Matrix3d::Identity() - W * (1.0 - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
            }
        }

    public:
        Matrix3d dR;

        // right jacobian
        Matrix3d right_J;
    };

    PreIntegration(const Vector3d& gyro_bias, const Vector3d& acc_bias, const ImuCalib& imu_calib);

    void IntegrateNewMeasurement(const Vector3d& angular_velocity, const Vector3d acceleration, const double delta_t);

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
    // gyroscope bias
    Vector3d gyro_bias_;

    // accelerometers bias
    Vector3d acc_bias_;

    // imu measurement noise
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

    std::vector<ImuMeasurement> imu_measurements_;
};
