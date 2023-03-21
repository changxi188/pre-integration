#pragma once

#include <vector>
#include "common.h"

class SimulateDataGen
{
public:
    SimulateDataGen();

    std::vector<IMU> GenerateGroundTruth();

    std::vector<IMU> AddNoise(const std::vector<IMU>& gt_imus);

private:
    // euler2Rotation:   body frame to interitail frame
    Matrix3d euler2Rotation(Vector3d eulerAngles);
    Matrix3d eulerRates2bodyRates(Vector3d eulerAngles);

    IMU MotionModel(double t);

    // time
    int    imu_frequency_ = 200;  // hz
    int    cam_frequency_ = 30;   // hz
    double imu_timestep_  = 1. / imu_frequency_;
    double cam_timestep_  = 1. / cam_frequency_;
    double t_start_       = 0.;
    double t_end_         = 20;  //  20 s

    // noise
    double gyro_bias_sigma_ = 1.0e-5;
    double acc_bias_sigma_  = 0.0001;

    double gyro_noise_sigma_ = 0.015;  // rad/s * 1/sqrt(hz)
    double acc_noise_sigma_  = 0.019;  //　m/(s^2) * 1/sqrt(hz)
};

