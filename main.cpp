#include <chrono>
#include <thread>
#include <vector>

#include <glog/logging.h>

#include "common.h"
#include "pre_integration.h"
#include "simulate_data_gen.h"
#include "visualizer.h"

int main(int argc, char** argv)
{
    std::string log_file("./a.log");
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::INFO, log_file.c_str());

    FLAGS_alsologtostderr  = 1;
    FLAGS_minloglevel      = 0;
    FLAGS_colorlogtostderr = true;

    SimulateDataGen simulate_data_gen;

    // step1: generate simulate data
    std::vector<IMU> ground_truth_imus = simulate_data_gen.GenerateGroundTruth();
    std::vector<IMU> noised_imus       = simulate_data_gen.AddNoise(ground_truth_imus);
    LOG(INFO) << "ground_truth_imus size : " << ground_truth_imus.size();
    LOG(INFO) << "noised_imus size : " << noised_imus.size();

    // step2: test simulate data with median integration
    std::vector<IMU> no_noised_media_integration = simulate_data_gen.TestIMU(ground_truth_imus);
    std::vector<IMU> noised_media_integration    = simulate_data_gen.TestIMU(noised_imus);

    // step3: pre-integration
    ImuCalib imu_calib = simulate_data_gen.GetImuCalib();
    LOG(INFO) << "imu_ calib : " << imu_calib;

    // step3.1: construct pre-integration
    Vector3d       gyro_bias = Vector3d::Zero();
    Vector3d       acc_bias  = Vector3d::Zero();
    PreIntegration pre_integration(gyro_bias, acc_bias, imu_calib);
    LOG(INFO) << pre_integration;

    // step3.2: integrate measurements and update pre-integration covariance

    for (size_t i = 0; i < ground_truth_imus.size() - 1; ++i)
    {
        IMU cur_imu = ground_truth_imus.at(i);
        IMU next_imu = ground_truth_imus.at(i + 1);

        double   dt               = next_imu.timestamp - cur_imu.timestamp;
        Vector3d angular_velocity = cur_imu.angular_velocity;
        Vector3d acceleration     = cur_imu.acceleration;

        pre_integration.IntegrateNewMeasurement(angular_velocity, acceleration, dt);
    }

    /*
    Visualizer visualizer;
    for (size_t i = 0; i < ground_truth_imus.size(); ++i)
    {
        std::vector<IMU> ground_truth_imuss(ground_truth_imus.begin(), ground_truth_imus.begin() + i);
        visualizer.SetGroundTruth(ground_truth_imuss);

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    for (size_t i = 0; i < no_noised_media_integration.size(); ++i)
    {
        std::vector<IMU> no_noised_media_integrations(no_noised_media_integration.begin(),
                                                      no_noised_media_integration.begin() + i);
        visualizer.SetNoNoisedMediaIntergration(no_noised_media_integrations);

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    for (size_t i = 0; i < noised_media_integration.size(); ++i)
    {
        std::vector<IMU> noised_media_integrations(noised_media_integration.begin(),
                                                   noised_media_integration.begin() + i);
        visualizer.SetNoisedMediaIntergration(noised_media_integrations);

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    */

    return 0;
}
