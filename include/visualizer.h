#pragma once

#include <mutex>
#include <thread>

#include <pangolin/pangolin.h>

#include "common.h"

class Visualizer
{
public:
    Visualizer()
    {
        visualize_thread_ = std::thread(&Visualizer::run, this);
    }

    void SetGroundTruth(const std::vector<IMU>& imus);

    void SetNoNoisedMediaIntergration(const std::vector<IMU>& imus);

    void SetNoisedMediaIntergration(const std::vector<IMU>& imus);

    virtual ~Visualizer()
    {
        visualize_thread_.join();
    }

private:
    void run();

    void drawAxis();

    void drawCamera(const IMU& imu_data);

    void drawGTOdometry();

    void drawNoNoisedMediaIntegration();

    void drawNoisedMediaIntegration();

private:
    std::thread visualize_thread_;

    std::mutex imu_data_mutex_;
    std::vector<IMU> ground_truth_imus_;
    std::vector<IMU> no_noised_media_integration_;
    std::vector<IMU> noised_media_integration_;
};

