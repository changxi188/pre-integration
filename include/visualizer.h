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

    virtual ~Visualizer()
    {
        visualize_thread_.join();
    }

private:
    void run();

    void drawAxis();

    void drawCamera(const IMU& imu_data);

    void drawGTOdometry();

private:
    std::thread visualize_thread_;

    std::mutex imu_data_mutex_;
    std::vector<IMU> ground_truth_imus_;
};

