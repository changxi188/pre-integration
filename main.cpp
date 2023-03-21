#include <chrono>
#include <thread>
#include <vector>
#include "common.h"
#include "simulate_data_gen.h"
#include "visualizer.h"

#include <glog/logging.h>

using namespace std;

int main(int argc, char** argv)
{
    std::string log_file("./a.log");
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::INFO, log_file.c_str());

    FLAGS_alsologtostderr  = 1;
    FLAGS_minloglevel      = 0;
    FLAGS_colorlogtostderr = true;

    SimulateDataGen simulate_data_gen;

    std::vector<IMU> ground_truth_imus = simulate_data_gen.GenerateGroundTruth();
    LOG(INFO) << "ground_truth_imus size : " << ground_truth_imus.size();

    std::vector<IMU> noised_imus = simulate_data_gen.AddNoise(ground_truth_imus);
    LOG(INFO) << "noised_imus size : " << ground_truth_imus.size();

    Visualizer visualizer;
    for (size_t i = 0; i < ground_truth_imus.size(); ++i)
    {
        std::vector<IMU> ground_truth_imuss(ground_truth_imus.begin(), ground_truth_imus.begin() + i);
        visualizer.SetGroundTruth(ground_truth_imuss);

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    return 0;
}
