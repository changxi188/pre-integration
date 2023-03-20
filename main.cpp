#include <chrono>
#include <thread>
#include "common.h"
#include "simulate_data_gen.h"
#include "visualizer.h"

using namespace std;

int main()
{
    SimulateDataGen simulate_data_gen;
    std::vector<IMU> ground_truth_imus = simulate_data_gen.GenerateGroundTruth();
    std::cout << "ground_truth_imus size : " << ground_truth_imus.size() << std::endl;

    Visualizer visualizer;
    for (size_t i = 0; i < ground_truth_imus.size(); ++i)
    {
        std::vector<IMU> ground_truth_imuss(ground_truth_imus.begin(), ground_truth_imus.begin() + i);
        visualizer.SetGroundTruth(ground_truth_imuss);

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    return 0;
}
