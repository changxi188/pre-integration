#include "simulate_data_gen.h"

SimulateDataGen::SimulateDataGen()
{
}

std::vector<IMU> SimulateDataGen::GenerateGroundTruth()
{
    std::vector<IMU> imus;

    for (double t = t_start_; t < t_end_; t += imu_timestep_)
    {
        IMU imu = MotionModel(t);

        imus.emplace_back(imu);
    }

    return imus;
}

IMU SimulateDataGen::MotionModel(double t)
{
    IMU data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    // z轴做sin运动
    float z = 1;
    // z轴的正弦频率是x，y的k1倍
    float K1 = 10;
    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周
    float K = M_PI / 10;

    // translation
    // twb:  body frame in world frame
    Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * sin(K * t) + 5, z * sin(K1 * K * t) + 5);
    // position导数　in world frame
    Vector3d        dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t));
    double          K2 = K * K;
    // position二阶导数
    Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t));

    // Rotation
    double k_roll  = 0.1;
    double k_pitch = 0.2;

    // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), K * t);

    // euler angles 的导数
    Vector3d eulerAnglesRates(-k_roll * sin(t), k_pitch * cos(t), K);

    // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
    // Eigen::Vector3d eulerAngles(0.0,0.0, K*t );
    // euler angles 的导数
    // Eigen::Vector3d eulerAnglesRates(0.,0. , K);

    // body frame to world frame
    Matrix3d Rwb = euler2Rotation(eulerAngles);

    // euler rates trans to body gyro
    Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;

    // gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Vector3d gn(0, 0, -9.81);

    // Rbw * Rwn * gn = gs
    Vector3d imu_acc = Rwb.transpose() * (ddp - gn);

    data.angular_velocity = imu_gyro;
    data.acceleration     = imu_acc;
    data.Rwb          = Rwb;
    data.twb          = position;
    data.imu_velocity = dp;
    data.timestamp    = t;
    return data;
}

// euler2Rotation:   body frame to interitail frame
Matrix3d SimulateDataGen::euler2Rotation(Vector3d eulerAngles)
{
    double roll  = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw   = eulerAngles(2);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    Matrix3d RIb;
    RIb << cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * cr * sp, sy * cp, cy * cr + sy * sr * sp,
        sp * sy * cr - cy * sr, -sp, cp * sr, cp * cr;
    return RIb;
}

Matrix3d SimulateDataGen::eulerRates2bodyRates(Vector3d eulerAngles)
{
    double roll  = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);

    Matrix3d R;
    R << 1, 0, -sp, 0, cr, sr * cp, 0, -sr, cr * cp;

    return R;
}