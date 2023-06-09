#include <random>
#include <vector>

#include "simulate_data_gen.h"

SimulateDataGen::SimulateDataGen()
{
}

std::vector<IMU> SimulateDataGen::TestIMU(const std::vector<IMU>& imus)
{
    std::vector<IMU> media_integration_result;

    IMU first_imu = imus.front();

    double             dt  = imu_timestep_;
    // position :    from  imu measurements
    Vector3d           Pwb = first_imu.twb;
    // quaterniond:  from imu measurements
    Eigen::Quaterniond Qwb(first_imu.Rwb);
    // velocity  :   from imu measurements
    Eigen::Vector3d Vw = first_imu.velocity;
    // ENU frame
    Eigen::Vector3d gw(0, 0, -9.81);

    for (int i = 1; i < imus.size(); ++i)
    {
        IMU last_imupose = imus[i - 1];
        IMU curr_imupose = imus[i];

        IMU integral_result;

        // delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Vector3d           last_ang_vel = last_imupose.angular_velocity;
        Vector3d           cur_ang_vel  = curr_imupose.angular_velocity;
        Vector3d           dtheta_half  = (last_ang_vel + cur_ang_vel) * dt / 4.0;
        Eigen::Quaterniond dq;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        dq.normalize();

        /// 中值积分
        // aw = Rwb * ( acc_body - acc_bias ) + gw
        Vector3d last_acc_w        = Qwb * (last_imupose.acceleration) + gw;
        Qwb                        = Qwb * dq;
        Vector3d curr_acc_w        = Qwb * (curr_imupose.acceleration) + gw;
        Vector3d acc_w             = (last_acc_w + curr_acc_w) / 2.0;
        Pwb                        = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw                         = Vw + acc_w * dt;

        integral_result.Rwb = Qwb.toRotationMatrix();
        integral_result.twb = Pwb;

        media_integration_result.emplace_back(integral_result);
    }

    return media_integration_result;
}

std::vector<IMU> SimulateDataGen::GenerateGroundTruth()
{
    std::vector<IMU> imus;

    for (double t = t_start_; t < t_end_; t += imu_timestep_)
    {
        IMU gt_imu = MotionModel(t);

        imus.emplace_back(gt_imu);
    }

    return imus;
}

ImuCalib SimulateDataGen::GetImuCalib()
{
    ImuCalib imu_calib;

    double gyro_cov = gyro_noise_sigma_ * gyro_noise_sigma_ / imu_timestep_;
    double acc_cov  = acc_noise_sigma_ * acc_noise_sigma_ / imu_timestep_;
    double gyro_bias_cov = gyro_bias_sigma_ * gyro_bias_sigma_ * imu_timestep_;
    double acc_bias_cov  = acc_bias_sigma_ * acc_bias_sigma_ * imu_timestep_;

    imu_calib.T_bc = T_bc_;
    imu_calib.ga_noise_cov.diagonal() << gyro_cov, gyro_cov, gyro_cov, acc_cov, acc_cov, acc_cov;
    imu_calib.ga_walk_noise_cov.diagonal() << gyro_bias_cov, gyro_bias_cov, gyro_bias_cov, acc_bias_cov, acc_bias_cov,
        acc_bias_cov;

    return imu_calib;
}

std::vector<IMU> SimulateDataGen::AddNoise(const std::vector<IMU>& gt_imus)
{
    std::vector<IMU> noised_imus;
    Vector3d acc_bias  = Vector3d::Zero();
    Vector3d gyro_bias = Vector3d::Zero();

    for (const auto& gt_imu : gt_imus)
    {
        IMU                              noised_imu = gt_imu;
        std::random_device               rd;
        std::default_random_engine       generator(rd());
        std::normal_distribution<double> noise(0.0, 1.0);

        Vector3d noise_gyro(noise(generator), noise(generator), noise(generator));
        Matrix3d gyro_sqrt_cov = gyro_noise_sigma_ * Matrix3d::Identity();
        noised_imu.angular_velocity =
            gt_imu.angular_velocity + gyro_sqrt_cov * noise_gyro / sqrt(imu_timestep_) + gyro_bias;

        Vector3d noise_acc(noise(generator), noise(generator), noise(generator));
        Matrix3d acc_sqrt_cov = acc_noise_sigma_ * Matrix3d::Identity();
        noised_imu.acceleration = gt_imu.acceleration + acc_sqrt_cov * noise_acc / sqrt(imu_timestep_) + acc_bias;

        // gyro_bias update
        Eigen::Vector3d noise_gyro_bias(noise(generator), noise(generator), noise(generator));
        Matrix3d        gyro_bias_sqrt_cov = gyro_bias_sigma_ * Matrix3d::Identity();
        gyro_bias += gyro_bias_sqrt_cov * sqrt(imu_timestep_) * noise_gyro_bias;
        noised_imu.gyro_bias = gyro_bias;

        // acc_bias update
        Eigen::Vector3d noise_acc_bias(noise(generator), noise(generator), noise(generator));
        Matrix3d        acc_bias_sqrt_cov = acc_bias_sigma_ * Matrix3d::Identity();
        acc_bias += acc_bias_sqrt_cov * sqrt(imu_timestep_) * noise_acc_bias;
        noised_imu.acc_bias = acc_bias;

        noised_imus.emplace_back(noised_imu);
    }

    return noised_imus;
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
    data.Rwb              = Rwb;
    data.twb              = position;
    data.velocity         = dp;
    data.timestamp        = t;
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
