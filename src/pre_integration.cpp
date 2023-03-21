#include "pre_integration.h"

PreIntegration::PreIntegration(const Vector3d& gyro_bias, const Vector3d& acc_bias, const ImuCalib& imu_calib)
  : gyro_bias_(gyro_bias), acc_bias_(acc_bias)
{
}
