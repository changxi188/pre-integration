#include "common.h"

Matrix3d NormalizeRotation(const Matrix3d& R)
{
    Eigen::JacobiSVD<Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);

    return svd.matrixU() * svd.matrixV().transpose();
}
