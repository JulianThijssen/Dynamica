#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

using FMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using FMatrix3x3 = Eigen::Matrix<float, 3, 3>;
using FVector3 = Eigen::Matrix<float, 3, 1>;
using FVectorX = Eigen::Matrix<float, Eigen::Dynamic, 1>;
#define vector3(a) block<3,1>(3*(a), 0)
#define vectorX(s, a) block<s,1>(3*(a), 0)
