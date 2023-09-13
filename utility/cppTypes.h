/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include "cTypes.h"
#include <eigen3/Eigen/Dense>

// Rotation Matrix

using RotMat = typename Eigen::Matrix<double, 3, 3>;

// 2x1 Vector

using Vec2 = typename Eigen::Matrix<double, 2, 1>;

// 3x1 Vector

using Vec3 = typename Eigen::Matrix<double, 3, 1>;

// 4x1 Vector

using Vec4 = typename Eigen::Matrix<double, 4, 1>;

// 4x1 Int Vector

using Vec4Int = typename Eigen::Matrix<int, 4, 1>;

// 6x1 Vector

using Vec6 = Eigen::Matrix<double, 6, 1>;

// 10x1 Vector

using Vec10 = Eigen::Matrix<double, 10, 1>;

// 12x1 Vector

using Vec12 = Eigen::Matrix<double, 12, 1>;

using Vec16 = Eigen::Matrix<double, 16, 1>;

// 19x1 Vector
using Vec19 = Eigen::Matrix<double, 19, 1>;

// 18x1 Vector
using Vec18 = Eigen::Matrix<double, 18, 1>;

// 22x1 vector

using Vec22 = Eigen::Matrix<double, 22, 1>;

// 23x1 vector

using Vec23 = Eigen::Matrix<double, 23, 1>;

// 28x1 vector

using Vec28 = Eigen::Matrix<double, 28, 1>;

// 2x2 Matrix

using Mat2 = typename Eigen::Matrix<double, 2, 2>;

// 3x3 Matrix

using Mat3 = typename Eigen::Matrix<double, 3, 3>;

// 4x1 Vector

using Quat = typename Eigen::Matrix<double, 4, 1>;

// Spatial Vector (6x1, all subspaces)

using SVec = typename Eigen::Matrix<double, 6, 1>;

// Spatial Transform (6x6)

using SXform = typename Eigen::Matrix<double, 6, 6>;

// 6x6 Matrix

using Mat6 = typename Eigen::Matrix<double, 6, 6>;

// 12x12 Matrix

using Mat12 = typename Eigen::Matrix<double, 12, 12>;

// 18x18 Matrix

using Mat18 = Eigen::Matrix<double, 18, 18>;

// 28x28 Matrix

using Mat28 = Eigen::Matrix<double, 28, 28>;

// 3x4 Matrix

using Mat34 = Eigen::Matrix<double, 3, 4>;

// 4x3 Matrix

using  Mat43 = Eigen::Matrix<double, 4,3>;

// 3x4 Matrix

using Mat23 = Eigen::Matrix<double, 2, 3>;

// 4x4 Matrix

using Mat4 = typename Eigen::Matrix<double, 4, 4>;

// 10x1 Vector

using MassProperties = typename Eigen::Matrix<double, 10, 1>;

// Dynamically sized vector

using DVec = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

// Dynamically sized matrix

using DMat = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix

using DMatInt = typename Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with spatial vector columns

using D6Mat = typename Eigen::Matrix<double, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns

using D3Mat = typename Eigen::Matrix<double, 3, Eigen::Dynamic>;


using D12Mat = typename Eigen::Matrix<double,12,Eigen::Dynamic>;

// std::vector (a list) of Eigen things

using vectorAligned = typename std::vector<double, Eigen::aligned_allocator<double>>;

enum class RobotType { CHEETAH_3, MINI_CHEETAH };

enum class ContactState{
  SWING,
  STANCE
};

#endif  // PROJECT_CPPTYPES_H
