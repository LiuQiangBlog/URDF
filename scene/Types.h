//
// Created by LiuQiang on 2025/1/5.
//

#ifndef SCENE_TYPES_H
#define SCENE_TYPES_H
#include "Eigen/Dense"

inline constexpr double Pi = 3.1415926535897932384626433832795;
inline constexpr double Epsilon = 1e-6;
inline constexpr double Deg2Rad = Pi / 180.0;
inline constexpr double Rad2Deg = 180.0 / Pi;
inline constexpr double MM2M = 0.001;
inline constexpr double M2MM = 1000.0;
using Index = int;
using T = double;
using FLOAT = float;

enum ErrorCodeType : int
{
    ERR_SUCCESS = 0,
    ERR_NO_SOLUTION,
    ERR_NOT_READY,
    ERR_NULLPTR,
    ERR_NOT_EXIST,
    ERR_INVALID_ARG
};

using Vec2d = Eigen::Vector<T, 2>;
using Vec3d = Eigen::Vector<T, 3>;
using Vec4d = Eigen::Vector<T, 4>;
using Vec5d = Eigen::Vector<T, 5>;
using Vec6d = Eigen::Vector<T, 6>;

using Vec3f = Eigen::Vector<FLOAT, 3>;

using Mat2d = Eigen::Matrix<T, 2, 2>;
using Mat3d = Eigen::Matrix<T, 3, 3>;
using Mat4d = Eigen::Matrix<T, 4, 4>;
using Mat6d = Eigen::Matrix<T, 6, 6>;

using Mat3x4d = Eigen::Matrix<T, 3, 4>;
using Mat3x6d = Eigen::Matrix<T, 3, 6>;
using Mat6x3d = Eigen::Matrix<T, 6, 3>;
using Mat6x6d = Eigen::Matrix<T, 6, 6>;


#endif // SCENE_TYPES_H
