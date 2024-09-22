// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_MATHUTILS_H__
#define __RDL_MATHUTILS_H__

/**
 * @file rdl_mathutils.hpp
 */

#include <assert.h>
#include <cmath>

#include "rdl_dynamics/rdl_eigenmath.hpp"

namespace RobotDynamics
{
struct Model;

namespace Math
{
/**
 * @enum LinearSolver
 * @brief Available solver methods for the linear systems.
 */
enum LinearSolver
{
    LinearSolverUnknown = 0,
    LinearSolverPartialPivLU,
    LinearSolverColPivHouseholderQR,
    LinearSolverHouseholderQR,
    LinearSolverLLT,
    LinearSolverLast,
};

extern Vector3d Vector3dZero;
extern Matrix3d Matrix3dIdentity;
extern Matrix3d Matrix3dZero;
extern Quaternion QuaternionIdentity;

extern SpatialVector SpatialVectorZero;
extern SpatialMatrix SpatialMatrixIdentity;
extern SpatialMatrix SpatialMatrixZero;

inline VectorNd vectorFromPtr(unsigned int n, double* ptr)
{
    // TODO: use memory mapping operators for Eigen
    VectorNd result(n);

    for (unsigned int i = 0; i < n; i++)
    {
        result[i] = ptr[i];
    }

    return result;
}

inline MatrixNd matrixFromPtr(unsigned int rows, unsigned int cols, double* ptr, bool row_major = true)
{
    MatrixNd result(rows, cols);

    if (row_major)
    {
        for (unsigned int i = 0; i < rows; i++)
        {
            for (unsigned int j = 0; j < cols; j++)
            {
                result(i, j) = ptr[i * cols + j];
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < rows; i++)
        {
            for (unsigned int j = 0; j < cols; j++)
            {
                result(i, j) = ptr[i + j * rows];
            }
        }
    }

    return result;
}

/// \brief Solves a linear system using gaussian elimination with pivoting
bool linSolveGaussElimPivot(MatrixNd A, VectorNd b, VectorNd& x);

/** \brief Translates the inertia matrix to a new center. */
Matrix3d parallel_axis(const Matrix3d& inertia, double mass, const Vector3d& com);

/**
 * @brief convert zyx euler angles and angle rates to angular velocity
 * @param xyz_angles (yaw, pitch, roll) euler angles
 * @param xyz_angle_rates (yaw_dot, pitch_dot, roll_dot)
 * @return angular velocities (wx, wy, wz)
 */
inline Vector3d angular_velocity_from_zyx_angle_rates(const Vector3d& zyx_angles, const Vector3d& zyx_angle_rates)
{
    double sy = sin(zyx_angles[1]);
    double cy = cos(zyx_angles[1]);
    double sx = sin(zyx_angles[2]);
    double cx = cos(zyx_angles[2]);

    return Vector3d(zyx_angle_rates[2] - sy * zyx_angle_rates[0], cx * zyx_angle_rates[1] + sx * cy * zyx_angle_rates[0],
                    -sx * zyx_angle_rates[1] + cx * cy * zyx_angle_rates[0]);
}

/**
 * @brief convert xyz euler angles and angle rates to angular velocity
 * @param xyz_angles (roll, pitch, yaw) euler angles
 * @param xyz_angle_rates (roll_dot, pitch_dot, yaw_dot)
 * @return angular velocities (wx, wy, wz)
 */
inline Vector3d angular_velocity_from_xyz_angle_rates(const Vector3d& xyz_angles, const Vector3d& xyz_angle_rates)
{
    double s1 = sin(xyz_angles[1]);
    double c1 = cos(xyz_angles[1]);
    double s2 = sin(xyz_angles[2]);
    double c2 = cos(xyz_angles[2]);

    return Vector3d(c2 * c1 * xyz_angle_rates[0] + s2 * xyz_angle_rates[1], -s2 * c1 * xyz_angle_rates[0] + c2 * xyz_angle_rates[1],
                    s1 * xyz_angle_rates[0] + xyz_angle_rates[2]);
}

/**
 * @brief compute the 3x3 jacobian matrix that when multiplied by the angular velocity, ordered (wx, wy, wz) returns the angular velocity vector, note that
 * the ordering of the returned angle rates is (yaw_dot, pitch_Dot, roll_dot)
 * @param zyx_angles yaw/pitch/roll euler angles, ordered (yaw, pitch, roll)
 * @param singularity threshold, if the cosine of pitch is less than this, then it is considered at singularity and will return a matrix of zeros and
 * set errno to EDOM, signifying a domain error
 * @return jacobian matrix, S, that when multiplied by angular velocity, v as in S*v returns a vector of angle rates
 */
inline Matrix3d angular_velocity_to_zyx_angle_rates_jacobian(const Vector3d& zyx_angles, double singularity_threshold = 1.e-10)
{
    double c1 = cos(zyx_angles[1]);
    double t1 = tan(zyx_angles[1]);
    double s2 = sin(zyx_angles[2]);
    double c2 = cos(zyx_angles[2]);

    if (c1 < singularity_threshold)
    {
        errno = EDOM;
        std::cerr << "Domain error - unable to convert angular velocities to ZYX euler rates with pitch = pi/2" << std::endl;
        return Matrix3dZero;
    }

    return Matrix3d(0., s2 / c1, c2 / c1, 0., c2, -s2, 1., s2 * t1, c2 * t1);
}

/**
 * @brief compute the 3x3 jacobian matrix that when multiplied by the angular velocity, ordered (wx, wy, wz) returns the angular velocity vector, note that
 * the ordering of the returned angle rates is (roll_dot, pitch_dot, yaw_dot)
 * @param xyz_angles roll/pitch/yaw euler angles, ordered (roll, pitch, yaw)
 * @param singularity threshold, if the cosine of pitch is less than this, then it is considered at singularity and will return a matrix of zeros and
 * set errno to EDOM, signifying a domain error
 * @return jacobian matrix, S, that when multiplied by angular velocity, v as in S*v returns a vector of angle rates
 */
inline Matrix3d angular_velocity_to_xyz_angle_rates_jacobian(const Vector3d& xyz_angles, double singularity_threshold = 1.e-10)
{
    double c1 = cos(xyz_angles[1]);
    double t1 = tan(xyz_angles[1]);
    double s2 = sin(xyz_angles[2]);
    double c2 = cos(xyz_angles[2]);

    if (c1 < singularity_threshold)
    {
        errno = EDOM;
        std::cerr << "Domain error - unable to convert angular velocities to XYZ euler rates with pitch = pi/2" << std::endl;
        return Matrix3dZero;
    }

    return Matrix3d(c2 / c1, -s2 / c1, 0., s2, c2, 0., -c2 * t1, s2 * t1, 1.);
}

/**
 * @brief convert angular velocity to zyx angle rates
 * @param zyx_angles yaw/pitch/roll euler angles, ordered (yaw, pitch, roll)
 * @param angular_velocity angular velocity (x, y, z)
 * @param singularity threshold, if the cosine of pitch is less than this, then it is considered at singularity and will return a matrix of zeros and
 * set errno to EDOM, signifying a domain error
 * @return given zyx angles and angular velocity, a vector of angle rates orderd (yaw_dot, pitch_dot, roll_dot)
 */
inline Vector3d angular_velocity_to_zyx_angle_rates(const Vector3d& zyx_angles, const Vector3d& angular_velocity, double singularity_threshold = 1.e-10)
{
    double c1 = cos(zyx_angles[1]);
    double t1 = tan(zyx_angles[1]);
    double s2 = sin(zyx_angles[2]);
    double c2 = cos(zyx_angles[2]);

    if (c1 < singularity_threshold)
    {
        errno = EDOM;
        std::cerr << "unable to compute angle rates from angular velocity due to singularity" << std::endl;
        return Vector3dZero;
    }

    return Vector3d((s2 / c1) * angular_velocity[1] + (c2 / c1) * angular_velocity[2], c2 * angular_velocity[1] - s2 * angular_velocity[2],
                    angular_velocity[0] + s2 * t1 * angular_velocity[1] + c2 * t1 * angular_velocity[2]);
}

/**
 * @brief convert angular velocity to xyz angle rates
 * @param xyz_angles yaw/pitch/roll euler angles, ordered (roll, pitch, yaw)
 * @param angular_velocity angular velocity (x, y, z)
 * @param singularity threshold, if the cosine of pitch is less than this, then it is considered at singularity and will return a vector of zeros and
 * set errno to EDOM, signifying a domain error
 * @return given xyz angles and angular velocity, a vector of angle rates orderd (roll_dot, pitch_dot, yaw_dot)
 */
inline Vector3d angular_velocity_to_xyz_angle_rates(const Vector3d& xyz_angles, const Vector3d& angular_velocity, double singularity_threshold = 1.e-10)
{
    double c1 = cos(xyz_angles[1]);
    double t1 = tan(xyz_angles[1]);
    double s2 = sin(xyz_angles[2]);
    double c2 = cos(xyz_angles[2]);

    if (c1 < singularity_threshold)
    {
        errno = EDOM;
        std::cerr << "unable to compute jacobian matrix for XYZ angles" << std::endl;
        return Vector3dZero;
    }

    return Vector3d((c2 / c1) * angular_velocity[0] - (s2 / c1) * angular_velocity[1], s2 * angular_velocity[0] + c2 * angular_velocity[1],
                    -c2 * t1 * angular_velocity[0] + s2 * t1 * angular_velocity[1] + angular_velocity[2]);
}

inline Vector3d global_angular_velocity_from_rates(const Vector3d& zyx_angles, const Vector3d& zyx_rates)
{
    double s0, c0, s1, c1;
    s0 = sin(zyx_angles[0]);
    c0 = cos(zyx_angles[0]);
    s1 = sin(zyx_angles[1]);
    c1 = cos(zyx_angles[1]);

    Matrix3d RzT(c0, s0, 0., -s0, c0, 0., 0., 0., 1.);
    RzT.transposeInPlace();
    Matrix3d RyT(c1, 0., -s1, 0., 1., 0., s1, 0., c1);
    RyT.transposeInPlace();

    return Vector3d(Vector3d(0., 0., zyx_rates[0]) + RzT * Vector3d(0., zyx_rates[1], 0.) + RzT * RyT * Vector3d(zyx_rates[2], 0., 0.));
}

/**
 * @brief calculate angular acceleration from zyx angles, angle rates, and angle accelerations
 * @param zyx_angles (yaw, pitch, roll) euler angles
 * @param zyx_angle_rates (yaw_dot, pitch_dot, roll_dot) angle rates
 * @param zyx_angle_accelerations (yaw_ddot, pitch_ddot, roll_ddot) angle accelerations
 *
 * @return angular accelerations (wx_dot, wy_dot, wz_dot)
 */
inline Vector3d angular_acceleration_from_zyx_angle_rates(const Vector3d& zyx_angles, const Vector3d& zyx_angle_rates, const Vector3d& zyx_angle_rates_dot)
{
    double sy = sin(zyx_angles[1]);
    double cy = cos(zyx_angles[1]);
    double sx = sin(zyx_angles[2]);
    double cx = cos(zyx_angles[2]);
    double xdot = zyx_angle_rates[2];
    double ydot = zyx_angle_rates[1];
    double zdot = zyx_angle_rates[0];
    double xddot = zyx_angle_rates_dot[2];
    double yddot = zyx_angle_rates_dot[1];
    double zddot = zyx_angle_rates_dot[0];

    return Vector3d(xddot - (cy * ydot * zdot + sy * zddot), -sx * xdot * ydot + cx * yddot + cx * xdot * cy * zdot + sx * (-sy * ydot * zdot + cy * zddot),
                    -cx * xdot * ydot - sx * yddot - sx * xdot * cy * zdot + cx * (-sy * ydot * zdot + cy * zddot));
}

/**
 * @brief calculate angular acceleration from zyx angles, angle rates, and angle accelerations
 * @param xyz_angles (roll, pitch, yaw) euler angles
 * @param xyz_angle_rates (roll_dot, pitch_dot, yaw_dot) angle rates
 * @param xyz_angle_accelerations (roll_ddot, pitch_ddot, yaw_ddot) angle accelerations
 *
 * @return angular accelerations (wx_dot, wy_dot, wz_dot)
 */
inline Vector3d angular_acceleration_from_xyz_angle_rates(const Vector3d& xyz_angles, const Vector3d& xyz_angle_rates, const Vector3d& xyz_angle_rates_dot)
{
    double s1 = sin(xyz_angles[1]);
    double c1 = cos(xyz_angles[1]);
    double s2 = sin(xyz_angles[2]);
    double c2 = cos(xyz_angles[2]);

    double xdot = xyz_angle_rates[0];
    double ydot = xyz_angle_rates[1];
    double zdot = xyz_angle_rates[2];

    double xddot = xyz_angle_rates_dot[0];
    double yddot = xyz_angle_rates_dot[1];
    double zddot = xyz_angle_rates_dot[2];
    return Vector3d(c2 * c1 * xddot + s2 * yddot + (-s2 * c1 * zdot - c2 * s1 * ydot) * xdot + c2 * zdot * ydot,
                    -s2 * c1 * xddot + c2 * yddot + (-c2 * c1 * zdot + s2 * s1 * ydot) * xdot - s2 * zdot * ydot, s1 * xddot + zddot + c1 * ydot * xdot);
}

void SparseFactorizeLTL(Model& model, Math::MatrixNd& H);

void SparseMultiplyHx(Model& model, Math::MatrixNd& L);

void SparseMultiplyLx(Model& model, Math::MatrixNd& L);

void SparseMultiplyLTx(Model& model, Math::MatrixNd& L);

void SparseSolveLx(Model& model, Math::MatrixNd& L, Math::VectorNd& x);

void SparseSolveLTx(Model& model, Math::MatrixNd& L, Math::VectorNd& x);
}  // namespace Math
}  // namespace RobotDynamics

/* __RDL_MATHUTILS_H__ */
#endif  // ifndef __RDL_MATHUTILS_H__
