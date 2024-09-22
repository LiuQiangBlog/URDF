// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include "rdl_dynamics/MotionVector.hpp"

namespace RobotDynamics
{
namespace Math
{
MotionVector MotionVector::cross(const MotionVector& v)
{
    double v_wx = v.wx();
    double v_wy = v.wy();
    double v_wz = v.wz();
    double v_vx = v.vx();
    double v_vy = v.vy();
    double v_vz = v.vz();

    double wx = this->wx();
    double wy = this->wy();
    double wz = this->wz();
    double vx = this->vx();
    double vy = this->vy();
    double vz = this->vz();

    MotionVector m(-wz * v_wy + wy * v_wz, wz * v_wx - wx * v_wz, -wy * v_wx + wx * v_wy, -vz * v_wy + vy * v_wz - wz * v_vy + wy * v_vz,
                   vz * v_wx - vx * v_wz + wz * v_vx - wx * v_vz, -vy * v_wx + vx * v_wy - wy * v_vx + wx * v_vy);

    return m;
}

ForceVector MotionVector::cross(const ForceVector& v)
{
    double v_mx = v.mx();
    double v_my = v.my();
    double v_mz = v.mz();
    double v_fx = v.fx();
    double v_fy = v.fy();
    double v_fz = v.fz();

    double wx = this->wx();
    double wy = this->wy();
    double wz = this->wz();
    double vx = this->vx();
    double vy = this->vy();
    double vz = this->vz();

    ForceVector f(-wz * v_my + wy * v_mz - vz * v_fy + vy * v_fz, wz * v_mx - wx * v_mz + vz * v_fx - vx * v_fz, -wy * v_mx + wx * v_my - vy * v_fx + vx * v_fy,
                  -wz * v_fy + wy * v_fz, wz * v_fx - wx * v_fz, -wy * v_fx + wx * v_fy);

    return f;
}
}  // namespace Math
}  // namespace RobotDynamics
