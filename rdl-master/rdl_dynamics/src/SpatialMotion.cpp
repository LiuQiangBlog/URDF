// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include "rdl_dynamics/SpatialMotion.hpp"

namespace RobotDynamics
{
namespace Math
{
void SpatialMotion::operator+=(const SpatialMotion& v)
{
    this->checkReferenceFramesMatch(&v);
    this->bodyFrame->checkReferenceFramesMatch(v.baseFrame);

    this->wx() += v.wx();
    this->wy() += v.wy();
    this->wz() += v.wz();

    this->vx() += v.vx();
    this->vy() += v.vy();
    this->vz() += v.vz();

    this->bodyFrame = v.bodyFrame;
}

void SpatialMotion::operator-=(const SpatialMotion& v)
{
    checkReferenceFramesMatch(&v);

    this->wx() -= v.wx();
    this->wy() -= v.wy();
    this->wz() -= v.wz();

    this->vx() -= v.vx();
    this->vy() -= v.vy();
    this->vz() -= v.vz();

    if (this->baseFrame == v.getBaseFrame())
    {
        this->baseFrame = v.bodyFrame;
    }
    else if (this->bodyFrame == v.bodyFrame)
    {
        this->bodyFrame = v.baseFrame;
    }
    else
    {
        throw ReferenceFrameException("Cannot perform -= operation on spatial motion vectors due to a reference frame mismatch!");
    }
}

void SpatialMotion::operator%=(const SpatialMotion& v)
{
    checkReferenceFramesMatch(&v);
    this->bodyFrame->checkReferenceFramesMatch(v.getReferenceFrame());

    set(MotionVector::operator%=(v));
}
}  // namespace Math
}  // namespace RobotDynamics
