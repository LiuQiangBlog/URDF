// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include "rdl_dynamics/SpatialAcceleration.hpp"

namespace RobotDynamics
{
namespace Math
{
void SpatialAcceleration::changeFrameWithRelativeMotion(ReferenceFramePtr newFrame, SpatialMotion twistOfCurrentFrameWithRespectToNewFrame,
                                                        const SpatialMotion& twistOfBodyWrtBaseExpressedInCurrent)
{
    if (this->referenceFrame == newFrame)
    {
        return;
    }

    referenceFrame->checkReferenceFramesMatch(twistOfCurrentFrameWithRespectToNewFrame.getReferenceFrame());
    referenceFrame->checkReferenceFramesMatch(twistOfCurrentFrameWithRespectToNewFrame.getBodyFrame());
    newFrame->checkReferenceFramesMatch(twistOfCurrentFrameWithRespectToNewFrame.getBaseFrame());

    checkReferenceFramesMatch(&twistOfBodyWrtBaseExpressedInCurrent);
    bodyFrame->checkReferenceFramesMatch(twistOfBodyWrtBaseExpressedInCurrent.getBodyFrame());
    baseFrame->checkReferenceFramesMatch(twistOfBodyWrtBaseExpressedInCurrent.getBaseFrame());

    twistOfCurrentFrameWithRespectToNewFrame %= twistOfBodyWrtBaseExpressedInCurrent;

    this->wx() += twistOfCurrentFrameWithRespectToNewFrame.wx();
    this->wy() += twistOfCurrentFrameWithRespectToNewFrame.wy();
    this->wz() += twistOfCurrentFrameWithRespectToNewFrame.wz();
    this->vx() += twistOfCurrentFrameWithRespectToNewFrame.vx();
    this->vy() += twistOfCurrentFrameWithRespectToNewFrame.vy();
    this->vz() += twistOfCurrentFrameWithRespectToNewFrame.vz();

    this->changeFrame(newFrame);
}

void SpatialAcceleration::operator+=(const SpatialAcceleration& v)
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

void SpatialAcceleration::operator-=(const SpatialAcceleration& v)
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
        throw ReferenceFrameException("Reference frame mismatch during subtraction of SpatialAccelerations!");
    }
}
}  // namespace Math
}  // namespace RobotDynamics
