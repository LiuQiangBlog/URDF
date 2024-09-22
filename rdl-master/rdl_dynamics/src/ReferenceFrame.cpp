// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include "rdl_dynamics/ReferenceFrame.hpp"

namespace RobotDynamics
{
ReferenceFramePtr ReferenceFrame::worldFrame = ReferenceFrame::createAWorldFrame();

RobotDynamics::Math::SpatialTransform ReferenceFrame::getTransformToDesiredFrame(ReferenceFramePtr desiredFrame)
{
    if (this == desiredFrame.get())
    {
        return RobotDynamics::Math::SpatialTransform();
    }

    verifyFramesHaveSameRoot(desiredFrame);

    return desiredFrame->getInverseTransformToRoot() * getTransformToRoot();
}

void ReferenceFrame::update()
{
    if (parentFrame == nullptr)
    {
        return;
    }

    inverseTransformToRoot = transformFromParent * parentFrame->getInverseTransformToRoot();
    transformToRoot = inverseTransformToRoot.inverse();
}

void ReferenceFrame::checkReferenceFramesMatch(ReferenceFramePtr referenceFrame) const
{
    checkReferenceFramesMatch(referenceFrame.get());
}

void ReferenceFrame::checkReferenceFramesMatch(ReferenceFrame* referenceFrame) const
{
    if (referenceFrame == nullptr)
    {
        throw ReferenceFrameException("Reference frame is nullptr!");
    }

    if (referenceFrame != this)
    {
        throw ReferenceFrameException("Reference frames do not match!");
    }
}

void ReferenceFrame::verifyFramesHaveSameRoot(ReferenceFramePtr frame)
{
    if (rootFrame != frame->getRootFrame())
    {
        std::string msg = "Frames " + frame->getName() + " and " + this->getName() + " have mismatched roots!";
        throw ReferenceFrameException(msg);
    }
}
}  // namespace RobotDynamics
