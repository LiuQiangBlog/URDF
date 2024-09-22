// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_DYNAMICS_FRAME_ORIENTATION_HPP__
#define __RDL_DYNAMICS_FRAME_ORIENTATION_HPP__

/**
 * @file FrameOrientation.hpp
 * @page frame_orientation Frame Orientation
 *
 * The RobotDynamics::Math::FrameOrientation object is a class for storing an orientation w.r.t a reference frame
 */

#include <rdl_dynamics/FrameObject.hpp>
#include <rdl_dynamics/rdl_eigenmath.hpp>

namespace RobotDynamics
{
namespace Math
{
/**
 * @class FrameOrientation
 * @ingroup reference_frame
 * @brief A Frame object that represents an orientation(quaternion) relative to a reference frame
 */
class FrameOrientation : public FrameObject, public Quaternion
{
  public:
    FrameOrientation() : FrameObject(nullptr), Quaternion(0., 0., 0., 1.)
    {
    }

    explicit FrameOrientation(ReferenceFramePtr referenceFrame) : FrameObject(referenceFrame), Quaternion(0., 0., 0., 1.)
    {
    }

    FrameOrientation(ReferenceFramePtr referenceFrame, Quaternion quat) : FrameObject(referenceFrame), Quaternion(quat)
    {
    }

    FrameOrientation(ReferenceFramePtr referenceFrame, double x, double y, double z, double w) : FrameObject(referenceFrame), Quaternion(x, y, z, w)
    {
    }

    FrameOrientation(ReferenceFramePtr referenceFrame, const Matrix3d& E) : FrameObject(referenceFrame), Quaternion(E)
    {
    }

    FrameOrientation(ReferenceFramePtr referenceFrame, const AxisAngle& axis_angle) : FrameObject(referenceFrame), Quaternion(axis_angle)
    {
    }

    FrameOrientation(ReferenceFramePtr referenceFrame, Vector3d axis, double angle) : FrameObject(referenceFrame), Quaternion(axis, angle)
    {
    }

    Math::TransformableGeometricObject* getTransformableGeometricObject()
    {
        return this;
    }

    [[deprecated("Use FrameOrientation(ReferenceFramePtr, Quaternion) instead.")]] void setIncludingFrame(const Quaternion& q, ReferenceFramePtr referenceFrame)
    {
        *this = FrameOrientation(referenceFrame, q);
    }

    [[deprecated("Use FrameOrientation(ReferenceFramePtr, Matrix3d) instead.")]] void setIncludingFrame(const Matrix3d& E, ReferenceFramePtr referenceFrame)
    {
        *this = FrameOrientation(referenceFrame, E);
    }

    [[deprecated("Use FrameOrientation(ReferenceFramePtr, AxisAngle) instead.")]] void setIncludingFrame(const AxisAngle& axis_angle, ReferenceFramePtr referenceFrame)
    {
        *this = FrameOrientation(referenceFrame, axis_angle);
    }

    [[deprecated("Use FrameOrientation(ReferenceFramePtr, Vector3d axis, double angle) instead.")]] void setIncludingFrame(Vector3d axis, double angle,
                                                                                                                           ReferenceFramePtr referenceFrame)
    {
        *this = FrameOrientation(referenceFrame, axis, angle);
    }

    void setOrientation(const Quaternion& o)
    {
        x() = o.x();
        y() = o.y();
        z() = o.z();
        w() = o.z();
    }

    void setOrientation(const Matrix3d& E)
    {
        Quaternion o(E);
        x() = o.x();
        y() = o.y();
        z() = o.z();
        w() = o.z();
    }

    void setOrientation(const AxisAngle& axis_angle)
    {
        Quaternion o(axis_angle);
        x() = o.x();
        y() = o.y();
        z() = o.z();
        w() = o.z();
    }

    void setOrientation(Vector3d axis, double angle)
    {
        Quaternion o(axis, angle);
        x() = o.x();
        y() = o.y();
        z() = o.z();
        w() = o.z();
    }

    FrameOrientation changeFrameAndCopy(ReferenceFramePtr referenceFrame) const
    {
        FrameOrientation ret = *this;
        ret.changeFrame(referenceFrame);
        return ret;
    }
};
}  // namespace Math
}  // namespace RobotDynamics

#endif  //__RDL_DYNAMICS_FRAME_ORIENTATION_HPP__
