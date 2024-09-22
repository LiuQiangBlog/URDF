// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_FRAME_OBJECT_HPP__
#define __RDL_FRAME_OBJECT_HPP__

/**
 * @page frame_object Frame Object
 *
 * The RobotDynamics::FrameObject class is inhereted by objects that perform operations in which reference frame checking
 * is important.
 */

#include "rdl_dynamics/ReferenceFrame.hpp"
#include "rdl_dynamics/rdl_eigenmath.hpp"

namespace RobotDynamics
{
/**
 * @class FrameObject
 * @ingroup reference_frame
 * @brief An interface that objects with a ReferenceFrame extend to inherit the FrameObject::changeFrame method
 */
class FrameObject
{
  public:
    explicit FrameObject(ReferenceFramePtr referenceFrame) : referenceFrame(referenceFrame)
    {
    }

    /**
     * @brief Destructor
     */
    virtual ~FrameObject()
    {
    }

    /**
     * @brief Change the ReferenceFrame this FrameObject is expressed in.
     * @param desiredFrame A pointer to the ReferenceFrame this FrameObject is to be transformed to
     */
    virtual void changeFrame(ReferenceFramePtr desiredFrame);

    /**
     * @brief Get a pointer to the reference frame this FrameObject is expressed in
     * @return Pointer to the ReferenceFrame this FrameObject is expressed in
     */
    inline ReferenceFramePtr getReferenceFrame() const
    {
        return referenceFrame;
    }

    /**
     * @brief Set frame objects internal reference frame
     * @param frame Pointer to new frame
     */
    inline void setReferenceFrame(ReferenceFramePtr frame)
    {
        referenceFrame = frame;
    }

    /**
     * @brief Check if two FrameObjects hold the same ReferenceFrame
     * @param frameObject
     */
    void checkReferenceFramesMatch(const FrameObject* frameObject) const
    {
        getReferenceFrame()->checkReferenceFramesMatch(frameObject->getReferenceFrame());
    }

    void checkReferenceFramesMatch(FrameObject* frameObject) const
    {
        getReferenceFrame()->checkReferenceFramesMatch(frameObject->getReferenceFrame());
    }

  protected:
    ReferenceFramePtr referenceFrame /**< Pointer to a ReferenceFrame*/;

    /**
     * @brief Pure virtual method that FrameObjects are required to implement so the FrameObject::changeFrame method is able
     * to access the TransformableGeometricObject which is required to implement the TransformableGeometricObject::transform
     **method
     * @return
     */
    virtual Math::TransformableGeometricObject* getTransformableGeometricObject() = 0;
};
}  // namespace RobotDynamics
#endif  // ifndef __RDL_FRAME_OBJECT_HPP__
