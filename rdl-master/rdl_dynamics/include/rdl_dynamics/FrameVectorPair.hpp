// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_FRAME_VECTOR_PAIR_HPP__
#define __RDL_FRAME_VECTOR_PAIR_HPP__

/**
 * @file FrameVectorPair.hpp
 * @page frame_vector_pair Frame Vector Pair
 *
 * A RobotDynamics::Math::FrameVectorPair is a data type containing two 3d vectors with an associated
 * reference frame. The prime motivation for this is for ensuring spatial transforms aren't performed on
 * 3d vectors and separate the storage of vectors of, for example, point velocities from spatial velocities
 */

#include "rdl_dynamics/FrameObject.hpp"
#include "rdl_dynamics/FrameVector.hpp"
#include "rdl_dynamics/SpatialMotion.hpp"

namespace RobotDynamics
{
namespace Math
{
/**
 * @class FrameVectorPair
 * @ingroup reference_frame
 * @brief A FrameVector is a pair of 3D vector with a ReferenceFrame
 */
class FrameVectorPair
{
  public:
    /**
     * @brief Default constructor. Initializes its ReferenceFrame to nullptr
     */
    FrameVectorPair() : lin(nullptr, 0., 0., 0.), ang(nullptr, 0., 0., 0.)
    {
    }

    /*
     * @brief constructor
     * @param linear Linear part
     * @param angular Angular part
     */
    FrameVectorPair(const FrameVector& linear, const FrameVector& angular) : lin(linear), ang(angular)
    {
        linear.checkReferenceFramesMatch(&angular);
    }

    /*
     * @brief constructor
     * @param referenceFrame pointer to a reference frame
     */
    explicit FrameVectorPair(ReferenceFramePtr referenceFrame) : lin(referenceFrame), ang(referenceFrame)
    {
    }

    /**
     * @brief Constructor
     * @param v Spatial motion
     */
    explicit FrameVectorPair(const SpatialMotion& v)
    {
        lin.setIncludingFrame(v.getLinearPart(), v.getReferenceFrame());
        ang.setIncludingFrame(v.getAngularPart(), v.getReferenceFrame());
    }

    /**
     * @brief Constructor
     * @param referenceFrame A pointer to a ReferenceFrame
     * @param linear linear component
     * @param angular angular component
     */
    FrameVectorPair(ReferenceFramePtr referenceFrame, const Vector3d& linear, const Vector3d& angular) : lin(referenceFrame, linear), ang(referenceFrame, angular)
    {
    }

    /**
     * @brief Constructor
     * @param referenceFrame
     * @param v
     */
    FrameVectorPair(ReferenceFramePtr referenceFrame, const SpatialVector& v) : lin(referenceFrame, v.getLinearPart()), ang(referenceFrame, v.getAngularPart())
    {
    }

    /**
     * @brief Destructor
     */
    virtual ~FrameVectorPair()
    {
    }

    /**
     * @brief Change the frame of the two 3d vectors. Equivalent to the following math expression
     * \f[
     * \begin{bmatrix}
     * E & 0 \\
     * 0 & E
     * \end{bmatrix}
     * \begin{bmatrix}
     * angular \\
     * linear
     * \end{bmatrix}
     * \f]
     *
     * @param referenceFrame
     */
    void changeFrame(ReferenceFramePtr referenceFrame)
    {
        assert(referenceFrame);
        lin.changeFrame(referenceFrame);
        ang.changeFrame(referenceFrame);
    }

    /**
     * @brief copy into new frame vector and change the frame of that
     * @param referenceFrame
     * @return Copy of frame vector pair expressed in new frame
     */
    FrameVectorPair changeFrameAndCopy(ReferenceFramePtr referenceFrame) const
    {
        FrameVectorPair p = *this;
        p.changeFrame(referenceFrame);
        return p;
    }

    /**
     * @brief Set x, y, and z components of linear and angular to 0
     */
    inline void setToZero()
    {
        lin.setZero();
        ang.setZero();
    }

    /**
     * @brief Set the reference frame
     * @param referenceFrame
     */
    void setReferenceFrame(ReferenceFramePtr referenceFrame)
    {
        lin.setReferenceFrame(referenceFrame);
        ang.setReferenceFrame(referenceFrame);
    }

    /**
     * @brief Set the components and the ReferenceFrame these components are expressed in
     * @param linear Linear component
     * @param angular Angular component
     * @param referenceFrame Pointer to a ReferenceFrame this point is expressed in
     */
    inline void setIncludingFrame(ReferenceFramePtr referenceFrame, const Vector3d& linear, const Vector3d& angular)
    {
        assert(referenceFrame);

        lin.setIncludingFrame(linear, referenceFrame);
        ang.setIncludingFrame(angular, referenceFrame);
    }

    /**
     * @brief Set the components and the ReferenceFrame these components are expressed in
     * @param v spatial vector
     * @param referenceFrame Pointer to a ReferenceFrame this point is expressed in
     */
    inline void setIncludingFrame(ReferenceFramePtr referenceFrame, const SpatialVector& v)
    {
        assert(referenceFrame);

        lin.setIncludingFrame(v.getLinearPart(), referenceFrame);
        ang.setIncludingFrame(v.getAngularPart(), referenceFrame);
    }

    /**
     * @brief Set the components and the ReferenceFrame these components are expressed in
     * @param v Motion vector
     * @param referenceFrame pointer to a reference frame
     */
    inline void setIncludingFrame(ReferenceFramePtr referenceFrame, const MotionVector& v)
    {
        assert(referenceFrame);

        lin.setIncludingFrame(v.getLinearPart(), referenceFrame);
        ang.setIncludingFrame(v.getAngularPart(), referenceFrame);
    }

    /**
     * @brief Set the components and the ReferenceFrame these components are expressed in
     * @param v spatial motion
     */
    inline void setIncludingFrame(const SpatialMotion& v)
    {
        lin.setIncludingFrame(v.getLinearPart(), v.getReferenceFrame());
        ang.setIncludingFrame(v.getAngularPart(), v.getReferenceFrame());
    }

    /**
     * @brief Set the linear vector
     * @param v
     */
    inline void setLinearPart(const Vector3d& v)
    {
        lin.set(v);
    }

    /**
     * @brief Set the angular vector
     * @param v
     */
    inline void setAngularPart(const Vector3d& v)
    {
        ang.set(v);
    }

    /**
     * @brief Get copy of linear component
     * @return Copy of linear part
     */
    inline FrameVector linear() const
    {
        return lin;
    }

    /**
     * @brief Get copy of angular component
     * @return Copy of angular component
     */
    inline FrameVector angular() const
    {
        return ang;
    }

    template <typename T>
    void operator*=(const T scale)
    {
        lin *= scale;
        ang *= scale;
    }

    void operator+=(const FrameVectorPair& v)
    {
        lin += v.linear();
        ang += v.angular();
    }

    void operator-=(const FrameVectorPair& v)
    {
        lin -= v.linear();
        ang -= v.angular();
    }

    /**
     * @brief Get pointer to linear vector
     * @return pointer
     */
    FrameVector* linearPtr()
    {
        return &lin;
    }

    /**
     * @brief Get pointer to angular vector
     * @return pointer
     */
    FrameVector* angularPtr()
    {
        return &ang;
    }

  protected:
    FrameVector lin; /**< Linear component */
    FrameVector ang; /**< Angular component */
};

template <typename T>
inline FrameVectorPair operator*(FrameVectorPair pair, const T scalar)
{
    pair *= scalar;
    return pair;
}

template <typename T>
inline FrameVectorPair operator*(const T scalar, FrameVectorPair pair)
{
    pair *= scalar;
    return pair;
}

// cppcheck-suppress passedByValue
inline FrameVectorPair operator+(FrameVectorPair v1, const FrameVectorPair v2)
{
    v1 += v2;
    return v1;
}

// cppcheck-suppress passedByValue
inline FrameVectorPair operator-(FrameVectorPair v1, const FrameVectorPair v2)
{
    v1 -= v2;
    return v1;
}
}  // namespace Math
}  // namespace RobotDynamics

#endif  // ifndef __RDL_FRAME_VECTOR_HPP__
