// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_FRAME_VECTOR_HPP__
#define __RDL_FRAME_VECTOR_HPP__

/**
 * @file FrameVector.hpp
 * @page frame_vector Frame Vector
 *
 * A RobotDynamics::Math::FrameVector is a 3d vector with an associated reference frame
 */

#include "rdl_dynamics/FrameObject.hpp"

namespace RobotDynamics
{
namespace Math
{
/**
 * @class FrameVector
 * @ingroup reference_frame
 * @brief A FrameVector is a 3D vector with a ReferenceFrame, and all operations between FrameVectors and other frame
 **objects
 * will perform runtime checks that objects are expressed in the same frames.
 * This class and its implementation are an adaptation of FrameVector.java by <a href="http://robots.ihmc.us/">Jerry
 **Pratt and the IHMC Robotics Group</a>.
 */
class FrameVector : public FrameObject, public Math::Vector3d
{
  public:
    /**
     * @brief Default constructor. Initializes its ReferenceFrame to nullptr
     */
    FrameVector() : FrameObject(nullptr), Math::Vector3d(0., 0., 0.)
    {
    }

    /**
     * @brief Constructor
     * @param referenceFrame A pointer to a ReferenceFrame
     */
    explicit FrameVector(ReferenceFramePtr referenceFrame) : FrameObject(referenceFrame), Math::Vector3d()
    {
    }

    /**
     * @brief Constructor
     * @param referenceFrame A pointer to a ReferenceFrame
     * @param x Value of the x-coordinate
     * @param y Value of the y-coordinate
     * @param z Value of the z-coordinate
     */
    FrameVector(ReferenceFramePtr referenceFrame, const double& x, const double& y, const double& z) : FrameObject(referenceFrame), Math::Vector3d(x, y, z)
    {
    }

    /**
     * @brief Constructor
     * @param referenceFrame Pointer to a ReferenceFrame
     * @param vector A Vector3d used to set the x,y, and z coordinates
     */
    FrameVector(ReferenceFramePtr referenceFrame, const Eigen::Vector3d& vector) : FrameObject(referenceFrame), Math::Vector3d(vector[0], vector[1], vector[2])
    {
    }

    /**
     * @brief Destructor
     */
    virtual ~FrameVector()
    {
    }

    /**
     * @brief Return pointer to this object as type TransformableGeometricObject. See FrameObject::changeFrame for
     * an example of where this method is used.
     * @return Pointer to this object as type TransformableGeometricObject
     */
    Math::TransformableGeometricObject* getTransformableGeometricObject()
    {
        return this;
    }

    /**
     * @brief copy into new frame vector and change the frame of that
     * @param referenceFrame
     * @return
     */
    FrameVector changeFrameAndCopy(ReferenceFramePtr referenceFrame) const
    {
        FrameVector p = *this;
        p.changeFrame(referenceFrame);
        return p;
    }

    /**
     * @brief Set x, y, and z components to 0
     */
    inline void setToZero()
    {
        set(0., 0., 0.);
    }

    /**
     * @brief Set the x, y, and z components and the ReferenceFrame these components are expressed in
     * @param x The x-component
     * @param y y-component
     * @param z z-component
     * @throws ReferenceFrameException If *referenceFrame=nullptr
     * @param referenceFrame Pointer to a ReferenceFrame this point is expressed in
     */
    inline void setIncludingFrame(const double x, const double y, const double z, ReferenceFramePtr referenceFrame)
    {
        if (!referenceFrame)
        {
            throw ReferenceFrameException("Reference frame cannot be nullptr!");
        }

        set(x, y, z);
        this->referenceFrame = referenceFrame;
    }

    /**
     * @brief Set the x, y, and z components and the ReferenceFrame these components are expressed in
     * @param vector Used to set the x,y, and z components of this point
     * @throws ReferenceFrameException If *referenceFrame=nullptr
     * @param referenceFrame Pointer to a ReferenceFrame this point is expressed in
     */
    inline void setIncludingFrame(const Eigen::Vector3d& vector, ReferenceFramePtr referenceFrame)
    {
        if (!referenceFrame)
        {
            throw ReferenceFrameException("Reference frame cannot be nullptr!");
        }

        set(vector[0], vector[1], vector[2]);
        this->referenceFrame = referenceFrame;
    }

    /**
     * @brief Dot product between two FrameVectors, i.e. \f$ v_1 \cdot v_2 \f$
     * @param frameVector
     * @throws ReferenceFrameException If the FrameVectors aren't expressed in the same ReferenceFrame
     * @return The dot product
     */
    inline double dot(const FrameVector& frameVector) const
    {
        checkReferenceFramesMatch(&frameVector);

        return this->x() * frameVector.x() + this->y() * frameVector.y() + this->z() * frameVector.z();
    }

    /**
     * @brief Cross product between two FrameVectors, i.e. \f$ v_1 \times v_2 \f$
     * @param vector
     * @throws ReferenceFrameException If the FrameVectors aren't expressed in the same ReferenceFrame
     * @return The cross product
     */
    inline FrameVector cross(const FrameVector& vector) const
    {
        checkReferenceFramesMatch(&vector);
        return FrameVector(this->referenceFrame, this->y() * vector.z() - this->z() * vector.y(), vector.x() * this->z() - vector.z() * this->x(),
                           this->x() * vector.y() - this->y() * vector.x());
    }

    /**
     * @brief Cross product, i.e. \f$ v_1 \times v_2 \f$
     * @param vector
     * @return The cross product
     */
    inline Vector3d cross(const Vector3d& vector) const
    {
        return RobotDynamics::Math::Vector3d::cross(vector);
    }

    /**
     * @brief Computer the angle between two FrameVectors, \f$ \arccos{\frac{v_1 \cdot v_2}{|v_1||v_2|}} \f$
     * @param frameVector
     * @return The angle between the vectors
     */
    inline double getAngleBetweenVectors(const FrameVector& frameVector) const
    {
        checkReferenceFramesMatch(&frameVector);

        return acos(std::max(-1., std::min(this->dot(frameVector) / (this->norm() * frameVector.norm()), 1.)));
    }

    inline Vector3d vec() const
    {
        return Math::Vector3d(x(), y(), z());
    }

    void operator+=(const Vector3d& v)
    {
        this->x() += v.x();
        this->y() += v.y();
        this->z() += v.z();
    }

    void operator-=(const Vector3d& v)
    {
        this->x() -= v.x();
        this->y() -= v.y();
        this->z() -= v.z();
    }

    /**
     * @brief Plus euals operator that performs runtime frame checks, \f$ v_1=v_1-v \f$
     * @param v
     */
    void operator+=(const FrameVector& v)
    {
        this->checkReferenceFramesMatch(&v);
        this->x() += v.x();
        this->y() += v.y();
        this->z() += v.z();
    }

    /**
     * @brief Minus euals operator that performs runtime frame checks, \f$ v_1=v_1-v \f$
     * @param v
     */
    void operator-=(const FrameVector& v)
    {
        this->checkReferenceFramesMatch(&v);
        this->x() -= v.x();
        this->y() -= v.y();
        this->z() -= v.z();
    }

    /**
     * @brief Times euals operator that performs runtime frame checks, \f$ v=v*scale \f$
     * @tparam typename of the scale parameter
     * @param scale
     */
    template <typename T>
    void operator*=(const T scale)
    {
        this->x() *= scale;
        this->y() *= scale;
        this->z() *= scale;
    }
};

template <typename T>
inline FrameVector operator*(FrameVector v1, const T scale)
{
    v1 *= scale;
    return v1;
}

template <typename T>
inline FrameVector operator*(const T scale, FrameVector v1)
{
    v1 *= scale;
    return v1;
}

inline FrameVector operator+(FrameVector v1, const Vector3d v2)
{
    v1 += v2;
    return v1;
}

inline FrameVector operator-(FrameVector v1, const Vector3d v2)
{
    v1 -= v2;
    return v1;
}

/**
 * @brief Add two FrameVectors together
 * @param v1
 * @param v2
 * @return THe result of \f$ v_1 + v_2 \f$
 */
// cppcheck-suppress passedByValue
inline FrameVector operator+(FrameVector v1, const FrameVector v2)
{
    v1 += v2;
    return v1;
}

/**
 * @brief Add two FrameVectors together
 * @param v1
 * @param v2
 * @return THe result of \f$ v_1 - v_2 \f$
 */
// cppcheck-suppress passedByValue
inline FrameVector operator-(FrameVector v1, const FrameVector v2)
{
    v1 -= v2;
    return v1;
}
typedef std::vector<FrameVector, Eigen::aligned_allocator<FrameVector>> FrameVectorV;
}  // namespace Math
}  // namespace RobotDynamics

#endif  // ifndef __RDL_FRAME_VECTOR_HPP__
