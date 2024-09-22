// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

/**
 * @file FramePoint.hpp
 * @brief File containing the FramePoint<T> object definition
 */

#ifndef __RDL_FRAME_POINT_HPP__
#define __RDL_FRAME_POINT_HPP__

#include "rdl_dynamics/FrameObject.hpp"
#include "rdl_dynamics/FrameVector.hpp"
#include "rdl_dynamics/Point3.hpp"
#include "rdl_dynamics/FrameExceptions.hpp"

/**
 * @page frame_point Frame Point
 *
 * A RobotDynamics::Math::FramePoint is a 3d point in Cartesian space with an associated reference frame
 */

namespace RobotDynamics
{
namespace Math
{
/**
 * @class FramePoint
 * @ingroup reference_frame
 * @brief A FramePoint is a 3D point that is expressed in a ReferenceFrame. To change the ReferenceFrame a
 * FramePoint is expressed in, you may call the inhereted FrameObject::changeFrame method and supply it a
 * pointer to the ReferenceFrame you wish to have the FramePoint expressed in. This class and its
 * implementation are an adaptation of FramePoint.java by <a href="http://robots.ihmc.us/">Jerry Pratt and the IHMC
 **Robotics Group</a>.
 */
class FramePoint : public FrameObject, public Math::Point3d
{
  public:
    /**
     * @brief Constructor
     * @param referenceFrame A pointer to the ReferenceFrame the point will be expressed in
     * @param x The x-component of the point
     * @param y The y-component of the point
     * @param z The z-component of the point
     */
    FramePoint(ReferenceFramePtr referenceFrame, const double x, const double y, const double z) : FrameObject(referenceFrame), Math::Point3d(x, y, z)
    {
    }

    /**
     * @brief Constructor
     * @param referenceFrame A pointer to the ReferenceFrame the point will be expressed in
     * @param v A Vector3d that will be used to set the components of this FramePoint
     */
    FramePoint(ReferenceFramePtr referenceFrame, Math::Vector3d v) : FrameObject(referenceFrame), Math::Point3d(v[0], v[1], v[2])
    {
    }

    /**
     * @brief Constructor
     * @param referenceFrame A pointer to the ReferenceFrame the point will be expressed in
     * @param point A Math::Point3 that will be used to set the components of this FramePoint
     */
    FramePoint(ReferenceFramePtr referenceFrame, const Math::Point3d& point) : FrameObject(referenceFrame), Math::Point3d(point)
    {
    }

    /**
     * @brief Copy constructor
     * @param framePoint A FramePoint to copy
     */
    FramePoint(const FramePoint& framePoint) : FrameObject(framePoint.getReferenceFrame()), Math::Point3d(framePoint.x(), framePoint.y(), framePoint.z())
    {
    }

    /**
     * @brief Constructor that initializes to (x,y,z) = (0,0,0)
     * @param referenceFrame A pointer to the ReferenceFrame the point will be expressed in
     */
    explicit FramePoint(ReferenceFramePtr referenceFrame) : FrameObject(referenceFrame), Math::Point3d()
    {
    }

    /**
     * @brief Empty constructor that creates a point with ReferencFrame=nullptr and (x,y,z)=(0,0,0)
     */
    FramePoint() : FrameObject(nullptr), Math::Point3d()
    {
    }

    FramePoint& operator=(const FramePoint& p)
    {
        this->referenceFrame = p.getReferenceFrame();
        this->x() = p.x();
        this->y() = p.y();
        this->z() = p.z();
        return *this;
    }

    /**
     * @brief Destructor
     */
    ~FramePoint()
    {
    }

    /**
     * @brief Get as point3d
     * @return Point as Point3d type
     */
    inline Math::Point3d point() const
    {
        return Math::Point3d(x(), y(), z());
    }

    /**
     * @brief Return a pointer to this as base class type Math::TransformableGeometricObject. See
     **FrameObject::changeFrame for how this method is used
     * @return Pointer to this object as type Math::TransformableGeometricObject
     */
    Math::TransformableGeometricObject* getTransformableGeometricObject()
    {
        return this;
    }

    /**
     * @brief copy into new frame point and change the frame of that
     * @param referenceFrame
     * @return
     */
    FramePoint changeFrameAndCopy(ReferenceFramePtr referenceFrame) const
    {
        FramePoint p = *this;
        p.changeFrame(referenceFrame);
        return p;
    }

    /**
     * @brief Set both the ReferenceFrame this object is expressed in as well as the (x,y,z) coordinates of the point
     * @param v Vector3d that this point will be set to
     * @param referenceFrame Pointer to the ReferenceFrame this object will be expressed in
     */
    EIGEN_STRONG_INLINE void setIncludingFrame(const Math::Vector3d& v, ReferenceFramePtr referenceFrame)
    {
        setIncludingFrame(v(0), v(1), v(2), referenceFrame);
    }

    /**
     * @brief Set both the ReferenceFrame the point is expressed in as well as the (x,y,z) coordinates
     * @param x The x coordinate
     * @param y The y coordinate
     * @param z The z coordinate
     * @param referenceFrame The ReferenceFrame this point is to be expressed in
     */
    void setIncludingFrame(const double x, const double y, const double z, ReferenceFramePtr referenceFrame)
    {
        if (!referenceFrame)
        {
            throw ReferenceFrameException("Reference frame is nullptr!");
        }

        this->set(x, y, z);
        this->referenceFrame = referenceFrame;
    }

    /**
     * @brief Set both the ReferenceFrame the point is expressed in as well as the (x,y,z) coordinates
     * @param point Math::Point3d to set this point to
     * @param referenceFrame Pointer to ReferenceFrame this point will be expressed in
     */
    void setIncludingFrame(const Math::Point3d& point, ReferenceFramePtr referenceFrame)
    {
        if (!referenceFrame)
        {
            throw ReferenceFrameException("Reference frame cannot be nullptr!");
        }

        this->x() = point.x();
        this->y() = point.y();
        this->z() = point.z();
        this->referenceFrame = referenceFrame;
    }

    /**
     * @brief Calculate the distance squared between two FramePoints.  \f$\Delta_x^2+\Delta_y^2+\Delta_z^2\f$
     * @throws ReferenceFrameException If both points are not expressed in the same ReferenceFrame
     * @param point FramePoint to calculate squared distance to
     * @return Distance squared
     */
    double distanceSquared(const FramePoint& point) const
    {
        checkReferenceFramesMatch(&point);

        double dx = this->x() - point.x();
        double dy = this->y() - point.y();
        double dz = this->z() - point.z();
        return dx * dx + dy * dy + dz * dz;
    }

    /**
     * @brief Calculate the 2D distance squared between two FramePoints
     * @param point FramePoint to calculate squared distance to
     * @param plane, 2 = z, 1 = y, 0 = x. Defaults to 2 which is the z = 0 plane
     * @throws ReferenceFrameException If both points are not expressed in the same ReferenceFrame
     * @throws std::runtime_error if the plane argument is less than zero or greater than 2
     * @return Distance squared
     */
    double distance2DSquared(const FramePoint& point, int plane = 2) const
    {
        checkReferenceFramesMatch(&point);
        if (plane < 0 || plane > 2)
        {
            throw std::runtime_error("FramePoint.distance2DSquared: Plane argument must be either 0, 1, or 2");
        }

        double dx = plane == 0 ? 0. : this->x() - point.x();
        double dy = plane == 1 ? 0. : this->y() - point.y();
        double dz = plane == 2 ? 0. : this->z() - point.z();
        return dx * dx + dy * dy + dz * dz;
    }

    /**
     * @brief Calculate the distance squared from this point to a Point3d
     * @param point Point3d to calculate squared distance to
     * @param plane, 2 = z, 1 = y, 0 = x. Defaults to 2 which is the z = 0 plane
     * @throws std::runtime_error if the plane argument is less than zero or greater than 2
     * @return Distance squared
     */
    double distance2DSquared(const Point3d& point, int plane = 2) const
    {
        if (plane < 0 || plane > 2)
        {
            throw std::runtime_error("FramePoint.distance2DSquared: Plane argument must be either 0, 1, or 2");
        }

        double dx = plane == 0 ? 0. : this->x() - point.x();
        double dy = plane == 1 ? 0. : this->y() - point.y();
        double dz = plane == 2 ? 0. : this->z() - point.z();
        return dx * dx + dy * dy + dz * dz;
    }

    /**
     * @brief Calculate the distance squared from this point to a Point3d
     * @param point Point3d to calculate squared distance to
     * @return Distance squared
     */
    double distanceSquared(const Point3d& point) const
    {
        double dx = this->x() - point.x();
        double dy = this->y() - point.y();
        double dz = this->z() - point.z();
        return dx * dx + dy * dy + dz * dz;
    }

    /**
     * @brief Calculate the distance between two FramePoints. \f$\sqrt{\Delta_x^2+\Delta_y^2+\Delta_z^2}\f$
     * @throws ReferenceFrameException If both points are not expressed in the same ReferenceFrame
     * @param point FramePoint to calculate distance to
     * @return Distance between points as template type T
     */
    double distance2D(const FramePoint& point, int plane = 2) const
    {
        checkReferenceFramesMatch(&point);

        return sqrt(distance2DSquared(point, plane));
    }

    /**
     * @brief Calculate the distance between two FramePoints. \f$\sqrt{\Delta_x^2+\Delta_y^2+\Delta_z^2}\f$
     * @param plane 2 = z, 1 = y, 0 = x. Defaults to 2 which is the z = 0 plane
     * @param point FramePoint to calculate distance to
     * @throws ReferenceFrameException If both points are not expressed in the same ReferenceFrame
     * @return Distance between points
     */
    double distance(const FramePoint& point) const
    {
        checkReferenceFramesMatch(&point);

        return sqrt(distanceSquared(point));
    }

    /**
     * @brief Calculate the distance to a Point3d
     * @param point Point3d to calculate distance to
     * @param plane 2 = z, 1 = y, 0 = x. Defaults to 2 which is the z = 0 plane
     * @return Distance between points
     */
    double distance2D(const Point3d& point, int plane = 2) const
    {
        return sqrt(distance2DSquared(point, plane));
    }

    /**
     * @brief Calculate the distance to a Point3d
     * @param point Point3d to calculate distance to
     * @return Distance between points
     */
    double distance(const Point3d& point) const
    {
        return sqrt(distanceSquared(point));
    }

    /**
     * @brief Calculate the L1 distance between two FramePoints by \f$|\Delta_x| + |\Delta_y| + |\Delta_z|\f$
     * @throws ReferenceFrameException If both points are not expressed in the same ReferenceFrame
     * @param point FramePoint to calculate distance to
     * @return Distance between points as template type T
     */
    double distanceL1(const FramePoint& point) const
    {
        checkReferenceFramesMatch(&point);

        return fabs(this->x() - point.x()) + fabs(this->y() - point.y()) + fabs(this->z() - point.z());
    }

    /**
     * @brief Calculate the LInfinity distance between two FramePoints by \f$max(|\Delta_x|,|\Delta_y|,|\Delta_z|)\f$
     * @throws ReferenceFrameException If both points are not expressed in the same ReferenceFrame
     * @param point FramePoint to calculate distance to
     * @return Distance between points as template type T
     */
    double distanceLinf(const FramePoint& point) const
    {
        checkReferenceFramesMatch(&point);

        double dx = this->x() - point.x();
        double dy = this->y() - point.y();
        double dz = this->z() - point.z();

        double tmp = fabs(dx) > fabs(dy) ? fabs(dx) : fabs(dy);

        return tmp > fabs(dz) ? tmp : fabs(dz);
    }

    /**
     * @brief Return true FramePoint argument is within epsilon of this, false otherwise
     * @param point The FramePoint to be compared
     * @param epsilon The tolerance of the comparison check
     * @throws ReferenceFrameException If both points are not expressed in the same ReferenceFrame
     *
     * @deprecated prefer isApprox instead
     */
    [[deprecated("Use isApprox(FramePoint, double) instead.")]] bool epsilonEquals(const FramePoint& point, const double epsilon) const
    {
        checkReferenceFramesMatch(&point);
        return std::abs(this->x() - point.x()) < epsilon && std::abs(this->y() - point.y()) < epsilon && std::abs(this->z() - point.z()) < epsilon;
    }

    /**
     * @brief Return true FramePoint argument is within epsilon of this and they have the same reference frame, false otherwise
     * @param point The FramePoint to be compared
     * @param epsilon The tolerance of the comparison check
     */
    bool isApprox(const FramePoint& point, const double epsilon) const
    {
        return referenceFrame == point.getReferenceFrame() && std::abs(this->x() - point.x()) < epsilon && std::abs(this->y() - point.y()) < epsilon &&
               std::abs(this->z() - point.z()) < epsilon;
    }

    /**
     * @brief Overloaded *= operator, performs this = this*scala
     * @param scale Scalar to scale each element of this FramePoint by
     * @return FramePoint representing this = this*scale
     */
    template <typename T>
    void operator*=(const T scale)
    {
        this->x() *= scale;
        this->y() *= scale;
        this->z() *= scale;
    }

    /**
     * @brief Overloaded /= operator, performs this = this*scale
     * @param scale Scalar to divide each element of this FramePoint by
     * @return FramePoint representing this = this/scale
     */
    template <typename T>
    void operator/=(const T scale)
    {
        this->x() /= scale;
        this->y() /= scale;
        this->z() /= scale;
    }

    void operator+=(const FrameVector& v)
    {
        checkReferenceFramesMatch(&v);
        this->x() += v.x();
        this->y() += v.y();
        this->z() += v.z();
    }

    void operator-=(const FrameVector& v)
    {
        checkReferenceFramesMatch(&v);
        this->x() -= v.x();
        this->y() -= v.y();
        this->z() -= v.z();
    }
};

inline FramePoint operator+(FramePoint p, const FrameVector& v)
{
    p += v;
    return p;
}

inline FramePoint operator-(FramePoint p, const FrameVector& v)
{
    p -= v;
    return p;
}

template <typename T>
inline FrameVector operator*(const T scale, FramePoint p)
{
    p *= scale;
    return FrameVector(p.getReferenceFrame(), p.x(), p.y(), p.z());
}

template <typename T>
inline FrameVector operator*(FramePoint p, const T scale)
{
    p *= scale;
    return FrameVector(p.getReferenceFrame(), p.x(), p.y(), p.z());
}

/**
 * @brief Check if two FramePoints are equal
 * @param lhs
 * @param rhs
 * @throws ReferenceFrameException If the two FramePoint arguments are not expressed in the same ReferenceFrame
 * @return bool true if equal, false if not equal
 */
inline bool operator==(const FramePoint& lhs, const FramePoint& rhs)
{
    lhs.checkReferenceFramesMatch(&rhs);

    if (lhs.x() != rhs.x())
    {
        return false;
    }

    if (lhs.y() != rhs.y())
    {
        return false;
    }

    if (lhs.z() != rhs.z())
    {
        return false;
    }

    return true;
}

/**
 * @brief Subtract two FramePoints and return result in newly created FramePoint
 * @param p1
 * @param p2
 * @throws ReferenceFrameException If the two FramePoint arguments are not expressed in the same ReferenceFrame
 * @return A FramePoint that is the difference of the two argument FramePoints, i.e. p1-=p2
 */
inline FrameVector operator-(FramePoint p1, const FramePoint& p2)
{
    p1.getReferenceFrame()->checkReferenceFramesMatch(p2.getReferenceFrame());
    return FrameVector(p1.getReferenceFrame(), p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
}

/**
 * @brief Check if two FramePoints are not equal
 * @param lhs
 * @param rhs
 * @throws ReferenceFrameException If the two FramePoint arguments are not expressed in the same ReferenceFrame
 * @return bool false if equal, true if not equal
 */
inline bool operator!=(const FramePoint& lhs, const FramePoint& rhs)
{
    return !operator==(lhs, rhs);
}

inline std::ostream& operator<<(std::ostream& output, const FramePoint& framePoint)
{
    output << "ReferenceFrame = " << framePoint.getReferenceFrame()->getName() << std::endl;
    output << "x = " << framePoint.x() << " y = " << framePoint.y() << " z = " << framePoint.z() << std::endl;
    return output;
}
typedef std::vector<FramePoint, Eigen::aligned_allocator<FramePoint>> FramePointV;
}  // namespace Math
}  // namespace RobotDynamics
#endif  // ifndef __RDL_FRAME_POINT_HPP__
