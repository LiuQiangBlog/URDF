// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_POINT_3_HPP__
#define __RDL_POINT_3_HPP__

/**
 * @file Point3.hpp
 */

#include <math.h>
#include <stdexcept>
#include <type_traits>
#include "rdl_dynamics/rdl_eigenmath.hpp"

namespace RobotDynamics
{
namespace Math
{
/**
 * @class Point3
 * @brief A generic 3D point
 */
class Point3d : public Math::TransformableGeometricObject
{
  public:
    Point3d(const double x, const double y, const double z)
    {
        point[0] = x;
        point[1] = y;
        point[2] = z;
    }

    Point3d(const Point3d& point) : Point3d(point.x(), point.y(), point.z())
    {
    }

    // cppcheck-suppress noExplicitConstructor
    explicit Point3d(const Vector3d& vector) : Point3d(vector[0], vector[1], vector[2])
    {
    }

    EIGEN_STRONG_INLINE Point3d() : Point3d(0., 0., 0.)
    {
    }

    virtual ~Point3d()
    {
    }

    /**
     * @brief Performs in place point transform. Given a point, \f$p\f$, this performs \f$ p = -X.E X.r + X.E p \f$
     * @param X
     */
    inline void transform(const Math::SpatialTransform& X)
    {
        set(-X.E * X.r + X.E * this->vec());
    }

    inline Point3d transform_copy(const Math::SpatialTransform& X) const
    {
        Point3d p = *this;
        p.transform(X);
        return p;
    }

    EIGEN_STRONG_INLINE void set(const std::vector<double>& vector)
    {
        set(vector[0], vector[1], vector[2]);
    }

    EIGEN_STRONG_INLINE void set(const Point3d& point)
    {
        set(point.x(), point.y(), point.z());
    }

    void set(const Math::Vector3d& v)
    {
        set(v[0], v[1], v[2]);
    }

    void set(const double x, const double y, const double z)
    {
        point[0] = x;
        point[1] = y;
        point[2] = z;
    }

    EIGEN_STRONG_INLINE void setToZero()
    {
        set(0., 0., 0.);
    }

    /**
     * Compares if two points are within epsilon of each other
     * @param point
     * @param epsilon
     * @return true if they are within epsilon, false otherwise
     */
    EIGEN_STRONG_INLINE bool epsilonEquals(const Point3d& point, const double epsilon) const
    {
        return fabs(this->x() - point.x()) < epsilon && fabs(this->y() - point.y()) < epsilon && fabs(this->z() - point.z()) < epsilon;
    }

    /**
     * @brief clamp any values that are less than min to min
     * @param min
     */
    void clampMin(const double min)
    {
        if (x() < min)
        {
            x() = min;
        }

        if (y() < min)
        {
            y() = min;
        }

        if (z() < min)
        {
            z() = min;
        }
    }

    /**
     * @brief clamp any values that are greater than make to max
     * @param max
     */
    void clampMax(const double max)
    {
        if (x() > max)
        {
            x() = max;
        }

        if (y() > max)
        {
            y() = max;
        }

        if (z() > max)
        {
            z() = max;
        }
    }

    /**
     * @brief clamp any values greater than max to max, and any value less than min to min
     * @param min
     * @param max
     */
    void clampMinMax(const double min, const double max)
    {
        this->clampMin(min);
        this->clampMax(max);
    }

    /**
     * @brief Set each element to the absolute value
     */
    void absoluteValue()
    {
        this->x() = fabs(this->x());
        this->y() = fabs(this->y());
        this->z() = fabs(this->z());
    }

    /**
     * @brief Square of the 2d distance between two points
     * @param point
     * @param plane, 2 = z, 1 = y, 0 = x. Defaults to 2 which is the z = 0 plane
     * @return Distance squared
     */
    double distance2DSquared(const Point3d& point, int plane = 2) const
    {
        if (plane < 0 || plane > 2)
        {
            throw std::runtime_error("Point3.distance2DSquared: Invalid plane arg. Must be 0, 1, or 2");
        }

        return plane == 2 ? std::pow(x() - point.x(), 2) + std::pow(y() - point.y(), 2) :
               plane == 1 ? std::pow(x() - point.x(), 2) + std::pow(z() - point.z(), 2) :
                            std::pow(y() - point.y(), 2) + std::pow(z() - point.z(), 2);
    }

    /**
     * @brief Square of the distance between two points, \f$ x^2 + y^2 + z^2 \f$
     * @param point
     * @return Distance squared
     */
    double distanceSquared(const Point3d& point) const
    {
        return std::pow(x() - point.x(), 2) + std::pow(y() - point.y(), 2) + std::pow(z() - point.z(), 2);
    }

    /**
     * brief 2D Distance between two points
     * @param point
     * @param plane, 2 = z, 1 = y, 0 = x. Defaults to 2 which is the z = 0 plane
     * @return Distance
     */
    double distance2D(const Point3d& point, int plane = 2) const
    {
        return std::sqrt(distance2DSquared(point, plane));
    }

    /**
     * brief Distance between two points, \f$ \sqrt{x^2 + y^2 + z^2} \f$
     * @param point
     * @return Distance
     */
    double distance(const Point3d& point) const
    {
        return std::sqrt(distanceSquared(point));
    }

    /**
     * @brief L1 norm of two points
     * @param point
     * @return L1 norm
     */
    double distanceL1(const Point3d& point) const
    {
        return fabs(x() - point.x()) + fabs(y() - point.y()) + fabs(z() - point.z());
    }

    /**
     * @brief Cross product between a point and vector
     * @param v
     * @return
     */
    Vector3d cross(const Vector3d& v)
    {
        return Vector3d(point[1] * v[2] - point[2] * v[1], point[2] * v[0] - point[0] * v[2], point[0] * v[1] - point[1] * v[0]);
    }

    /**
     * L-infinity norm
     * @param point
     * @return
     */
    double distanceLinf(const Point3d& point) const
    {
        double dx = x() - point.x();
        double dy = y() - point.y();
        double dz = z() - point.z();

        double tmp = fabs(dx) > fabs(dy) ? fabs(dx) : fabs(dy);

        return tmp > fabs(dz) ? tmp : fabs(dz);
    }

    EIGEN_STRONG_INLINE double& x()
    {
        return point[0];
    }

    EIGEN_STRONG_INLINE double x() const
    {
        return point[0];
    }

    EIGEN_STRONG_INLINE double& y()
    {
        return point[1];
    }

    EIGEN_STRONG_INLINE double y() const
    {
        return point[1];
    }

    EIGEN_STRONG_INLINE double& z()
    {
        return point[2];
    }

    EIGEN_STRONG_INLINE double z() const
    {
        return point[2];
    }

    EIGEN_STRONG_INLINE double* data()
    {
        return point;
    }

    EIGEN_STRONG_INLINE Math::Vector3d vec() const
    {
        return Math::Vector3d(point[0], point[1], point[2]);
    }

    Point3d& operator=(const Point3d& other)
    {
        set(other);
        return *this;
    }

    template <typename T>
    void operator*=(const T scale)
    {
        static_assert(std::is_pod<T>::value, "T must be POD");
        this->x() *= scale;
        this->y() *= scale;
        this->z() *= scale;
    }

    template <typename T>
    void operator/=(const T scale)
    {
        static_assert(std::is_pod<T>::value, "T must be POD");
        this->x() /= scale;
        this->y() /= scale;
        this->z() /= scale;
    }

    bool operator==(const Point3d& rhs)
    {
        if ((this->x() != rhs.x()) || (this->y() != rhs.y()) || (this->z() != rhs.z()))
        {
            return false;
        }

        return true;
    }

    bool operator!=(const Point3d& rhs)
    {
        return !this->operator==(rhs);
    }

    void operator+=(const Vector3d& v)
    {
        point[0] += v[0];
        point[1] += v[1];
        point[2] += v[2];
    }

    void operator-=(const Vector3d& v)
    {
        point[0] -= v[0];
        point[1] -= v[1];
        point[2] -= v[2];
    }

  protected:
    double point[3];
};

inline Point3d operator+(Point3d p, const Vector3d& v)
{
    p += v;
    return p;
}

inline Point3d operator-(Point3d p, const Vector3d& v)
{
    p -= v;
    return p;
}

template <typename T>
inline Point3d operator*(Point3d p, const T scale)
{
    static_assert(std::is_pod<T>::value, "T must be POD");
    p *= scale;
    return p;
}

template <typename T>
inline Point3d operator*(const T scale, Point3d p)
{
    static_assert(std::is_pod<T>::value, "T must be POD");
    p *= scale;
    return p;
}

inline Vector3d operator-(Point3d p1, const Point3d& p2)
{
    return Vector3d(p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
}

inline std::ostream& operator<<(std::ostream& os, const Point3d& point)
{
    os << "x: " << point.x() << '\n' << "y: " << point.y() << '\n' << "z: " << point.z() << "\n";
    return os;
}

static inline Matrix3d toTildeForm(const Point3d& p)
{
    return Matrix3d(0., -p.z(), p.y(), p.z(), 0., -p.x(), -p.y(), p.x(), 0.);
}
typedef std::vector<Point3d, Eigen::aligned_allocator<Point3d>> Point3V;
}  // namespace Math
}  // namespace RobotDynamics
#endif  // ifndef __RDL_POINT_3_HPP__
