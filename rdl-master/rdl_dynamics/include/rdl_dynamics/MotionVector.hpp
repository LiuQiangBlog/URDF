// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_MOTION_VECTOR_HPP__
#define __RDL_MOTION_VECTOR_HPP__

/**
 * @file MotionVector.hpp
 * @brief Contains various geometric objects that have methods for transforming themselves into different frames of reference
 */

#include "rdl_dynamics/ForceVector.hpp"
#include "rdl_dynamics/rdl_eigenmath.hpp"

namespace RobotDynamics
{
namespace Math
{
class MotionVector : public SpatialVector, public TransformableGeometricObject
{
  public:
    /**
     * @brief Constructor
     * @param other
     */
    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    MotionVector(const Eigen::MatrixBase<OtherDerived>& other) : SpatialVector(other)
    {
    }

    MotionVector(const MotionVector& v) : SpatialVector(v)
    {
    }

    /**
     * @brief Overload equal operator
     * @param other
     * @return A motion vector
     */
    MotionVector& operator=(const MotionVector& other)
    {
        SpatialVector::operator=(other);
        return *this;
    }

    /**
     * @brief Empty constructor
     */
    EIGEN_STRONG_INLINE MotionVector() : SpatialVector(0., 0., 0., 0., 0., 0.)
    {
    }

    /**
     * @brief Constructor
     * @param v0 x-angular
     * @param v1 y-angular
     * @param v2 z-angular
     * @param v3 x-linear
     * @param v4 y-linear
     * @param v5 z-linear
     */
    MotionVector(const double v0, const double v1, const double v2, const double v3, const double v4, const double v5)
    {
        Base::_check_template_params();

        (*this) << v0, v1, v2, v3, v4, v5;
    }

    /**
     * @brief Get a copy of a MotionVector as a SpatialVector
     * @return SpatialVector copy of a MotionVectors
     */
    EIGEN_STRONG_INLINE SpatialVector toSpatialVector() const
    {
        return *this;
    }

    /**
     * @brief Setter
     * @param v Sets the values equal to those stored in v
     */
    EIGEN_STRONG_INLINE void set(const MotionVector& v)
    {
        (*this) << v[0], v[1], v[2], v[3], v[4], v[5];
    }

    /**
     * @brief Get a reference to the angular-x component
     * @return A reference to the angular x-component
     */
    EIGEN_STRONG_INLINE double& wx()
    {
        return this->operator[](0);
    }

    /**
     * @brief Get a reference to the angular-y component
     * @return A reference to the angular y-component
     */
    EIGEN_STRONG_INLINE double& wy()
    {
        return this->operator[](1);
    }

    /**
     * @brief Get a reference to the angular-z component
     * @return A copy reference to the angular z-component
     */
    EIGEN_STRONG_INLINE double& wz()
    {
        return this->operator[](2);
    }

    /**
     * @brief Get a copy of the angular-x component
     * @return A copy of the angular x-component
     */
    EIGEN_STRONG_INLINE double wx() const
    {
        return this->operator[](0);
    }

    /**
     * @brief Get a copy of the angular-y component
     * @return A copy of the angular y-component
     */
    EIGEN_STRONG_INLINE double wy() const
    {
        return this->operator[](1);
    }

    /**
     * @brief Get a copy of the angular-z component
     * @return A copy of the angular z-component
     */
    EIGEN_STRONG_INLINE double wz() const
    {
        return this->operator[](2);
    }

    /**
     * @brief Get a reference to the linear-x component
     * @return A reference to the linear x-component
     */
    EIGEN_STRONG_INLINE double& vx()
    {
        return this->operator[](3);
    }

    /**
     * @brief Get a reference to the linear-y component
     * @return A reference to the linear y-component
     */
    EIGEN_STRONG_INLINE double& vy()
    {
        return this->operator[](4);
    }

    /**
     * @brief Get a reference to the linear-z component
     * @return A reference to the linear z-component
     */
    EIGEN_STRONG_INLINE double& vz()
    {
        return this->operator[](5);
    }

    /**
     * @brief Get a copy of the linear-x component
     * @return A copy of the linear x-component
     */
    EIGEN_STRONG_INLINE double vx() const
    {
        return this->operator[](3);
    }

    /**
     * @brief Get a copy of the linear-y component
     * @return A copy of the linear y-component
     */
    EIGEN_STRONG_INLINE double vy() const
    {
        return this->operator[](4);
    }

    /**
     * @brief Get a copy of the linear-z component
     * @return A copy of the linear z-component
     */
    EIGEN_STRONG_INLINE double vz() const
    {
        return this->operator[](5);
    }

    /**
     * @brief Transforms a motion vector. Performs \f$ v= X v \f$
     * @param X
     */
    inline void transform(const SpatialTransform& X)
    {
        this->setLinearPart(-X.E * X.r.cross(this->getAngularPart()) + X.E * this->getLinearPart());
        this->setAngularPart(X.E * this->getAngularPart());
    }

    /**
     * @brief Copies, transforms, and returns a MotionVector. Performs \f$ v_2=X v_1 \f$
     * @param X
     * @return Copied, transformed MotionVector
     */
    MotionVector transform_copy(const SpatialTransform& X) const
    {
        MotionVector v = *this;
        v.transform(X);
        return v;
    }

    /**
     * @brief See V. Duindum thesis p.25 for an explanation of what \f$ad_T\f$ operator is.
     * It is also in Featherstone p. 25 eq. 2.31 & 2.32. For featherstone notation,
     * it is essentially the \f$\times\f$ operator for spatial vectors. Given two SpatialMotion vectors, \f$v_1\f$ and
     * \f$v_2\f$, this method returns \f$ v_3 = (v_1\times) v_2 \f$. Expanded, it looks like,
     ** \f[
     * v_3 = (v_1 \times) v_2 = ad_{v_1} v_2 =
     *  \begin{bmatrix}
     * \omega_{v_1} \times & \mathbf{0} \\
     * v_{v_1}\times & \omega_{v_1}\times
     *  \end{bmatrix} v_2
     * \f]
     * The 3d vector \f$\times\f$ operator is equivalent to the \f$\sim\f$ operator. See Math::toTildeForm.
     */
    MotionVector cross(const MotionVector& v);

    /**
     * @brief See Featherstone p. 25 eq. 2.31 & 2.32. For featherstone notation,
     * it is essentially the \f$\times *\f$ operator for spatial vectors. Given a SpatialMotion vector, \f$v_1\f$, and
     * SpatialForceVector, \f$f_1\f$, this method returns \f$ f_2 = (v_1\times *) f_1 \f$. Expanded, it looks like,
     * \f[
     *  =(v_1\times *) f_1 =
     *  \begin{bmatrix}
     * \omega_{v_1}\times & v_{v_1}\times \\
     * \mathbf{0} & \omega_{v_1}\times
     *  \end{bmatrix} f
     * \f]
     * The 3d vector \f$\times\f$ operator is equivalent to the \f$\sim\f$ operator. See Math::toTildeForm.
     */
    ForceVector cross(const ForceVector& v);

    /**
     * @brief Get the spatial motion cross matrix,
     * \f[
     *  m\times =
     *  \begin{bmatrix}
     *   \omega\times & \mathbf{0} \\
     *   v\times & \omega\times
     *  \end{bmatrix}
     * \f]
     * @return \f$ v\times \f$
     */
    inline SpatialMatrix crossm()
    {
        return SpatialMatrix(0, -this->operator[](2), this->operator[](1), 0, 0, 0, this->operator[](2), 0, -this->operator[](0), 0, 0, 0, -this->operator[](1),
                             this->operator[](0), 0, 0, 0, 0, 0, -this->operator[](5), this->operator[](4), 0, -this->operator[](2), this->operator[](1),
                             this->operator[](5), 0, -this->operator[](3), this->operator[](2), 0, -this->operator[](0), -this->operator[](4), this->operator[](3), 0,
                             -this->operator[](1), this->operator[](0), 0);
    }

    /**
     * @brief Get the spatial force cross matrix
     * @return \f$ v\times* \f$
     */
    inline SpatialMatrix crossf()
    {
        return SpatialMatrix(0, -this->operator[](2), this->operator[](1), 0, -this->operator[](5), this->operator[](4), this->operator[](2), 0, -this->operator[](0),
                             this->operator[](5), 0, -this->operator[](3), -this->operator[](1), this->operator[](0), 0, -this->operator[](4), this->operator[](3), 0, 0,
                             0, 0, 0, -this->operator[](2), this->operator[](1), 0, 0, 0, this->operator[](2), 0, -this->operator[](0), 0, 0, 0, -this->operator[](1),
                             this->operator[](0), 0);
    }

    /**
     * @brief Operator for performing the RBDA \f$\times\f$ operator for two motion vectors, i.e.
     **\f$m_2=(m_1\times)m_2\f$.
     * @param v
     * @return A MotionVector
     */
    inline MotionVector operator%=(const MotionVector& v)
    {
        return this->cross(v);
    }

    /**
     * @brief Operator for performing the RBDA \f$ \times * \f$ operator for a motion vector and a force vector, i.e.
     **\f$ f=(m\times *)f \f$.
     * @param v
     * @return A ForceVector
     */
    inline ForceVector operator%=(const ForceVector& v)
    {
        return this->cross(v);
    }

    /**
     * @brief Overloaded += operator for a MotionVector
     * @return MotionVector
     */
    inline MotionVector operator+=(const MotionVector& v)
    {
        (*this) << (this->wx() += v.wx()), (this->wy() += v.wy()), (this->wz() += v.wz()), (this->vx() += v.vx()), (this->vy() += v.vy()), (this->vz() += v.vz());
        return *this;
    }
};

/**
 * @brief Operator for transforming a MotionVector
 * @param X SpatialTransform to the desired frame
 * @param v
 * @return Transformed MotionVector
 */
inline MotionVector operator*(const SpatialTransform& X, MotionVector v)
{
    v.transform(X);
    return v;
}

/**
 * Operator for performing the spatial vector \f$\times\f$ operator.
 * @param v
 * @param v2
 * @return Returns \f$ ret = (v \times)v2 \f$
 */
inline MotionVector operator%(MotionVector v, const MotionVector& v2)
{
    return v.MotionVector::operator%=(v2);
}

/**
 * Operator for performing the spatial vector \f$\times*\f$ operator
 * @param v
 * @param v2
 * @return Returns \f$ ret = (v\times*)v2 \f$
 */
inline ForceVector operator%(MotionVector v, const ForceVector& v2)
{
    return v.MotionVector::operator%=(v2);
}
typedef std::vector<MotionVector, Eigen::aligned_allocator<MotionVector>> MotionVectorV;
}  // namespace Math
}  // namespace RobotDynamics

#endif
