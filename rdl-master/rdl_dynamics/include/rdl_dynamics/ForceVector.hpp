// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_FORCE_VECTOR_HPP__
#define __RDL_FORCE_VECTOR_HPP__

/**
 * @file ForceVector.hpp
 * @brief Contains various geometric objects that have methods for transforming themselves into different frames of reference
 */
#include "rdl_dynamics/rdl_eigenmath.hpp"

namespace RobotDynamics
{
namespace Math
{
/**
 * @class ForceVector
 * @brief A ForceVector is a SpatialVector containing 3 moments and 3 linear forces.
 */
class ForceVector : public SpatialVector, public TransformableGeometricObject
{
  public:
    /**
     * @brief Constructor
     * @param other
     */
    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    ForceVector(const Eigen::MatrixBase<OtherDerived>& other) : SpatialVector(other)
    {
    }

    /**
     * Overloaded equal operator
     * @param other
     * @return Copied ForceVector
     */
    template <typename OtherDerived>
    ForceVector& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        SpatialVector::operator=(other);
        return *this;
    }

    /**
     * @brief Empty constructor
     */
    EIGEN_STRONG_INLINE ForceVector() : SpatialVector(0., 0., 0., 0., 0., 0.)
    {
    }

    /**
     * @brief Get a copy of a ForceVector as type SpatialVector
     */
    EIGEN_STRONG_INLINE SpatialVector toSpatialVector() const
    {
        return *this;
    }

    /**
     * @brief Constructor
     * @param mx x-Moment
     * @param my y-Moment
     * @param mz z-Moment
     * @param fx x-Linear Force
     * @param fy y-Linear Force
     * @param fz z-Linear Force
     */
    ForceVector(const double mx, const double my, const double mz, const double fx, const double fy, const double fz)
    {
        Base::_check_template_params();

        (*this) << mx, my, mz, fx, fy, fz;
    }

    /**
     * @brief Setter
     * @param f
     */
    EIGEN_STRONG_INLINE void set(const ForceVector& f)
    {
        (*this) << f[0], f[1], f[2], f[3], f[4], f[5];
    }

    /**
     * @brief Get reference to x-angular component
     * @return Reference to x-angular component
     */
    EIGEN_STRONG_INLINE double& mx()
    {
        return this->operator[](0);
    }

    /**
     * @brief Get reference to y-angular component
     * @return Reference to y-angular component
     */
    EIGEN_STRONG_INLINE double& my()
    {
        return this->operator[](1);
    }

    /**
     * @brief Get reference to z-angular component
     * @return Reference to z-angular component
     */
    EIGEN_STRONG_INLINE double& mz()
    {
        return this->operator[](2);
    }

    /**
     * @brief Get copy of x-angular component
     * @return Copy of x-angular component
     */
    EIGEN_STRONG_INLINE double mx() const
    {
        return this->operator[](0);
    }

    /**
     * @brief Get copy of y-angular component
     * @return Copy of y-angular component
     */
    EIGEN_STRONG_INLINE double my() const
    {
        return this->operator[](1);
    }

    /**
     * @brief Get copy of z-angular component
     * @return Copy of z-angular component
     */
    EIGEN_STRONG_INLINE double mz() const
    {
        return this->operator[](2);
    }

    /**
     * @brief Get reference to x-linear component
     * @return Reference to x-linear component
     */
    EIGEN_STRONG_INLINE double& fx()
    {
        return this->operator[](3);
    }

    /**
     * @brief Get reference to y-linear component
     * @return Reference to y-linear component
     */
    EIGEN_STRONG_INLINE double& fy()
    {
        return this->operator[](4);
    }

    /**
     * @brief Get reference to z-linear component
     * @return Reference to z-linear component
     */
    EIGEN_STRONG_INLINE double& fz()
    {
        return this->operator[](5);
    }

    /**
     * @brief Get copy of x-linear component
     * @return Copy of x-linear component
     */
    EIGEN_STRONG_INLINE double fx() const
    {
        return this->operator[](3);
    }

    /**
     * @brief Get copy of y-linear component
     * @return Copy of y-linear component
     */
    EIGEN_STRONG_INLINE double fy() const
    {
        return this->operator[](4);
    }

    /**
     * @brief Get copy of z-linear component
     * @return Copy of z-linear component
     */
    EIGEN_STRONG_INLINE double fz() const
    {
        return this->operator[](5);
    }

    /**
     * @brief Performs the following in place transform \f[
     * f =
     * \begin{bmatrix}
     * X.E & -X.E(X.r\times) \\
     * \mathbf{0} & X.E
     * \end{bmatrix}
     * f
     * \f]
     * @param X
     */
    void transform(const SpatialTransform& X)
    {
        this->setAngularPart(X.E * (this->getAngularPart() - X.r.cross(this->getLinearPart())));
        this->setLinearPart(X.E * this->getLinearPart());
    }

    /**
     * @brief Copy then transform a ForceVector by \f[
     * f_2 =
     * \begin{bmatrix}
     * X.E & -X.E(X.r\times) \\
     * \mathbf{0} & X.E
     * \end{bmatrix}
     * f_1
     * \f]
     * @param X
     * @return Copied, transformed ForceVector
     */
    ForceVector transform_copy(const SpatialTransform& X) const
    {
        ForceVector f = *this;
        f.transform(X);
        return f;
    }

    /**
     * @brief Overloaded plus-equals operator
     * @return \f$ f_2=f_2+f_1 \f$
     */
    inline ForceVector operator+=(const ForceVector& v)
    {
        (*this) << (this->mx() += v.mx()), (this->my() += v.my()), (this->mz() += v.mz()), (this->fx() += v.fx()), (this->fy() += v.fy()), (this->fz() += v.fz());
        return *this;
    }
};

/**
 * @brief Operator for transforming a ForceVector. Calls the ForceVector::transform method.
 * @param X SpatialTransform
 * @param f ForceVector to be transformed
 * @return Transformed ForceVector
 */
inline ForceVector operator*(const SpatialTransform& X, ForceVector f)
{
    f.transform(X);
    return f;
}
typedef std::vector<ForceVector, Eigen::aligned_allocator<ForceVector>> ForceVectorV;
}  // namespace Math
}  // namespace RobotDynamics

#endif
