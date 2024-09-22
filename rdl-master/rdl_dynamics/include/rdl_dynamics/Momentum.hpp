// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_MOMENTUM_HPP__
#define __RDL_MOMENTUM_HPP__

/**
 * @file Momentum.hpp
 */

#include "rdl_dynamics/ForceVector.hpp"
#include "rdl_dynamics/MotionVector.hpp"
#include "rdl_dynamics/RigidBodyInertia.hpp"

namespace RobotDynamics
{
namespace Math
{
/**
 * @class Momentum
 * @brief Momentum is mass/inertia multiplied by velocity
 */
class Momentum : public ForceVector
{
  public:
    Momentum() : ForceVector()
    {
    }

    Momentum(const double kx, const double ky, const double kz, const double lx, const double ly, const double lz) : ForceVector(kx, ky, kz, lx, ly, lz)
    {
    }

    Momentum(const Vector3d& k, const Vector3d l) : ForceVector(k.x(), k.y(), k.z(), l.x(), l.y(), l.z())
    {
    }

    Momentum(const Momentum& momentum) : ForceVector(momentum)
    {
    }

    explicit Momentum(const ForceVector& forceVector) : ForceVector(forceVector)
    {
    }

    Momentum& operator=(const Momentum& m)
    {
        SpatialVector::operator=(m);
        return *this;
    }

    Momentum(const RigidBodyInertia& inertia, const MotionVector& vector) : Momentum(computeMomentum(inertia, vector))
    {
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
    Momentum transform_copy(const SpatialTransform& X) const
    {
        Momentum m = *this;
        m.transform(X);
        return m;
    }

    EIGEN_STRONG_INLINE double& kx()
    {
        return this->operator[](0);
    }

    EIGEN_STRONG_INLINE double& ky()
    {
        return this->operator[](1);
    }

    EIGEN_STRONG_INLINE double& kz()
    {
        return this->operator[](2);
    }

    EIGEN_STRONG_INLINE double kx() const
    {
        return this->operator[](0);
    }

    EIGEN_STRONG_INLINE double ky() const
    {
        return this->operator[](1);
    }

    EIGEN_STRONG_INLINE double kz() const
    {
        return this->operator[](2);
    }

    EIGEN_STRONG_INLINE double& lx()
    {
        return this->operator[](3);
    }

    EIGEN_STRONG_INLINE double& ly()
    {
        return this->operator[](4);
    }

    EIGEN_STRONG_INLINE double& lz()
    {
        return this->operator[](5);
    }

    EIGEN_STRONG_INLINE double lx() const
    {
        return this->operator[](3);
    }

    EIGEN_STRONG_INLINE double ly() const
    {
        return this->operator[](4);
    }

    EIGEN_STRONG_INLINE double lz() const
    {
        return this->operator[](5);
    }

    /**
     * @brief Overloaded plus-equals operator
     * @return \f$ f_2=f_2+f_1 \f$
     */
    inline Momentum operator+=(const Momentum& v)
    {
        (*this) << (this->mx() += v.mx()), (this->my() += v.my()), (this->mz() += v.mz()), (this->fx() += v.fx()), (this->fy() += v.fy()), (this->fz() += v.fz());
        return *this;
    }

    /**
     * @brief Overloaded plus-equals operator
     * @return \f$ f_2=f_2-f_1 \f$
     */
    inline Momentum operator-=(const Momentum& v)
    {
        (*this) << (this->mx() -= v.mx()), (this->my() -= v.my()), (this->mz() -= v.mz()), (this->fx() -= v.fx()), (this->fy() -= v.fy()), (this->fz() -= v.fz());
        return *this;
    }

    /**
     * @brief Operator for computing kinetic energy. With momentum, \f$ m \f$ and Math::MotionVector, \f$ v \f$ this
     **performs performs \f$ \frac{m\cdot v}{2} \f$
     * @param vector
     * @return kinetic energy
     */
    EIGEN_STRONG_INLINE double operator*(const MotionVector& vector)
    {
        return this->dot(vector) * 0.5;
    }

    /**
     * @brief Computes momentum from inertia and velocity
     * @param I
     * @param v
     */
    static inline Momentum computeMomentum(const RigidBodyInertia& I, const MotionVector& v)
    {
        return Momentum(I.Ixx * v[0] + I.Iyx * v[1] + I.Izx * v[2] + I.h[1] * v[5] - I.h[2] * v[4],
                        I.Iyx * v[0] + I.Iyy * v[1] + I.Izy * v[2] - I.h[0] * v[5] + I.h[2] * v[3],
                        I.Izx * v[0] + I.Izy * v[1] + I.Izz * v[2] + I.h[0] * v[4] - I.h[1] * v[3], -I.h[1] * v[2] + I.h[2] * v[1] + I.m * v[3],
                        I.h[0] * v[2] - I.h[2] * v[0] + I.m * v[4], -I.h[0] * v[1] + I.h[1] * v[0] + I.m * v[5]);
    }
};

/**
 * @brief Multiplying a Math::RigidBodyInertia by a Math::MotionVector returns a Momentum
 * @param I
 * @param v
 * @return I*V
 */
static inline Momentum operator*(const RigidBodyInertia& I, const MotionVector& v)
{
    return Momentum::computeMomentum(I, v);
}

/**
 * @brief Add two momentums
 * @param m1
 * @param m2
 * @return \f$ m = m1+m2 /f$
 */
static inline Momentum operator+(Momentum m1, const Momentum& m2)
{
    m1 += m2;
    return m1;
}

/**
 * @brief subtract momentums two momentums
 * @param m1
 * @param m2
 * @return \f$ m = m1-m2 /f$
 */
static inline Momentum operator-(Momentum m1, const Momentum& m2)
{
    m1 -= m2;
    return m1;
}

}  // namespace Math
}  // namespace RobotDynamics

#endif  // ifndef __RDL_MOMENTUM_HPP__
