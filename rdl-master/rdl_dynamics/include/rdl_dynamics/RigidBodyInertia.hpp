// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_RIGID_BODY_INERTIA_HPP__
#define __RDL_RIGID_BODY_INERTIA_HPP__

/**
 * @file RigidBodyInertia.hpp
 * @brief See V. Duindum p39-40 & Featherstone p32-33
 */

#include "rdl_dynamics/rdl_eigenmath.hpp"

namespace RobotDynamics
{
namespace Math
{
/**
 * @class RigidBodyInertia
 * @brief This class stores a bodies mass, center of mass, and inertia information. The inertia elements are
 * stored individually since the inertia matrix is a 3x3 symmetric matrix. The bodies inertia matrix, expressed
 * about its center of mass, can be reconstructed as
 * \f[
 * I_c =
 * \begin{bmatrix}
 * I_xx & I_yx & I_zx \\
 * I_yx & I_yy & I_zy \\
 * I_zx & I_zy & I_zz
 * \end{bmatrix}
 * \f]
 * The full RigidBodyInertia matrix has the following structure,
 * \f[
 * \begin{bmatrix}
 * I_c + (h\times)h & h\times \\
 * -h\times & \mathbf{1}_{3\times3}m
 * \end{bmatrix}
 * \f]
 * where \f$ \mathbf{1}_{3\times 3} \f$ is a 3x3 identity matrix
 */
class RigidBodyInertia : public TransformableGeometricObject
{
  public:
    /**
     * @brief Constructor
     */
    RigidBodyInertia() : m(0.), h(Vector3d::Zero(3, 1)), Ixx(0.), Iyx(0.), Iyy(0.), Izx(0.), Izy(0.), Izz(0.)
    {
    }

    /**
     * @brief Constructor
     * @param mass Body mass
     * @param com_mass Vector pointing to center of mass scaled by the body mass
     * @param inertia 3x3 Inertia tensor about the bodies center of mass
     */
    RigidBodyInertia(double mass, const Vector3d& com_mass, const Matrix3d& inertia)
      : m(mass), h(com_mass), Ixx(inertia(0, 0)), Iyx(inertia(1, 0)), Iyy(inertia(1, 1)), Izx(inertia(2, 0)), Izy(inertia(2, 1)), Izz(inertia(2, 2))
    {
    }

    /**
     * @brief Constructor
     * @param m Body mass
     * @param h Vector pointing to center of mass scaled by the body mass
     * @param Ixx
     * @param Iyx
     * @param Iyy
     * @param Izx
     * @param Izy
     * @param Izz
     */
    RigidBodyInertia(double m, const Vector3d& h, const double Ixx, const double Iyx, const double Iyy, const double Izx, const double Izy, const double Izz)
      : m(m), h(h), Ixx(Ixx), Iyx(Iyx), Iyy(Iyy), Izx(Izx), Izy(Izy), Izz(Izz)
    {
    }

    /**
     * @brief Copy constructor
     * @param inertia
     */
    RigidBodyInertia(const RigidBodyInertia& inertia)
      : m(inertia.m), h(inertia.h), Ixx(inertia.Ixx), Iyx(inertia.Iyx), Iyy(inertia.Iyy), Izx(inertia.Izx), Izy(inertia.Izy), Izz(inertia.Izz)
    {
    }

    /**
     * @brief Setter
     * @param I
     */
    inline void set(const RigidBodyInertia& I)
    {
        this->set(I.m, I.h, I.Ixx, I.Iyx, I.Iyy, I.Izx, I.Izy, I.Izz);
    }

    /**
     * @brief Setter
     * @param m Body mass
     * @param h Vector pointing to center of mass scaled by the body mass
     * @param Ixx
     * @param Iyx
     * @param Iyy
     * @param Izx
     * @param Izy
     * @param Izz
     */
    void set(double m, const Vector3d& h, const double Ixx, const double Iyx, const double Iyy, const double Izx, const double Izy, const double Izz)
    {
        this->m = m;
        this->h = h;
        this->Ixx = Ixx;
        this->Iyx = Iyx;
        this->Iyy = Iyy;
        this->Izx = Izx;
        this->Izy = Izy;
        this->Izz = Izz;
    }

    /**
     * @brief Overloaded plus-equals operator. Adds two inertia matrices
     * @param rbi
     */
    void operator+=(const RigidBodyInertia& rbi);

    /**
     * @brief Transform a Math::RigidBodyInertia matrix
     * @param X
     *
     */
    void transform(const SpatialTransform& X);

    /**
     * @brief Copy, transform, and return a Math::RigidBodyInertia
     * @param X
     * @return Returns a copied and transformed Math::RigidBodyInertia
     */
    inline RigidBodyInertia transform_copy(const SpatialTransform& X) const
    {
        RigidBodyInertia I = *this;
        I.transform(X);
        return I;
    }

    /**
     * @brief Create a Math::RigidBodyInertia object from a 6x6 Math::SpatialMatrix
     * @param Ic
     */
    void createFromMatrix(const SpatialMatrix& Ic);

    /**
     * Get a Math::RigidBodyInertia as a 6x6 Math::SpatialMatrix
     * @return A 6x6 matrix representing a Math::SpatialInertia matrix
     */
    SpatialMatrix toMatrix() const;

    /**
     * @brief Store a Math::RigidBodyInertia in the Math::SpatialMatrix
     * @param mat Modified to store a Math::RigidBodyInertia in the Math::SpatialMatrix
     */
    void setSpatialMatrix(SpatialMatrix& mat) const;

    /**
     * @brief Given Math::RigidBodyInertia \f$ I_r \f$ ad Math::SpatialMatrix \f$ M_I \f$, returns Math::SpatialMatrix
     **\f$ M_r \f$
     * such that \f$ M_r = I_r - M_I \f$
     * @param m
     * @return New Math::SpatialMatrix that is the Math::SpatialMatrix subtracted from a Math::RigidBodyInertia
     */
    inline SpatialMatrix subtractSpatialMatrix(const SpatialMatrix& m) const
    {
        return SpatialMatrix(Ixx - m(0, 0), Iyx - m(0, 1), Izx - m(0, 2), -m(0, 3), -h[2] - m(0, 4), h[1] - m(0, 5), Iyx - m(1, 0), Iyy - m(1, 1), Izy - m(1, 2),
                             h[2] - m(1, 3), -m(1, 4), -h[0] - m(1, 5), Izx - m(2, 0), Izy - m(2, 1), Izz - m(2, 2), -h[1] - m(2, 3), h[0] - m(2, 4), -m(2, 5), -m(3, 0),
                             h[2] - m(3, 1), -h[1] - m(3, 2), this->m - m(3, 3), -m(3, 4), -m(3, 5), -h[2] - m(4, 0), -m(4, 1), h[0] - m(4, 2), -m(4, 3),
                             this->m - m(4, 4), -m(4, 5), h[1] - m(5, 0), -h[0] - m(5, 1), -m(5, 2), -m(5, 3), -m(5, 4), this->m - m(5, 5));
    }

    /**
     * @brief Multiply a Math::RigidBodyInertia by a Math::SpatialVector and return the result as a new
     **Math::SpatialVector
     * @param v
     * @return The result of a Math::RigidBodyInertia multiplied by a Math::SpatialVector
     */
    inline SpatialVector timesSpatialVector(const SpatialVector& v) const
    {
        return SpatialVector(Ixx * v[0] + Iyx * v[1] + Izx * v[2] + h[1] * v[5] - h[2] * v[4], Iyx * v[0] + Iyy * v[1] + Izy * v[2] - h[0] * v[5] + h[2] * v[3],
                             Izx * v[0] + Izy * v[1] + Izz * v[2] + h[0] * v[4] - h[1] * v[3], -h[1] * v[2] + h[2] * v[1] + m * v[3],
                             h[0] * v[2] - h[2] * v[0] + m * v[4], -h[0] * v[1] + h[1] * v[0] + m * v[5]);
    }

    /**
     * @brief A helper method that returns a 6x3 matrix that is a Math::RigidBodyInertia multiplied by a 6x3 matrix
     * @param m
     * @return The result of multiplying a Math::RigidBodyInertia by a 6x3 matrix
     */
    Matrix63 multiplyMatrix63(Matrix63 m) const
    {
        m.col(0) = this->timesSpatialVector(m.col(0));
        m.col(1) = this->timesSpatialVector(m.col(1));
        m.col(2) = this->timesSpatialVector(m.col(2));
        return m;
    }

    RigidBodyInertia& operator=(const RigidBodyInertia& I)
    {
        m = I.m;
        h = I.h;
        Ixx = I.Ixx;
        Iyx = I.Iyx;
        Iyy = I.Iyy;
        Izx = I.Izx;
        Izy = I.Izy;
        Izz = I.Izz;
        return *this;
    }

    double m; /**< Mass */

    Vector3d h; /**< Vector pointing to body center of mass in body frame, scaled by body mass */

    double Ixx; /**< Element in body's inertia matrix */
    double Iyx; /**< Element in body's inertia matrix */
    double Iyy; /**< Element in body's inertia matrix */
    double Izx; /**< Element in body's inertia matrix */
    double Izy; /**< Element in body's inertia matrix */
    double Izz; /**< Element in body's inertia matrix */
};

/**
 * @brief Add two Math::RigidBodyInertia objects together
 * @param rbi_in A copy that is modified and returned.
 * @param rbi
 * @return A Math::RigidBodyInertia that is the addition of the two arguments
 */
inline RigidBodyInertia operator+(RigidBodyInertia rbi_in, const RigidBodyInertia& rbi)
{
    rbi_in += rbi;
    return rbi_in;
}

/**
 * @brief Operator for multiplying a Math::RigidBodyInertia by a Math::SpatialVector
 * @param I
 * @param v
 * @return Math::RigidBodyInertia times Math::SpatialVector
 */
inline SpatialVector operator*(const RigidBodyInertia& I, const SpatialVector& v)
{
    return I.timesSpatialVector(v);
}

/**
 * @brief Operator for multiplying a Math::RigidBodyInertia by a Math::Matrix63
 * @param I
 * @param m
 * @return The result of multiplying a Math::RigidBodyInertia by a Math::Matrix63
 */
inline Matrix63 operator*(const RigidBodyInertia& I, const Matrix63& m)
{
    return I.multiplyMatrix63(m);
}

/**
 * Creates a RigidBodyInertia from a body's  mass, center of mass location, and 3x3 inertia matrix
 * @param mass Body mass
 * @param com Center of mass location in body frame
 * @param inertia_C 3x3 inertia tensor about from located at center of mass and aligned with body frame
 * @return RigidBodyInertia
 */
static inline RigidBodyInertia createFromMassComInertiaC(double mass, const Vector3d& com, const Matrix3d& inertia_C)
{
    RigidBodyInertia result;

    result.m = mass;
    result.h = com * mass;
    Matrix3d I = inertia_C + com.toTildeForm() * com.toTildeForm().transpose() * mass;
    result.Ixx = I(0, 0);
    result.Iyx = I(1, 0);
    result.Iyy = I(1, 1);
    result.Izx = I(2, 0);
    result.Izy = I(2, 1);
    result.Izz = I(2, 2);

    return RigidBodyInertia(mass, com * mass, inertia_C + com.toTildeForm() * com.toTildeForm().transpose() * mass);
}
typedef std::vector<RigidBodyInertia, Eigen::aligned_allocator<RigidBodyInertia>> RigidBodyInertiaV;
}  // namespace Math
}  // namespace RobotDynamics

#endif  //__SPATIAL_INERTIA_HPP__
