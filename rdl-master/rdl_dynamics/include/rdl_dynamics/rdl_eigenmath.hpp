// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef __RDL_EIGENMATH_H__
#define __RDL_EIGENMATH_H__

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/QR>
#include <eigen3/Eigen/Eigen>

namespace RobotDynamics
{
/** \brief Math types such as vectors and matrices and utility functions. */
namespace Math
{
typedef Eigen::Matrix<double, 6, 3> Matrix63;
typedef Eigen::VectorXd VectorNd;
typedef Eigen::MatrixXd MatrixNd;
typedef Eigen::AngleAxisd AxisAngle;
}  // namespace Math
}  // namespace RobotDynamics

namespace RobotDynamics
{
namespace Math
{
struct SpatialTransform;
class Quaternion;
class Matrix3d;
class Vector3d;
class SpatialMatrix;

/**
 * @class TransformableGeometricObject
 * @brief The TransformableGeometricObject class is an essential interface because it forces all geometric objects
 * to implement a method that tells how to transform them. This makes in possible for frame transformations of any
 * TransformableGeometricObject can be done via the FrameObject::changeFrame method.
 */
class TransformableGeometricObject
{
  public:
    /**
     * @brief Pure virtual object. This object forces objects that inherit from it to have a method that tells
     * how that object is transformed.
     * @param X SpatialTransform
     */
    virtual void transform(const RobotDynamics::Math::SpatialTransform& X) = 0;
};

class Vector3d : public Eigen::Vector3d, public TransformableGeometricObject
{
  public:
    typedef Eigen::Vector3d Base;

    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    Vector3d(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Vector3d(other)
    {
    }

    template <typename OtherDerived>
    Vector3d& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    EIGEN_STRONG_INLINE Vector3d() : Vector3d(0., 0., 0.)
    {
    }

    EIGEN_STRONG_INLINE Vector3d(const double& v0, const double& v1, const double& v2)
    {
        Base::_check_template_params();

        (*this) << v0, v1, v2;
    }

    void set(const Eigen::Vector3d& v)
    {
        Base::_check_template_params();

        set(v[0], v[1], v[2]);
    }

    void set(const double& v0, const double& v1, const double& v2)
    {
        Base::_check_template_params();

        (*this) << v0, v1, v2;
    }

    inline void transform(const RobotDynamics::Math::SpatialTransform& X);

    inline Vector3d transform_copy(const RobotDynamics::Math::SpatialTransform& X) const;

    Matrix3d toTildeForm() const;
};

class Matrix3d : public Eigen::Matrix3d
{
  public:
    typedef Eigen::Matrix3d Base;

    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    Matrix3d(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix3d(other)
    {
    }

    Matrix3d(const Quaternion& o);

    Matrix3d(const AxisAngle& a) : Eigen::Matrix3d(a)
    {
    }

    template <typename OtherDerived>
    Matrix3d& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    EIGEN_STRONG_INLINE Matrix3d()
    {
    }

    EIGEN_STRONG_INLINE Matrix3d(const double& m00, const double& m01, const double& m02, const double& m10, const double& m11, const double& m12, const double& m20,
                                 const double& m21, const double& m22)
    {
        Base::_check_template_params();

        (*this) << m00, m01, m02, m10, m11, m12, m20, m21, m22;
    }

    /**
     * @brief Convert rotation matrix to intrinsic ZYX euler angles
     * @param m Rotation matrix to convert
     * @param yaw_at_pitch_singularity At Y = +- PI/2 is a singularity in which there are multiple solutions. This will be the yaw value in the output and the
     * roll value is dependent on this. To get the most accurate results at singularity, provide this value as close as possible to desired/reality and the resulting
     * roll value will be near the expected
     */
    Vector3d toIntrinsicZYXAngles(double yaw_at_pitch_singularity = 0.) const;
};

class Vector4d : public Eigen::Vector4d
{
  public:
    typedef Eigen::Vector4d Base;

    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    Vector4d(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Vector4d(other)
    {
    }

    template <typename OtherDerived>
    Vector4d& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    EIGEN_STRONG_INLINE Vector4d()
    {
    }

    EIGEN_STRONG_INLINE Vector4d(const double& v0, const double& v1, const double& v2, const double& v3)
    {
        Base::_check_template_params();

        (*this) << v0, v1, v2, v3;
    }

    void set(const double& v0, const double& v1, const double& v2, const double& v3)
    {
        Base::_check_template_params();

        (*this) << v0, v1, v2, v3;
    }
};

class SpatialVector : public Eigen::Matrix<double, 6, 1>
{
  public:
    typedef Eigen::Matrix<double, 6, 1> Base;

    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    SpatialVector(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, 6, 1>(other)
    {
    }

    template <typename OtherDerived>
    SpatialVector& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    EIGEN_STRONG_INLINE SpatialVector()
    {
        (*this) << 0., 0., 0., 0., 0., 0.;
    }

    EIGEN_STRONG_INLINE SpatialVector(const double& v0, const double& v1, const double& v2, const double& v3, const double& v4, const double& v5)
    {
        Base::_check_template_params();

        (*this) << v0, v1, v2, v3, v4, v5;
    }

    EIGEN_STRONG_INLINE SpatialVector(const Vector3d& angularPart, const Vector3d& linearPart)
    {
        Base::_check_template_params();

        (*this) << angularPart[0], angularPart[1], angularPart[2], linearPart[0], linearPart[1], linearPart[2];
    }

    EIGEN_STRONG_INLINE void set(const double& v0, const double& v1, const double& v2, const double& v3, const double& v4, const double& v5)
    {
        Base::_check_template_params();

        (*this) << v0, v1, v2, v3, v4, v5;
    }

    EIGEN_STRONG_INLINE Vector3d getAngularPart() const
    {
        return Vector3d(this->operator[](0), this->operator[](1), this->operator[](2));
    }

    EIGEN_STRONG_INLINE Vector3d getLinearPart() const
    {
        return Vector3d(this->operator[](3), this->operator[](4), this->operator[](5));
    }

    void zeroAngularPart()
    {
        this->operator[](0) = 0.;
        this->operator[](1) = 0.;
        this->operator[](2) = 0.;
    }

    void zeroLinearPart()
    {
        this->operator[](3) = 0.;
        this->operator[](4) = 0.;
        this->operator[](5) = 0.;
    }

    inline void setAngularPart(const Vector3d& v)
    {
        setAngularPart(v(0), v(1), v(2));
    }

    inline void setAngularPart(double x, double y, double z)
    {
        this->operator[](0) = x;
        this->operator[](1) = y;
        this->operator[](2) = z;
    }

    inline void setLinearPart(const Vector3d& v)
    {
        setLinearPart(v(0), v(1), v(2));
    }

    inline void setLinearPart(double x, double y, double z)
    {
        this->operator[](3) = x;
        this->operator[](4) = y;
        this->operator[](5) = z;
    }

    EIGEN_STRONG_INLINE void set(const Vector3d& angularPart, const Vector3d& linearPart)
    {
        Base::_check_template_params();

        (*this) << angularPart[0], angularPart[1], angularPart[2], linearPart[0], linearPart[1], linearPart[2];
    }

    /**
     * @brief Get the spatial motion cross matrix
     * @param v
     * @return \f$ v\times \f$
     */
    SpatialMatrix crossm();
    /**
     * @brief Spatial motion cross times spatial motion
     * @param v1
     * @param v2
     * @return \f$ this \times v \f$
     */
    SpatialVector crossm(const SpatialVector& v);
    /**
     * @brief Get the spatial force cross matrix
     * @param v
     * @return \f$ v\times* \f$
     */
    SpatialMatrix crossf();
    /**
     * @brief Spatial motion cross spatial force
     * @param v1 Spatial motion
     * @param v2 Spatial force
     * @return \f$ this \times* v \f$
     */
    SpatialVector crossf(const SpatialVector& v);
};

class Matrix4d : public Eigen::Matrix<double, 4, 4>
{
  public:
    typedef Eigen::Matrix<double, 4, 4> Base;

    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    Matrix4d(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, 4, 4>(other)
    {
    }

    template <typename OtherDerived>
    Matrix4d& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    EIGEN_STRONG_INLINE Matrix4d()
    {
    }

    EIGEN_STRONG_INLINE Matrix4d(const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m10, const Scalar& m11, const Scalar& m12,
                                 const Scalar& m13, const Scalar& m20, const Scalar& m21, const Scalar& m22, const Scalar& m23, const Scalar& m30, const Scalar& m31,
                                 const Scalar& m32, const Scalar& m33)
    {
        Base::_check_template_params();

        (*this) << m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33;
    }

    void set(const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m10, const Scalar& m11, const Scalar& m12, const Scalar& m13,
             const Scalar& m20, const Scalar& m21, const Scalar& m22, const Scalar& m23, const Scalar& m30, const Scalar& m31, const Scalar& m32, const Scalar& m33)
    {
        Base::_check_template_params();

        (*this) << m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33;
    }
};

class SpatialMatrix : public Eigen::Matrix<double, 6, 6>
{
  public:
    typedef Eigen::Matrix<double, 6, 6> Base;

    template <typename OtherDerived>
    // cppcheck-suppress noExplicitConstructor
    SpatialMatrix(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Matrix<double, 6, 6>(other)
    {
    }

    template <typename OtherDerived>
    SpatialMatrix& operator=(const Eigen::MatrixBase<OtherDerived>& other)
    {
        this->Base::operator=(other);
        return *this;
    }

    EIGEN_STRONG_INLINE SpatialMatrix()
    {
        Base::_check_template_params();

        (*this) << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    }

    EIGEN_STRONG_INLINE SpatialMatrix(const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m04, const Scalar& m05, const Scalar& m10,
                                      const Scalar& m11, const Scalar& m12, const Scalar& m13, const Scalar& m14, const Scalar& m15, const Scalar& m20, const Scalar& m21,
                                      const Scalar& m22, const Scalar& m23, const Scalar& m24, const Scalar& m25, const Scalar& m30, const Scalar& m31, const Scalar& m32,
                                      const Scalar& m33, const Scalar& m34, const Scalar& m35, const Scalar& m40, const Scalar& m41, const Scalar& m42, const Scalar& m43,
                                      const Scalar& m44, const Scalar& m45, const Scalar& m50, const Scalar& m51, const Scalar& m52, const Scalar& m53, const Scalar& m54,
                                      const Scalar& m55)
    {
        Base::_check_template_params();

        (*this) << m00, m01, m02, m03, m04, m05, m10, m11, m12, m13, m14, m15, m20, m21, m22, m23, m24, m25, m30, m31, m32, m33, m34, m35, m40, m41, m42, m43, m44, m45,
            m50, m51, m52, m53, m54, m55;
    }

    void set(const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m04, const Scalar& m05, const Scalar& m10, const Scalar& m11,
             const Scalar& m12, const Scalar& m13, const Scalar& m14, const Scalar& m15, const Scalar& m20, const Scalar& m21, const Scalar& m22, const Scalar& m23,
             const Scalar& m24, const Scalar& m25, const Scalar& m30, const Scalar& m31, const Scalar& m32, const Scalar& m33, const Scalar& m34, const Scalar& m35,
             const Scalar& m40, const Scalar& m41, const Scalar& m42, const Scalar& m43, const Scalar& m44, const Scalar& m45, const Scalar& m50, const Scalar& m51,
             const Scalar& m52, const Scalar& m53, const Scalar& m54, const Scalar& m55)
    {
        Base::_check_template_params();

        (*this) << m00, m01, m02, m03, m04, m05, m10, m11, m12, m13, m14, m15, m20, m21, m22, m23, m24, m25, m30, m31, m32, m33, m34, m35, m40, m41, m42, m43, m44, m45,
            m50, m51, m52, m53, m54, m55;
    }
};

/** \brief Compact representation of spatial transformations.
 *
 * Instead of using a verbose 6x6 matrix, this structure only stores a 3x3
 * matrix and a 3-d vector to store spatial transformations. It also
 * encapsulates efficient operations such as concatenations and
 * transformation of spatial vectors.
 */
struct SpatialTransform
{
    /**
     * @brief Constructor
     */
    SpatialTransform() : E(Matrix3d::Identity(3, 3)), r(Vector3d::Zero(3, 1))
    {
    }

    /**
     * @brief Constructor
     * @param rotation Orthogonal rotation matrix
     * @param x X component
     * @param y Y component
     * @param z Z component
     */
    SpatialTransform(const Matrix3d& rotation, const double x, const double y, const double z) : E(rotation), r(x, y, z)
    {
    }

    SpatialTransform(const Quaternion& o, double x, double y, double z) : E(o), r(x, y, z)
    {
    }

    SpatialTransform(double x, double y, double z, double w);

    SpatialTransform(const Quaternion& o) : E(o), r(0., 0., 0.)
    {
    }

    /**
     * @brief Constructor
     * @param rotation Orthogonal rotation matrix
     * @param translation 3D translational component
     */
    SpatialTransform(const Matrix3d& rotation, const Vector3d& translation) : E(rotation), r(translation)
    {
    }

    /**
     * @brief Constructor
     * @param rotation Orthogonal rotation matrix
     */
    SpatialTransform(const Matrix3d& rotation) : E(rotation), r(0., 0., 0.)
    {
    }

    /**
     * @brief Constructor
     * @param translation 3D translational component
     */
    SpatialTransform(const Vector3d& translation) : E(1., 0., 0., 0., 1., 0., 0., 0., 1.), r(translation)
    {
    }

    /**
     * @brief Transform a spatial vector. Same as \f$ X * v \f$
     * @param v_sp Spatial motion vector to be copied/transformed
     *
     * @returns Transformed spatial vector. \f$ \begin{bmatrix} E * w \\ - E * (r \times w) + E * v \end{bmatrix} \f$
     */
    SpatialVector apply(const SpatialVector& v_sp) const
    {
        Vector3d v_rxw(v_sp[3] - r[1] * v_sp[2] + r[2] * v_sp[1], v_sp[4] - r[2] * v_sp[0] + r[0] * v_sp[2], v_sp[5] - r[0] * v_sp[1] + r[1] * v_sp[0]);

        return SpatialVector(E(0, 0) * v_sp[0] + E(0, 1) * v_sp[1] + E(0, 2) * v_sp[2], E(1, 0) * v_sp[0] + E(1, 1) * v_sp[1] + E(1, 2) * v_sp[2],
                             E(2, 0) * v_sp[0] + E(2, 1) * v_sp[1] + E(2, 2) * v_sp[2], E(0, 0) * v_rxw[0] + E(0, 1) * v_rxw[1] + E(0, 2) * v_rxw[2],
                             E(1, 0) * v_rxw[0] + E(1, 1) * v_rxw[1] + E(1, 2) * v_rxw[2], E(2, 0) * v_rxw[0] + E(2, 1) * v_rxw[1] + E(2, 2) * v_rxw[2]);
    }

    /**
     * @brief Applies \f$ X^T * f \f$
     * @param f_sp Spatial force
     *
     * @returns \f$ \begin{bmatrix} E^T * n + r \times * E^T * f \\ E^T * f \end{bmatrix} \f$
     */
    SpatialVector applyTranspose(const SpatialVector& f_sp) const
    {
        Vector3d E_T_f(E(0, 0) * f_sp[3] + E(1, 0) * f_sp[4] + E(2, 0) * f_sp[5], E(0, 1) * f_sp[3] + E(1, 1) * f_sp[4] + E(2, 1) * f_sp[5],
                       E(0, 2) * f_sp[3] + E(1, 2) * f_sp[4] + E(2, 2) * f_sp[5]);

        return SpatialVector(E(0, 0) * f_sp[0] + E(1, 0) * f_sp[1] + E(2, 0) * f_sp[2] - r[2] * E_T_f[1] + r[1] * E_T_f[2],
                             E(0, 1) * f_sp[0] + E(1, 1) * f_sp[1] + E(2, 1) * f_sp[2] + r[2] * E_T_f[0] - r[0] * E_T_f[2],
                             E(0, 2) * f_sp[0] + E(1, 2) * f_sp[1] + E(2, 2) * f_sp[2] - r[1] * E_T_f[0] + r[0] * E_T_f[1], E_T_f[0], E_T_f[1], E_T_f[2]);
    }

    /**
     * @brief Applies \f$ X * f \f$ where \f$ f \f$ is a spatial force
     * @param f_sp Spatial force vector
     *
     * @return \f$ \begin{bmatrix} E * n - E * (r \times f) \\ E * f \end{bmatrix} \f$
     */
    SpatialVector applyAdjoint(const SpatialVector& f_sp) const
    {
        return SpatialVector(E * (f_sp.getAngularPart() - r.cross(f_sp.getLinearPart())), E * f_sp.getLinearPart());
    }

    /**
     * @brief Return transform as 6x6 spatial matrix
     *
     * @return \f$ \begin{bmatrix} E & \mathbf{0} \\ -E * r\times & E \end{bmatrix} \f$
     */
    SpatialMatrix toMatrix() const
    {
        SpatialMatrix result;

        result.block<3, 3>(0, 0) = E;
        result.block<3, 3>(0, 3) = Matrix3d::Zero(3, 3);
        result.block<3, 3>(3, 0) = -E * r.toTildeForm();
        result.block<3, 3>(3, 3) = E;

        return result;
    }

    /**
     * @brief Returns Spatial transform that transforms spatial force vectors
     *
     * @return \f$ \begin{bmatrix} E & -E * r\times \\ \mathbf{0} & E \end{bmatrix} \f$
     */
    SpatialMatrix toMatrixAdjoint() const
    {
        SpatialMatrix result;

        result.block<3, 3>(0, 0) = E;
        result.block<3, 3>(0, 3) = -E * r.toTildeForm();
        result.block<3, 3>(3, 0) = Matrix3d::Zero(3, 3);
        result.block<3, 3>(3, 3) = E;

        return result;
    }

    /**
     * @brief Returns spatial force transform transposed
     *
     * @return \f$ \begin{bmatrix} E^{T} & (-E r\times)^{T} \\ \mathbf{0} & E^{T} \end{bmatrix} \f$
     */
    SpatialMatrix toMatrixTranspose() const
    {
        Matrix3d _Erx = E * Matrix3d(0., -r[2], r[1], r[2], 0., -r[0], -r[1], r[0], 0.);
        SpatialMatrix result;

        result.block<3, 3>(0, 0) = E.transpose();
        result.block<3, 3>(0, 3) = -_Erx.transpose();
        result.block<3, 3>(3, 0) = Matrix3d::Zero(3, 3);
        result.block<3, 3>(3, 3) = E.transpose();

        return result;
    }

    /**
     * @brief Returns inverse of transform
     *
     * @return \f$ X^{-1} \f$
     */
    SpatialTransform inverse() const
    {
        return SpatialTransform(E.transpose(), -E * r);
    }

    /**
     * @brief Inverts in place. \f$ this = this^{-1} \f$
     */
    void invert()
    {
        r = -E * r;
        E.transposeInPlace();
    }

    /**
     * @brief Overloaded * operator for combining transforms
     * @param XT
     * @return Combined rotation
     */
    SpatialTransform operator*(const SpatialTransform& XT) const
    {
        return SpatialTransform(E * XT.E, XT.r + XT.E.transpose() * r);
    }

    void operator*=(const SpatialTransform& XT)
    {
        r = XT.r + XT.E.transpose() * r;
        E *= XT.E;
    }

    Matrix3d E;
    Vector3d r;
};

Vector3d Vector3d::transform_copy(const RobotDynamics::Math::SpatialTransform& X) const
{
    return X.E * (*this);
}

void Vector3d::transform(const RobotDynamics::Math::SpatialTransform& X)
{
    *this = X.E * (*this);
}

/** \brief Quaternion that are used for \ref joint_singularities "singularity free" joints.
 *
 * order: x,y,z,w
 */
class Quaternion : public Vector4d, public TransformableGeometricObject
{
  public:
    /**
     * @brief Constructor
     */
    Quaternion() : Vector4d(0., 0., 0., 1.)
    {
    }

    // cppcheck-suppress noExplicitConstructor
    Quaternion(const Eigen::Quaterniond& q) : Vector4d(q.x(), q.y(), q.z(), q.w())
    {
    }

    // cppcheck-suppress noExplicitConstructor
    Quaternion(const RobotDynamics::Math::Quaternion& q) : Vector4d(q.x(), q.y(), q.z(), q.w())
    {
    }

    // cppcheck-suppress noExplicitConstructor
    Quaternion(const Eigen::Product<Eigen::Matrix<double, 3, 3>, Eigen::Matrix<double, 3, 3>, 0> E) : Quaternion(Matrix3d(E))
    {
    }

    // cppcheck-suppress noExplicitConstructor
    Quaternion(const Eigen::Matrix3d& E);

    // cppcheck-suppress noExplicitConstructor
    Quaternion(const Eigen::Matrix3d::ConstTransposeReturnType& E) : Quaternion(Eigen::Matrix3d(E))
    {
    }

    Quaternion(Eigen::Transpose<Eigen::Matrix<double, 3, 3> > E) : Quaternion(Eigen::Matrix3d(E))
    {
    }

    Quaternion(const Vector4d& v) : Vector4d(v[0], v[1], v[2], v[3])
    {
    }

    // cppcheck-suppress noExplicitConstructor
    Quaternion(const AxisAngle& axis_angle) : Quaternion(axis_angle.axis(), axis_angle.angle())
    {
    }

    Quaternion(Vector3d axis, double angle)
    {
        double d = axis.norm();
        double s2 = std::sin(angle * 0.5) / d;
        (*this) << axis[0] * s2, axis[1] * s2, axis[2] * s2, std::cos(angle * 0.5);
    }

    /**
     * @brief Constructor
     * @param x
     * @param y
     * @param z
     * @param w
     */
    Quaternion(double x, double y, double z, double w) : Vector4d(x, y, z, w)
    {
    }

    AxisAngle toAxisAngle() const
    {
        double n = vec().norm();
        if (n < std::numeric_limits<double>::epsilon())
        {
            n = vec().stableNorm();
        }

        AxisAngle axisAngle;
        if (n > 0.0)
        {
            axisAngle.angle() = 2.0 * atan2(n, w());
            axisAngle.axis() = vec() / n;
        }
        else
        {
            axisAngle.angle() = 0.0;
            axisAngle.axis() << 1.0, 0.0, 0.0;
        }

        return axisAngle;
    }

    bool isApprox(Quaternion o, double eps) const
    {
        if ((std::abs(x() - o.x()) < eps && std::abs(y() - o.y()) < eps && std::abs(z() - o.z()) < eps && std::abs(w() - o.w()) < eps) ||
            (std::abs(x() + o.x()) < eps && std::abs(y() + o.y()) < eps && std::abs(z() + o.z()) < eps && std::abs(w() + o.w()) < eps))
        {
            return true;
        }

        return false;
    }

    bool isApprox(Matrix3d E, double eps) const
    {
        return isApprox(Quaternion(E), eps);
    }

    EIGEN_STRONG_INLINE double& x()
    {
        return this->operator[](0);
    }

    EIGEN_STRONG_INLINE double x() const
    {
        return this->operator[](0);
    }

    EIGEN_STRONG_INLINE double& y()
    {
        return this->operator[](1);
    }

    EIGEN_STRONG_INLINE double y() const
    {
        return this->operator[](1);
    }

    EIGEN_STRONG_INLINE double& z()
    {
        return this->operator[](2);
    }

    EIGEN_STRONG_INLINE double z() const
    {
        return this->operator[](2);
    }

    EIGEN_STRONG_INLINE double& w()
    {
        return this->operator[](3);
    }

    EIGEN_STRONG_INLINE double w() const
    {
        return this->operator[](3);
    }

    /**
     * @brief Get vector part
     * @return Vector part
     */
    EIGEN_STRONG_INLINE Vector3d vec() const
    {
        return Vector3d(this->operator[](0), this->operator[](1), this->operator[](2));
    }

    Quaternion& operator=(const Eigen::Quaterniond& q)
    {
        (*this) << q.x(), q.y(), q.z(), q.w();
        return *this;
    }

    /**
     * @brief sanitize the quaternion by negating each element if the w element is less than zero
     */
    void sanitize()
    {
        if (!std::signbit(w()))
        {
            return;
        }

        x() *= -1.0;
        y() *= -1.0;
        z() *= -1.0;
        w() *= -1.0;
    }

    Quaternion& operator=(const RobotDynamics::Math::Quaternion& q)
    {
        (*this) << q.x(), q.y(), q.z(), q.w();
        return *this;
    }

    Quaternion& operator=(const Vector4d& q)
    {
        (*this) << q[0], q[1], q[2], q[3];
        return *this;
    }

    Quaternion& operator=(const Matrix3d& E)
    {
        *this = Quaternion(E);
        return *this;
    }

    Quaternion& operator=(const AxisAngle& axis_angle)
    {
        *this = Quaternion(axis_angle);
        return *this;
    }

    /**
     * @brief Method to scale the elements of a quaternion by a constant. Normalization is NOT performed
     * @param s
     * @return Scaled quaternion
     */
    Quaternion operator*(const double& s) const
    {
        return Quaternion((*this)[0] * s, (*this)[1] * s, (*this)[2] * s, (*this)[3] * s);
    }

    /**
     * @brief Quaternion multiplication
     * @param q Quaternion to multiply by
     * @return New multiplied quaternion result
     */
    Quaternion operator*(const Quaternion& q) const
    {
        return Quaternion(
            (*this)[3] * q[0] + (*this)[0] * q[3] + (*this)[1] * q[2] - (*this)[2] * q[1], (*this)[3] * q[1] + (*this)[1] * q[3] + (*this)[2] * q[0] - (*this)[0] * q[2],
            (*this)[3] * q[2] + (*this)[2] * q[3] + (*this)[0] * q[1] - (*this)[1] * q[0], (*this)[3] * q[3] - (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2]);
    }

    /**
     * @brief Overloaded *= operator for quaternion multiplication
     * @param q Quaternion to multiply by
     * @return Modified result of the multiplication
     */
    Quaternion& operator*=(const Quaternion& q)
    {
        set((*this)[3] * q[0] + (*this)[0] * q[3] + (*this)[1] * q[2] - (*this)[2] * q[1], (*this)[3] * q[1] + (*this)[1] * q[3] + (*this)[2] * q[0] - (*this)[0] * q[2],
            (*this)[3] * q[2] + (*this)[2] * q[3] + (*this)[0] * q[1] - (*this)[1] * q[0], (*this)[3] * q[3] - (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2]);
        return *this;
    }

    void transform(const RobotDynamics::Math::SpatialTransform& X)
    {
        *this = Quaternion(X.E.transpose()) * (*this);
    }

    /**
     * @brief From Wikipedia: In computer graphics, Slerp is shorthand for spherical linear interpolation,
     * introduced by Ken Shoemake in the context of quaternion interpolation for the
     * purpose of animating 3D rotation. It refers to constant-speed motion along a unit-radius
     * great circle arc, given the ends and an interpolation parameter between 0 and 1
     * @note Only unit quaternions are valid rotations, so make sure to normalize
     * @param alpha Interpolation parameter. Should be between 0 and 1
     * @param quat Quaternion to interpolate between
     * @return Interpolated quaternion
     */
    Quaternion slerp(double alpha, const Quaternion& quat) const;

    Quaternion conjugate() const
    {
        return Quaternion(-(*this)[0], -(*this)[1], -(*this)[2], (*this)[3]);
    }

    Quaternion timeStep(const Vector3d& omega, double dt)
    {
        double omega_norm = omega.norm();

        return Quaternion(omega / omega_norm, dt * omega_norm) * (*this);
    }

    Vector3d rotate(const Vector3d& vec) const
    {
        Vector3d vn(vec);
        Quaternion vec_quat(vn[0], vn[1], vn[2], 0.f), res_quat;

        res_quat = vec_quat * (*this);
        res_quat = conjugate() * res_quat;

        return Vector3d(res_quat[0], res_quat[1], res_quat[2]);
    }

    /**
     * @brief Decompose a quaternion into a swing then twist quaternion where the twist is about the given axis
     * @arg twist_axis Unit vector for the axis of the twist, e.g. (0., 0., 1.) for the z-axis
     * @arg swing Modified. Location for the resulting swing quaternion to be stored
     * @arg twist Modified. Location for the resulting twist quaternion to be stored
     *
     * @note This implementation can be found in PRZEMYSLAW DOBROWOLSKI's thesis titled, "SWING-TWIST DECOMPOSITION IN CLIFFORD ALGEBRA"
     */
    void swingTwistDecomposition(const Vector3d& twist_axis, Quaternion& swing, Quaternion& twist)
    {
        double u = twist_axis.dot(vec());
        double n = twist_axis.squaredNorm();
        double m = w() * n;
        double l = std::sqrt(std::pow(m, 2) + std::pow(u, 2) * n);
        twist << twist_axis.x() * (u / l), twist_axis.y() * (u / l), twist_axis.z() * (u / l), m / l;
        swing = *this * twist.conjugate();
    }

    /**
     * @brief Convert to intrinsic ZYX euler angles
     * @param yaw_at_pitch_singularity At Y = +- PI/2 is a singularity in which there are multiple solutions. This will be the yaw value in the output and the
     * roll value is dependent on this. To get the most accurate results at singularity, provide this value as close as possible to desired/reality and the resulting
     * roll value will be near the expected
     */
    Vector3d toIntrinsicZYXAngles(double yaw_at_pitch_singularity = 0.) const;
};

/**
 * @brief Convert YPR angles to quaternion
 */
static inline Quaternion intrinsicZYXAnglesToQuaternion(double yaw, double pitch, double roll)
{
    return Quaternion(Vector3d(0., 0., 1.), yaw) * Quaternion(Vector3d(0., 1., 0.), pitch) * Quaternion(Vector3d(1., 0., 0.), roll);
}

/**
 * @brief Convert YPR angles to quaternion
 */
static inline Quaternion intrinsicZYXAnglesToQuaternion(const Vector3d& zyx_angles)
{
    return intrinsicZYXAnglesToQuaternion(zyx_angles[0], zyx_angles[1], zyx_angles[2]);
}

/**
 * @brief Convert PRY angles to quaternion
 */
static inline Quaternion intrinsicYXZAnglesToQuaternion(double pitch, double roll, double yaw)
{
    return Quaternion(Vector3d(0., 1., 0.), pitch) * Quaternion(Vector3d(1., 0., 0.), roll) * Quaternion(Vector3d(0., 0., 1.), yaw);
}

/**
 * @brief Convert PRY angles to quaternion
 */
static inline Quaternion intrinsicYXZAnglesToQuaternion(const Vector3d& yxz_angles)
{
    return intrinsicYXZAnglesToQuaternion(yxz_angles[0], yxz_angles[1], yxz_angles[2]);
}

/**
 * @brief Convert RPY angles to quaternion
 */
static inline Quaternion intrinsicXYZAnglesToQuaternion(double roll, double pitch, double yaw)
{
    return Quaternion(Vector3d(1., 0., 0.), roll) * Quaternion(Vector3d(0., 1., 0.), pitch) * Quaternion(Vector3d(0., 0., 1.), yaw);
}

/**
 * @brief Convert RPY angles to quaternion
 */
static inline Quaternion intrinsicXYZAnglesToQuaternion(const Vector3d& xyz_angles)
{
    return intrinsicXYZAnglesToQuaternion(xyz_angles[0], xyz_angles[1], xyz_angles[2]);
}

static inline Matrix3d toTildeForm(const double x, const double y, const double z)
{
    return Matrix3d(0., -z, y, z, 0., -x, -y, z, 0.);
}

inline std::ostream& operator<<(std::ostream& output, const SpatialTransform& X)
{
    output << "X.E = " << std::endl << X.E << std::endl;
    output << "X.r = " << X.r.transpose();
    return output;
}

/**
 * @brief Get spatial transform from angle and axis
 * @param angle_rad angle magnitude
 * @param axis normalized 3d vector
 * @return Spatial transform
 */
inline SpatialTransform Xrot(double angle_rad, const Vector3d& axis)
{
    double s, c;

    s = sin(angle_rad);
    c = cos(angle_rad);

    return SpatialTransform(Matrix3d(axis[0] * axis[0] * (1.0f - c) + c, axis[1] * axis[0] * (1.0f - c) + axis[2] * s, axis[0] * axis[2] * (1.0f - c) - axis[1] * s,

                                     axis[0] * axis[1] * (1.0f - c) - axis[2] * s, axis[1] * axis[1] * (1.0f - c) + c, axis[1] * axis[2] * (1.0f - c) + axis[0] * s,

                                     axis[0] * axis[2] * (1.0f - c) + axis[1] * s, axis[1] * axis[2] * (1.0f - c) - axis[0] * s, axis[2] * axis[2] * (1.0f - c) + c

                                     ),
                            Vector3d(0., 0., 0.));
}

/**
 * @brief Get transform with zero translation and pure rotation about x axis
 * @param xrot
 * @return Transform with zero translation and x-rotation
 */
inline SpatialTransform Xrotx(const double& xrot)
{
    double s, c;

    s = sin(xrot);
    c = cos(xrot);
    return SpatialTransform(Matrix3d(1., 0., 0., 0., c, s, 0., -s, c), Vector3d(0., 0., 0.));
}

/**
 * @brief Get transform with zero translation and pure rotation about y axis
 * @param yrot
 * @return Transform with zero translation and y-rotation
 */
inline SpatialTransform Xroty(const double& yrot)
{
    double s, c;

    s = sin(yrot);
    c = cos(yrot);
    return SpatialTransform(Matrix3d(c, 0., -s, 0., 1., 0., s, 0., c), Vector3d(0., 0., 0.));
}

/**
 * @brief Get transform with zero translation and pure rotation about z axis
 * @param zrot
 * @return Transform with zero translation and z-rotation
 */
inline SpatialTransform Xrotz(const double& zrot)
{
    double s, c;

    s = sin(zrot);
    c = cos(zrot);
    return SpatialTransform(Matrix3d(c, s, 0., -s, c, 0., 0., 0., 1.), Vector3d(0., 0., 0.));
}

/**
 * @brief Get transform with zero translation and intrinsic euler z/y/x rotation
 * @param yaw
 * @param pitch
 * @param roll
 * @return Transform with zero translation and z/y/x rotation
 */
inline SpatialTransform XeulerZYX(double yaw, double pitch, double roll)
{
    return Xrotx(roll) * Xroty(pitch) * Xrotz(yaw);
}

/**
 * @brief Get transform defined by intrinsic YPR(yaw->pitch->roll) euler angles
 * @param ypr Vector of euler angles where ypr[0] is yaw, ypr[1] is pitch, and ypr[2] is roll
 *
 * @return spatial transform where rotation component is defined by the YPR rotations
 */
inline SpatialTransform XeulerZYX(const Vector3d& ypr)
{
    return XeulerZYX(ypr[0], ypr[1], ypr[2]);
}

/**
 * @brief Get transform with zero translation and intrinsic euler x/y/z rotation
 * @param roll
 * @param pitch
 * @param yaw
 * @return Transform with zero translation and x/y/z rotation
 */
inline SpatialTransform XeulerXYZ(double roll, double pitch, double yaw)
{
    return Xrotz(yaw) * Xroty(pitch) * Xrotx(roll);
}

/**
 * @brief Get transform with zero translation and euler x/y/z rotation
 * @param xyz_angles xyz_angles xyz angles where element 0 is roll, 1 is pitch, and 2 is yaw
 * @return Transform with zero translation and x/y/z rotation
 */
inline SpatialTransform XeulerXYZ(const Vector3d& xyz_angles)
{
    return XeulerXYZ(xyz_angles[0], xyz_angles[1], xyz_angles[2]);
}

inline SpatialTransform XrotQuat(double x, double y, double z, double w)
{
    return SpatialTransform(Quaternion(x, y, z, w));
}

inline SpatialTransform XrotQuat(const Quaternion& orientation)
{
    return SpatialTransform(orientation);
}

/**
 * @brief Get pure translation transform
 * @param r
 * @return Transform with identity rotation and translation \f$ r \f$
 */
inline SpatialTransform Xtrans(const Vector3d& r)
{
    return SpatialTransform(Matrix3d::Identity(3, 3), r);
}

inline SpatialTransform Xtrans(double x, double y, double z)
{
    return SpatialTransform(Matrix3d::Identity(3, 3), Vector3d(x, y, z));
}
}  // namespace Math
}  // namespace RobotDynamics

#endif  // ifndef __RDL_EIGENMATH_H__
