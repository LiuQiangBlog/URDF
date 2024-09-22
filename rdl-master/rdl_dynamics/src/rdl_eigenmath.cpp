// Copyright (c) 2023 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <rdl_dynamics/rdl_eigenmath.hpp>

namespace RobotDynamics
{
namespace Math
{
SpatialTransform::SpatialTransform(double x, double y, double z, double w) : E(Quaternion(x, y, z, w)), r(0., 0., 0.)
{
}

Matrix3d::Matrix3d(const Quaternion& o)
{
    double x = o.x();
    double y = o.y();
    double z = o.z();
    double w = o.w();

    (*this) << 1. - 2. * (y * y + z * z), 2. * x * y + 2. * w * z, 2. * x * z - 2. * w * y, 2. * x * y - 2. * w * z, 1. - 2. * (x * x + z * z), 2. * y * z + 2. * w * x,
        2. * x * z + 2. * w * y, 2. * y * z - 2. * w * x, 1. - 2. * (x * x + y * y);
}

Quaternion::Quaternion(const Eigen::Matrix3d& E)
{
    double trace = E.trace();

    if (trace > 0.)
    {
        double s = 2. * std::sqrt(trace + 1.);
        (*this) << (E(1, 2) - E(2, 1)) / s, (E(2, 0) - E(0, 2)) / s, (E(0, 1) - E(1, 0)) / s, 0.25 * s;
    }
    else if ((E(0, 0) > E(1, 1)) && (E(0, 0) > E(2, 2)))
    {
        double s = 2. * std::sqrt(1. + E(0, 0) - E(1, 1) - E(2, 2));
        (*this) << -0.25 * s, (-E(0, 1) - E(1, 0)) / s, (-E(0, 2) - E(2, 0)) / s, (E(2, 1) - E(1, 2)) / s;
    }
    else if (E(1, 1) > E(2, 2))
    {
        double s = 2. * std::sqrt(1. + E(1, 1) - E(0, 0) - E(2, 2));
        (*this) << (-E(0, 1) - E(1, 0)) / s, -0.25 * s, (-E(1, 2) - E(2, 1)) / s, (E(0, 2) - E(2, 0)) / s;
    }
    else
    {
        double s = 2. * std::sqrt(1. + E(2, 2) - E(0, 0) - E(1, 1));
        (*this) << (-E(0, 2) - E(2, 0)) / s, (-E(1, 2) - E(2, 1)) / s, -0.25 * s, (E(1, 0) - E(0, 1)) / s;
    }
}

Quaternion Quaternion::slerp(double alpha, const Quaternion& quat) const
{
    // check whether one of the two has 0 length
    double s = std::sqrt(squaredNorm() * quat.squaredNorm());

    // division by 0.f is unhealthy!
    assert(s != 0.);

    double angle = acos(dot(quat) / s);

    if ((angle == 0.) || std::isnan(angle))
    {
        return *this;
    }
    assert(!std::isnan(angle));

    double d = 1. / std::sin(angle);
    double p0 = std::sin((1. - alpha) * angle);
    double p1 = std::sin(alpha * angle);

    if (dot(quat) < 0.)
    {
        return Quaternion(Vector4d(((*this) * p0 - quat * p1) * d));
    }
    return Quaternion(Vector4d(((*this) * p0 + quat * p1) * d));
}

Vector3d Quaternion::toIntrinsicZYXAngles(double yaw_at_pitch_singularity) const
{
    double qx = this->operator[](0);
    double qy = this->operator[](1);
    double qz = this->operator[](2);
    double qw = this->operator[](3);
    double sqw = qw * qw;
    double sqx = qx * qx;
    double sqy = qy * qy;
    double sqz = qz * qz;
    double unit = sqx + sqy + sqz + sqw;  // if normalised is one, otherwise is correction factor
    double test = qw * qy - qz * qx;
    if (test > 0.499 * unit)
    {  // singularity at north pole
        return Vector3d(yaw_at_pitch_singularity, M_PI_2, 2. * atan2(qx, qy) + yaw_at_pitch_singularity);
    }
    if (test < -0.499 * unit)
    {  // singularity at south pole
        return Vector3d(yaw_at_pitch_singularity, -M_PI_2, 2. * atan2(qx, qw) - yaw_at_pitch_singularity);
    }

    return Vector3d(atan2(2. * (qw * qz + qx * qy), 1. - 2. * (sqy + sqz)), asin(2. * test / unit), atan2(2. * (qw * qx + qy * qz), 1. - 2. * (sqx + sqy)));
}

Vector3d Matrix3d::toIntrinsicZYXAngles(double yaw_at_pitch_singularity) const
{
    // // Assuming the angles are in radians.
    if ((*this)(0, 2) < -0.999)
    {  // singularity at north pole
        return Vector3d(yaw_at_pitch_singularity, M_PI_2, yaw_at_pitch_singularity + atan2((*this)(1, 0), (*this)(1, 1)));
    }
    if ((*this)(0, 2) > 0.999)
    {  // singularity at south pole
        return Vector3d(yaw_at_pitch_singularity, -M_PI_2, -yaw_at_pitch_singularity + atan2(-(*this)(1, 0), (*this)(1, 1)));
    }

    return Vector3d(atan2((*this)(0, 1), (*this)(0, 0)), asin(-(*this)(0, 2)), atan2((*this)(1, 2), (*this)(2, 2)));
}

Matrix3d Vector3d::toTildeForm() const
{
    return Matrix3d(0., -(*this)[2], (*this)[1], (*this)[2], 0., -(*this)[0], -(*this)[1], (*this)[0], 0.);
}

SpatialMatrix SpatialVector::crossm()
{
    return SpatialMatrix(0, -this->operator[](2), this->operator[](1), 0, 0, 0, this->operator[](2), 0, -this->operator[](0), 0, 0, 0, -this->operator[](1),
                         this->operator[](0), 0, 0, 0, 0, 0, -this->operator[](5), this->operator[](4), 0, -this->operator[](2), this->operator[](1), this->operator[](5),
                         0, -this->operator[](3), this->operator[](2), 0, -this->operator[](0), -this->operator[](4), this->operator[](3), 0, -this->operator[](1),
                         this->operator[](0), 0);
}

SpatialVector SpatialVector::crossm(const SpatialVector& v)
{
    return SpatialVector(-this->operator[](2) * v[1] + this->operator[](1) * v[2], this->operator[](2) * v[0] - this->operator[](0) * v[2],
                         -this->operator[](1) * v[0] + this->operator[](0) * v[1],
                         -this->operator[](5) * v[1] + this->operator[](4) * v[2] - this->operator[](2) * v[4] + this->operator[](1) * v[5],
                         this->operator[](5) * v[0] - this->operator[](3) * v[2] + this->operator[](2) * v[3] - this->operator[](0) * v[5],
                         -this->operator[](4) * v[0] + this->operator[](3) * v[1] - this->operator[](1) * v[3] + this->operator[](0) * v[4]);
}

SpatialMatrix SpatialVector::crossf()
{
    return SpatialMatrix(0, -this->operator[](2), this->operator[](1), 0, -this->operator[](5), this->operator[](4), this->operator[](2), 0, -this->operator[](0),
                         this->operator[](5), 0, -this->operator[](3), -this->operator[](1), this->operator[](0), 0, -this->operator[](4), this->operator[](3), 0, 0, 0,
                         0, 0, -this->operator[](2), this->operator[](1), 0, 0, 0, this->operator[](2), 0, -this->operator[](0), 0, 0, 0, -this->operator[](1),
                         this->operator[](0), 0);
}

SpatialVector SpatialVector::crossf(const SpatialVector& v)
{
    return SpatialVector(-this->operator[](2) * v[1] + this->operator[](1) * v[2] - this->operator[](5) * v[4] + this->operator[](4) * v[5],
                         this->operator[](2) * v[0] - this->operator[](0) * v[2] + this->operator[](5) * v[3] - this->operator[](3) * v[5],
                         -this->operator[](1) * v[0] + this->operator[](0) * v[1] - this->operator[](4) * v[3] + this->operator[](3) * v[4],
                         -this->operator[](2) * v[4] + this->operator[](1) * v[5], +this->operator[](2) * v[3] - this->operator[](0) * v[5],
                         -this->operator[](1) * v[3] + this->operator[](0) * v[4]);
}
}  // namespace Math
}  // namespace RobotDynamics
