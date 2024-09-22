/*
 * Copyright (c) 2022, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include "Utils.h"

namespace GeoRobotics
{

#define GEO_EPSILON 1e-6
#define GEO_UNIT_EPSILON 1e-6

struct TwistCoordinate
{
  TwistCoordinate() {}
  explicit TwistCoordinate(const Vector3d & axis)
  {
    prismatic_reset(axis);
  }
  TwistCoordinate(const Vector3d & point, const Vector3d & axis)
  {
    revolute_reset(point, axis);
  }

  ~TwistCoordinate() {}

  Vector3d linear = Vector3d::Zero();
  Vector3d angular = Vector3d::Zero();
  bool operator==(const TwistCoordinate & t) const;
  TwistCoordinate operator*(const TwistCoordinate & t) const;
  TwistCoordinate operator+(const TwistCoordinate & t) const;
  TwistCoordinate operator+(const Vector6d & t) const;
  TwistCoordinate operator-(const TwistCoordinate & t) const;
  TwistCoordinate operator-(const Vector6d & t) const;
  Eigen::Isometry3d expMap() const;
  Vector6d vector() const;
  Matrix6d crossMatrix() const;

  void prismatic_reset(const Vector3d & axis);
  void revolute_reset(const Vector3d & point, const Vector3d & axis);
  double norm() const;
  void print() const;
};

/*! \brief Check if the norm equals to one
 */
bool checkUnit(const Vector3d & axis);
bool checkNull(const Vector3d & axis);

/*! \brief Computes the 4x4 matrix
 */
Matrix4d hat(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular);
void hat(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular, Matrix4d & twist);

/*! \brief Computes the 4x4 matrix  from a twist coordinate
 */
Matrix4d hat(const Vector6d & xi);
Matrix4d hat(const TwistCoordinate & xi);

/*! \brief Computes the skew-symmetric matrix
 */
Matrix3d skew(const Eigen::Vector3d & input);
void skew(const Eigen::Vector3d & input, Matrix3d & output);

/*! \brief Computes the cross operator matrix
 */
Matrix6d crossMatrix(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular);

/*! \brief Computes the cross operator matrix assumming vel = [linear; angular]
 */
Matrix6d crossMatrix(const Vector6d & vel);

/*! \brief Computes the cross product: output = one X two;
 */
Vector6d cross(const Vector6d & one, const Vector6d & two);

/*! \brief Computes the dual-cross operator matrix assumming vel = [linear; angular]
 */
Matrix6d dualCrossMatrix(const Vector6d & vel);

/*! \brief Computes the dual of the cross product: output = -(one X two)^T, P34 ofR.Featherstone;
 */
Vector6d dualCross(const Vector6d & one, const Vector6d & two);

/*! \brief Computes the rotation matrix from an axis
 */
Matrix3d expMap(const Vector3d & vec);
/*! \brief Computes the rotation matrix from an axis
 */
Matrix3d expMap(const Vector3d & axis, const double & theta);

/*! \brief For computing the twist coordinate exponential
 * This is an intermediate step.
 */
void expMap(const Vector3d & vec, Matrix3d & R, Matrix3d & A);

/*! \brief Computes the exponential map of the twist coordinate [v, w]
 */
Eigen::Isometry3d expMap(const Vector3d & linear, const Vector3d & angular);

/*! \brief Computes the exponential map of the twist coordinate \xi [v, w]
 */
Eigen::Isometry3d expMap(const Vector6d & xi);

/*! \brief Computes the exponential map of the twist coordinate \xi [v, w]
 */
Eigen::Isometry3d expMap(const TwistCoordinate & xi);

/*! \brief Computes the exponential map of the twist coordinate \xi [v, w]
 */
Eigen::Isometry3d expMap(const TwistCoordinate & xi, const double & theta);

/*! \brief Computes the exponential map of the twist \hat{\xi}, e.g., hat(xi)
 */
Eigen::Isometry3d expMap(const Matrix4d & hat_xi);

/*! \brief Performs the adjointTransform * twistCoordinate
 */
void metaAdg(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output);
Matrix6d metaAdgMatrix(const Iso3d & transform);
TwistCoordinate metaAdg(const Iso3d & transform, const TwistCoordinate & input);

/*! \brief Performs the inverse(adjointTransform) * twistCoordinate
 */
void metaAdgInv(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output);
Matrix6d metaAdgInvMatrix(const Iso3d & transform);

TwistCoordinate metaAdgInv(const Iso3d & transform, const TwistCoordinate & input);

void metaAdgInvTwo(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output);
Matrix6d metaAdgInvTwoMatrix(const Iso3d & transform);

/*! \brief Performs the transpose(adjointTransform) * twistCoordinate
 */
void metaAdgT(const Iso3d & transform, const TwistCoordinate & input, TwistCoordinate & output);
Matrix6d metaAdgTMatrix(const Iso3d & transform);

/*! \brief Performs the adjointInvTransform * twistCoordinate. The transform from refrence frame to the target frame
 */
TwistCoordinate metaTwistTransform(const Iso3d & transform, const TwistCoordinate & input);

/*! \brief Performs the g * twist * g^{-1}
 */
// Matrix4d metaTwistTransform(const Iso3d & transform, const Matrix4d & input);

/*! \brief Generates a vector of random 3D vector from a uniform distribution.
 *  \param bag is the container
 *  \param number of the vectors
 *  \param lb lower bound
 *  \param ub upper bound
 *  \param normalize if we normalize the vectors
 */
void grVec3d(std::vector<Vector3d> & bag,
             const size_t & size,
             const double & lb = 0.0,
             const double & ub = 1.0,
             const bool & normalize = true);

/*! \brief Generates a random integer in the range of [0, n -1]
 */
template<typename Integer>
Integer randInt(Integer n);

Eigen::Vector3d grVec3d(double lb, double ub, bool normalize);

Iso3d grTF(double lb = 0.0, double ub = 1.0);

Vector3d grVec3d(std::default_random_engine & re,
                 const double & lb = 0.0,
                 const double & ub = 1.0,
                 const bool & normalize = true);

Eigen::VectorXd grVec(double lb, double ub, bool normalize, int size);

Matrix3d grRotation();
AngleAxisd grAngleAxis();

/*! \brief Each call generates a random double in the range of [0, n -1]
 */
double grNumber(const double & lb = -m_pi, const double & ub = m_pi);

double grNumber(std::default_random_engine & re, const double & lb = -m_pi, const double & ub = m_pi);

} // namespace GeoRobotics
