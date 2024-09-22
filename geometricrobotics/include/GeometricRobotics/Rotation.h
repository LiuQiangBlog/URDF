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

#include "GeometricRobotics/Frame.h"
#include "GeometricRobotics/Utils.h"

namespace GeoRobotics
{

struct RotationProperties
{
  Frame referenceFrame = {"InertialFrame"};
  Frame targetFrame = {"objectFrame"};
};

Vector3d parseVector3d(const std::string & vector_str);

class Rotation
{
public:
  Rotation(double x, double y, double z, double w)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
  }
  Rotation()
  {
    reset();
  }
  const double & x() const
  {
    return x_;
  }
  const double & y() const
  {
    return y_;
  }
  const double & z() const
  {
    return z_;
  }
  const double & w() const
  {
    return w_;
  }

  void getQuaternion(double & quat_x, double & quat_y, double & quat_z, double & quat_w) const
  {
    quat_x = x_;
    quat_y = y_;
    quat_z = z_;
    quat_w = w_;
  }
  void getRPY(double & roll, double & pitch, double & yaw) const;

  void setFromQuaternion(double quat_x, double quat_y, double quat_z, double quat_w)
  {
    x_ = quat_x;
    y_ = quat_y;
    z_ = quat_z;
    w_ = quat_w;
    normalize();
  }

  // Multiplication operator (copied from gazebo)
  Rotation operator*(const Rotation & qt) const;
  /// Rotate a vector using the quaternion
  Vector3d operator*(Vector3d vec) const;

  /*! \brief  Get the inverse of this quaternion
   */
  Rotation inverseQ() const;

  void normalize();
  void setFromRPY(double roll, double pitch, double yaw);

  void init(const std::string & rotation_str)
  {
    reset();
    Vector3d rpy;
    rpy = parseVector3d(rotation_str);
    setFromRPY(rpy.x(), rpy.y(), rpy.z());
  }

  void reset()
  {
    x_ = y_ = z_ = 0.0;
    w_ = 1.0;
  }

  const Eigen::Matrix3d & matrix()
  {
    return matrix_;
  }

  const Frame & targetFrame()
  {
    return properties_.targetFrame;
  }

  const Frame & referenceFrame()
  {
    return properties_.referenceFrame;
  }

protected:
  double x_, y_, z_, w_;

  Eigen::Matrix3d matrix_ = Eigen::Matrix3d::Identity();

  RotationProperties properties_;
};

} // namespace GeoRobotics
