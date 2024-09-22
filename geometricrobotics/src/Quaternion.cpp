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
#include "GeometricRobotics/urdf/Quaternion.h"

namespace GeoRobotics
{
void Quaternion::setFromRPY(double roll, double pitch, double yaw)
{
  double phi, the, psi;

  phi = roll / 2.0;
  the = pitch / 2.0;
  psi = yaw / 2.0;

  x_ = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  y_ = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  z_ = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
  w_ = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

  normalize();
}

// Multiplication operator (copied from gazebo)
Quaternion Quaternion::operator*(const Quaternion & qt) const
{

  double cx = w() * qt.x() + x() * qt.w() + y() * qt.z() - z() * qt.y();
  double cy = w() * qt.y() - x() * qt.z() + y() * qt.w() + z() * qt.x();
  double cz = w() * qt.z() + x() * qt.y() - y() * qt.x() + z() * qt.w();
  double cw = w() * qt.w() - x() * qt.x() - y() * qt.y() - z() * qt.z();

  return Quaternion(cx, cy, cz, cw);
}

Vector3d Quaternion::operator*(Vector3d vec) const
{
  Vector3d result;

  double tw = 0.0;
  double tx = vec.x();
  double ty = vec.y();
  double tz = vec.z();
  Quaternion tmp(tx, ty, tz, tw);

  tmp = (*this) * (tmp * inverse());

  result.x() = tmp.x();
  result.y() = tmp.y();
  result.z() = tmp.z();

  return result;
}

void Quaternion::getRPY(double & roll, double & pitch, double & yaw) const
{
  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = x_ * x_;
  sqy = y_ * y_;
  sqz = z_ * z_;
  sqw = w_ * w_;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (x_ * z_ - w_ * y_);
  const double pi_2 = 1.57079632679489661923;
  if(sarg <= -0.99999)
  {
    pitch = -pi_2;
    roll = 0;
    yaw = 2 * atan2(x_, -y_);
  }
  else if(sarg >= 0.99999)
  {
    pitch = pi_2;
    roll = 0;
    yaw = 2 * atan2(-x_, y_);
  }
  else
  {
    pitch = asin(sarg);
    roll = atan2(2 * (y_ * z_ + w_ * x_), sqw - sqx - sqy + sqz);
    yaw = atan2(2 * (x_ * y_ + w_ * z_), sqw + sqx - sqy - sqz);
  }
}

Quaternion Quaternion::inverse() const
{
  Quaternion q;

  double norm = w() * w() + x() * x() + y() * y() + z() * z();

  if(norm > 0.0)
  {
    double qw = w() / norm;
    double qx = -x() / norm;
    double qy = -y() / norm;
    double qz = -z() / norm;
    return Quaternion(qx, qy, qz, qw);
  }
  else
  {
    RoboticsUtils::throw_runtime_error("The norm of the quaternion is not greater than 0. ", __FILE__, __LINE__);
  }

  return q;
}

void Quaternion::normalize()
{
  double s = sqrt(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_);
  if(s == 0.0)
  {
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    w_ = 1.0;
  }
  else
  {
    x_ /= s;
    y_ /= s;
    z_ /= s;
    w_ /= s;
  }
}
} // namespace GeoRobotics
