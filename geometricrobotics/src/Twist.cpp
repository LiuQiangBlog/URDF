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
#include "GeometricRobotics/Twist.h"

namespace GeoRobotics
{

void Twist::reset(const Vector3d & axis, const std::string & referenceFrameName)
{

  // (1) Update the twist coordinate:
  tc_.prismatic_reset(axis);
  // (2) Update the twist matrix:
  hat(tc_.linear, tc_.angular, twist_);

  // (3) Reference frame name:
  properties_.refFrameName = referenceFrameName;
}

void Twist::reset(const Vector3d & point, const Vector3d & axis, const std::string & referenceFrameName)
{
  // (1) Update the twist coordinate:
  tc_.revolute_reset(point, axis);
  // (2) Update the twist matrix:
  hat(tc_.linear, tc_.angular, twist_);

  // (3) Reference frame name:
  properties_.refFrameName = referenceFrameName;
}
/*
void Twist::updateTwist_()
{
  // The translational part:
  twist_.block<3, 1>(0, 3) = getTwistCoordinate().v;

  // The rotational part:
  const Eigen::Vector3d & w = getTwistCoordinate().w;

  twist_(0, 1) = -w(2);
  twist_(1, 0) = w(2);

  twist_(0, 2) = w(1);
  twist_(2, 0) = -w(1);

  twist_(1, 2) = -w(0);
  twist_(2, 1) = w(0);
}
*/

} // namespace GeoRobotics
