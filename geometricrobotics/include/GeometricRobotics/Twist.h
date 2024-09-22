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

#include "Computations.h"
#include "Utils.h"

namespace GeoRobotics
{

struct TwistProperties
{
  std::string name{"Twist"};

  std::string refFrameName{"inertialFrame"};
};

class Twist
{
  /*! \brief
   * Element of \f$ se(3): \hat{\xi} \in \mathcal{R}^{4 \times 4} \f$, which is a infinitesimal generator of the
   * Euclidean group.
   */
public:
  explicit Twist() {}
  ~Twist() {}

  /*! \brief Revolute joint: update the twist coordinate and the twist
   * TODO: CHECK: on P87 of Murray, it says that all vectors and points are specified w.r.t. the base frame.
   * \param point on the axis
   * \param axis a unit vector
   * \param referenceFramename is the unique name in a model or a robot.
   */
  void reset(const Vector3d & point, const Vector3d & axis, const std::string & referenceFrameName);

  /*! \brief Prismatic joint: update the twist coordinate and the twist
   * \param axis the unit vector
   * \param referenceFramename is the unique name in a model or a robot.
   */
  void reset(const Vector3d & axis, const std::string & referenceFrameName);

  /*! \brief returns the \f$ 4\times4 \f$ twist matrix
   */
  Matrix4d const & matrix()
  {
    return twist_;
  }

  TwistProperties const & getProperties()
  {
    return properties_;
  }

  /*! \brief get the twist coordinate
   */
  TwistCoordinate const & tc()
  {
    return tc_;
  }

  /*
  void setReferenceFrame(const std::string & refFrameName)
  {
    properties_.refFrameName = refFrameName;
  }

  void setLinear(const Vector3d & linear)
  {
    tc_.linear = linear;
  }
  void setAngular(const Vector3d & angular)
  {
    tc_.angular = angular;
  }
  */
protected:
  TwistProperties properties_{};
  TwistCoordinate tc_{};

  // Update the twist according to the twist coordinates.
  // void updateTwist_();
  Matrix4d twist_ = Matrix4d::Zero();
};

} // namespace GeoRobotics
