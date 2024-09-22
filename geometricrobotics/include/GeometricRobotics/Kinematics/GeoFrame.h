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

#include "GeometricRobotics/Computations.h"
#include "GeometricRobotics/urdf/Link.h"

namespace GeoRobotics
{

class KinematicTree;

class GeoFrame : public std::enable_shared_from_this<GeoFrame>
{
public:
  GeoFrame(std::string name, std::shared_ptr<KinematicTree> const robot) : name_(name), robot_(robot)
  {
    // RoboticsUtils::quickInfo("Creating GeoFrame: " + name);

    // RoboticsUtils::quickInfo("Created GeoLink: " + urdfLink()->name);
  }

  // std::shared_ptr<GeoFrame> getPtr()
  // {
  //   return shared_from_this();
  // }

  ~GeoFrame() {}

  std::string frameName() const
  {
    return name_;
  }

  virtual Iso3d const & transform() = 0;

  virtual Vector6d const inertialFrameBodyVel() = 0;

  virtual MatrixXd computeBodyJacobian(bool const & fullJacobian = true) const = 0;

  std::shared_ptr<KinematicTree const> const robot() const
  {
    return robot_;
  }

private:
protected:
  std::string name_ = "emptyFrame";
  std::shared_ptr<KinematicTree const> robot_;
  // Iso3d inertialTransform_{Iso3d::Identity()};

}; // end of class

// class FixedFrame : public GeoFrame
// {
//   FixedFrame(std::string name, std::shared_ptr<KinematicTree> const robot) : GeoFrame(name, robot)
//   {
//
//   }
//
//   ~FixedFrame() {}
//
//   Iso3d const & transform() override;
//
//   /*! \brief returns a full-zero size-compatible matrix
//    */
//   MatrixXd computeBodyJacobian(bool const & fullJacobian = true) override
//   {
//
//   }
//
// };

} // namespace GeoRobotics
