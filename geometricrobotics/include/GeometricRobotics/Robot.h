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

#include "GeometricRobotics/Kinematics/KinematicTree.h"

namespace GeoRobotics
{

class Robot
{
public:
  Robot(std::shared_ptr<Model> model);
  ~Robot(){};

  /*! @return the URDF model
   */
  std::shared_ptr<Model> const & getModel() const
  {
    return modelPtr_;
  }

  std::shared_ptr<KinematicTree> const & createFloatingBaseKinematicTree(std::string const & treeName,
                                                                         std::string const & rootLinkName,
                                                                         Iso3d const & initialTf = Iso3d::Identity());

  /*
  std::shared_ptr<Joint> floatingBaseJoint(const std::string & rootLinkName,
                                           const Iso3d & initialTf = Iso3d::Identity());
             */

protected:
  std::shared_ptr<KinematicTree> root_tree_;

  std::shared_ptr<Model> modelPtr_;

  std::unordered_map<std::string, std::shared_ptr<KinematicTree>> trees_;

}; // End of class: Robot

} // namespace GeoRobotics
