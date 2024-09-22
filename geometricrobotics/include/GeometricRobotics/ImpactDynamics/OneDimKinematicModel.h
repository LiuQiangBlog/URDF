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

#include "GeometricRobotics/ImpactDynamics/ImpactDynamicsModel.h"

namespace GeoRobotics
{

class OneDimKinematicModel : public ImpactDynamicsModel
/*! \brief It implements the one dimensional frictionless-impact dynamics model employing the kinematic restitution.
 */
{
public:
  OneDimKinematicModel(const std::shared_ptr<KinematicChain> robot,
                       const std::shared_ptr<EndEffector> ee,
                       const ImpactModelParams & params);

  virtual ~OneDimKinematicModel()
  {
    RoboticsUtils::quickInfo("Destructing OneDimKinematicModel.");
  }

  const PostImpactStates & objectPostImpactStates() override;

  /*! \brief Compute the post-impact states.
   * \param av The average velocity of the robot. This param is not used for this model.
   */
  void update(const Matrix6d & equivalentMass, const Vector6d & av) override;

  void setFrictionCoefficient(const double & miu) override
  {
    RoboticsUtils::throw_runtime_error(
        "We can not set friction coefficient for the frictionless impact dynamics model.");
  }

  /*! \brief Set the kinematic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    params_.coeR = e;
  }

protected:
  void writeSolution_(const Vector3d & impulse, PostImpactStates & input);
}; // End of class: OneDimKinematicModel

} // namespace GeoRobotics
