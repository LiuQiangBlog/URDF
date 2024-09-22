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
#include "GeometricRobotics/ImpactDynamics/OneDimKinematicModel.h"

namespace GeoRobotics
{

OneDimKinematicModel::OneDimKinematicModel(const std::shared_ptr<KinematicChain> robot,
                                           const std::shared_ptr<EndEffector> ee,
                                           const ImpactModelParams & params)
: ImpactDynamicsModel(robot, ee, params)
{
  RoboticsUtils::quickInfo("OneDimKinematicModel is created.");
}

void OneDimKinematicModel::update(const Matrix6d & equivalentMass, const Vector6d & av)
{
  updateImpactNormal_();
  // printMatrix("ImpactNormal", impactNormal());

  const Matrix3d & P = impactNormal() * impactNormal().transpose();
  // printMatrix("P", P);

  const Vector3d & eeJump = -(1 + params().coeR) * P * robotContactPointVelocity();

  // printMatrix("eeJump", eeJump);
  const Matrix3d & em = equivalentMass.block<3, 3>(0, 0);
  // printMatrix("equivalent mass", em);
  // printMatrix("equivalent mass inverse", em.inverse());

  // const Vector3d & impulse = equivalentMass.block<3, 3>(0, 0) * eeJump;
  // I should always compute it in body coordinates
  Vector3d impulse = Vector3d::Zero();
  double impulseValue = (impactNormal().transpose() * em * impactNormal());
  ems_ = impulseValue;

  impulseValue *= eeJump.norm();
  impulse.z() = impulseValue;

  writeSolution_(impulse, robotPostImpactStates_);
}

void OneDimKinematicModel::writeSolution_(const Vector3d & impulse, PostImpactStates & input)
{
  input.impulse = impulse;

  input.linearVelJump = input.impulse / robot()->mass();

  // The angular and linear velocities are not touched.
}

const PostImpactStates & OneDimKinematicModel::objectPostImpactStates()
{
  RoboticsUtils::throw_runtime_error("We can not check the object post-impact state for the kinematic model.", __FILE__,
                                     __LINE__);
}

} // namespace GeoRobotics
