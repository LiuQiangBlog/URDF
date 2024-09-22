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
#include "GeometricRobotics/Kinematics/GeoLink.h"

namespace GeoRobotics
{

void GeoLink::calcWrench()
{
  wrench_ =
      localInertiaTensor() * bodyAcceleration() + dualCross(bodyVelocity(), localInertiaTensor() * bodyVelocity());
}

void GeoLink::printInertiaTensors(const int & choice) const
{

  RoboticsUtils::quickInfo("Link " + urdfLink()->name + " has the inertia tensor: ");
  std::cout << "Mass: " << urdfLink()->metaInertia->mass << std::endl;
  // std::cout<<"Rotational inertia: " << std::endl<<urdfLink()->inertia().rInertia()<<std::endl;
  // std::cout<<"mass inertia: " << std::endl<<urdfLink()->inertia().mMatrix()<<std::endl;
  switch(choice)
  {
    case 0:
      std::cout << "At the local COM frame: " << std::endl
                << RoboticsUtils::info << comInertiaTensor() << RoboticsUtils::reset << std::endl;
      break;
    case 1:
      std::cout << "At the local link frame: " << std::endl
                << RoboticsUtils::info << localInertiaTensor() << RoboticsUtils::reset << std::endl;
      break;
    default:
      RoboticsUtils::throw_runtime_error("Unknown choice " + std::to_string(choice) + " to print the inertia tensor.");
  }
}

void GeoLink::initializeInertiaTensors_(const MetaInertia & metaInertia)
{

  // RoboticsUtils::quickInfo("Inertia tensor: " + urdfLink()->name);
  // transform to the COM in the local body frame
  // localCOMtransform_.linear() = mLink_->metaInertia->origin.rotation.getEigenQuaternion().toRotationMatrix();
  // localCOMtransform_.linear() = metaInertia.origin.rotation.getEigenQuaternion().toRotationMatrix();

  // localCOMtransform_.translation() = metaInertia.origin.position;
  localCOMtransform_ = metaInertia.origin.toMatrix();

  // RoboticsUtils::quickPrint("localCOMtransform_", localCOMtransform_.matrix());
  // These two matrcies are set once for all.
  comInertiaTensor_ = metaInertia.constructInertiaTensor();
  // RoboticsUtils::quickPrint("comInertiaTensor_", comInertiaTensor_);
  // Apply the transform of the COM w.r.t. the local link frame
  const Matrix6d & adgInvM = metaAdgInvMatrix(localCOMTransform());
  localInertiaTensor_ = adgInvM.transpose() * comInertiaTensor_ * adgInvM;
  // RoboticsUtils::quickPrint("localInertiaTensor_", localInertiaTensor_);
  // RoboticsUtils::quickInfo("finished Inertia tensor: " + urdfLink()->name);
}

Matrix6d GeoLink::spatialInertia(const Iso3d & transform)
{
  const Matrix6d & adgInvM = metaAdgInvMatrix(transform);
  return adgInvM.transpose() * localInertiaTensor() * adgInvM;
}

} // namespace GeoRobotics
