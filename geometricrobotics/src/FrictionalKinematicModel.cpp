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
#include "GeometricRobotics/ImpactDynamics/FrictionalKinematicModel.h"

namespace GeoRobotics
{
FrictionalKinematicModel::FrictionalKinematicModel(const std::shared_ptr<KinematicChain> robotPtr,
                                                   const std::shared_ptr<EndEffector> ee,
                                                   const ImpactModelParams & params,
                                                   const bool & useFrictionCone)
: ImpactDynamicsModel(robotPtr, ee, params), useFrictionCone_(useFrictionCone)
{

  updateImpactNormal_();
  // 1. Create the impulse generator K_
  initializeImpulseGenerator_(useFrictionCone);
  // 2. Create the post-impact velocity predictor P_
  initializeVelocityPredictor_();
  // 3. Check the frame consistency
  updateBuildingBlocks_(params.bodyCoordinates);

  RoboticsUtils::quickInfo("FrictionalKinematicModel is created.");
}

void FrictionalKinematicModel::initializeVelocityPredictor_()
{
  PBody_ = -Matrix3d::Identity();
  // Suppose the z-axis is the impact direction.
  PBody_(2, 2) -= params().coeR;

  P_ = PBody_;
}

void FrictionalKinematicModel::initializeImpulseGenerator_(const bool & useFrictionCone)
{
  if(useFrictionCone)
  {
    KBody_.resize(3, 4);

    double angle = atan2(params().coeF, 1.0);
    // double angle = atan(0.5);
    // std::cout<<"The angle is: "<<angle * 180 / M_PI <<" degrees."<<std::endl;
    const Vector3d & iNormal = impactNormal();
    // printMatrix("iNormal", iNormal);
    // printMatrix("rotation matrix", AngleAxisd(angle, tangentY_).matrix());
    const Vector3d & generator = AngleAxisd(angle, tangentY_) * iNormal;
    // printMatrix("generator", generator);

    double angleStep = 2 * m_pi / 4;

    for(int ii = 0; ii < 4; ii++)
    {
      KBody_.col(ii) = AngleAxisd(ii * angleStep, iNormal) * generator;
    }
  }
  else
  {
    KBody_.resize(3, 3);
    // Suppose the third coloumn correspond to the impact normal, the other two spans the tangent plane
    KBody_.setIdentity();
  }

  // Initialize K_ to KBody_
  K_ = KBody_;
  // printMatrix("KBody", K_);
  robotPostImpactStates_.K_ = K_;
}

void FrictionalKinematicModel::updateBuildingBlocks_(const bool & bodyCoordinate)
{
  if(!bodyCoordinate)
  {
    K_ = endEffector()->transform().rotation() * KBody_;
    P_ = endEffector()->transform().rotation() * PBody_;
  }
}
const PostImpactStates & FrictionalKinematicModel::objectPostImpactStates()
{
  RoboticsUtils::throw_runtime_error(
      "We can not check the object post-impact state for the Frictional kinematic model.", __FILE__, __LINE__);
}

void FrictionalKinematicModel::update(const Matrix6d & equivalentMass, const Vector6d & av)
{

  // calcCoeStick_();
  coeStick_ = calcCoeStick_(equivalentMass.block<3, 3>(0, 0).partialPivLu().inverse());
  updateImpactNormal_();
  // printMatrix("ImpactNormal", impactNormal());
  // Check the frame consistency of the building-blocks
  updateBuildingBlocks_(params().bodyCoordinates);

  const Vector3d & eeJump = Projector_() * robotContactPointVelocity();

  // printMatrix("eeJump", eeJump);

  const MatrixXd & K = Generator_();
  // const Vector4d & lambda = K.transpose() * KKT.colPivHouseholderQr().solve(equivalentMass.block<3, 3>(0, 0) *
  // eeJump); printMatrix("lambda", lambda);
  printMatrix("Equivalent mass", equivalentMass.block<3, 3>(0, 0));
  // printMatrix("l-impulse", K_ * lambda);

  Vector3d impulse;
  if(useFrictionCone_)
  {
    const Matrix3d KKT = K * K.transpose();
    // impulse = KKT * KKT.colPivHouseholderQr().solve(equivalentMass.block<3, 3>(0, 0) * eeJump);
    impulse = KKT
              * (Matrix3d::Identity() + K * K.transpose())
                    .colPivHouseholderQr()
                    .solve(equivalentMass.block<3, 3>(0, 0) * eeJump);
    // printMatrix("Friction cone ", K);
    // printMatrix("b ", equivalentMass.block<3, 3>(0, 0) * eeJump);
    // printMatrix("Friction cone pseudo inverse:", K.transpose() * (K*K.transpose()).inverse());
    // printMatrix("Friction cone impulse", impulse);
  }
  else
  {
    impulse = K.colPivHouseholderQr().solve(equivalentMass.block<3, 3>(0, 0) * eeJump);
  }
  // const Vector3d & impulse = K.transpose() * KKT.colPivHouseholderQr().solve(equivalentMass.block<3, 3>(0, 0) *
  // eeJump);
  // printMatrix("Meq * du", equivalentMass.block<3, 3>(0, 0) * eeJump);

  writeSolution_(impulse, robotPostImpactStates_);
}

void FrictionalKinematicModel::writeSolution_(const Vector3d & impulse, PostImpactStates & input)
{
  input.impulse = impulse;
  input.linearVelJump = input.impulse / robot()->mass();
}

} // namespace GeoRobotics
