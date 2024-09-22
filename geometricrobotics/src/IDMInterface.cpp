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
#include "GeometricRobotics/ImpactDynamics/IDMInterface.h"
namespace GeoRobotics
{

IDMInterface::IDMInterface(const std::shared_ptr<KinematicChain> robot,
                           const std::shared_ptr<EndEffector> ee,
                           const IDMInterfaceParams & params)
: robot_(robot), ee_(ee), params_(params)
{

  // 1. Create the impact-dynamics model
  switch(params_.idModel)
  {
    case IDModel::FrictionalPlanar:
    {
      idModel_ = std::make_shared<TwoDimModelBridge>(robot, ee, params.idParams, false);
      break;
    }
    case IDModel::FrictionalPlanarEM:
    {
      idModel_ = std::make_shared<TwoDimModelBridge>(robot, ee, params.idParams, true);
      break;
    }
    case IDModel::Kinematic:
    {
      idModel_ = std::make_shared<OneDimKinematicModel>(robot, ee, params.idParams);
      break;
    }
    case IDModel::Frictionless3DAssembly:
    {
      idModel_ = std::make_shared<FLAssemblyModelBridge>(robot, ee, params.idParams);
      break;
    }
    case IDModel::Frictionless3D:
    {
      idModel_ = std::make_shared<FLModelBridge>(robot, ee, params.idParams);
      break;
    }
    case IDModel::Frictionless3DCRB:
    {
      idModel_ = std::make_shared<FLCRBModelBridge>(robot, ee, params.idParams);
      break;
    }
    case IDModel::Frictionless3DCRB_FB:
    {
      idModel_ = std::make_shared<FL_CRB_FB_ModelBridge>(robot, ee, params.idParams);
      break;
    }
    case IDModel::Frictionless3DCRB_RV:
    {
      idModel_ = std::make_shared<FL_CRB_RV_ModelBridge>(robot, ee, params.idParams);
      break;
    }
    case IDModel::Frictionless3DGM:
    {
      idModel_ = std::make_shared<FL_GM_ModelBridge>(robot, ee, params.idParams);
      break;
    }
    case IDModel::FrictionalKinematic:
    {
      idModel_ = std::make_shared<FrictionalKinematicModel>(robot, ee, params.idParams, false);
      break;
    }
    case IDModel::FrictionConeKinematic:
    {
      idModel_ = std::make_shared<FrictionalKinematicModel>(robot, ee, params.idParams, true);
      break;
    }
    default:
    {
      // clang-format off
      RoboticsUtils::throw_runtime_error("The Impact Dynamics type: " + ToString(params.idModel) + " is not defined.", __FILE__, __LINE__);
      // clang-format on
    }
  }

  RoboticsUtils::quickHL("Initialized the impact dynamics model.");

  // 2. Initialize the state-jump predictors
  initializeStateJumpPredictors_(ee);
}

void IDMInterface::initializeStateJumpPredictors_(const std::shared_ptr<EndEffector> ee)
{
  ppParams_.reset();
  pParams_.reset();

  if(params().idParams.bodyCoordinates)
  {
    ppParams_.spatialFrame = false;
    pParams_.spatialFrame = false;
  }

  pParams_.sensorTF = ee->relativeTransform();
  ppParams_.sensorTF = ee->relativeTransform();

  p_ = std::make_shared<PostImpactPredictor>(robot_, pParams_);
  pp_ = std::make_shared<ProjectionPredictor>(robot_, ppParams_);

  RoboticsUtils::quickHL("Initialized the post-impact state predictors.");
}

void IDMInterface::update()
{
  // 1. Update the equivalent mass at the impact application point.
  updateEM_();
  // 2. Update the impulse by the impact model.
  updateID_();

  // 3. Update the post-impact state jumps
  bool sensorFrame = !ppParams_.spatialFrame;
  Vector6d input = Vector6d::Zero();
  input.head(3) = robotPostImpactStates().impulse;
  pp_->compute(getEndEffector(), -input, sensorFrame);
  p_->compute(-input, sensorFrame);
}

void IDMInterface::updateID_()
{
  switch(params().idModel)
  {
    case IDModel::FrictionalPlanar:
    {
      const Matrix6d & ci = robot()->computeCentroidalInertia();
      idModel_->update(ci, robot()->computeAverageVelocity(ci, robot()->computeCMM()));
      break;
    }
    case IDModel::FrictionalPlanarEM:
    {
      const Matrix6d & ci = robot()->computeCentroidalInertia();
      idModel_->update(eMass(), robot()->computeAverageVelocity(ci, robot()->computeCMM()));
      break;
    }
    case IDModel::Frictionless3DAssembly:
    {
      // This model does not need equivalent mass or additional velocity.
      idModel_->update(Matrix6d::Identity(), Vector6d::Zero());
      break;
    }
    case IDModel::Frictionless3D:
    {
      // This model does not need equivalent mass or additional velocity.
      idModel_->update(Matrix6d::Identity(), Vector6d::Zero());
      break;
    }
    case IDModel::Frictionless3DCRB:
    {
      // This model does not need equivalent mass or additional velocity.
      idModel_->update(Matrix6d::Identity(), Vector6d::Zero());
      break;
    }
    case IDModel::Frictionless3DCRB_FB:
    {
      // This model does not need equivalent mass or additional velocity.
      idModel_->update(Matrix6d::Identity(), Vector6d::Zero());
      break;
    }
    case IDModel::Frictionless3DCRB_RV:
    {
      // This model does not need equivalent mass or additional velocity.
      idModel_->update(Matrix6d::Identity(), Vector6d::Zero());
      break;
    }
    case IDModel::Frictionless3DGM:
    {
      // This model does not need equivalent mass or additional velocity.
      idModel_->update(Matrix6d::Identity(), Vector6d::Zero());
      break;
    }
    case IDModel::Kinematic:
    {
      const Matrix6d & ci = robot()->computeCentroidalInertia();
      // The second argument is not used for the kinematic model
      idModel_->update(eMass(), robot()->computeAverageVelocity(ci, robot()->computeCMM()));
      break;
    }
    case IDModel::FrictionalKinematic:
    {
      // The second argument is not used for the kinematic model
      idModel_->update(eMass(), Vector6d::Zero());
      break;
    }
    case IDModel::FrictionConeKinematic:
    {
      // The second argument is not used for the kinematic model
      idModel_->update(eMass(), Vector6d::Zero());
      break;
    }
    default:
    {
      // clang-format off
      RoboticsUtils::throw_runtime_error("The Impact Dynamics type: " + ToString(params().idModel) + " is not defined.", __FILE__, __LINE__);
      // clang-format on
    }
  }
}
void IDMInterface::updateEM_()
{
  switch(params().emChoice)
  {
    case EMChoice::Projection:
    {
      // Use the origin as the point: q
      computeEMbyProjection(eM_);
      break;
    }
    case EMChoice::CRB:
    {
      computeEMbyCRB(eM_);
      break;
    }
    default:
    {
      // clang-format off
      RoboticsUtils::throw_runtime_error("The equivalent mass type: " + ToString(params().emChoice) + " is not defined.", __FILE__, __LINE__);
      // clang-format on
    }
  }
}

void IDMInterface::metaCalcEMProjection_(const MatrixXd & J, const MatrixXd & M, Matrix6d & em)
{
  em = (J * M.inverse() * J.transpose()).inverse();
}

void IDMInterface::computeEMbyProjection(Matrix6d & em)
{
  if(params().idParams.bodyCoordinates)
  {
    // const MatrixXd & J = robot()->bodyJacobian();
    const MatrixXd & J = getEndEffector()->bodyJacobian();
    const MatrixXd & M = robot()->M();
    metaCalcEMProjection_(J, M, em);
  }
  else
  {
    // const MatrixXd & J = robot()->spatialJacobian();
    const MatrixXd & J = getEndEffector()->spatialJacobian();
    const MatrixXd & M = robot()->M();
    metaCalcEMProjection_(J, M, em);
  }
}

void IDMInterface::computeEMbyCRB(Matrix6d & em)
{
  // Applies the centroidal inertia
  em = robot()->computeCentroidalInertia();
}

/*
std::string IDMInterface::saveData()
{
  using namespace RoboticsUtils;
  std::ofstream myfile;
  time_t t = time(0); // get time now
  struct tm * now = localtime(&t);
  char buffer[80];
  strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", now);

  const IDMInterfaceParams & iParams = params();

  myfile << "EquivalentMass: " << ToString(iParams.emChoice) << "\n";
  myfile << "ImpactDynamicsModel: " << ToString(iParams.idModel) << "\n";

  const ImpactModelParams & p = impactDynamicsModel()->params();

  std::string name = "PostImpactStates_" + std::string(buffer) + ".yaml";
  myfile.open(name);
  std::string space(" ");
  std::string tripleSpace("  ");

  myfile << "ImpactDynamicsModelConfiguration:\n";
  myfile << tripleSpace + "BodyCoordinate: " << p.bodyCoordinates << "\n";
  myfile << tripleSpace + "FrictionCoefficient: " << p.coeF << "\n";
  myfile << tripleSpace + "RestitutionCoefficient: " << p.coeR << "\n";
  myfile << tripleSpace + "ImpactDuration: " << p.iDuration << "\n";
  myfile << tripleSpace + "TimeStep: " << p.timeStep << "\n";

  myfile << "ImpactDynamics:\n";

  auto id = impactDynamicsModel();

  EigenVecToYaml(myfile, tripleSpace + "contactVelocity", id->robotContactPointVelocity());
  EigenVecToYaml(myfile, tripleSpace + "impulse", id->robotPostImpactStates().impulse);
  EigenVecToYaml(myfile, tripleSpace + "linearVelJump", id->robotPostImpactStates().linearVelJump);

  myfile << "ProjectionPredictor:\n";

  EigenVecToYaml(myfile, tripleSpace + "jointVelJump", projectionPredictor()->dq());
  EigenVecToYaml(myfile, tripleSpace + "jointTorqueJump", projectionPredictor()->tau());
  EigenVecToYaml(myfile, tripleSpace + "eevJump", projectionPredictor()->eeV());

  myfile << "AssemblyPredictor:\n";
  myfile << space + "jointSpaceJump:\n";

  EigenVecToYaml(myfile, tripleSpace + "jointVel", postImpactPredictor()->jv().dq);
  EigenVecToYaml(myfile, tripleSpace + "jointTorque", postImpactPredictor()->jv().tau);
  EigenVecToYaml(myfile, tripleSpace + "actuationWrench", postImpactPredictor()->jv().jaw);
  EigenVecToYaml(myfile, tripleSpace + "constraintWrench", postImpactPredictor()->jv().jcw);
  EigenVecToYaml(myfile, tripleSpace + "lambda", postImpactPredictor()->jv().lambda);
  EigenVecToYaml(myfile, tripleSpace + "vel", postImpactPredictor()->jv().vel);

  myfile << space + "linkSpaceJump:\n";
  EigenVecToYaml(myfile, tripleSpace + "vel", postImpactPredictor()->lv().vel);
  EigenVecToYaml(myfile, tripleSpace + "wrench", postImpactPredictor()->lv().w);

  RoboticsUtils::quickInfo("Wrote to file: " + name);
  myfile.close();
  return name;
}
*/
} // namespace GeoRobotics
