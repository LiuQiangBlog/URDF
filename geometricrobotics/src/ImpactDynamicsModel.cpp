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
#include "GeometricRobotics/ImpactDynamics/ImpactDynamicsModel.h"

namespace GeoRobotics
{

// clang-format off
ImpactDynamicsModel::ImpactDynamicsModel(
		const std::shared_ptr<KinematicChain> robot,
		const std::shared_ptr<EndEffector> ee,
		const ImpactModelParams & params)
: robot_(robot), ee_(ee), params_(params)
// clang-format on 
{
  RoboticsUtils::quickInfo("ImpactDynamicsModel is created.");
}

const PostImpactStates & ImpactDynamicsModel::robotPostImpactStates()
{
  return robotPostImpactStates_;
}

const PostImpactStates & ImpactDynamicsModel::objectPostImpactStates()
{
  return objectPostImpactStates_;
}


/*
double ImpactDynamicsModel::calcCoeStick_(const Matrix3d & iim) const
{

  // Compute coeStick_ by Eq.~4.12 of Impact Mechanics by Stronge 2000 
  auto coe_calc_1 = [](const double & a, const double & b, const double & c, const double &d)->double { 
	  return a * b  - c * d;
  };

  auto coe_calc_2 = [&coe_calc_1](const double & a, const double & b, const double & c, const double &d)->double { 
	  return pow(coe_calc_1(a, b, c, d), 2);
  };


  return sqrt(coe_calc_2(iim(0,0), iim(1,2), iim(0,1), iim(0,2)) + coe_calc_2(iim(1,1), iim(0,2), iim(0,1), iim(1,2)))/coe_calc_1(iim(0,0), iim(1,1), iim(0,1), iim(0,1));

}
*/


double ImpactDynamicsModel::calcCoeStick_(const Matrix3d & iim) const
{

  // Compute coeStick_ = ||B^-1 * d|| in line with Yan-bin.
  

  MatrixXd tangentialMatrix;
  tangentialMatrix.resize(3,2);

  tangentialMatrix.block<3,1>(0, 0) = tangentialX();
  tangentialMatrix.block<3,1>(0, 1) = tangentialY();


  Eigen::Matrix2d B;
  B = tangentialMatrix.transpose() * iim * tangentialMatrix;

  Eigen::Vector2d d;
  d = tangentialMatrix.transpose() * iim * impactNormal();

  Eigen::Vector2d miu_stick = B.colPivHouseholderQr().solve(d);

  return miu_stick.norm();

}


//double ImpactDynamicsModel::calcCoeStick_(const Matrix3d & iim) const
//{
//
//  // Compute coeStick_ = ||B^-1 * d||
//  
//
//  MatrixXd tangentialMatrix;
//  tangentialMatrix.resize(3,1);
//
//  tangentialMatrix.block<3,1>(0, 0) = tangentialY();
//
//
//  double B = (tangentialMatrix.transpose() * iim * tangentialMatrix)(0);
//
//  double d = (tangentialMatrix.transpose() * iim * impactNormal())(0);
//
//  return (1/B * d);
//}


TwoDimModelBridge::TwoDimModelBridge(const std::shared_ptr<KinematicChain> robotPtr, 
		const std::shared_ptr<EndEffector> ee,
		const ImpactModelParams & params,
		bool useProjectedMass
		)
: ImpactDynamicsModel(robotPtr, ee, params), case_(params.modelCase), useProjectedMass_(useProjectedMass)
{

  // Initialize the two-dim-model
  // Energetic coefficient of restitution
  piParams_.e = params.coeR;
  // Coefficient of friction
  piParams_.miu = params.coeF;
  // rotation_.setZero();
  // rotationFull_.setIdentity();

  twoDimModelPtr_.reset(new FIDynamics::TwoDimModel(piParams_));

  RoboticsUtils::quickInfo("TwoDimModelBridge is created.");
}

void TwoDimModelBridge::update(const Matrix6d & inertia,
                               const Vector6d & v)
{
   if(projectedMass_())
   {
     updateWithPI_(inertia, v);
   }
   else
   {
     updateWithCI_(inertia, v);
   }
}
void TwoDimModelBridge::updateWithPI_(
                               const Matrix6d & inertia,
                               const Vector6d & v)
{

 // Supoose the z-axis of the end-effector defines the impact normal.
 
  // By default, Jia's paper work with spatial representations.
  updateSpatialImpactNormal_();

  printMatrix("impactNormal", impactNormal());

  rAverageLinearVel_ = v.head(3);
  rAverageAngularVel_ = v.tail(3);
  rCentroidalInertia_ = inertia.block<3, 3>(3, 3);

  // The transform from COM to the contact/impact point
  const Iso3d & tf = robot()->centroidalTransform().inverse() * endEffector()->transform();

  // std::cout<<"vc parameters updated"<<std::endl;

  params_.eePosition = tf.translation();
  params_.eeRotation = tf.rotation();

  // (3) Update the twoDim model
  //updatePiParams_(params().eePosition, params().eeRotation);

  //projectPiParams_(robotContactPointVelocity(), params().eePosition, 2);

  const Vector3d & result = RoboticsUtils::vectorOnSurface(impactNormal(), Eigen::Vector3d::UnitZ());
  piParams_.nu = result.segment<2>(0);
  piParams_.contactPoint = RoboticsUtils::vectorOnSurface(params().eePosition, Eigen::Vector3d::UnitZ()).segment<2>(0);

  Vector3d offset; 
  offset<< 0.01, 0.01, 0.01;
  piParams_.batParams.com = RoboticsUtils::vectorOnSurface(params().eePosition + offset, Eigen::Vector3d::UnitZ()).segment<2>(0);
  piParams_.batParams.preVel =
      RoboticsUtils::vectorOnSurface(
		      endEffector()->pointVelocity(false),
		      Eigen::Vector3d::UnitZ()).segment<2>(0);


  // case 2: // X-Y plane with surface normal z: 
  
  // (1) Rest of the bat information
  piParams_.batParams.name = "robot";
  piParams_.batParams.mass =  inertia(2,2);
  piParams_.batParams.inertia = inertia(5,5);
  piParams_.batParams.preW = v(5);
  // (2) Wall
  piParams_.objectParams.preVel << 0.0, 0.0;
  piParams_.objectParams.preW = 0.0;
  piParams_.objectParams.name = "wall";
  piParams_.objectParams.mass = std::numeric_limits<double>::max();
  piParams_.objectParams.inertia = std::numeric_limits<double>::max();
  piParams_.objectParams.com = piParams_.contactPoint;


  printMatrix("Planar impact direction", piParams_.nu);
  // Eigen::Vector2d rotatedZ = rotation_*Eigen::Vector3d::UnitZ();
  piParams_.tu(0) = -piParams_.nu(1);
  piParams_.tu(1) = piParams_.nu(0);

  printMatrix("Planar tangential direction", piParams_.tu);




  // Do not need to modify
  twoDimModelPtr_->updateParams(planarImpactParams());
  // std::cout<<"twoDimModelPtr_->updated params "<<std::endl;
  twoDimModelPtr_->update();

  // (4) Convert the twoDim model solution back to 3D:
  // Keep the same as the CI case
  planarSolutionTo3D_();
} // end of update


void TwoDimModelBridge::updateWithCI_(
                               const Matrix6d & centroidalInertia,
                               const Vector6d & av)
// void TwoDimModelBridge::update(const Eigen::Vector3d & impactNormal, const Eigen::Vector3d & impactLinearVel)
{

 // Supoose the z-axis of the end-effector defines the impact normal.
 
  //updateImpactNormal_();
  
  // By default, Jia's paper work with spatial representations.
  updateSpatialImpactNormal_();

  printMatrix("impactNormal", impactNormal());

  // std::cout<<"Updating TwoDimModelBridge"<<std::endl;
  // (0) Compute the whole-body inertia and average velocity
  // const Matrix6d & centroidalInertia = robot()->computeCentroidalInertia();
  // const MatrixXd & cmm = robot()->computeCMM();
  // Vector6d av = centroidalInertia.colPivHouseholderQr().solve(cmm * robot()->jointVelocities());

  // std::cout<<"inertia updated"<<std::endl;
  // Assert that the average com velocity is equal to the com velocity
  
  //assert(mc_impact::areSame(av(0), getRobot().comVelocity()(0)));
  //assert(mc_impact::areSame(av(1), getRobot().comVelocity()(1)));
  //assert(mc_impact::areSame(av(2), getRobot().comVelocity()(2)));
  

  rAverageLinearVel_ = av.head(3);
  rAverageAngularVel_ = av.tail(3);
  rCentroidalInertia_ = centroidalInertia.block<3, 3>(3, 3);

  // std::cout<<"The average angular velocity is: "<<rAverageAngularVel_.transpose()<<std::endl;
  // std::cout<<"The average linear velocity is: "<<getRobot().comVelocity().transpose()<<std::endl;

  // The transform from COM to the contact/impact point
  const Iso3d & tf = robot()->centroidalTransform().inverse() * endEffector()->transform();

  // std::cout<<"vc parameters updated"<<std::endl;

  params_.eePosition = tf.translation();
  params_.eeRotation = tf.rotation();

  // (3) Update the twoDim model
  updatePiParams_(params().eePosition, params().eeRotation);

  twoDimModelPtr_->updateParams(planarImpactParams());
  // std::cout<<"twoDimModelPtr_->updated params "<<std::endl;
  twoDimModelPtr_->update();

  // std::cout<<"twoDimModelPtr_->updated "<<std::endl;
  // (4) Convert the twoDim model solution back to 3D:
  planarSolutionTo3D_();
} // end of update

void TwoDimModelBridge::planarSolutionTo3D_()
{
  switch(getCase_())
  {
    case TwoDimModelCase::HRP4PushWall:
      // Convert the solution using the Push-Wall assumptions
      planarSolutionTo3D_(robotPostImpactStates_, 2);
      break;
    case TwoDimModelCase::PandaPushWall:
      planarSolutionTo3D_(robotPostImpactStates_, 2);
      break;
    case TwoDimModelCase::PandaPushTable:
      planarSolutionTo3D_(robotPostImpactStates_, 0);
      break;
    default:
      RoboticsUtils::throw_runtime_error("The assumptions are not set for the TwoDimModelBridge.", __FILE__, __LINE__);
  }
}

const PostImpactStates & TwoDimModelBridge::objectPostImpactStates()
{
  switch(getCase_())
  {
    case TwoDimModelCase::HRP4PushWall:
      // In this case the object(wall)  is supposed to be stationary.
      RoboticsUtils::throw_runtime_error(
          "In the HRP4PushWall case, the wall is stationary. Thus there is no need to check its post-impact states.",
          __FILE__, __LINE__);
      exit(-1);
    default:
      return objectPostImpactStates_;
  }
}

void TwoDimModelBridge::planarSolutionTo3D_(PostImpactStates & input, const int & idx)
{
  switch(idx)
  {
    case 0: // Y-Z plane with surface normal x:
	    {
    input.impulse.y() = -twoDimModelPtr_->getSolution().I_r.x();
    input.impulse.z() = -twoDimModelPtr_->getSolution().I_r.y();
    input.linearVel.y() = twoDimModelPtr_->getImpactBodies().first.postVel.x();
    input.linearVel.z() = twoDimModelPtr_->getImpactBodies().first.postVel.y();


		    break;
	    }
    case 1: // X-Z plane with surface normal y:
	    {
    input.impulse.x() = -twoDimModelPtr_->getSolution().I_r.x();
    input.impulse.z() = -twoDimModelPtr_->getSolution().I_r.y();
    input.linearVel.x() = twoDimModelPtr_->getImpactBodies().first.postVel.x();
    input.linearVel.z() = twoDimModelPtr_->getImpactBodies().first.postVel.y();


		    break;
	    }
    case 2: // X-Y plane with surface normal z: 
	    {
    input.impulse.x() = -twoDimModelPtr_->getSolution().I_r.x();
    input.impulse.y() = -twoDimModelPtr_->getSolution().I_r.y();
    input.linearVel.x() = twoDimModelPtr_->getImpactBodies().first.postVel.x();
    input.linearVel.y() = twoDimModelPtr_->getImpactBodies().first.postVel.y();

    break;
	    }
    default:
      RoboticsUtils::throw_runtime_error("The case: " + std::to_string(idx) + " is not known for projection.", __FILE__, __LINE__);
  }
  // Convert the post-impact impulse:
  // The robot applies the impulse "I", thus it receives impulse "-I".
  // input.impulse = -rotation_.transpose() * twoDimModelPtr_->getSolution().I_r;

  // Fix the angle: if the impulse has a different sign than the impact normal, we flip it.
  double innerP = -twoDimModelPtr_->getSolution().I_r.transpose() * planarImpactParams().nu;
  if(innerP < 0)
  {
    input.impulse = -input.impulse;
  }

  // std::cout<<"I_nr is: "<< twoDimModelPtr_->getSolution().I_nr<<std::endl;
  // std::cout<<"I_nc is: "<< twoDimModelPtr_->getSolution().I_nc<<std::endl;
  // std::cout<<"I_r is: "<< twoDimModelPtr_->getSolution().I_r.transpose()<<std::endl;
  // std::cout<<"The impulse is: "<< robotPostImpactStates_.impulse<<std::endl;
  // Convert the post-impact velocities:
  // robot:
  // input.linearVel = rotation_.transpose() * twoDimModelPtr_->getImpactBodies().first.postVel;
  //input.linearVel.x() = twoDimModelPtr_->getImpactBodies().first.postVel.x();
  //input.linearVel.y() = twoDimModelPtr_->getImpactBodies().first.postVel.y();
  input.linearVelJump = input.impulse / robot()->mass();
  // std::cout<<RoboticsUtils::info<<"linear Vel jump:
  // "<<input.linearVelJump.transpose()<<RoboticsUtils::reset<<std::endl;
  // std::cout<<"The post-imapct linear velocity is: "<< twoDimModelPtr_->getImpactBodies().first.postVel<<std::endl;
  //  Compute: wJump = (1 / params().inertia) * cross2(r, I_r);
  Eigen::Vector3d rb;
  rb = params().eePosition - robot()->com();
  input.anguleVelJump = rCentroidalInertia_.inverse() * rb.cross(input.impulse);
  input.anguleVel = rAverageAngularVel_ + input.anguleVelJump;
}

void TwoDimModelBridge::printResult() const
{
  std::cout << RoboticsUtils::info << "I_nr is: " << twoDimModelPtr_->getSolution().I_nr << RoboticsUtils::reset
            << std::endl;
  std::cout << RoboticsUtils::info << "I_nc is: " << twoDimModelPtr_->getSolution().I_nc << RoboticsUtils::reset
            << std::endl;
  std::cout << RoboticsUtils::info << "I_r is: " << twoDimModelPtr_->getSolution().I_r.transpose()
            << RoboticsUtils::reset << std::endl;
  std::cout << RoboticsUtils::info << "The impulse is: " << robotPostImpactStates_.impulse << RoboticsUtils::reset
            << std::endl;

  std::cout << RoboticsUtils::info
            << "The post-imapct linear velocity is: " << twoDimModelPtr_->getImpactBodies().first.postVel
            << RoboticsUtils::reset << std::endl;
}
void TwoDimModelBridge::printPIParams() const
{
  std::cout << RoboticsUtils::info << "The nu is: " << piParams_.nu.transpose() << RoboticsUtils::reset << std::endl;
  std::cout << RoboticsUtils::info << "The tu is: " << piParams_.tu.transpose() << RoboticsUtils::reset << std::endl;

  std::cout << RoboticsUtils::info << "The rotated contact point is: " << piParams_.contactPoint.transpose()
            << RoboticsUtils::reset << std::endl;

  std::cout << RoboticsUtils::info << "The rotated com is: " << piParams_.batParams.com.transpose()
            << RoboticsUtils::reset << std::endl;

  std::cout << RoboticsUtils::info << "The bat inertia is: " << piParams_.batParams.inertia << RoboticsUtils::reset
            << std::endl;

  std::cout << RoboticsUtils::info << "The bat preimpact vel is: " << piParams_.batParams.preVel << RoboticsUtils::reset
            << std::endl;

  std::cout << RoboticsUtils::info << "The bat preimpact angular vel is: " << piParams_.batParams.preW
            << RoboticsUtils::reset << std::endl;
}

void TwoDimModelBridge::projectPiParams_(const Vector3d & robotVel, const Eigen::Vector3d & eeP, const int & idx)
{
  switch(idx)
  {
    case 0: // Y-Z plane with surface normal x:
	    {const Vector3d & result = RoboticsUtils::vectorOnSurface(impactNormal(), Eigen::Vector3d::UnitX());
      piParams_.nu = result.segment<2>(1);

      piParams_.contactPoint = RoboticsUtils::vectorOnSurface(eeP, Eigen::Vector3d::UnitX()).segment<2>(1);

      piParams_.batParams.com = RoboticsUtils::vectorOnSurface(robot()->com(), Eigen::Vector3d::UnitX()).segment<2>(1);
      piParams_.batParams.preVel =
      RoboticsUtils::vectorOnSurface(robotVel, Eigen::Vector3d::UnitX()).segment<2>(1);

      break;
	    }
    case 1: // X-Z plane with surface normal y:
	    {
	 const Vector3d & result = RoboticsUtils::vectorOnSurface(impactNormal(), Eigen::Vector3d::UnitY());
      piParams_.nu(0) = result(0);
      piParams_.nu(1) = result(2);

      const Vector3d & rP = RoboticsUtils::vectorOnSurface(eeP, Eigen::Vector3d::UnitY());
      piParams_.contactPoint(0) = rP(0);
      piParams_.contactPoint(1) = rP(2);

      const Vector3d rCom = RoboticsUtils::vectorOnSurface(robot()->com(), Eigen::Vector3d::UnitY());
      piParams_.batParams.com(0) = rCom(0);
      piParams_.batParams.com(1) = rCom(2);

      const Vector3d rV = RoboticsUtils::vectorOnSurface(robotVel, Eigen::Vector3d::UnitY());
      piParams_.batParams.preVel(0) = rV(0);
      piParams_.batParams.preVel(1) = rV(2);

      break;
	    }
    case 2: // X-Y plane with surface normal z: 
	    {const Vector3d & result = RoboticsUtils::vectorOnSurface(impactNormal(), Eigen::Vector3d::UnitZ());
      piParams_.nu = result.segment<2>(0);
      piParams_.contactPoint = RoboticsUtils::vectorOnSurface(eeP, Eigen::Vector3d::UnitZ()).segment<2>(0);

      piParams_.batParams.com = RoboticsUtils::vectorOnSurface(robot()->com(), Eigen::Vector3d::UnitZ()).segment<2>(0);
      piParams_.batParams.preVel =
      RoboticsUtils::vectorOnSurface(robotVel, Eigen::Vector3d::UnitZ()).segment<2>(0);

      break;
	    }
    default:
      RoboticsUtils::throw_runtime_error("The case: " + std::to_string(idx) + " is not known for projection.", __FILE__, __LINE__);
  }

  // (1) Rest of the bat information
  piParams_.batParams.name = "robot";
  piParams_.batParams.mass = robot()->mass();
  piParams_.batParams.inertia = rCentroidalInertia_(idx, idx);
  piParams_.batParams.preW = rAverageAngularVel_(idx);
  // (2) Wall
  piParams_.objectParams.preVel << 0.0, 0.0;
  piParams_.objectParams.preW = 0.0;
  piParams_.objectParams.name = "wall";
  piParams_.objectParams.mass = std::numeric_limits<double>::max();
  piParams_.objectParams.inertia = std::numeric_limits<double>::max();
  piParams_.objectParams.com = piParams_.contactPoint;


  printMatrix("Planar impact direction", piParams_.nu);
  // Eigen::Vector2d rotatedZ = rotation_*Eigen::Vector3d::UnitZ();
  piParams_.tu(0) = -piParams_.nu(1);
  piParams_.tu(1) = piParams_.nu(0);

  printMatrix("Planar tangential direction", piParams_.tu);


  }

void TwoDimModelBridge::updatePiParams_(
                                        const Eigen::Vector3d & eeP,
                                        const Eigen::Matrix3d & eeR)
{
  
  // Evaluate piParams.nu, piParams.tu, and piParams_.contactPoint 
  switch(getCase_())
  {
    case TwoDimModelCase::HRP4PushWall:
      // Apply impact dynamics in the X-Y plane 
      projectPiParams_(robotCOMVelocity(), eeP, 2);
      //projectPiParams_(robotContactPointVelocity(), eeP, 2);
      break;
    case TwoDimModelCase::PandaPushWall:
      // Apply impact dynamics in the X-Y plane 
      projectPiParams_(robotCOMVelocity(), eeP, 2);
      //projectPiParams_(robotContactPointVelocity(), eeP, 2);
      break;
    case TwoDimModelCase::PandaPushTable:
      // Apply impact dynamics in the Y-Z plane 
      projectPiParams_(robotCOMVelocity(), eeP, 0);
      //projectPiParams_(robotContactPointVelocity(), eeP, 1);
      break;
    default:
      RoboticsUtils::throw_runtime_error("The assumptions are not set for the TwoDimModelBridge.", __FILE__, __LINE__);
  }
  // Z-axis in the inertial frame:
  // const Vector3d & iNormal = eeR.rightCols(1);
  //RoboticsUtils::vectorOnSurface(eeP, Eigen::Vector3d::UnitZ()).block<2, 1>(0, 0);
  //piParams_.nu = RoboticsUtils::projectToGround(impactNormal()).block<2, 1>(0, 0);
  //const Vector3d & contactVel = robotContactPointVelocity();
  /*
  Vector3d projectionSurfaceNormal;

  if(fabs(contactVel.x()) > fabs(contactVel.y()))
  {
    projectionSurfaceNormal = eeR.col(1);
  }
  else
  {
    projectionSurfaceNormal = eeR.col(0);
  }
  */

  //piParams_.nu = RoboticsUtils::projectToSurface(iNormal, ).block<2, 1>(0, 0);
    // std::cout<<RoboticsUtils::info<<"The nu is: "<<piParams_.nu.transpose()<<RoboticsUtils::reset<<std::endl;
  // std::cout<<RoboticsUtils::info<<"The tu is: "<<piParams_.tu.transpose()<<RoboticsUtils::reset<<std::endl;

  // (2) Contact Point:
  //piParams_.contactPoint = RoboticsUtils::vectorOnSurface(eeP, Eigen::Vector3d::UnitZ()).block<2, 1>(0, 0);

  // std::cout<<RoboticsUtils::info<<"The contact point is: "<<piParams_.contactPoint.transpose()<<std::endl;
  // (3) Parmams of the bat and the object:
  //switch(getCase_())
  //{
  //  case TwoDimModelCase::HRP4PushWall:
  //    // Update the parameters using the Push-Wall assumptions
  //    paramUpdateHRP4PushWall_();
  //    break;
  //  default:
  //    RoboticsUtils::throw_runtime_error("The assumptions are not set for the TwoDimModelBridge.", __FILE__, __LINE__);
  //}
}

void TwoDimModelBridge::paramUpdateHRP4PushWall_()
{
  // Bat is supposed to be the robot:
  // (1) Robot
  piParams_.batParams.com = RoboticsUtils::vectorOnSurface(robot()->com(), Eigen::Vector3d::UnitZ()).block<2, 1>(0, 0);
  // piParams_.batParams.com = rotation_ * getRobot()->com();
  // std::cout<<RoboticsUtils::info<<"The impact com is:
  // "<<piParams_.batParams.com.transpose()<<RoboticsUtils::reset<<std::endl;
  piParams_.batParams.mass = robot()->mass();
  // Get the z-axis diagonal element:
  // piParams_.batParams.inertia = (rotationFull_ * rCentroidalInertia_)(2, 2);
  piParams_.batParams.inertia = rCentroidalInertia_(2, 2);
  // std::cout<<green<<"The bat inertia is: "<<piParams_.batParams.inertia<<std::endl;

  // std::cout<<RoboticsUtils::error<<"The applied impactLinearVel is:
  // "<<impactLinearVel.transpose()<<RoboticsUtils::reset<<std::endl;

  // std::cout<<"Projection is: "<<RoboticsUtils::vectorOnSurface(impactLinearVel,
  // Eigen::Vector3d::UnitZ()).transpose()<<std::endl;
  piParams_.batParams.preVel =
      RoboticsUtils::vectorOnSurface(robotContactPointVelocity(), Eigen::Vector3d::UnitZ()).block<2, 1>(0, 0);
  // piParams_.batParams.preVel = rotation_ * impactLinearVel;

  // std::cout<<RoboticsUtils::error<<"The bat preimpact vel is:
  // "<<piParams_.batParams.preVel.transpose()<<RoboticsUtils::reset<<std::endl;
  // Get the z-axis average angular velocity:
  piParams_.batParams.preW = rAverageAngularVel_(2);
  // std::cout<<RoboticsUtils::info<<"The bat preimpact angular vel is:
  // "<<piParams_.batParams.preW<<RoboticsUtils::reset<<std::endl;

  piParams_.batParams.name = "robot";

  // Object is suppose to be the wall:

  // (2) Wall
  piParams_.objectParams.preVel << 0.0, 0.0;
  piParams_.objectParams.preW = 0.0;
  piParams_.objectParams.name = "wall";

  // mass and inertia of the wall are set to be infinite.
  // piParams_.objectParams.mass = std::numeric_limits<double>::infinity();
  piParams_.objectParams.mass = std::numeric_limits<double>::max();
  // piParams_.objectParams.inertia = std::numeric_limits<double>::infinity();
  piParams_.objectParams.inertia = std::numeric_limits<double>::max();

  // com of the wall is set to be the contact point such that r = cp - com == 0.
  // Suppose that piParams_.contactPoint is already set.
  piParams_.objectParams.com = piParams_.contactPoint;
}

} // namespace GeoRobotics
