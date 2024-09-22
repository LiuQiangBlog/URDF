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

#include "GeometricRobotics/Kinematics/KinematicChain.h"
//#include <RobotInterface/RobotInterface.h>
#include <RoboticsUtils/utils.h>
#include <TwoDimModel/TwoDimModel.h>

namespace GeoRobotics
{

enum class TwoDimModelCase
{
  HRP4PushWall, // Impact dynamics plane: X-Y, with normal Eigen::Vector3d::UnitZ()

  PandaPushWall, // Impact dynamics plane: X-Y, with normal Eigen::Vector3d::UnitZ()
  PandaPushTable, // Impact dynamics plane: X-Z, with normal Eigen::Vector3d::UnitY()
};

struct ImpactModelParams
{
  bool bodyCoordinates = true;
  double iDuration = 0.005;
  double timeStep = 0.005;
  double coeF = 0.8;
  double coeR = 0.2;
  int dim = 3;
  Eigen::Vector3d eePosition = Eigen::Vector3d::Zero();
  Eigen::Matrix3d eeRotation = Eigen::Matrix3d::Zero();

  TwoDimModelCase modelCase = TwoDimModelCase::HRP4PushWall;

}; // End of the ImpactModelParams.

struct PostImpactStates
{
  Eigen::Vector3d linearVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d linearVelJump = Eigen::Vector3d::Zero();
  Eigen::Vector3d anguleVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d anguleVelJump = Eigen::Vector3d::Zero();
  Eigen::Vector3d impulse = Eigen::Vector3d::Zero();
  Eigen::VectorXd deltaDq;
  Eigen::VectorXd deltaTau;
  MatrixXd K_;
  // double c = 0.0;
  // double c0 = 0.0;
  // double c1 = 0.0;
};

class ImpactDynamicsModel
{
public:
  /*! \brief Interface to frictional/frictionless impact dynamics models
   * \param robot The robot that applies impact.
   * \param ee The impacting end-effector, whose transform() denotes the impact point. Supoose the z-axis of the
   * end-effector defines the impact normal, i.e., tf.rotation().
   *
   */
  ImpactDynamicsModel(const std::shared_ptr<KinematicChain> robot,
                      const std::shared_ptr<EndEffector> ee,
                      const ImpactModelParams & params);

  virtual ~ImpactDynamicsModel()
  {
    RoboticsUtils::quickInfo("Destructing ImpactDynamicsModel.");
  }

  virtual const PostImpactStates & robotPostImpactStates();
  virtual const PostImpactStates & objectPostImpactStates();

  const Vector3d & impactNormal() const
  {
    return impactNormal_;
  }
  const Vector3d & tangentialX() const
  {
    return tangentX_;
  }
  const Vector3d & tangentialY() const
  {
    return tangentY_;
  }

  const Vector3d & robotCOMVelocity() const
  {
    return robot()->comVelocity();
  }
  const Vector3d & robotContactPointVelocity() const
  {
    return endEffector()->pointVelocity(params().bodyCoordinates);
  }

  inline const std::shared_ptr<KinematicChain> robot() const
  {
    return robot_;
  }
  inline const std::shared_ptr<EndEffector> endEffector() const
  {
    return ee_;
  }
  inline const ImpactModelParams & params() const
  {
    return params_;
  }

  // virtual void update(const Eigen::Vector3d & impactNormal, const Eigen::Vector3d & impactLinearVel) = 0;

  // virtual void update(const Eigen::Vector3d & impactNormal, const Eigen::Vector3d & impactLinearVel, const Matrix6d &
  // centroidalInertia) = 0;

  virtual void update(const Matrix6d & centroidalInertia, const Vector6d & av) = 0;

  // virtual void update(const Eigen::Vector3d & impactNormal, const
  // std::shared_ptr<mc_impact::McZMPArea<Eigen::Vector2d>> mcZMPAreaPtr_, StandingStabilityParams & params) = 0;
  // virtual void update() = 0;

  /*! \brief Set the friction coefficient.
   */
  virtual void setFrictionCoefficient(const double & miu) = 0;

  /*! \brief Set the restitution coefficient.
   */
  virtual void setRestitutionCoefficient(const double & e) = 0;

  /*! \brief Set the name of the impact body / surface
   */
  // virtual void setImpactSurfaceName(const std::string & iBody) = 0;

  /*! \brief returns the scalar equivalent mass, i.e., n^T * EM * n.
   */
  const double & EMs() const
  {
    return ems_;
  }

  /*! \brief Returns the coefficient of stick.
   *
   * If coeStick <= coeFriction, the contact mode is: slip-stick.
   */
  double coeStick() const
  {
    return coeStick_;
  }

protected:
  std::shared_ptr<KinematicChain> robot_;
  std::shared_ptr<EndEffector> ee_;
  ImpactModelParams params_;
  PostImpactStates robotPostImpactStates_;
  PostImpactStates objectPostImpactStates_;

  Vector3d impactNormal_, tangentX_, tangentY_;

  double coeStick_ = 0.0; ///< The coefficient of stick.
  double calcCoeStick_(const Matrix3d & iim) const;

  double ems_ = 0.0;

  void resetNormals_()
  {
    tangentX_ = Eigen::Vector3d::UnitX();
    tangentY_ = Eigen::Vector3d::UnitY();

    impactNormal_ = Eigen::Vector3d::UnitZ();
  }

  void updateImpactNormal_()
  {
    if(params().bodyCoordinates)
    {
      // By default, we assume the z-axis as the impact normal
      resetNormals_();
    }
    else
    {
      updateSpatialImpactNormal_();
    }
  }
  void updateSpatialImpactNormal_()
  {
    tangentX_ = endEffector()->transform().rotation().matrix().col(0);
    tangentY_ = endEffector()->transform().rotation().matrix().col(1);

    // z-axis in the inertial frame
    impactNormal_ = endEffector()->transform().rotation().matrix().col(2);
  }
};

class TwoDimModelBridge : public ImpactDynamicsModel
/*! \brief Interface of the planar frictional-impact-dynamics model.
 */
{
public:
  TwoDimModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                    const std::shared_ptr<EndEffector> ee,
                    const ImpactModelParams & params,
                    bool useProjectedMass);

  virtual ~TwoDimModelBridge()
  {
    RoboticsUtils::quickInfo("Destructing TwoDimModelBridge.");
  }
  const PostImpactStates & objectPostImpactStates() override;

  // void update(const Eigen::Vector3d & impactNormal, const Eigen::Vector3d & impactLinearVel) override;
  // void update(const Eigen::Vector3d & impactNormal, const Eigen::Vector3d & impactLinearVel) override;

  void update(const Matrix6d & inertia, const Vector6d & v) override;

  const FIDynamics::PIParams & planarImpactParams()
  {
    return piParams_;
  }

  const Eigen::Vector3d & getAverageLinerVel()
  {
    return rAverageLinearVel_;
  }
  const Eigen::Vector3d & getAverageAngularVel()
  {
    return rAverageAngularVel_;
  }
  void printPIParams() const;
  void printResult() const;

  /*! \return the moment of inertia of the entire robot in its centroidal frame.
   */
  const Eigen::Matrix3d & getRobotCentroidalInertia()
  {
    return rCentroidalInertia_;
  }

  void setFrictionCoefficient(const double & miu) override
  {
    // twoDimModelPtr_->setFrictionCoefficient(miu);
    piParams_.miu = miu;
  }

  /*! \brief Set the energetic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    // twoDimModelPtr_->setEnergeticRestitutionCoefficient(e);
    piParams_.e = e;
  }

  /*
  void setImpactSurfaceName(const std::string & iSurface) override
  {
    params_.iSurfaceName = iSurface;
  }
  */

protected:
  /*! \brief update with the centroidal inertia
   */
  void updateWithCI_(const Matrix6d & centroidalInertia, const Vector6d & av);

  /*! \brief update with the projected inertia
   */
  void updateWithPI_(const Matrix6d & inertia, const Vector6d & v);

  // std::vector<std::string> logEntries_;

  // Compute the planar impact parameters using 3D data.
  /*!
   * \param in The impact normal
   * \param in The impact normal
   */
  void updatePiParams_(const Eigen::Vector3d & eeP, const Eigen::Matrix3d & eeR);
  void projectPiParams_(const Vector3d & robotVel, const Eigen::Vector3d & eeP, const int & idx);
  FIDynamics::PIParams piParams_;

  Eigen::Matrix3d rCentroidalInertia_; ///< Robot centroidal inertia.
  Eigen::Vector3d rAverageAngularVel_; ///< Robot average angular velocity
  Eigen::Vector3d rAverageLinearVel_; ///< Robot average angular velocity

  std::shared_ptr<FIDynamics::TwoDimModel> twoDimModelPtr_;

  // Eigen::Vector3d impactNormal_ = Eigen::Vector3d::Zero();

  // Convert the 2D solution to 3D
  void planarSolutionTo3D_();

  void planarSolutionTo3D_(PostImpactStates & input, const int & idx);

  /*! \brief indicates what assumptions are used.
   */
  inline const TwoDimModelCase & getCase_()
  {
    return case_;
  }

  TwoDimModelCase case_ = TwoDimModelCase::HRP4PushWall;

  inline const bool & projectedMass_()
  {
    return useProjectedMass_;
  }
  bool useProjectedMass_ = false;
  /*! \brief update the piParmas with the Push-Wall assumption: object mass and moment of inertia are infinite.
   */
  void paramUpdateHRP4PushWall_();

}; // end of the TwoDimModelBridge

} // namespace GeoRobotics
