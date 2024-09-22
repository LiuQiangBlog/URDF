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
#include "GeometricRobotics/Prediction/PostImpactPredictor.h"
#include "ImpactDynamicsModel.h"
#include <RoboticsUtils/utils.h>
#include <ThreeDimModel/FLModel.h>

namespace GeoRobotics
{

class FLModelBridge : public ImpactDynamicsModel
/*! \brief Interface of the frictionaless-impact-dynamics model.
 */
{
public:
  FLModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                const std::shared_ptr<EndEffector> ee,
                const ImpactModelParams & params);

  virtual ~FLModelBridge()
  {
    RoboticsUtils::quickInfo("Destructing FLModelBridge.");
  }

  const PostImpactStates & objectPostImpactStates() override
  {
    return objectPostImpactStates_;
  }

  void update(const Matrix6d & inertia, const Vector6d & v) override;

  const FIDynamics::FLParams & FrictionlessImpactParams()
  {
    return flParams_;
  }
  void setFrictionCoefficient(const double & miu) override
  {
    RoboticsUtils::throw_runtime_error("The frictionless model can not set friction coefficient");
  }
  /*! \brief Set the energetic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    // twoDimModelPtr_->setEnergeticRestitutionCoefficient(e);
    flParams_.e = e;
  }

protected:
  std::shared_ptr<FIDynamics::FLModel> FLModelPtr_;

  FIDynamics::FLParams flParams_;

}; // end of the FLModelBridge

class FLCRBModelBridge : public ImpactDynamicsModel
/*! \brief Interface of the frictionaless-impact-dynamics model.
 */
{
public:
  FLCRBModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                   const std::shared_ptr<EndEffector> ee,
                   const ImpactModelParams & params);

  virtual ~FLCRBModelBridge()
  {
    RoboticsUtils::quickInfo("Destructing FLCRBModelBridge.");
  }

  const PostImpactStates & objectPostImpactStates() override
  {
    return objectPostImpactStates_;
  }

  void update(const Matrix6d & inertia, const Vector6d & v) override;

  const FIDynamics::FLParams & FrictionlessImpactParams()
  {
    return flParams_;
  }
  void setFrictionCoefficient(const double & miu) override
  {
    RoboticsUtils::throw_runtime_error("The frictionless model can not set friction coefficient");
  }
  /*! \brief Set the energetic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    // twoDimModelPtr_->setEnergeticRestitutionCoefficient(e);
    flParams_.e = e;
  }

protected:
  std::shared_ptr<FIDynamics::FLModel> FLModelPtr_;

  FIDynamics::FLParams flParams_;

}; // end of the FLModelBridge

class FLAssemblyModelBridge : public ImpactDynamicsModel
/*! \brief Interface of the frictionaless-impact-dynamics model.
 */
{
public:
  FLAssemblyModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                        const std::shared_ptr<EndEffector> ee,
                        const ImpactModelParams & params);

  virtual ~FLAssemblyModelBridge()
  {
    RoboticsUtils::quickInfo("Destructing FLAssemblyModelBridge.");
  }

  const PostImpactStates & objectPostImpactStates() override
  {
    return objectPostImpactStates_;
  }

  void update(const Matrix6d & inertia, const Vector6d & v) override;

  const FIDynamics::FLParams & FrictionlessImpactParams()
  {
    return flParams_;
  }
  void setFrictionCoefficient(const double & miu) override
  {
    RoboticsUtils::throw_runtime_error("The frictionless model can not set friction coefficient");
  }
  /*! \brief Set the energetic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    // twoDimModelPtr_->setEnergeticRestitutionCoefficient(e);
    flParams_.e = e;
  }

protected:
  std::shared_ptr<FIDynamics::FLModel> FLModelPtr_;

  FIDynamics::FLParams flParams_;

  std::shared_ptr<PostImpactPredictor> bPip_;

}; // end of the FLModelBridge

class FL_CRB_FB_ModelBridge : public ImpactDynamicsModel
/*! \brief Frictionaless-impact-dynamics model using the CRB inertia with joint flexibility.
 */
{
public:
  FL_CRB_FB_ModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                        const std::shared_ptr<EndEffector> ee,
                        const ImpactModelParams & params);

  virtual ~FL_CRB_FB_ModelBridge()
  {
    RoboticsUtils::quickInfo("Destructing the FL_CRB_FB_ModelBridge.");
  }

  const PostImpactStates & objectPostImpactStates() override
  {
    return objectPostImpactStates_;
  }

  void update(const Matrix6d & inertia, const Vector6d & v) override;

  const FIDynamics::FLParams & FrictionlessImpactParams()
  {
    return flParams_;
  }
  void setFrictionCoefficient(const double & miu) override
  {
    RoboticsUtils::throw_runtime_error("The frictionless model can not set friction coefficient");
  }
  /*! \brief Set the energetic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    // twoDimModelPtr_->setEnergeticRestitutionCoefficient(e);
    flParams_.e = e;
  }

protected:
  std::shared_ptr<FIDynamics::FLModel> FLModelPtr_;

  FIDynamics::FLParams flParams_;

}; // end of the FLModelBridge

class FL_CRB_RV_ModelBridge : public ImpactDynamicsModel
/*! \brief Frictionaless-impact-dynamics model using the CRB inertia and relative velocity
 */
{
public:
  FL_CRB_RV_ModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                        const std::shared_ptr<EndEffector> ee,
                        const ImpactModelParams & params);

  virtual ~FL_CRB_RV_ModelBridge()
  {
    RoboticsUtils::quickInfo("Destructing the FL_CRB_RV_ModelBridge.");
  }

  const PostImpactStates & objectPostImpactStates() override
  {
    return objectPostImpactStates_;
  }

  void update(const Matrix6d & inertia, const Vector6d & v) override;

  const FIDynamics::FLParams & FrictionlessImpactParams()
  {
    return flParams_;
  }
  void setFrictionCoefficient(const double & miu) override
  {
    RoboticsUtils::throw_runtime_error("The frictionless model can not set friction coefficient");
  }
  /*! \brief Set the energetic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    // twoDimModelPtr_->setEnergeticRestitutionCoefficient(e);
    flParams_.e = e;
  }

protected:
  std::shared_ptr<FIDynamics::FLModel> FLModelPtr_;

  FIDynamics::FLParams flParams_;

}; // end of the FLModelBridge

class FL_GM_ModelBridge : public ImpactDynamicsModel
/*! \brief Frictionaless-impact-dynamics model using the generalized momentum, i.e., M*dq
 */
{
public:
  FL_GM_ModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                    const std::shared_ptr<EndEffector> ee,
                    const ImpactModelParams & params);

  virtual ~FL_GM_ModelBridge()
  {
    RoboticsUtils::quickInfo("Destructing the FL_GM_ModelBridge.");
  }

  const PostImpactStates & objectPostImpactStates() override
  {
    return objectPostImpactStates_;
  }

  void update(const Matrix6d & inertia, const Vector6d & v) override;

  const FIDynamics::FLParams & FrictionlessImpactParams()
  {
    return flParams_;
  }
  void setFrictionCoefficient(const double & miu) override
  {
    RoboticsUtils::throw_runtime_error("The frictionless model can not set friction coefficient");
  }
  /*! \brief Set the energetic restitution coefficient.
   */
  void setRestitutionCoefficient(const double & e) override
  {
    // twoDimModelPtr_->setEnergeticRestitutionCoefficient(e);
    flParams_.e = e;
  }

protected:
  std::shared_ptr<FIDynamics::FLModel> FLModelPtr_;

  FIDynamics::FLParams flParams_;

}; // end of the FLModelBridge

} // namespace GeoRobotics
