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

#include "GeometricRobotics/ImpactDynamics/FrictionalKinematicModel.h"
#include "GeometricRobotics/ImpactDynamics/FrictionlessModel.h"
#include "GeometricRobotics/ImpactDynamics/ImpactDynamicsModel.h"
#include "GeometricRobotics/ImpactDynamics/OneDimKinematicModel.h"
#include "GeometricRobotics/Kinematics/KinematicChain.h"
#include "GeometricRobotics/Prediction/PostImpactPredictor.h"
#include "GeometricRobotics/Prediction/ProjectionPredictor.h"
#include <fstream>
#include <iostream>
#include <time.h> /* time_t, struct tm, time, localtime, strftime */

namespace GeoRobotics
{

enum struct IDModel
/*! \brief  Choice of Impact Dynamics model
 */
{
  Kinematic, ///< Applies the kinematic restitution.
  Frictionless3D, ///< Frictionless 3D model.
  Frictionless3DCRB, ///< Frictionless 3D model using the Composite-rigid body assumption.
  Frictionless3DGM, ///< Frictionless 3D model using the generalized momentum.
  Frictionless3DCRB_FB, ///< Frictionless 3D model using the Composite-rigid body assumption and joint flexibility.
  Frictionless3DCRB_RV, ///< Frictionless 3D model using the Composite-rigid body assumption and relative velocity.
  Frictionless3DAssembly, ///< Frictionless 3D model using the assembly approach.
  FrictionalPlanar, ///< Applies Yanbin-2019.
  FrictionalPlanarEM, ///< Applies Yanbin-2019 and the projected mass.
  FrictionalKinematic, ///< Applies the model by Roy Featherstone.
  FrictionConeKinematic ///< Applies the model by Roy Featherstone with friction cone.
};

inline const std::string ToString(IDModel m)
{
  switch(m)
  {
    case IDModel::Kinematic:
      return "OneDimKinematicModel";
    case IDModel::Frictionless3D:
      return "Frictionless3D";
    case IDModel::Frictionless3DCRB:
      return "Frictionless3DCRB";
    case IDModel::Frictionless3DCRB_FB:
      return "Frictionless3DCRB_FB";
    case IDModel::Frictionless3DCRB_RV:
      return "Frictionless3DCRB_RV";
    case IDModel::Frictionless3DAssembly:
      return "Frictionless3DAssembly";
    case IDModel::FrictionalPlanar:
      return "FrictionalPlanarModel";
    case IDModel::FrictionalPlanarEM:
      return "FrictionalPlanarModel";
    case IDModel::FrictionalKinematic:
      return "FrictionalKinematicModel";
    default:
      return "[Unknown Impact-Dynamics-Model]";
  }
}

enum struct EMChoice
/*! \brief  Choice of equivalent mass.
 */
{
  Projection, ///<  (J M ^-1 J^T)^-1
  CRB ///< The composite-rigid-body inertia.
};

inline const std::string ToString(EMChoice e)
{
  switch(e)
  {
    case EMChoice::Projection:
      return "Projection";
    case EMChoice::CRB:
      return "CRB";
    default:
      return "[Unknown Equivalent-Mass-Choice]";
  }
}

struct IDMInterfaceParams
{
  ///< Choice of apparent mass.
  EMChoice emChoice = EMChoice::Projection;

  ///< Choice of method.
  IDModel idModel = IDModel::Kinematic;

  ///< If the parent joint constraint of the root link applies
  bool pjConstraint = false;

  ///< Spatial coordinates representation
  // bool spatial = false;

  ImpactModelParams idParams;
};

class IDMInterface
/*! \brief It provides API to (frictional) impact dynamics model.
 * 1. Frictionless case:
 *    1.1 Zheng's model.
 * 2. Frictional case:
 *    2.1. It iterfaces the frictional-impact-dynamics model by Yanbin.
 *    2.2. It implements the model covered by R.Featherstone -- Not implemented.
 *
 * 3. Equivalent (apparent ) mass (inertia tensor):
 *    3.1 Projection approach.
 *    3.2 Composite-rigid-body (Centroidal) inertia.
 *    3.3 Constrained dynamics -- Not implemented.
 */
{
public:
  IDMInterface(const std::shared_ptr<KinematicChain> robot,
               const std::shared_ptr<EndEffector> ee,
               const IDMInterfaceParams & params);
  ~IDMInterface() {}

  const IDMInterfaceParams & params() const
  {
    return params_;
  }

  /*! \brief The equivalent mass (inertia tensor)
   */
  const Matrix6d & eMass() const
  {
    return eM_;
  }

  const std::shared_ptr<KinematicChain> & robot() const
  {
    return robot_;
  }

  void update();

  const PostImpactStates & robotPostImpactStates() const
  {
    return idModel_->robotPostImpactStates();
  }

  const std::shared_ptr<ImpactDynamicsModel> & impactDynamicsModel() const
  {
    return idModel_;
  }

  const std::shared_ptr<ProjectionPredictor> & projectionPredictor() const
  {
    return pp_;
  }

  const std::shared_ptr<PostImpactPredictor> & postImpactPredictor() const
  {
    return p_;
  }

  // std::string saveData();

  const std::shared_ptr<EndEffector> & getEndEffector() const
  {
    return ee_;
  }

protected:
  std::shared_ptr<KinematicChain> robot_;
  std::shared_ptr<EndEffector> ee_;
  IDMInterfaceParams params_;

  void updateID_();
  std::shared_ptr<ImpactDynamicsModel> idModel_;

  Matrix6d eM_ = Matrix6d::Zero();
  void updateEM_();
  void computeEMbyProjection(Matrix6d & em);
  void computeEMbyCRB(Matrix6d & em);
  void metaCalcEMProjection_(const MatrixXd & J, const MatrixXd & M, Matrix6d & em);

  void initializeStateJumpPredictors_(const std::shared_ptr<EndEffector> ee);
  ProjectionPredictorParams ppParams_;
  std::shared_ptr<ProjectionPredictor> pp_;

  PredictorParams pParams_;
  std::shared_ptr<PostImpactPredictor> p_;
};

} // namespace GeoRobotics
