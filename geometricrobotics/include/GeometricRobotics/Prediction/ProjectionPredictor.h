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
namespace GeoRobotics
{

struct ProjectionPredictorParams
{
  bool spatialFrame = true; ///< Following the spatial or body frame representations.
  Iso3d sensorTF = Iso3d::Identity(); ///< In the n-th (end-effector) link frame , the transform to the external wrench
                                      ///< (Force-torque sensor), i.e., g_{ne}.

  ///< Tranform Wrench from ee to link frame
  Matrix6d FTtoLinkFrame = Matrix6d::Identity();

  ///< Tranform Vel from ee to link frame
  Matrix6d VeltoLinkFrame = Matrix6d::Identity();

  void reset()
  {
    FTtoLinkFrame = metaAdgTMatrix(sensorTF.inverse());
    VeltoLinkFrame = metaAdgMatrix(sensorTF);
  }
  ProjectionPredictorParams()
  {
    reset();
  }
  ~ProjectionPredictorParams() {}
};

class ProjectionPredictor
/*! Empolying the Jacobian projection approach, computes the joint torque, velocity jumps given the impulse.
 */
{
public:
  ProjectionPredictor(const std::shared_ptr<KinematicChain> robot, const ProjectionPredictorParams & params);
  ~ProjectionPredictor() {}

  // clang-format off
  /*! \brief Assuming the joints are locked, we use the IIM derived from the centroidal dynamics. By default, we assume the wrench jump is given in the end-effector link frame, whose origin coinsides the nth joint axis. 
   * \param eWrenchJump 
   * \param sensorBodyFrame If the wrench is given in the sensor frame, we need to convert it.
   */
  void compute(const std::shared_ptr<EndEffector> & impactSEE, const Vector6d & eWrenchJump, const bool & sensorBodyFrame = false);
  // clang-format on 
  //
  // clang-format off
  /*! \brief Assuming the joints are flexible, we use the IIM derived from the joint-space inertia tensor. By default, we assume the wrench jump is given in the end-effector link frame, whose origin coinsides the nth joint axis. 
   * \param eWrenchJump 
   * \param sensorBodyFrame If the wrench is given in the sensor frame, we need to convert it.
   */
  void computeGM(const std::shared_ptr<EndEffector> & impactSEE, const Vector6d & eWrenchJump, const bool & sensorBodyFrame = false);
  // clang-format on 


  // clang-format off
  /*! \brief By default, we assume the wrench jump is given in the last linkframe, whose origin coinsides the nth joint axis. 
   * \param linear velocity jump 
   * \param endEffectorFrame If it is the endEffectorVel or the endLinkVel 
   */
  void computeVel(const std::shared_ptr<EndEffector> & impactSEE, const Vector3d & velJump, const bool & endEffectorFrame = false, const bool minimumNorm = false);
  // clang-format on 
  




  // clang-format off
  /*! \brief Drop the last joint, compute qd jump by the joint 0-5; 
   * \param linear velocity jump 
   * \param endEffectorFrame If it is the endEffectorVel or the endLinkVel 
   */
  void nonRedundantComputeVel(const Vector3d & velJump, const bool & endEffectorFrame = false);
  // clang-format on 
  

  // clang-format off
  /*! \brief By default, we assume the wrench jump is given in the end-effector link frame, whose origin coinsides the nth joint axis. 
   * \param eWrenchJump given in the sensorBodyFrame 
   * \param bodyVelJump the velocity jump in the body frame.
   */
  //void computeBoth(const Vector3d & eWrenchJump, const Vector3d & bodyVelJump);
  // clang-format on 

  void computeImpulseAndVel(const Vector3d & eWrenchJump, const Vector3d & bodyVelJump);

  void computeNullspace(const Vector3d & eWrenchJump, const Vector3d & bodyVelJump);

  void computeWeightedCMMAndVel(const Vector3d & eWrenchJump, const Vector3d & bodyVelJump);

  const ProjectionPredictorParams & params() const
  {
    return params_;
  }
   const std::shared_ptr<KinematicChain> & robot() const
  {
    return robot_;
  }
  const Vector6d & inputVelJump() const
  {
    return ivJump_;
  }
  const Vector6d & inputWrenchJump() const
  {
    return iwJump_;
  }

  /*! \brief The equivalent mass (inertia tensor)
   */ 
  const Matrix6d & eMass() const
  {
    return eM_; 
  }

  /*! \brief The End-effector velocity jump
   *  In the frame specified by the params()
   */ 
  const Vector6d & eeV() const
  {
    return eeV_; 
  }

  const VectorXd & dq() const
  {
    return dq_; 
  }

  const VectorXd & tau() const
  {
    return tau_; 
  }

  /*! \brief 
   *  \return the inverse-inertia-matrix 
   */ 
  const Matrix3d & iim() const
  {
    return iim_; 
  }
  void print() const;
protected:
  std::shared_ptr<KinematicChain> robot_;

  void initializeVars_();

  ProjectionPredictorParams params_;

  ///< The input wrench jump
  Vector6d iwJump_ = Vector6d::Zero();

  ///< The input velociy jump
  Vector6d ivJump_ = Vector6d::Zero();

  void solveTwoTasks_(VectorXd & output, const MatrixXd & J1, const VectorXd & b1, const MatrixXd & J2, const VectorXd & b2 );

  void solveTwoWeightedTasks_(VectorXd & output, const MatrixXd & J1, const VectorXd & b1, const double & w1, const MatrixXd & J2, const VectorXd & b2 );

  void computeFT_(const std::shared_ptr<EndEffector> & impactSEE, const bool & spatialFrame);
  void computeFTGM_(const std::shared_ptr<EndEffector> & impactSEE, const bool & spatialFrame);
  void computeVel_(const std::shared_ptr<EndEffector> & impactSEE, const bool & spatialFrame, const bool robustMethod = true);
  ///< The equivalent mass (inertia tensor)
  Matrix6d eM_ = Matrix6d::Zero();
  Matrix3d iim_ = Matrix3d::Zero();
  Vector6d eeV_ = Vector6d::Zero();


  VectorXd tau_;
  VectorXd dq_;

}; // End of Predictor
} // namespace GeoRobotics
