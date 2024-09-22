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

struct PredictorParams
{
  bool spatialFrame = true; ///< Following the spatial or body frame representations.
  Iso3d sensorTF = Iso3d::Identity(); ///< In the n-th (end-effector) link frame , the transform to the external wrench
                                      ///< (Force-torque sensor), i.e., g_{ne}.

  Matrix6d toLinkFrame = Matrix6d::Identity();

  void reset()
  {
    toLinkFrame = metaAdgTMatrix(sensorTF.inverse());
  }
  PredictorParams()
  {
    reset();
  }
  ~PredictorParams() {}
};

struct LinkVariables
{
  ///< The link velocities w.r.t. inertial frame: 6dof x 1
  VectorXd vel;

  ///< The link wrenches: 6dof x 1
  VectorXd w;

  void reset(const int & dof, const int & sd);
};

struct JointVariables
{
  ///< The relative velocities between the parent/child links: 6dof x 1
  // We can speicify either in the spatial frame or in the body frame
  VectorXd vel;

  ///< The joint velocities: dof x 1
  VectorXd dq;

  ///< The joint torques: dof x 1
  VectorXd tau;

  ///< The Lagrange multipliers to compute jcw: 5dof x 1
  VectorXd lambda;

  ///< The joint actuation wrenches: 6dof x 1
  VectorXd jaw;

  ///< The joint constraint wrenches: 6dof x 1
  VectorXd jcw;

  void reset(const int & dof, const int & fd, const int & sd);
};

class PostImpactPredictor
/*! Given the impulse, computes the robot state jumps empolying the spatial frame representation.
 */
{
public:
  PostImpactPredictor(const std::shared_ptr<KinematicChain> robot, const PredictorParams & params);
  ~PostImpactPredictor() {}

  // clang-format off
  /*! \brief By default, we assume the wrench jump is given in the end-effector link frame, whose origin coinsides the nth joint axis. 
   * \param eWrenchJump 
   * \param sensorBodyFrame If the wrench is given in the sensor frame, we need to convert it.
   */
  void compute(const Vector6d & eWrenchJump, const bool & sensorBodyFrame = false);
  // clang-format on 
  
  const JointVariables & jv() const
  {
    return jv_; 
  }

  const LinkVariables & lv() const
  {
    return lv_; 
  }

  /*! \brief returns the Lagrange multipliers that computes the constraint wrench for each joint or link.
   */
  //const VectorXd & lambda() const
  //{
  //  return lambda_;
  //}

  /*! \brief returns the joint velocity jumps
   */
  //const VectorXd & dq() const
  //{
  //  return dq_;
  //}

  /*! \brief returns the velocity jumps of the origin of the joint frames
   * returns a vector of 6 * dof
   */
  //const VectorXd & jVel() const
  //{
  //  return jv_.vel_;
  //}
  /*
  const VectorXd & jfVel(const std::string & jName) const
  {
    int idx = robot()->jointNameToIndex(jName);
    return vel_.segment<6>(idx * 6);
  }
  */

  /*! \brief returns the joint torque jumps
   */
  //const VectorXd & tau() const
  //{
  //  return jv_.tau;
  //}

  const std::shared_ptr<KinematicChain> & robot() const
  {
    return robot_;
  }

  const PredictorParams & params() const
  {
    return params_;
  }

  const Vector6d & inputWrenchJump() const
  {
    return iwJump_;
  }
  const MatrixXd & P() const
  {
    return P_;
  }
  const MatrixXd & M() const
  {
    return M_;
  }
  const MatrixXd & Ta() const
  {
    return Ta_;
  }
  const MatrixXd & T() const
  {
    return T_;
  }
protected:
  std::shared_ptr<KinematicChain> robot_;

  LinkVariables lv_;
  JointVariables jv_;

  void initializeVars_();

  int dof_ = 0; ///< dof
  int sd_ = 0; ///< 6dof
  int fd_ = 0; ///< 5dof

  /////< The link velocity jump
  //VectorXd lVel_;

  /////< The link velocity jump
  //VectorXd jVel_;
  //VectorXd dq_;
  //VectorXd tau_;
  //// The Lagrange
  //VectorXd lambda_;
  /////< Joint actuation wrenches
  //VectorXd jaw_;
  /////< Joint constraint wrenches
  //VectorXd jcw_;

  ///< The input wrench jump
  Vector6d iwJump_ = Vector6d::Zero();

  PredictorParams params_;

  MatrixXd P_, M_, Ta_, T_;

  /*!  \brief construct the building blocks
   */
  void constructProblem_(MatrixXd & M, MatrixXd & Ta, MatrixXd & T);

  /*!  \brief construct the building blocks in body coordinates
   *   This should be called only once for each robot, as M, Ta, T are set once for all!
   */
  void constructProblemB_(MatrixXd & M, MatrixXd & Ta, MatrixXd & T);

  /*! \brief udpate the P matrix for each iteration when the body coordinates is applied.
   */
  void updatePB_(MatrixXd & P);

  /*!  \brief construct the building blocks
   */
  void solveProblem_(const MatrixXd & M, const MatrixXd & Ta, const MatrixXd & T,  const MatrixXd & jac, const Vector6d & iwJump);

  /*! \brief Given Ta, T, and x, it evaluates: vel_, tau_, lambda_, jaw_, and jcw_.
   */
  void evaluateSolution_(const MatrixXd & Ta, const MatrixXd T, const VectorXd & x);

  /*! \brief Solve: dq_ from J * dq = vel
   */
  void solveDq_(const VectorXd & vel, const MatrixXd & J);

  /*! \brief In body coordinates, solve: dq_ from J * dq = vel
   * 
   */
  void solveDqB_(const VectorXd & vel);

};

} // namespace GeoRobotics
