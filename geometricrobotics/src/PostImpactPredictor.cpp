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
#include "GeometricRobotics/Prediction/PostImpactPredictor.h"

namespace GeoRobotics
{

void LinkVariables::reset(const int & dof, const int & sd)
{
  vel.resize(sd);
  w.resize(sd);
}

void JointVariables::reset(const int & dof, const int & fd, const int & sd)
{
  vel.resize(sd);
  dq.resize(dof);
  tau.resize(dof);
  lambda.resize(fd);
  jaw.resize(sd);
  jcw.resize(sd);
}

void PostImpactPredictor::initializeVars_()
{
  dof_ = robot_->dof();

  sd_ = 6 * dof_;
  fd_ = 5 * dof_;

  jv_.reset(dof_, fd_, sd_);
  lv_.reset(dof_, sd_);

  // V_J = P^T * V_L, P is fixed for the spatial coordinates representation.
  P_.resize(sd_, sd_);
  P_.setIdentity();

  M_.resize(sd_, sd_);
  M_.setZero();

  Ta_.resize(sd_, dof_);
  Ta_.setZero();

  T_.resize(sd_, fd_);
  T_.setZero();

  // The block size
  int bs = 6;

  if(params().spatialFrame)
  {
    // If the spatial coordinates, P matrix is set once for all
    // Fill the negative components
    for(int ii = 0; ii < dof_ - 1; ii++)
    {
      int row = ii * bs;
      int column = (ii + 1) * bs;
      P_.block<6, 6>(row, column) = -Matrix6d::Identity();
    }
  }
  else
  {
    // If applies the body frame computation, M, T and Ta are fixed w.r.t. the body frame!
    constructProblemB_(M_, Ta_, T_);
  }

  // printMatrix("P matrix", P());
  // RoboticsUtils::quickInfo("PostImpactPredictor variables initialized.");
}

PostImpactPredictor::PostImpactPredictor(const std::shared_ptr<KinematicChain> robot, const PredictorParams & params)
: robot_(robot), params_(params)
{

  initializeVars_();
  RoboticsUtils::quickInfo("PostImpactPredictor is created.");
}

void PostImpactPredictor::compute(const Vector6d & eWrenchJump, const bool & sensorBodyFrame)
{
  // iwJump_ is supposed to be given in the end-effector link frame.
  if(sensorBodyFrame)
  {
    // Computes the end-effector frame wrench
    iwJump_ = params().toLinkFrame * eWrenchJump;
  }
  else
  {
    iwJump_ = eWrenchJump;
  }

  if(params().spatialFrame)
  {
    // 1. Spatial coordinates case:
    // Evaluating (1) inertia tensors: M, (2) Actuation wrench basis: Ta, (3) constraint wrench basis: T.
    constructProblem_(M_, Ta_, T_);

    // Transform the external wrench to the inertial frame
    iwJump_ = metaAdgTMatrix(robot()->transform().inverse()) * iwJump_;
  }
  else
  {
    // 2. Body coordinates case: Update the P matrix
    updatePB_(P_);
  }

  // printMatrix("M", M());
  // printMatrix("Ta", Ta());
  // printMatrix("T", T());
  // RoboticsUtils::quickInfo("Problem constructed.");
  // RoboticsUtils::quickInfo("Problem constructed.");
  if(params().spatialFrame)
  {
    solveProblem_(M(), Ta(), T(), robot()->spatialJacobian(), inputWrenchJump());
    solveDq_(jv().vel, robot()->spatialJacobian());
  }
  else
  {
    solveProblem_(M(), Ta(), T(), robot()->bodyJacobian(), inputWrenchJump());
    solveDqB_(jv().vel);
  }
}

void PostImpactPredictor::updatePB_(MatrixXd & P)
{
  int bs = 6;

  for(int ii = 0; ii < dof_ - 1; ii++)
  {
    int row = ii * bs;
    int column = (ii + 1) * bs;
    // Assumes a sequential order
    // Iso3d const & tf = robot()->joint(ii + 1)->relativeTransform(); // This is dirty and wrong
    Iso3d const & tf = Iso3d::Identity();
    P.block<6, 6>(row, column) = -metaAdgTMatrix(tf.inverse());
  }
}

void PostImpactPredictor::constructProblemB_(MatrixXd & M, MatrixXd & Ta, MatrixXd & T)
{
  M.setZero();
  Ta.setZero();
  T.setZero();

  int ii = 0;
  // The P matrix assumes a sequential order of the links!
  // Hence, we need to loop by the pre-defined order.
  for(auto & link : robot()->links())
  {
    auto pj = robot()->joint(link->parentJointName());
    // (1) Each diagonal block of M corresponds to the spatial inertia tensor of a link
    // (2) The links are taken sequentially from a chain. The order matters, otherwise the P matrix can not compute the
    // relative velocities.

    // 1. Inertia tensors
    M.block<6, 6>(ii * 6, ii * 6).noalias() = link->localInertiaTensor();

    // 2. Joint actuation wrench basis: Ta in (6dof x dof);
    Ta.block<6, 1>(ii * 6, ii).noalias() = pj->awBasis();

    // 3. Joint constraint wrench basis: T in (6dof x 5dof);
    T.block<6, 5>(ii * 6, ii * 5).noalias() = pj->cwBasis();

    // ----
    ii++;
  }
}

void PostImpactPredictor::constructProblem_(MatrixXd & M, MatrixXd & Ta, MatrixXd & T)
{
  M.setZero();
  Ta.setZero();
  T.setZero();

  int ii = 0;
  // The P matrix assumes a sequential order of the links!
  // Hence, we need to loop by the pre-defined order.
  for(auto & link : robot()->links())
  {
    auto pj = robot()->joint(link->parentJointName());
    // (1) Each diagonal block of M corresponds to the spatial inertia tensor of a link
    // (2) The links are taken sequentially from a chain. The order matters, otherwise the P matrix can not compute the
    // relative velocities.

    // 1. Inertia tensors
    M.block<6, 6>(ii * 6, ii * 6).noalias() = link->spatialInertia(robot()->transform(pj->name()));

    // 2. Joint actuation wrench basis: Ta in (6dof x dof);
    const Matrix6d & adgM = metaAdgTMatrix(robot()->transform(pj->name()).inverse());

    Ta.block<6, 1>(ii * 6, ii).noalias() = adgM * pj->awBasis();

    // 3. Joint constraint wrench basis: T in (6dof x 5dof);
    T.block<6, 5>(ii * 6, ii * 5).noalias() = adgM * pj->cwBasis();

    // ----
    ii++;
  }
}

void PostImpactPredictor::solveProblem_(const MatrixXd & M,
                                        const MatrixXd & Ta,
                                        const MatrixXd & T,
                                        const MatrixXd & jac,
                                        const Vector6d & iwJump)
{
  // Check the Eigen reference:
  // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
  // To solve Ax = b, i.e., A^-1 * b, we can perform: x = A.colPivHouseholderQr().solve(b)

  MatrixXd A;
  VectorXd b;

  // A.resize(sd_ + sd_ + 6, sd_ + sd_);
  A.resize(sd_ + sd_, sd_ + sd_);
  A.setZero();
  // b.resize(sd_ + sd_ + 6);
  b.resize(sd_ + sd_);
  b.setZero();

  //--- constract A

  const MatrixXd & PT = P() * T;
  const MatrixXd & PTa = P() * Ta;

  assert(PT.rows() == sd_);
  assert(PT.cols() == fd_);

  assert(PTa.rows() == sd_);
  assert(PTa.cols() == dof_);

  // 1. Newton-Euler
  A.block(0, 0, sd_, sd_).noalias() = M;
  A.block(0, sd_, sd_, dof_).noalias() = PTa;
  A.block(0, sd_ + dof_, sd_, fd_).noalias() = PT;
  // 2. External Wrench, by default, we apply the Jacobian of the last joint.
  A.block(sd_, sd_, dof_, dof_).setIdentity();
  // 3. Joint motion constraint

  A.block(sd_ + dof_, 0, fd_, sd_).noalias() = PT.transpose();
  //--- construct b ...

  // 1. Newton-Euler, only applies to the last joint/link
  b.segment<6>(6 * (dof_ - 1)).noalias() = iwJump;
  // 2. External wrench
  b.segment(sd_, dof_).noalias() = jac.transpose() * iwJump;

  // 3. end-effector Velocity jump
  // A.block(2*sd_, 6*(dof_ - 1), 7, 6).noalias() = MatrixXd::Identity(6,6);
  // b.segment(2*sd_, 6) = - robot_->bodyJacobian() * robot_->jointVelocities();

  // const VectorXd & x = ( A.transpose()*A).colPivHouseholderQr().inverse() * A.transpose() * b;

  const VectorXd & x = A.colPivHouseholderQr().solve(b);

  // RoboticsUtils::quickInfo("Problem solved.");
  // Evaluates: vel_, tau_, lambda_, jaw_, and jcw_.
  evaluateSolution_(Ta, T, x);

  // RoboticsUtils::quickInfo("Solution evaluated.");
  // Evaluates: dq_

  /*
  VectorXd eW;
  eW.resize(sd_);
  eW.setZero();
  eW.tail(6) = iwJump;

  const VectorXd & result = M * lv().vel + PTa * jv().tau + PT * jv().lambda - eW;

  RoboticsUtils::quickInfo("Result is: " + std::to_string(result.norm()) );


  const VectorXd & result_2 = A*x - b;
  RoboticsUtils::quickInfo("Result 2 is: " + std::to_string(result_2.norm()) );

  const VectorXd & result_3 = A.block(0, 0, sd_, sd_ + sd_)*x - b.head(sd_);
  RoboticsUtils::quickInfo("Result 3 is: " + std::to_string(result_3.norm()) );

  const VectorXd & result_4 = A.block(sd_, 0, sd_, sd_ + sd_)*x - b.tail(sd_);
  RoboticsUtils::quickInfo("Result 4 is: " + std::to_string(result_4.norm()) );
  */

  // RoboticsUtils::quickInfo("dq evaluated.");
}

void PostImpactPredictor::evaluateSolution_(const MatrixXd & Ta, const MatrixXd T, const VectorXd & x)
{
  // link velocity jumps
  lv_.vel.noalias() = x.head(sd_);

  // Joint torque jumps
  // jv_.tau.noalias() = x.segment(sd_, dof_);
  jv_.tau.noalias() = x.segment(sd_, dof_);

  // Lagrange multipliers
  jv_.lambda.noalias() = x.tail(fd_);

  // The joint actuation wrench:
  jv_.jaw.noalias() = Ta * jv().tau;
  jv_.jcw.noalias() = T * jv().lambda;

  // Solve link wrench jumps from the joint wrench jumps
  lv_.w.noalias() = -P() * (jv().jaw + jv().jcw);
  // Add the external wrench to the last link:
  lv_.w.tail(6) += inputWrenchJump();

  // Solve joint velocity jumps from link velocity jumps
  jv_.vel.noalias() = P().transpose() * lv().vel;
}

void PostImpactPredictor::solveDqB_(const VectorXd & vel)
{
  for(auto & j : robot()->joints())
  {

    auto jName = j->name();
    int idx = robot()->jointNameToIndex(jName);

    // Take the corresponding joint velocity:
    const Vector6d & jv = vel.segment<6>(idx * 6);

    // Take xi_prime from the Jacobian:
    // const Vector6d & jj = robot()->bodyJacobian(jName).rightCols(1);
    const Vector6d & jj = metaAdgMatrix(robot()->transform(jName).inverse()) * robot()->spatialJacobian().col(idx);

    // jj * dq_i = jv
    // A^+ = (A^T * A)^−1 *  A^T
    jv_.dq(idx) = (1.0 / (jj.transpose() * jj)) * jj.transpose() * jv;
  }
}

void PostImpactPredictor::solveDq_(const VectorXd & vel, const MatrixXd & J)
{

  for(auto & j : robot()->joints())
  {

    auto jName = j->name();
    int idx = robot()->jointNameToIndex(jName);

    // Take the corresponding joint velocity:
    const Vector6d & jv = vel.segment<6>(idx * 6);

    // Take xi_prime from the Jacobian:
    const Vector6d & jj = J.col(idx);

    // jj * dq_i = jv
    // A^+ = (A^T * A)^−1 *  A^T
    jv_.dq(idx) = (1.0 / (jj.transpose() * jj)) * jj.transpose() * jv;
  }

  /*
  MatrixXd psJac;
  psJac.resize(J.cols(), J.rows());

  // Evaluate the coloumn-wise pseudo inverse.
  for(int i = 0; i < J.cols(); i++)
  {
    double c = 1.0 / (J.col(i).transpose() * J.col(i));
    psJac.row(i) = c * J.col(i).transpose();
  }

  jv_.dq.setZero();

  for(int ii = 0; ii < dof_; ii++)
  {
    int idx = ii * 6;
    jv_.dq(ii) = psJac.row(ii) * vel.segment<6>(idx);
  }
  */
}

} // namespace GeoRobotics
