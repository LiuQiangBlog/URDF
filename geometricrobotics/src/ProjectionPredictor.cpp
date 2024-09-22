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
#include "GeometricRobotics/Prediction/ProjectionPredictor.h"

namespace GeoRobotics
{
ProjectionPredictor::ProjectionPredictor(const std::shared_ptr<KinematicChain> robot,
                                         const ProjectionPredictorParams & params)
: robot_(robot), params_(params)
{

  initializeVars_();
  RoboticsUtils::quickInfo("ProjectionPredictor is created.");
}

void ProjectionPredictor::initializeVars_()
{
  tau_.resize(robot()->dof());
  dq_.resize(robot()->dof());
  tau_.setZero();
  dq_.setZero();
}

void ProjectionPredictor::nonRedundantComputeVel(const Vector3d & ivj, const bool & endEffectorFrame)
{
  Vector6d velJump;
  velJump.setZero();
  velJump.head(3) = ivj;

  // iwJump_ is supposed to be given in the end-effector link frame.
  if(endEffectorFrame)
  {
    // Computes the link frame velocity
    // ivJump_ = params().VeltoLinkFrame * velJump;
    ivJump_ = robot()->velTransform(robot()->name() + "_impactBody", robot()->lastJoint()) * velJump;
  }
  else
  {
    ivJump_ = velJump;
  }

  // RoboticsUtils::quickPrint("Non-redundant computation: vel jump", ivJump_);
  MatrixXd J, M;
  Vector6d vj = inputVelJump();

  if(params().spatialFrame)
  {
    J = robot()->spatialJacobian().block<6, 6>(0, 0);
    M = robot()->M();
  }
  else
  {
    J = robot()->bodyJacobian().block<6, 6>(0, 0);
    M = robot()->M();
  }

  dq_ = M.llt().solve(J.transpose() * (J * M.colPivHouseholderQr().inverse() * J.transpose()).inverse() * vj);

  eeV_ = J * dq_;
}

void ProjectionPredictor::computeVel(const std::shared_ptr<EndEffector> & ee,
                                     const Vector3d & inputVelJump,
                                     const bool & endEffectorFrame,
                                     const bool minimumNorm)
{
  Vector6d velJump;
  velJump.setZero();
  velJump.head(3) = inputVelJump;

  // iwJump_ is supposed to be given in the end-effector link frame.
  if(endEffectorFrame)
  {
    // Computes the link frame velocity
    // ivJump_ = params().VeltoLinkFrame * velJump;
    ivJump_ = robot()->velTransform(robot()->name() + "_impactBody", robot()->lastJoint()) * velJump;
  }
  else
  {
    ivJump_ = velJump;
  }

  computeVel_(ee, params().spatialFrame, minimumNorm);

} // end of compute

void ProjectionPredictor::computeGM(const std::shared_ptr<EndEffector> & ee,
                                    const Vector6d & eWrenchJump,
                                    const bool & sensorBodyFrame)
{
  // iwJump_ is supposed to be given in the end-effector link frame.
  if(sensorBodyFrame)
  {
    // Computes the link frame wrench
    iwJump_ = params().FTtoLinkFrame * eWrenchJump;
  }
  else
  {
    iwJump_ = eWrenchJump;
  }

  RoboticsUtils::quickError("Start the GM computation");
  computeFTGM_(ee, params().spatialFrame);

} // end of compute

void ProjectionPredictor::compute(const std::shared_ptr<EndEffector> & ee,
                                  const Vector6d & eWrenchJump,
                                  const bool & sensorBodyFrame)
{
  // iwJump_ is supposed to be given in the end-effector link frame.
  if(sensorBodyFrame)
  {
    // Computes the link frame wrench
    iwJump_ = params().FTtoLinkFrame * eWrenchJump;
  }
  else
  {
    iwJump_ = eWrenchJump;
  }

  RoboticsUtils::quickError("Start the correct computation");
  computeFT_(ee, params().spatialFrame);

} // end of compute

void ProjectionPredictor::computeFTGM_(const std::shared_ptr<EndEffector> & ee, const bool & spatialFrame)
{

  // RoboticsUtils::quickError("Calling computeFT_");
  MatrixXd J, M;
  Vector6d wj;

  if(spatialFrame)
  {
    J = ee->spatialJacobian();
    M = robot()->M();
    // Convert the input wrench from the end-effector link frame to spatial frame
    // Transform the external wrench to the inertial frame
    iwJump_ = metaAdgTMatrix(robot()->transform().inverse()) * inputWrenchJump();

    wj = inputWrenchJump();
  }
  else
  {
    J = ee->bodyJacobian();
    M = robot()->M();
    wj = inputWrenchJump();
  }

  MatrixXd tJ = J.block(0, 0, 3, robot()->dof());
  printMatrix("tJ", tJ);
  // 2. Joint torque jump
  tau_ = tJ.transpose() * wj;

  // 3. Joint velocity jump

  iim_ = tJ * M.inverse() * tJ.transpose();

  printMatrix("iim", iim_);
  printMatrix("wj", wj.head(3));
  dq_ = tJ.transpose() * (tJ * tJ.transpose()).colPivHouseholderQr().solve(iim_ * wj.head(3));
  // dq_ =  tJ.transpose() * (tJ * tJ.transpose()).inverse() * (iim * wj.head(3));

  // Note that the input is already the impulse or momentum
  printMatrix("dq", dq_);
  eeV_ = J * dq_;
  printMatrix("eev", eeV_);
  RoboticsUtils::quickError("Finished the GM calculation");
}

void ProjectionPredictor::computeFT_(const std::shared_ptr<EndEffector> & ee, const bool & spatialFrame)
{

  // RoboticsUtils::quickError("Calling computeFT_");
  MatrixXd J, M;
  Vector6d wj;

  if(spatialFrame)
  {
    J = ee->spatialJacobian();
    M = robot()->M();
    // Convert the input wrench from the end-effector link frame to spatial frame
    // Transform the external wrench to the inertial frame
    iwJump_ = metaAdgTMatrix(robot()->transform().inverse()) * inputWrenchJump();

    wj = inputWrenchJump();
  }
  else
  {
    J = ee->bodyJacobian();
    M = robot()->M();
    wj = inputWrenchJump();
  }

  MatrixXd tJ = J.block(0, 0, 3, robot()->dof());
  printMatrix("tJ", tJ);
  // 2. Joint torque jump
  tau_ = tJ.transpose() * wj;

  // 3. Joint velocity jump

  const Iso3d & tf = robot()->centroidalTransform().inverse() * robot()->transform();
  printMatrix("tf", tf.matrix());
  // const Matrix6d & ci = robot()->computeCentroidalInertia();
  const Matrix6d & ci = robot()->centroidalInertia();
  printMatrix("ci", ci);

  iim_ = Matrix3d::Identity();
  iim_ *= 1 / robot()->mass();
  const Matrix3d & t_skew = skew(tf.translation());
  iim_ -= tf.rotation().transpose() * t_skew * ci.block<3, 3>(3, 3).inverse() * t_skew * tf.rotation();

  printMatrix("iim", iim_);
  printMatrix("wj", wj.head(3));
  dq_ = tJ.transpose() * (tJ * tJ.transpose()).colPivHouseholderQr().solve(iim_ * wj.head(3));
  // dq_ =  tJ.transpose() * (tJ * tJ.transpose()).inverse() * (iim * wj.head(3));

  // Note that the input is already the impulse or momentum
  printMatrix("dq", dq_);
  eeV_ = J * dq_;
  printMatrix("eev", eeV_);
  RoboticsUtils::quickError("Finished the calculation");
}

void ProjectionPredictor::computeVel_(const std::shared_ptr<EndEffector> & ee,
                                      const bool & spatialFrame,
                                      const bool minimumNorm)
{
  MatrixXd J, M;
  Vector6d vj = inputVelJump();

  if(spatialFrame)
  {
    J = ee->spatialJacobian();
    M = robot()->M();
    // Convert the input wrench from the end-effector link frame to spatial frame
    // Transform the external wrench to the inertial frame
    // iwJump_ = metaAdgTMatrix(robot()->transform().inverse()) * inputWrenchJump();
  }
  else
  {
    J = ee->bodyJacobian();
    M = robot()->M();
  }

  // const Matrix6d & emInv = J * M.inverse() * J.transpose();
  // Matrix6d emInv = J * M.colPivHouseholderQr().inverse() * J.transpose();

  // print();

  // printMatrix("mass inertia", M);
  // eM_ = emInv.colPivHouseholderQr().inverse();

  // printMatrix("Equivalent mass", eM_);

  // 2. Joint torque jump
  // tau_ = J.transpose() * wj;

  // 3. Joint velocity jump
  // dq_ = M.householderQr().solve(J.transpose() * wj);
  // dq_ = M.householderQr().solve(J.transpose() * wj);
  // dq_ = M.colPivHouseholderQr().solve(J.transpose() * wj);
  // dq_ = M.jacobiSvd().solve(J.transpose() * wj);
  // dq_ = M.llt().solve(J.transpose() * wj);
  // dq_ = M.llt().solve(J.transpose() * eM_ * wj);

  if(minimumNorm)
  {
    dq_ = M.llt().solve(J.transpose()
                        * (MatrixXd::Identity(6, 6) + J * M.colPivHouseholderQr().inverse() * J.transpose()).inverse()
                        * vj);
  }
  else
  {
    dq_ = M.llt().solve(J.transpose() * (J * M.colPivHouseholderQr().inverse() * J.transpose()).inverse() * vj);
  }
  eeV_ = J * dq_;
}

void ProjectionPredictor::print() const
{
  if(params().spatialFrame)
  {
    std::cout << RoboticsUtils::info << "The predictor for robot: " << RoboticsUtils::hlight << robot()->name()
              << RoboticsUtils::info << " is expressed in " << RoboticsUtils::hlight << "spatial coordinates"
              << RoboticsUtils::reset << std::endl;
  }
  else
  {
    std::cout << RoboticsUtils::info << "The predictor for robot: " << RoboticsUtils::hlight << robot()->name()
              << RoboticsUtils::info << " is expressed in " << RoboticsUtils::hlight << "body coordinates"
              << RoboticsUtils::reset << std::endl;
  }

  RoboticsUtils::quickPrint("Jacobian", robot()->bodyJacobian());
  RoboticsUtils::quickPrint("COM Jacobian", robot()->comBodyJac());
  RoboticsUtils::quickPrint("COM Jacobian", robot()->bodyJacobian());
  RoboticsUtils::quickPrint("Joint velocities", robot()->jointVelocities().transpose());
  RoboticsUtils::quickPrint("cmm", robot()->cmm());
  RoboticsUtils::quickPrint("Inertia tensor", robot()->M());
  RoboticsUtils::quickPrint("Wrench transform", robot()->wrenchTransform(robot()->lastJoint(), "com"));
}

/*
void ProjectionPredictor::computeBoth(const Vector3d & wj, const Vector3d & bodyVelJump)
{

  const Iso3d g_ce = robot()->centroidalTransform().inverse() * robot()->transform();

  Vector6d iw = Vector6d::Zero();
  iw.head(3) = wj;

  Vector6d iv = Vector6d::Zero();
  iv.head(3) = bodyVelJump;

  const Vector6d & hJump = metaAdgTMatrix(g_ce.inverse())* iw;

  VectorXd rhs;
  rhs.resize(12);
  rhs.setZero();
  rhs.head(6) = hJump;
  rhs.tail(6) = iv;

  int dof = robot()->dof();
  MatrixXd lhs;
  lhs.resize(12, dof);
  lhs.setZero();

  const MatrixXd & J = robot()->bodyJacobian();

  lhs.block(0, 0, 6, dof).noalias() = robot()->cmm();
  lhs.block(6, 0, 6, dof).noalias() = J;

  // dq_ = (lhs.transpose() * lhs + MatrixXd::Identity(7,7)).inverse() * lhs.transpose() * rhs;
  dq_ = (lhs.transpose() * lhs).inverse() * lhs.transpose() * rhs;
  tau_ = J.transpose() * iw;
  eeV_ = J * dq_;

} // end of computeBoth
*/

void ProjectionPredictor::computeImpulseAndVel(const Vector3d & wj, const Vector3d & bodyVelJump)
{

  Vector6d iw = Vector6d::Zero();
  iw.head(3) = wj;

  Vector6d iv = Vector6d::Zero();
  iv.head(3) = bodyVelJump;

  const MatrixXd & J = robot()->bodyJacobian();

  /*
  VectorXd rhs;
  rhs.resize(13);
  rhs.setZero();
  rhs.head(7) = J.transpose() * iw;
  rhs.tail(6) = iv;

  int dof = robot()->dof();
  MatrixXd lhs;
  lhs.resize(13, dof);
  lhs.setZero();

  lhs.block(0, 0, 7, dof).noalias() = robot()->M();
  lhs.block(7, 0, 6, dof).noalias() = J;

  MatrixXd w = MatrixXd::Identity(6 + 7, 6 + 7);
  double w1 = 0.8;
  w.block<7, 7>(0, 0) = w1 * MatrixXd::Identity(7, 7);
  w.block<6, 6>(7, 7) = (1 - w1) * MatrixXd::Identity(6, 6);
;
*/

  // dq_ = (lhs.transpose() * lhs + MatrixXd::Identity(7,7)).inverse() * lhs.transpose() * rhs;
  // const MatrixXd & w_inv = w.inverse();
  // dq_ = (lhs.transpose() * w_inv * lhs).inverse() * lhs.transpose() * w_inv * rhs;

  solveTwoWeightedTasks_(dq_, robot()->M(), J.transpose() * iw, 0.2, J, iv);
  tau_ = J.transpose() * iw;
  eeV_ = J * dq_;

} // end of computeBoth

/*
void ProjectionPredictor::computeThree(const Vector3d & wj, const Vector3d & bodyVelJump)
{

  const Iso3d g_ce = robot()->centroidalTransform().inverse() * robot()->transform();

  Vector6d iw = Vector6d::Zero();
  iw.head(3) = wj;

  Vector6d iv = Vector6d::Zero();
  iv.head(3) = bodyVelJump;

  const Vector6d & hJump = metaAdgTMatrix(g_ce.inverse())* iw;

  const MatrixXd & J = robot()->bodyJacobian();

  VectorXd rhs;
  rhs.resize(19);
  rhs.setZero();
  rhs.head(6) = hJump;
  rhs.segment<7>(6) = J.transpose() * iw;
  rhs.tail(6) = iv;

  int dof = robot()->dof();
  MatrixXd lhs;
  lhs.resize(19, dof);
  lhs.setZero();

  lhs.block(0, 0, 6, dof).noalias() = robot()->cmm();
  lhs.block(6, 0, 6, dof).noalias() = robot()->M();
  lhs.block(13, 0, 6, dof).noalias() = J;

  // dq_ = (lhs.transpose() * lhs + MatrixXd::Identity(7,7)).inverse() * lhs.transpose() * rhs;
  dq_ = (lhs.transpose() * lhs).inverse() * lhs.transpose() * rhs;
  tau_ = J.transpose() * iw;
  eeV_ = J * dq_;

} // end of computeBoth
*/

void ProjectionPredictor::solveTwoWeightedTasks_(VectorXd & output,
                                                 const MatrixXd & J1,
                                                 const VectorXd & b1,
                                                 const double & w1,
                                                 const MatrixXd & J2,
                                                 const VectorXd & b2)
{

  if(J1.cols() != J2.cols())
  {
    RoboticsUtils::throw_runtime_error("The two tasks have different columns.", __FILE__, __LINE__);
  }

  int dof = robot()->dof();

  if(J1.cols() != dof)
  {
    RoboticsUtils::throw_runtime_error("Task column does not equal to robot's dof", __FILE__, __LINE__);
  }

  int r1 = static_cast<int>(J1.rows());
  int r2 = static_cast<int>(J2.rows());
  int r_sum = r1 + r2;

  VectorXd rhs;
  rhs.resize(r_sum);
  rhs.setZero();

  rhs.head(r1) = b1;
  rhs.tail(r2) = b2;

  MatrixXd lhs;
  lhs.resize(r_sum, dof);
  lhs.setZero();

  lhs.block(0, 0, r1, dof).noalias() = J1;
  lhs.block(r1, 0, r2, dof).noalias() = J2;

  MatrixXd w = MatrixXd::Identity(r_sum, r_sum);

  w.block(0, 0, r1, r1) = w1 * MatrixXd::Identity(r1, r1);
  w.block(r1, r1, r2, r2) = (1 - w1) * MatrixXd::Identity(r2, r2);
  ;

  const MatrixXd & w_inv = w.inverse();

  if(r_sum < dof)
  { // More columns than rows:
    output = w_inv * lhs.transpose() * (lhs * w_inv * lhs.transpose()).inverse() * rhs;
  }
  else
  { // More rows than columns:
    output = (lhs.transpose() * w_inv * lhs).inverse() * lhs.transpose() * w_inv * rhs;
  }
}

void ProjectionPredictor::solveTwoTasks_(VectorXd & output,
                                         const MatrixXd & J1,
                                         const VectorXd & b1,
                                         const MatrixXd & J2,
                                         const VectorXd & b2)
{
  VectorXd q1, q2;

  MatrixXd N1, J1_PI; // Nullspace projector, Pseudo-inverse matrix

  // RoboticsUtils::pseudoInverseMatrix(J1_PI, J1, true);

  RoboticsUtils::pseudoInverseMNMatrix(J1_PI, J1, true);

  q1 = J1_PI * b1;

  RoboticsUtils::nullSpaceProjector(J1, N1);

  VectorXd b2_correct;
  b2_correct = b2 - J2 * q1;

  RoboticsUtils::pseudoInverseSolve(q2, J2 * N1, b2_correct, true);

  output = q1 + q2;
}

void ProjectionPredictor::computeWeightedCMMAndVel(const Vector3d & wj, const Vector3d & bodyVelJump)
{
  Vector6d iw = Vector6d::Zero();
  iw.head(3) = wj;

  Vector6d iv = Vector6d::Zero();
  iv.head(3) = bodyVelJump;

  const MatrixXd & J = robot()->bodyJacobian();

  const MatrixXd & cmm = robot()->cmm();

  const Iso3d g_ce = robot()->centroidalTransform().inverse() * robot()->transform();

  const Vector6d & hJump = metaAdgTMatrix(g_ce.inverse()) * iw;

  solveTwoWeightedTasks_(dq_, cmm, hJump, 0.2, J, iv);

  // RoboticsUtils::pseudoInverseSolve(dq_, J, iv);
  tau_ = J.transpose() * wj;
  eeV_ = J * dq_;
}

void ProjectionPredictor::computeNullspace(const Vector3d & wj, const Vector3d & bodyVelJump)
{

  Vector6d iw = Vector6d::Zero();
  iw.head(3) = wj;

  Vector6d iv = Vector6d::Zero();
  iv.head(3) = bodyVelJump;

  const MatrixXd & J = robot()->bodyJacobian();

  const MatrixXd & cmm = robot()->cmm();

  const Iso3d g_ce = robot()->centroidalTransform().inverse() * robot()->transform();

  const Vector6d & hJump = metaAdgTMatrix(g_ce.inverse()) * iw;

  // solveTwoTasks_(dq_, J, iv, cmm, hJump);
  solveTwoTasks_(dq_, cmm, hJump, J, iv);

  // RoboticsUtils::pseudoInverseSolve(dq_, J, iv);
  tau_ = J.transpose() * wj;
  eeV_ = J * dq_;
}

} // namespace GeoRobotics
