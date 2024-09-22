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
#include "GeometricRobotics/ImpactDynamics/FrictionlessModel.h"

namespace GeoRobotics
{

FLModelBridge::FLModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                             const std::shared_ptr<EndEffector> ee,
                             const ImpactModelParams & params)
: ImpactDynamicsModel(robotPtr, ee, params)
{

  // Initialize the two-dim-model
  // Energetic coefficient of restitution
  flParams_.e = params.coeR;

  FLModelPtr_.reset(new FIDynamics::FLModel(flParams_));

  if(!params.bodyCoordinates)
  {
    RoboticsUtils::throw_runtime_error("By default, the FL model assumes body coordinates representation.", __FILE__,
                                       __LINE__);
  }

  resetNormals_();
  RoboticsUtils::quickInfo("FLModelBridge is created.");
}

void FLModelBridge::update(const Matrix6d & inertia, const Vector6d & v)
{

  printMatrix("impactNormal", impactNormal());

  const Vector3d bodyTV = endEffector()->pointVelocity(true);

  printMatrix("bodyTV", bodyTV);
  double contactVel = (impactNormal().transpose() * bodyTV)(0);

  int dof = robot()->dof();
  // MatrixXd Jm; // Block-diagonal matrix. The ith non-zero block corresponds to the ith joint writes: T^T * \xi

  // Jm.resize(5 * dof, dof);
  // Jm.setZero();

  // for(int ii = 0; ii < dof; ii++)
  //{
  //  auto j = robot()->joint(ii);
  //  int idx = robot()->jointNameToIndex(j->name());

  //  // Last column of the body Jacobian.
  //   const Vector6d & jj = metaAdgMatrix(robot()->transform(j->name()).inverse()) *
  //   robot()->spatialJacobian().col(ii);
  //  //const Vector6d & jj = metaAdgMatrix(robot()->transform(j->name()).inverse()) *
  //  robot()->spatialJacobian().col(idx); Jm.block<5, 1>(ii * 5, ii).noalias() = j->cwBasis().transpose() * jj;
  //   std::cout<<"Joint: "<<ii<<std::endl;
  //   printMatrix("cwbasis transpose", j->cwBasis().transpose());
  //   printMatrix("jacobian", jj);
  //   printMatrix("product", j->cwBasis().transpose() * jj);
  //}

  // printMatrix("Jm", Jm);
  const MatrixXd & mInv = robot()->M().partialPivLu().inverse();
  // const MatrixXd & mInv = robot()->M().inverse();
  // printMatrix("mInv", mInv);
  // const MatrixXd & eJac = endEffector()->bodyJacobian().block(0, 0, 3, dof);
  const MatrixXd & eJac = endEffector()->bodyJacobian();
  // printMatrix("ejac", eJac);
  // const MatrixXd temp = Jm * mInv;
  // printMatrix("temp", temp);
  // const MatrixXd & H = -(temp * Jm.transpose()).partialPivLu().inverse() * temp;

  // const Matrix3d & iim = temp * (Jm.transpose() * H + MatrixXd::Identity(dof, dof)) * eJac.transpose();
  const MatrixXd & js_iim = eJac * mInv * eJac.transpose();
  printMatrix("Textbook jointspace iim", js_iim);

  // const Matrix6d & em = js_m.partialPivLu().inverse();
  //// printMatrix("em block inverse", em.block<3,3>(0,0).partialPivLu().inverse());
  //// const Matrix3d & iim = (eJac * mInv * eJac.transpose()).block<3,3>(0,0);
  // const Matrix3d & iim = em.block<3, 3>(0, 0).partialPivLu().inverse();

  // printMatrix("iim", iim);

  const Matrix3d & iim = js_iim.block<3, 3>(0, 0);

  double iims = impactNormal().transpose() * iim * impactNormal();
  ems_ = 1.0 / iims;

  coeStick_ = calcCoeStick_(iim);
  printMatrix("iim", iim.inverse());
  FLModelPtr_->update(contactVel, iim);
  //
  // const Matrix6d & ci = robot()->computeCentroidalInertia();
  // const Vector6d & comd = robot()->computeAverageVelocity(ci, robot()->computeCMM());
  // printMatrix("COM velocity", comd);
  // FLModelPtr_->update(comd(0), (ci.block<3, 3>(0, 0)).inverse());

  robotPostImpactStates_.impulse = FLModelPtr_->getSolution().I_r;
  robotPostImpactStates_.deltaTau = eJac.transpose() * FLModelPtr_->getSolution().I_r;
  robotPostImpactStates_.deltaDq = mInv * eJac.transpose() * FLModelPtr_->getSolution().I_r;
  printMatrix("Joint velocity jump", robotPostImpactStates_.deltaDq);
  printMatrix("Joint torque jump", robotPostImpactStates_.deltaTau);
}

FLCRBModelBridge::FLCRBModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                                   const std::shared_ptr<EndEffector> ee,
                                   const ImpactModelParams & params)
: ImpactDynamicsModel(robotPtr, ee, params)
{

  // Initialize the two-dim-model
  // Energetic coefficient of restitution
  flParams_.e = params.coeR;

  FLModelPtr_.reset(new FIDynamics::FLModel(flParams_));

  if(!params.bodyCoordinates)
  {
    RoboticsUtils::throw_runtime_error("By default, the FL model assumes body coordinates representation.", __FILE__,
                                       __LINE__);
  }

  resetNormals_();
  RoboticsUtils::quickInfo("FLCRBModelBridge is created.");
}

void FLCRBModelBridge::update(const Matrix6d & inertia, const Vector6d & v)
{

  printMatrix("impactNormal", impactNormal());

  const Vector3d bodyTV = endEffector()->pointVelocity(true);

  printMatrix("bodyTV", bodyTV);
  // double contactVel = (impactNormal().transpose() * bodyTV)(0);

  int dof = robot()->dof();
  const MatrixXd & eJac = endEffector()->bodyJacobian().block(0, 0, 3, dof);
  // printMatrix("ejac", eJac);

  // The transform from COM to the contact/impact point
  const Iso3d & tf = robot()->centroidalTransform().inverse() * endEffector()->transform();

  const Matrix6d & ci = robot()->computeCentroidalInertia();
  const Vector6d & comd = robot()->computeAverageVelocity(ci, robot()->computeCMM());
  const MatrixXd & adgInvCP = metaAdgInvMatrix(tf);
  const MatrixXd & adgInvPC = adgInvCP.inverse();
  const Vector3d newBodyTV = (adgInvCP * comd).head(3);
  printMatrix("new bodyTV", newBodyTV);
  printMatrix("ci", ci);
  printMatrix("r1", endEffector()->transform().translation() - robot()->com());
  const Matrix6d & iim_full = adgInvCP * ci.inverse() * adgInvCP.transpose();

  printMatrix("new iim", iim_full.block<3, 3>(0, 0));
  // printMatrix("full iim inverse", iim_full.inverse());

  double contactVel = (impactNormal().transpose() * newBodyTV)(0);
  // double contactVel = (impactNormal().transpose() * eJac * robot()->jointVelocities())(0);

  Matrix3d iim = Matrix3d::Identity();
  iim *= 1 / robot()->mass();
  const Matrix3d & t_skew = skew(tf.translation());
  iim -= tf.rotation().transpose() * t_skew * ci.block<3, 3>(3, 3).inverse() * t_skew * tf.rotation();

  printMatrix("iim", iim);
  printMatrix("iim inverse", iim.inverse());
  const MatrixXd & EM = adgInvPC.transpose() * ci * adgInvPC;
  printMatrix("EM", EM);

  double iims = impactNormal().transpose() * iim * impactNormal();
  ems_ = 1.0 / iims;

  coeStick_ = calcCoeStick_(iim);

  FLModelPtr_->update(contactVel, iim);
  //
  // const Vector6d & comd = robot()->computeAverageVelocity(ci, robot()->computeCMM());
  // printMatrix("COM velocity", comd);
  // FLModelPtr_->update(comd(0), (ci.block<3, 3>(0, 0)).inverse());

  robotPostImpactStates_.impulse = FLModelPtr_->getSolution().I_r;
  robotPostImpactStates_.deltaTau = eJac.transpose() * FLModelPtr_->getSolution().I_r;

  const MatrixXd & mInv = robot()->M().partialPivLu().inverse();
  robotPostImpactStates_.deltaDq = mInv * eJac.transpose() * FLModelPtr_->getSolution().I_r;
  printMatrix("Joint velocity jump", robotPostImpactStates_.deltaDq);
  printMatrix("Joint torque jump", robotPostImpactStates_.deltaTau);
}

FLAssemblyModelBridge::FLAssemblyModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                                             const std::shared_ptr<EndEffector> ee,
                                             const ImpactModelParams & params)
: ImpactDynamicsModel(robotPtr, ee, params)
{

  // Initialize the two-dim-model
  // Energetic coefficient of restitution
  flParams_.e = params.coeR;

  FLModelPtr_.reset(new FIDynamics::FLModel(flParams_));

  if(!params.bodyCoordinates)
  {
    RoboticsUtils::throw_runtime_error("By default, the FL model assumes body coordinates representation.", __FILE__,
                                       __LINE__);
  }

  resetNormals_();

  PredictorParams pipParams;
  pipParams.spatialFrame = false;
  pipParams.reset();

  bPip_ = std::make_shared<PostImpactPredictor>(robotPtr, pipParams);

  RoboticsUtils::quickInfo("FLAssemblyModelBridge is created.");
}

void FLAssemblyModelBridge::update(const Matrix6d & inertia, const Vector6d & v)
{

  printMatrix("impactNormal", impactNormal());
  // Pseudo update of the post-impact predictor. We use the constructed building blocks to compute the iim matrix.
  Vector6d wJump = Vector6d::Ones();
  bPip_->compute(wJump);

  int sd = 6 * robot()->dof();
  // int fd = 5*robot()->dof();

  // Starting index of the last block
  int ld = (robot()->dof() - 1) * 6;

  MatrixXd mInv;
  mInv.resize(sd, sd);
  mInv.setZero();

  for(int ii = 0; ii < robot()->dof(); ii++)
  {
    mInv.block<6, 6>(ii * 6, ii * 6).noalias() = bPip_->M().block<6, 6>(ii * 6, ii * 6).partialPivLu().inverse();
  }

  MatrixXd TTPT, PT, A, ID, WW, Pe, P_compose;
  TTPT.noalias() = bPip_->T().transpose() * bPip_->P().transpose();
  PT.noalias() = bPip_->P() * bPip_->T();
  A = TTPT * mInv * PT;

  ID.resize(sd, sd);
  ID.setIdentity();

  Pe.resize(sd, 6);
  Pe.setZero();
  const MatrixXd & FTe2n = metaAdgTMatrix(endEffector()->relativeTransform().inverse());
  // MatrixXd FTe2n = Matrix6d::Identity();
  Pe.block<6, 6>(ld, 0) = FTe2n;

  const MatrixXd & temp = PT * A.partialPivLu().inverse() * TTPT * mInv;
  // printMatrix("PTAinvTTPTmInv", temp);
  WW = mInv
       * (ID - temp)
       //* (Pe - bPip_->P() * bPip_->Ta() * robot()->bodyJacobian().transpose() * FTe2n);
       * (Pe - bPip_->P() * bPip_->Ta() * endEffector()->bodyJacobian().transpose());
  //* ( - bPip_->P() * bPip_->Ta() * robot()->bodyJacobian().transpose() * FTe2n);
  // printMatrix("WW", WW);

  // MatrixXd Ps;
  P_compose.resize(6, sd);
  P_compose.setZero();

  int dof = robot()->dof();
  Vector6d equivalentM = Vector6d::Zero();
  for(int ii = 0; ii < dof; ii++)
  {
    Iso3d j2n = robot()->transform(static_cast<size_t>(ii)).inverse() * endEffector()->transform();
    const Matrix6d & adgInv_j2n = metaAdgInvMatrix(j2n);
    P_compose.block<6, 6>(0, ii * 6) = adgInv_j2n;
    equivalentM += metaAdgTMatrix(j2n) * robot()->linkMomentum(static_cast<size_t>(ii));
  }
  // printMatrix("P_compose", P_compose);
  printMatrix("Equivalent momentum", equivalentM);
  printMatrix("CRB momentum", robot()->computeCMM() * robot()->jointVelocities());
  printMatrix("CRB inertia * com velocity ",
              robot()->computeCentroidalInertia().block<3, 3>(0, 0) * robot()->comVelocity());

  const Vector3d bodyTV = endEffector()->pointVelocity(true);

  printMatrix("bodyTV", bodyTV);

  RoboticsUtils::quickHL("The kinetic energy is: " + std::to_string(robot()->kineticEnergy()));

  double contactVel = (impactNormal().transpose() * bodyTV)(0);

  // int dof = robot()->dof();
  const MatrixXd & eJac = endEffector()->bodyJacobian().block(0, 0, 3, dof);

  Matrix3d iim;
  // iim = WW.block<3, 3>(ld, 0);
  const MatrixXd & result = P_compose * bPip_->P().transpose() * WW;
  iim = result.block<3, 3>(0, 0);

  printMatrix("iim", iim);
  double iims = impactNormal().transpose() * iim * impactNormal();
  ems_ = 1.0 / iims;

  printMatrix("iim inverse", iim.inverse());
  const VectorXd & dq = robot()->jointVelocities();
  std::cout << "robot kinetic energy: " << dq.transpose() * robot()->M() * dq << std::endl;
  ;
  std::cout << "robot joint-space Momemtum: " << robot()->M() * dq << std::endl;
  ;

  coeStick_ = calcCoeStick_(iim);
  FLModelPtr_->update(contactVel, iim);
  //
  // const Vector6d & comd = robot()->computeAverageVelocity(ci, robot()->computeCMM());
  // printMatrix("COM velocity", comd);
  // FLModelPtr_->update(comd(0), (ci.block<3, 3>(0, 0)).inverse());

  robotPostImpactStates_.impulse = FLModelPtr_->getSolution().I_r;
  robotPostImpactStates_.deltaTau = eJac.transpose() * FLModelPtr_->getSolution().I_r;

  // robotPostImpactStates_.deltaDq = mInv * eJac.transpose() * FLModelPtr_->getSolution().I_r;
  // printMatrix("Joint velocity jump", robotPostImpactStates_.deltaDq);
  printMatrix("Joint torque jump", robotPostImpactStates_.deltaTau);
}

FL_CRB_FB_ModelBridge::FL_CRB_FB_ModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                                             const std::shared_ptr<EndEffector> ee,
                                             const ImpactModelParams & params)
: ImpactDynamicsModel(robotPtr, ee, params)
{

  // Initialize the two-dim-model
  // Energetic coefficient of restitution
  flParams_.e = params.coeR;

  FLModelPtr_.reset(new FIDynamics::FLModel(flParams_));

  if(!params.bodyCoordinates)
  {
    RoboticsUtils::throw_runtime_error("By default, the FL model assumes body coordinates representation.", __FILE__,
                                       __LINE__);
  }

  resetNormals_();
  RoboticsUtils::quickInfo("FL_CRB_FB_ModelBridge is created.");
}

void FL_CRB_FB_ModelBridge::update(const Matrix6d & inertia, const Vector6d & v)
{

  printMatrix("impactNormal", impactNormal());

  const Vector3d bodyTV = endEffector()->pointVelocity(true);

  printMatrix("bodyTV", bodyTV);
  double contactVel = (impactNormal().transpose() * bodyTV)(0);

  int dof = robot()->dof();
  // printMatrix("ejac", eJac);

  // The transform from COM to the contact/impact point
  const Iso3d & tf = robot()->centroidalTransform().inverse() * endEffector()->transform();

  const Matrix6d & ci = robot()->computeCentroidalInertia();
  const MatrixXd & cmm = robot()->computeCMM();
  const Vector6d & comd = robot()->computeAverageVelocity(ci, cmm);

  const Matrix6d & adgInvCP = metaAdgInvMatrix(tf);
  const MatrixXd & adgInvPC = adgInvCP.inverse();
  // const Vector3d newBodyTV = (adgInvCP * comd).head(3);
  // printMatrix("new bodyTV", newBodyTV);

  // double contactVel = (impactNormal().transpose() * newBodyTV)(0);

  Matrix3d iim = Matrix3d::Identity();
  iim *= 1 / robot()->mass();
  const Matrix3d & t_skew = skew(tf.translation());
  iim -= tf.rotation().transpose() * t_skew * ci.block<3, 3>(3, 3).inverse() * t_skew * tf.rotation();

  const MatrixXd & mInv = robot()->M().partialPivLu().inverse();

  const MatrixXd & eJac = endEffector()->bodyJacobian();
  const MatrixXd & eJacTV = endEffector()->bodyJacobian().block(0, 0, 3, dof);
  const MatrixXd & comJac6d = ci.inverse() * cmm;
  const MatrixXd & A = eJac - adgInvCP * comJac6d;
  auto jv = robot()->jointVelocities();
  printMatrix("Contact point body TV: ", eJac * jv);
  printMatrix("transformed COM Vel: ", adgInvCP * comJac6d * jv);
  printMatrix("Vel diff: ", A * jv);

  printMatrix("iim", iim);
  printMatrix("iim inverse", iim.inverse());

  printMatrix("original addon", A * mInv * eJac.transpose());
  printMatrix("original addon inverse", (A * mInv * eJac.transpose()).inverse());
  // const MatrixXd addon = ((A * mInv * eJac.transpose()).inverse()).block<3, 3>(0, 0).inverse();
  const MatrixXd addon = (A * mInv * eJac.transpose()).block<3, 3>(0, 0);

  printMatrix("addon", addon);
  printMatrix("addon inverse", addon.inverse());

  iim += addon;

  const MatrixXd new_em = adgInvPC.transpose() * ci * adgInvPC + (A * mInv * eJac.transpose()).inverse();

  printMatrix("new em ", new_em);
  printMatrix("new em inverse", new_em.inverse());
  // iim = ((adgInvCP * comJac6d + A) * mInv * eJac.transpose()).block<3,3>(0,0);

  printMatrix("iim with flexibility", iim);
  double iims = impactNormal().transpose() * iim * impactNormal();
  ems_ = 1.0 / iims;

  printMatrix("iim with flexibility inverse", iim.inverse());

  coeStick_ = calcCoeStick_(iim);
  FLModelPtr_->update(contactVel, iim);
  //
  // const Vector6d & comd = robot()->computeAverageVelocity(ci, robot()->computeCMM());
  // printMatrix("COM velocity", comd);
  // FLModelPtr_->update(comd(0), (ci.block<3, 3>(0, 0)).inverse());

  robotPostImpactStates_.impulse = FLModelPtr_->getSolution().I_r;
  robotPostImpactStates_.deltaTau = eJacTV.transpose() * FLModelPtr_->getSolution().I_r;

  // const MatrixXd & mInv = robot()->M().partialPivLu().inverse();
  robotPostImpactStates_.deltaDq = mInv * eJacTV.transpose() * FLModelPtr_->getSolution().I_r;
  printMatrix("Joint velocity jump", robotPostImpactStates_.deltaDq);
  printMatrix("Joint torque jump", robotPostImpactStates_.deltaTau);
}

FL_CRB_RV_ModelBridge::FL_CRB_RV_ModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                                             const std::shared_ptr<EndEffector> ee,
                                             const ImpactModelParams & params)
: ImpactDynamicsModel(robotPtr, ee, params)
{

  // Initialize the two-dim-model
  // Energetic coefficient of restitution
  flParams_.e = params.coeR;

  FLModelPtr_.reset(new FIDynamics::FLModel(flParams_));

  if(!params.bodyCoordinates)
  {
    RoboticsUtils::throw_runtime_error("By default, the FL model assumes body coordinates representation.", __FILE__,
                                       __LINE__);
  }

  resetNormals_();
  RoboticsUtils::quickInfo("FL_CRB_RV_ModelBridge is created.");
}

void FL_CRB_RV_ModelBridge::update(const Matrix6d & inertia, const Vector6d & v)
{

  printMatrix("impactNormal", impactNormal());

  const Vector3d bodyTV = endEffector()->pointVelocity(true);

  printMatrix("bodyTV", bodyTV);
  double contactVel = (impactNormal().transpose() * bodyTV)(0);

  int dof = robot()->dof();
  // printMatrix("ejac", eJac);

  // The transform from COM to the contact/impact point
  const Iso3d & tf = robot()->centroidalTransform().inverse() * endEffector()->transform();

  const Matrix6d & ci = robot()->computeCentroidalInertia();
  const MatrixXd & cmm = robot()->computeCMM();
  const Vector6d & comd = robot()->computeAverageVelocity(ci, cmm);

  const Matrix6d & adgInvCP = metaAdgInvMatrix(tf);
  const MatrixXd & adgInvPC = adgInvCP.inverse();
  // const Vector3d newBodyTV = (adgInvCP * comd).head(3);
  // printMatrix("new bodyTV", newBodyTV);

  // double contactVel = (impactNormal().transpose() * newBodyTV)(0);

  Matrix3d iim = Matrix3d::Identity();
  iim *= 1 / robot()->mass();
  const Matrix3d & t_skew = skew(tf.translation());
  iim -= tf.rotation().transpose() * t_skew * ci.block<3, 3>(3, 3).inverse() * t_skew * tf.rotation();

  const MatrixXd & mInv = robot()->M().partialPivLu().inverse();

  const MatrixXd & eJac = endEffector()->bodyJacobian();
  const MatrixXd & eJacTV = endEffector()->bodyJacobian().block(0, 0, 3, dof);
  const MatrixXd & comJac6d = robot()->avJacobian();
  const MatrixXd & A = eJac - adgInvCP * comJac6d;
  const Matrix6d & rv_em = A * robot()->relativeVelocityInertia().partialPivLu().inverse() * A.transpose();
  // const Matrix6d & rv_em = A * robot()->relativeVelocityInertia() * A.transpose();

  auto jv = robot()->jointVelocities();

  printMatrix("Contact point body TV: ", eJac * jv);
  printMatrix("transformed COM Vel: ", adgInvCP * comJac6d * jv);
  printMatrix("Vel diff: ", A * jv);

  printMatrix("iim", iim);
  printMatrix("iim inverse", iim.inverse());

  // const MatrixXd addon = ((A * mInv * eJac.transpose()).inverse()).block<3, 3>(0, 0).inverse();
  // const MatrixXd addon = (rv_em.inverse()).block<3, 3>(0, 0).inverse();
  const MatrixXd addon = rv_em.block<3, 3>(0, 0);
  // const MatrixXd addon = rv_em.block<3, 3>(0, 0).inverse();
  // const MatrixXd addon = (A * mInv * eJac.transpose()).block<3, 3>(0, 0);

  printMatrix("rv_em", rv_em);
  printMatrix("addon", addon);

  iim += addon;

  // const MatrixXd new_em = adgInvPC.transpose() * ci * adgInvPC + (A * mInv * eJac.transpose()).inverse();

  // printMatrix("new em ", new_em);
  // printMatrix("new em inverse", new_em.inverse());
  // iim = ((adgInvCP * comJac6d + A) * mInv * eJac.transpose()).block<3,3>(0,0);

  printMatrix("iim with flexibility", iim);
  double iims = impactNormal().transpose() * iim * impactNormal();
  ems_ = 1.0 / iims;

  printMatrix("iim with flexibility inverse", iim.inverse());

  coeStick_ = calcCoeStick_(iim);
  FLModelPtr_->update(contactVel, iim);
  //
  // const Vector6d & comd = robot()->computeAverageVelocity(ci, robot()->computeCMM());
  // printMatrix("COM velocity", comd);
  // FLModelPtr_->update(comd(0), (ci.block<3, 3>(0, 0)).inverse());

  robotPostImpactStates_.impulse = FLModelPtr_->getSolution().I_r;
  robotPostImpactStates_.deltaTau = eJacTV.transpose() * FLModelPtr_->getSolution().I_r;

  // const MatrixXd & mInv = robot()->M().partialPivLu().inverse();
  robotPostImpactStates_.deltaDq = mInv * eJacTV.transpose() * FLModelPtr_->getSolution().I_r;
  printMatrix("Joint velocity jump", robotPostImpactStates_.deltaDq);
  printMatrix("Joint torque jump", robotPostImpactStates_.deltaTau);
}

FL_GM_ModelBridge::FL_GM_ModelBridge(const std::shared_ptr<KinematicChain> robotPtr,
                                     const std::shared_ptr<EndEffector> ee,
                                     const ImpactModelParams & params)
: ImpactDynamicsModel(robotPtr, ee, params)
{

  // Initialize the two-dim-model
  // Energetic coefficient of restitution
  flParams_.e = params.coeR;

  FLModelPtr_.reset(new FIDynamics::FLModel(flParams_));

  if(!params.bodyCoordinates)
  {
    RoboticsUtils::throw_runtime_error("By default, the FL model assumes body coordinates representation.", __FILE__,
                                       __LINE__);
  }

  resetNormals_();
  RoboticsUtils::quickInfo("FL_GM_ModelBridge is created.");
}

void FL_GM_ModelBridge::update(const Matrix6d & inertia, const Vector6d & v)
{

  printMatrix("impactNormal", impactNormal());

  const Vector3d bodyTV = endEffector()->pointVelocity(true);

  printMatrix("bodyTV", bodyTV);
  double contactVel = (impactNormal().transpose() * bodyTV)(0);

  int dof = robot()->dof();

  const MatrixXd & tJ = endEffector()->bodyJacobian().block(0, 0, 3, dof);
  const MatrixXd & mInv = robot()->M().partialPivLu().inverse();
  const Matrix3d & iim = tJ * mInv * tJ.transpose();

  auto jv = robot()->jointVelocities();
  // const MatrixXd new_em = adgInvPC.transpose() * ci * adgInvPC + (A * mInv * eJac.transpose()).inverse();

  // printMatrix("new em ", new_em);
  // printMatrix("new em inverse", new_em.inverse());
  // iim = ((adgInvCP * comJac6d + A) * mInv * eJac.transpose()).block<3,3>(0,0);

  printMatrix("iim by GM", iim);

  coeStick_ = calcCoeStick_(iim);
  FLModelPtr_->update(contactVel, iim);

  // const Vector6d & comd = robot()->computeAverageVelocity(ci, robot()->computeCMM());
  // printMatrix("COM velocity", comd);
  // FLModelPtr_->update(comd(0), (ci.block<3, 3>(0, 0)).inverse());

  robotPostImpactStates_.impulse = FLModelPtr_->getSolution().I_r;
  robotPostImpactStates_.deltaTau = tJ.transpose() * FLModelPtr_->getSolution().I_r;

  // const MatrixXd & mInv = robot()->M().partialPivLu().inverse();
  robotPostImpactStates_.deltaDq = mInv * tJ.transpose() * FLModelPtr_->getSolution().I_r;
  printMatrix("Joint velocity jump", robotPostImpactStates_.deltaDq);
  printMatrix("Joint torque jump", robotPostImpactStates_.deltaTau);
}

} // namespace GeoRobotics
