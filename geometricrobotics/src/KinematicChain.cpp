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
#include "GeometricRobotics/Kinematics/KinematicChain.h"

#include <RoboticsUtils/utils.h>

namespace GeoRobotics
{

Iso3d const & KinematicChain::transform(std::string const & name) const
{
  if(name == "null")
  {
    return transform_;
  }
  else if(name == "inertial")
  {
    return inertial_transform_;
  }
  else
  {
    int const & type = frameType(name);
    switch(type)
    {
      case 1: // joint
      {
        return tfs_[static_cast<size_t>(jointNameToIndex(name))];
      }
      case 2: // end-effector
      {
        return endEffector(name)->transform();
      }
      case 3: // com
      {
        return centroidalTransform();
      }
      default:
      {
        RoboticsUtils::throw_runtime_error("The requested transform: " + name + " is not found", __FILE__, __LINE__);
      }
    }
  }
}

int KinematicChain::frameType(std::string const & name) const
{
  if(frames_.find(name) == frames_.end())
  {
    RoboticsUtils::throw_runtime_error("The frame: " + name + " is not found", __FILE__, __LINE__);
    return -1;
  }
  else
    return frames_.find(name)->second;
}

void KinematicChain::removeFrame_(std::string const & name)
{
  if(frames_.find(name) == frames_.end())
  {
    RoboticsUtils::throw_runtime_error("The frame: " + name + " is not found", __FILE__, __LINE__);
  }
  else
  {
    frames_.erase(name);
  }
}

void KinematicChain::addFrame_(std::string const & name, int const & frameType)
{
  if((frameType >= 0) && (frameType <= 2))
  {
    frames_[name] = frameType;
    return;
  }
  else if((frameType == 3) && (name == "com"))
  {
    frames_[name] = frameType;
    return;
  }
  else
  {
    RoboticsUtils::throw_runtime_error(
        "The frame: " + name + ", and type: " + std::to_string(frameType) + " is unknown", __FILE__, __LINE__);
  }
}

void KinematicChain::removeEndEffector(std::string const & name)
{
  if(endEffectors_.find(name) == endEffectors_.end())
  {
    RoboticsUtils::throw_runtime_error("Unable to find End-Effector: " + name, __FILE__, __LINE__);
  }
  else
  {
    endEffectors_.erase(endEffectors_.find(name));
    removeFrame_(name);
  }
}

std::shared_ptr<EndEffector> KinematicChain::metaGetEndEffector_(std::string const & name) const
{
  if(endEffectors_.find(name) == endEffectors_.end())
  {
    RoboticsUtils::throw_runtime_error("Unable to find End-Effector: " + name, __FILE__, __LINE__);
  }
  return endEffectors_.find(name)->second;
}
void KinematicChain::createEndEffector(std::string const & name, std::string const & parentJointName, Iso3d const & tf)
{
  endEffectors_[name] = std::make_shared<EndEffector>(name, parentJointName, tf, shared_from_this());

  addFrame_(name, 2);
}

void KinematicChain::printKinematicChain() const
{
  RoboticsUtils::quickInfo("The kinematic chain: " + chainName_ + " has the following joints: ");

  for(auto & joint : joints_)
  {
    std::cout << RoboticsUtils::info << "Joint: " << joint->name() << ": " << joint->q() << ", " << RoboticsUtils::reset
              << std::endl;
  }

  std::cout << std::endl;
}

void KinematicChain::readJointPositions(VectorXd & q) const
{
  q.resize(dof());
  for(auto joint : joints_)
  {
    q(jointNameToIndex(joint->name())) = joint->q();
  }
}
void KinematicChain::readJointPositions(std::vector<double> & q) const
{
  q.resize(static_cast<size_t>(dof()));
  for(auto joint : joints_)
  {
    q[static_cast<size_t>(jointNameToIndex(joint->name()))] = joint->q();
    // q.push_back(joint->q());
  }
}

void KinematicChain::readJointVelocities(VectorXd & dq) const
{
  dq.resize(dof());
  for(auto joint : joints_)
  {
    dq(jointNameToIndex(joint->name())) = joint->dq();
  }
}
void KinematicChain::readJointVelocities(std::vector<double> & dq) const
{
  dq.resize(static_cast<size_t>(dof()));
  for(auto joint : joints_)
  {
    dq[static_cast<size_t>(jointNameToIndex(joint->name()))] = joint->dq();
  }
}
void KinematicChain::readJointAccelerations(VectorXd & ddq) const
{
  ddq.resize(dof());
  for(auto joint : joints_)
  {
    ddq(jointNameToIndex(joint->name())) = joint->ddq();
  }
}
void KinematicChain::readJointAccelerations(std::vector<double> & ddq) const
{
  ddq.resize(static_cast<size_t>(dof()));
  for(auto joint : joints_)
  {
    ddq[static_cast<size_t>(jointNameToIndex(joint->name()))] = joint->ddq();
  }
}
void KinematicChain::readJointTorques(VectorXd & tau) const
{
  tau.resize(dof());
  for(auto joint : joints_)
  {
    tau(jointNameToIndex(joint->name())) = joint->tau();
  }
}
void KinematicChain::readJointTorques(std::vector<double> & tau) const
{
  tau.resize(static_cast<size_t>(dof()));
  for(auto joint : joints_)
  {
    tau[static_cast<size_t>(jointNameToIndex(joint->name()))] = joint->tau();
  }
}
void KinematicChain::velIntegration(double dt)
{
  for(auto joint : joints_)
  {
    joint->velIntegration(dt);
  }

  updateChain_();
}

void KinematicChain::setRandom(size_t status, std::default_random_engine & re)
{
  for(auto joint : joints_)
  {
    joint->setRandom(status, re);
  }

  updateChain_();
}

void KinematicChain::setJointVelocities(VectorXd const & dq)
{
  assert(dq.size() == dof());
  for(int idx = 0; idx < dq.size(); idx++)
  {
    auto & joint = joints_[static_cast<size_t>(idx)];
    joint->updateDq(dq(idx));
  }

  // Update the current joint velocities !!!
  readJointVelocities(dq_);
  // updateJointVelDependentComponents_();

  updateChain_();
}
void KinematicChain::setJointVelocities(std::vector<double> const & dq)
{

  assert(dq.size() == dof());
  for(size_t idx = 0; idx < dq.size(); idx++)
  {
    auto & joint = joints_[static_cast<size_t>(idx)];
    joint->updateDq(dq[idx]);
  }

  // Update the current joint velocities !!!
  readJointVelocities(dq_);
  // updateJointVelDependentComponents_();

  updateChain_();
}
void KinematicChain::setJointPositions(VectorXd const & q)
{
  assert(q.size() == dof());
  for(int idx = 0; idx < q.size(); idx++)
  {
    joints_[static_cast<size_t>(idx)]->update(q(idx));
  }
  updateChain_();

  // Update the current joint positions !!!
  readJointPositions(q_);
}
void KinematicChain::setJointPositions(std::vector<double> const & q)
{
  assert(q.size() == dof());
  for(size_t idx = 0; idx < q.size(); idx++)
  {
    joints_[idx]->update(q[idx]);
  }
  updateChain_();

  // Update the current joint positions !!!
  readJointPositions(q_);
}
std::string const & KinematicChain::linkIndexToName(size_t const & idx) const
{
  if(idx >= links_.size())
  {
    RoboticsUtils::throw_runtime_error("idx: " + std::to_string(idx) + "is out of the range.", __FILE__, __LINE__);
  }
  return links_[idx]->name();
}

std::string const & KinematicChain::jointIndexToName(size_t const & idx) const
{
  if(idx >= joints_.size())
  {
    RoboticsUtils::throw_runtime_error("idx: " + std::to_string(idx) + "is out of the range.", __FILE__, __LINE__);
  }
  return joints_[idx]->getURDFJoint()->name;
}

int KinematicChain::linkNameToIndex(std::string const & name) const
{
  if(linkIndices_.find(name) == linkIndices_.end())
  {
    RoboticsUtils::throw_runtime_error("The joint: " + name + " is not found", __FILE__, __LINE__);
    return -1;
  }
  else
    return linkIndices_.find(name)->second;
}

int KinematicChain::jointNameToIndex(std::string const & name) const
{
  auto it = jointIndices_.find(name);
  if(it == jointIndices_.end())
  {
    RoboticsUtils::throw_runtime_error("The joint: " + name + " is not found", __FILE__, __LINE__);
    return -1;
  }

  return it->second;
}

Iso3d KinematicChain::incrementalTransform(std::string const & jointName) const
{
  return expMap(twistCoordinate(jointName), joint(jointName)->q());
}

void KinematicChain::updateJacobians_()
{
  com_.setZero();
  comJac_.setZero();
  NN_.setZero();
  Vp_ = 0;

  spatialJacobian_.noalias() = computeSpatialJacobian();
  bodyJacobian_.noalias() = metaAdgInvMatrix(transform()) * spatialJacobian();

  // update COM
  // for (auto l : links())
  //{
  //
  //  double weight = l->urdfLink()->metaInertia->mass / mass();
  //  const Vector3d & inertialCom = transform(l->parentJointName()) * l->localCOM();
  //  com_ += weight * inertialCom;
  //}

  // Update the link COM body Jacobians.
  for(auto const & j : joints())
  {
    // j->childLink();
    auto const & l = j->childLink();

    auto jName = j->name();
    Iso3d const & j_tf = transform(jName);
    int jIdx = jointNameToIndex(jName);
    spatialJacs_[static_cast<size_t>(jIdx)].noalias() = spatialJacobian_.leftCols(jIdx + 1);
    spatialJacsFull_[static_cast<size_t>(jIdx)].block(0, 0, 6, jIdx + 1).noalias() =
        spatialJacobian_.leftCols(jIdx + 1);

    bodyJacs_[static_cast<size_t>(jIdx)].noalias() = metaAdgInvMatrix(j_tf) * spatialJacobian(jName);
    bodyJacsFull_[static_cast<size_t>(jIdx)].noalias() = metaAdgInvMatrix(j_tf) * spatialJacobian(jName, true);
    // Update the link momentum:

    // COM related quantities:
    auto lName = j->childLink()->name();
    size_t lIdx = static_cast<size_t>(linkNameToIndex(lName));

    linkMomentums_[lIdx] = l->localInertiaTensor() * bodyJacs_[static_cast<size_t>(jIdx)]
                           * jointVelocities().head(static_cast<int>(lIdx + 1));

    linkComBodyJacs_[lIdx].noalias() = computeLinkComBodyJac_(lName);

    double weight = l->urdfLink()->metaInertia->mass / mass();
    const Vector3d & inertialCom = transform(j->name()) * l->localCOM();

    com_ += weight * inertialCom;

    const MatrixXd & bodyJac = linkComBodyJacs_[lIdx];

    const MatrixXd & temp = j_tf.rotation() * bodyJac;

    auto dof = bodyJac.cols();

    comJac_.block(0, 0, 3, bodyJac.cols()).noalias() += weight * temp;

    // size_t lIdx = static_cast<size_t>(linkNameToIndex(link->name()));

    linkComVels_[lIdx] = temp * dq_.head(dof);

    // Update N:
    NN_.head(dof) += l->mass() * (-GRAVITY) * temp.block(2, 0, 1, dof).transpose();

    // Update potential energy
    Vp_ += l->mass() * GRAVITY * inertialCom.z();
  }

  // double baseWeight = baselink_->urdfLink()->metaInertia->mass / mass();
  // const Vector3d & bctf = baselink_->localCOM();
  // com_ += baseWeight * bctf;
  cmmtf_.translation() = com();
  comVelocity_ = comJac_ * dq_;
}

void KinematicChain::updateChain_()
{
  readJointPositions(q_);
  readJointVelocities(dq_);
  readJointAccelerations(ddq_);
  // readJointTorques(tau_);

  updateTransform_();

  updateJacobians_();

  updateInertia_CoriolisAndCentrifugalForces_();

  for(auto e : endEffectors_)
  {
    // e.second.lock()->update();
    e.second->update();
  }

  updateCentroidalDynamics_();
}

MatrixXd KinematicChain::computeMs() const
{
  MatrixXd m = MatrixXd::Zero(dof(), dof());

  for(auto j : joints_)
  {

    std::shared_ptr<GeoLink> l = this->link(j->getURDFJoint()->childLinkName);

    long const size = this->spatialJacobian(j->name()).cols();

    MatrixXd const & sj = this->spatialJacobian(j->name());
    m.block(0, 0, size, size).noalias() += sj.transpose() * l->spatialInertia(transform(j->name())) * sj;
  }

  return m;
}

void KinematicChain::updateInertiaTensor_()
{
  M_.setZero();
  // Md_.setZero();
  // M_.setZero();

  for(auto const & joint : joints_)
  {

    std::shared_ptr<GeoLink> link = this->link(joint->getURDFJoint()->childLinkName);
    long const size = this->spatialJacobian(joint->name()).cols();
    // 1. compute by body-frame coordinates, i.e., all the Jacobians are body Jacobians
    MatrixXd const & bj = this->bodyJacobian(joint->name());
    // const MatrixXd & bjd = computeBodyJacobianDerivative(joint->name());

    MatrixXd const & Mbj = link->localInertiaTensor() * bj;

    M_.block(0, 0, size, size).noalias() += bj.transpose() * Mbj;
    // Md_.block(0, 0, size, size).noalias() += bjd.transpose() * Mbj + bj.transpose() * link->localInertiaTensor() *
    // bjd;
  }
}

Eigen::MatrixXd KinematicChain::computeMd(int const & idx) const
{
  MatrixXd output = MatrixXd::Zero(dof(), dof());

  for(auto const & joint : joints_)
  {
    int jIdx = jointNameToIndex(joint->name());
    if(jIdx < idx)
    {
      continue;
    }
    std::shared_ptr<GeoLink> link = this->link(joint->getURDFJoint()->childLinkName);
    long const size = this->spatialJacobian(joint->name()).cols();
    // 1. The body-frame inertia tensor, i.e., all the Jacobians are body Jacobians
    MatrixXd const & bj = this->bodyJacobian(joint->name());
    MatrixXd const & bjd = computeBodyJacobianDerivative(jIdx, idx);

    output.block(0, 0, size, size).noalias() +=
        bjd.transpose() * link->localInertiaTensor() * bj + bj.transpose() * link->localInertiaTensor() * bjd;
  }

  return output;
}

void KinematicChain::updateCentroidalDynamics_()
{
  // (0) cmmtf is updated after the COM is udpated.

  // (1) Update the CI and the CMM
  ci_.noalias() = computeCentroidalInertia();
  cmm_.noalias() = computeCMM();

  aVel_.noalias() = computeAverageVelocity(centroidalInertia(), cmm());
  // aVel_ = centroidalInertia().partialPivLu().inverse() * cmm() * jointVelocities();

  aVelJac_.noalias() = centroidalInertia().colPivHouseholderQr().inverse() * cmm();

  // Update the relative velocity Jacobians and the relative velocity inertia.
  rCi_.setZero();
  for(auto const & j : joints())
  {
    auto jtf = transform(j->name());
    // This is the transform from i->cmm frame: g_ci = g_sc ^ (-1) * g_si
    Iso3d const & g_ic = jtf.inverse() * centroidalTransform();
    // Momentum transform from the parent joint frame to the CMM frame

    size_t const idx = static_cast<size_t>(jointNameToIndex(j->name()));
    rVelJacs_[idx].noalias() = bodyJacobian(j->name(), true) - metaAdgMatrix(g_ic) * aVelJac_;

    rCi_.noalias() +=
        rVelJacobian(j->name()).transpose() * j->childLink()->localInertiaTensor() * rVelJacobian(j->name());
  }
}

void KinematicChain::updateInertia_CoriolisAndCentrifugalForces_()
{

  M_.setZero();
  CC_.setZero();
  Mdv_.clear();
  Md_.setZero();
  Ke_ = 0.0;

  // 2. Compute the Inertia partial derivatives:

  std::vector<MatrixXd> MJv;

  for(auto const & l : links())
  {

    MatrixXd const & bj = bodyJacobian(l->parentJointName());

    long const size = bj.cols();

    MatrixXd const & Mbj = l->localInertiaTensor() * bj;
    MJv.push_back(Mbj);

    M_.block(0, 0, size, size).noalias() += bj.transpose() * Mbj;
  }
  // Update the kinetic energy:
  Ke_ = 0.5 * jointVelocities().dot(M_ * jointVelocities());

  for(auto const & j : joints())
  {
    // Update the coriolis and centrifugal forces:
    MatrixXd output = MatrixXd::Zero(dof(), dof());

    int idx = jointNameToIndex(j->name());
    for(auto const & joint : joints_)
    {
      int jIdx = jointNameToIndex(joint->name());
      if(jIdx < idx)
      {
        continue;
      }
      long const size = this->spatialJacobian(joint->name()).cols();
      // 1. The body-frame inertia tensor, i.e., all the Jacobians are body Jacobians
      MatrixXd const & bjd = computeBodyJacobianDerivative(jIdx, idx);

      output.block(0, 0, size, size).noalias() += bjd.transpose() * MJv[jIdx] + MJv[jIdx].transpose() * bjd;
    }

    Mdv_.push_back(output);
    Md_ += output * dq_(jointNameToIndex(j->name()));
  }
  // 3. Construct the CC matrix

  for(int i = 0; i < dof(); i++)
  {
    for(int j = 0; j < dof(); j++)
    {
      for(int k = 0; k < dof(); k++)
      {
        CC_(i, j) += (Mdv_[k](i, j) + Mdv_[j](i, k) - Mdv_[i](k, j)) * dq_(k);
      }
    }
  }

  CC_ *= 0.5;
}

/*
void KinematicChain::updateJointVelDependentComponents_()
{
  // Update the derivative of the inertia matrix
  for(auto joint : joints_)
  {

    std::shared_ptr<GeoLink> link = this->link(joint->getURDFJoint()->childLinkName);
    long size = bodyJacobian(joint->name()).cols();
    // 1. The body-frame inertia tensor, i.e., all the Jacobians are body Jacobians
    MatrixXd bj = bodyJacobian(joint->name());

    MatrixXd bjd = computeBodyJacobianDerivative(joint->name());

    Mbd_.block(0, 0, size, size) +=
        bjd.transpose() * link->localInertiaTensor() * bj + bj.transpose() * link->localInertiaTensor() * bjd;
  }
}
*/

void KinematicChain::updateTransform_()
{
  // Following eq.~3.3 of Murray et al. 1994
  transform_.setIdentity();
  for(auto const & joint : joints_)
  {
    transform_ = transform_ * incrementalTransform(joint->name());

    tfs_[static_cast<size_t>(jointNameToIndex(joint->name()))] =
        transform_
        * initialTransform(joint->name()); // Note that the initial transform is not accumulated into transform_
  }

  transform_ = transform_ * initialTransform();
}

void KinematicChain::initializeChain_()
{
  // Set the initial transform and the spatial twist coordinates once for all
  Iso3d initialTransform = Iso3d::Identity();

  addFrame_("com", 3);

  int jIdx = 0;
  // RoboticsUtils::quickInfo("Initialize the KinematicChain: " + chainName_);
  for(auto joint : joints_)
  {

    // 0. Store the indices
    jointIndices_[joint->getURDFJoint()->name] = jIdx;
    addFrame_(joint->getURDFJoint()->name, 1);
    jIdx++;

    // 2. The initial transforms:
    // std::cout<<"The initial transform of the joint: " << joint->getURDFJoint()->name <<std::endl;
    // std::cout<<RoboticsUtils::hlight<< joint->initialRelativeTransform().matrix()<<RoboticsUtils::reset<<std::endl;
    initialTfs_.push_back(initialTransform * joint->initialRelativeTransform());
    // initialTransform_ = joint->initialRelativeTransform() * initialTransform_;
    initialTransform = initialTfs_.back();

    // std::cout<<"The current initial transform is: " <<std::endl;
    // std::cout<<RoboticsUtils::error<< initialTransform_.matrix()<<RoboticsUtils::reset<<std::endl;

    // 1. the spatial twists (w.r.t. the root link frame)
    joint->setTwistCoordinate(joint->computeTwistCoordinate(initialTransform));
    joint->setInitialTransform(initialTransform);
    tcs_.push_back(joint->twistCoordinate());
  }
  tfs_ = initialTfs_;

  linkMomentums_.resize(static_cast<size_t>(dof()));

  int lIdx = 0;
  mass_ = 0.0;
  for(auto link : links_)
  {

    // 0. Store the indices
    linkIndices_[link->urdfLink()->name] = lIdx;
    std::cout << "linkname: " << link->urdfLink()->name << ", with index: " << lIdx << std::endl;

    linkMomentums_[static_cast<size_t>(lIdx)].setZero();

    lIdx++;
    // Accumulating the mass:
    mass_ += link->mass();
  }
  // mass_ += baselink_->urdfLink()->metaInertia->mass;

  transform_ = initialTransform;

  initializeVars_();

  RoboticsUtils::quickInfo("Initialized the KinematicChain: " + chainName_);
}

void KinematicChain::initializeVars_()
{
  CC_ = MatrixXd::Zero(dof(), dof());
  NN_ = VectorXd::Zero(dof());

  M_ = MatrixXd::Zero(dof(), dof());
  Md_ = MatrixXd::Zero(dof(), dof());
  // M_ = MatrixXd::Zero(dof(), dof());
  spatialJacobian_ = MatrixXd::Zero(6, dof());
  spatialJacobianDot_ = MatrixXd::Zero(6, dof());
  bodyJacobian_ = MatrixXd::Zero(6, dof());
  bodyJacobianDot_ = MatrixXd::Zero(6, dof());

  comJac_.resize(3, dof());

  size_t dof_size_t = static_cast<size_t>(dof());
  linkComBodyJacs_.resize(dof_size_t);

  bodyJacs_.resize(dof_size_t);
  bodyJacsFull_.resize(dof_size_t);
  spatialJacs_.resize(dof_size_t);
  spatialJacsFull_.resize(dof_size_t);

  for(size_t ii = 0; ii < dof_size_t; ii++)
  {
    // spatialJacs_[ii].resize(6, dof());
    // spatialJacs_[ii].setZero();

    spatialJacsFull_[ii] = MatrixXd::Zero(6, dof());
  }

  linkComVels_.resize(dof_size_t);

  for(auto link : links())
  {
    linkComBodyJacs_[static_cast<size_t>(linkNameToIndex(link->name()))] = computeLinkComBodyJac_(link->name());
  }

  q_.resize(dof());
  dq_.resize(dof());
  ddq_.resize(dof());

  // The centroidal dynamics:
  cmm_ = MatrixXd::Zero(6, dof());

  aVelJac_ = MatrixXd::Zero(6, dof());

  rVelJacs_.resize(dof_size_t);
  rCi_.resize(dof(), dof());
}

MatrixXd KinematicChain::computeLinkComBodyJac_(std::string const & linkName) const
{
  std::shared_ptr<GeoLink> const & l = link(linkName);
  MatrixXd const & bodyJac = bodyJacobian(l->parentJointName());
  MatrixXd const & linearJac = bodyJac.block(0, 0, 3, bodyJac.cols());
  MatrixXd const & angularJac = bodyJac.block(3, 0, 3, bodyJac.cols());
  MatrixXd comJac;
  comJac.resize(3, bodyJac.cols());
  comJac.noalias() = -skew(l->localCOM()) * angularJac + linearJac;
  return comJac;
}
void KinematicChain::printInertiaTensors(int const & choice) const
{

  RoboticsUtils::quickInfo("The KinematicChain: " + chainName_ + " has the inertial tensors: ");

  for(auto link : links_)
  {
    link->printInertiaTensors(choice);
  }
}

void KinematicChain::printSpatialTwists() const
{
  // The relative transform from root to leaf
  Iso3d tfs;
  tfs.setIdentity();
  RoboticsUtils::quickInfo("The KinematicChain: " + chainName_ + " has the twists: ");
  for(auto joint : joints_)
  {

    tfs = tfs * joint->initialRelativeTransform();
    std::cout << "Joint: " << joint->getURDFJoint()->name << std::endl;
    // compute tc in the root frame:
    std::cout << RoboticsUtils::info << "transform: " << std::endl << tfs.matrix() << std::endl;
    std::cout << RoboticsUtils::info << "inertial frame twist: " << std::endl
              << joint->twistCoordinate().vector() << std::endl;
    // TwistCoordinate xi_prime = metaTwistTransform(tfs, joint->localTwistCoordinate());

    std::cout << RoboticsUtils::error << "Spatial frame twist: " << std::endl;
    // std::cout<<RoboticsUtils::hlight<<xi_prime.vector()<<RoboticsUtils::reset<<std::endl;
    std::cout << RoboticsUtils::hlight << twistCoordinate(joint->getURDFJoint()->name).vector() << RoboticsUtils::reset
              << std::endl;

    // tfs = tfs * joint->relativeTransform();
  }
}

Eigen::MatrixXd KinematicChain::computeBodyJacobian(std::string const & jointName, bool const & fullSize) const
{
  Eigen::MatrixXd output;

  computeBodyJacobian_(static_cast<size_t>(jointNameToIndex(jointName)), output, fullSize);
  return output;
}

void KinematicChain::computeBodyJacobian_(size_t const & idx, Eigen::MatrixXd & output, bool const & fullSize) const
{
  size_t local_dof = idx + 1;
  setJacFullSize_(output, static_cast<int>(local_dof), fullSize);
  output.setZero();

  Iso3d accumulated_tf = initialTransform(jointIndexToName(idx));

  for(size_t ii = 0; ii < local_dof; ii++)
  {
    // size_t jj = joints_.size() - ii - 1;
    size_t jj = local_dof - ii - 1;

    accumulated_tf = incrementalTransform(jointIndexToName(jj)) * accumulated_tf;

    output.col(static_cast<int>(jj)) = metaAdgInv(accumulated_tf, tcs_[jj]).vector();
  }
}

Eigen::MatrixXd KinematicChain::computeBodyJacobian() const
{
  Eigen::MatrixXd output;

  computeBodyJacobian_(static_cast<size_t>(dof() - 1), output);

  return output;
}
/*
Eigen::MatrixXd KinematicChain::computeBodyJacobian() const
{
  Eigen::MatrixXd output;
  output.resize(6, dof());
  output.setZero();

  Iso3d accumulated_tf = initialTransform();

  for(size_t ii = 0; ii < joints_.size(); ii++)
  {
    size_t idx = joints_.size() - ii - 1;

    accumulated_tf = incrementalTransform(jointIndexToName(idx)) * accumulated_tf;

    output.col(static_cast<int>(idx)) = metaAdgInv(accumulated_tf, tcs_[idx]).vector();
  }

  return output;
}
*/

void KinematicChain::computeBodyJacobianDot_(MatrixXd const & jacobian,
                                             size_t const & idx,
                                             VectorXd const & dq,
                                             Eigen::MatrixXd & output) const
{

  // The number of non-zero coloumns
  int cols = static_cast<int>(idx) + 1;
  output = MatrixXd::Zero(6, cols);

  // Loop through the cols
  for(size_t ii = 0; ii < static_cast<size_t>(cols); ii++)
  {
    // output.col(static_cast<int>(ii)) = tc_prime_dot_(jacobian, ii);
    Vector6d tc_dot;
    tc_dagger_dot_(jacobian, ii, cols, dq, tc_dot);
    output.col(static_cast<int>(ii)) = tc_dot;
  }
}

void KinematicChain::tc_dagger_dot_(MatrixXd const & jacobian,
                                    size_t const & idx,
                                    int const & localDof,
                                    VectorXd const & dq,
                                    Vector6d & output) const
{
  MatrixXd matrix = MatrixXd::Zero(6, localDof);

  VectorXd const & tc_ = jacobian.col(static_cast<int>(idx));
  for(int j = static_cast<int>(idx) + 1; j < localDof; j++)
  {
    matrix.col(j) = cross(tc_, jacobian.col(j));
  }

  output = matrix * dq.segment(0, localDof);
}

void KinematicChain::computeSpatialJacobianDot_(MatrixXd const & jacobian,
                                                size_t const & idx,
                                                VectorXd const & dq,
                                                Eigen::MatrixXd & output) const
{

  // printMatrix("joint velocities:", dq);
  // The number of non-zero coloumns
  int cols = static_cast<int>(idx) + 1;
  // According to Sec.~.114 of Robot-Dynamics
  output = MatrixXd::Zero(6, cols);

  // Loop through the cols
  for(size_t ii = 0; ii < static_cast<size_t>(cols); ii++)
  {
    // output.col(static_cast<int>(ii)) = tc_prime_dot_(jacobian, ii);
    Vector6d tc_dot;
    tc_prime_dot_(jacobian, ii, dq, tc_dot);
    output.col(static_cast<int>(ii)) = tc_dot;
  }
}

void KinematicChain::tc_prime_dot_(MatrixXd const & jacobian,
                                   size_t const & idx,
                                   VectorXd const & dq,
                                   Vector6d & output) const
{
  MatrixXd matrix = MatrixXd::Zero(6, dof());

  VectorXd const & tc_ = jacobian.col(static_cast<int>(idx));
  // The inequality:  "i< static_cast<int>(idx)" indicating the dependency, i.e., joint idx depends on joint i;
  // If i = j, it is also zero.
  // The rest of the columns are all zero.
  for(int j = 0; j < static_cast<int>(idx); j++)
  {
    matrix.col(j).noalias() = cross(jacobian.col(j), tc_);
  }

  output = matrix * dq;
}
void KinematicChain::computeSpatialJacobian_(size_t const & idx, Eigen::MatrixXd & output, bool const & fullSize) const
{

  setJacFullSize_(output, static_cast<int>(idx) + 1, fullSize);

  output.setZero();

  output.col(0).noalias() = tcs_[0].vector();

  Iso3d accumulated_tf;
  accumulated_tf.setIdentity();

  // Loop through the rest
  for(size_t ii = 1; ii < idx + 1; ii++)
  {
    Iso3d const & tf = incrementalTransform(jointIndexToName(ii - 1));
    accumulated_tf = accumulated_tf * tf;

    output.col(static_cast<int>(ii)).noalias() = metaAdg(accumulated_tf, tcs_[ii]).vector();
  }
}

Eigen::MatrixXd KinematicChain::computeSpatialJacobian(std::string const & jointName, bool const & fullSize) const
{
  size_t idx = static_cast<size_t>(jointNameToIndex(jointName));

  Eigen::MatrixXd output;

  computeSpatialJacobian_(idx, output, fullSize);

  return output;
}

Eigen::MatrixXd KinematicChain::computeSpatialJacobianDerivative(int const & idx, int const & deltaIdx) const
{

  Eigen::MatrixXd output;
  // We do not need to reconstruct smaller Jacobians
  Eigen::MatrixXd const & jac = spatialJacobian();

  // long dof = jac.cols();
  output = MatrixXd::Zero(6, idx + 1);

  if(idx <= deltaIdx)
  {
    return output;
  }
  else
  {
    // (0 - deltaIdx) are zero columns
    for(int ii = deltaIdx; ii <= idx; ii++)
    // Iterate over the axes
    {
      output.col(ii).noalias() = cross(jac.col(deltaIdx), jac.col(ii));
    }
    return output;
  }
}

MatrixXd KinematicChain::computeBodyJacobianDerivative(int const & idx, int const & deltaIdx) const
{
  if(deltaIdx > idx)
  {
    RoboticsUtils::throw_runtime_error(
        "Joint " + std::to_string(deltaIdx) + " dopes not affect Jacobian: " + std::to_string(idx), __FILE__, __LINE__);
  }
  MatrixXd output = MatrixXd::Zero(6, idx + 1);

  // In the frame of idx
  MatrixXd const & jac = bodyJacobian(jointIndexToName(idx));

  // Iterate over the axes
  // for(int ii = 0; ii<jac.cols(); ii++)
  for(int ii = 0; ii < deltaIdx; ii++)
  {
    output.col(ii).noalias() = cross(jac.col(ii), jac.col(deltaIdx));
  }
  return output;
}

Eigen::MatrixXd KinematicChain::computeBodyJacobianDerivativeRF(std::string const & jointName) const
{
  Eigen::MatrixXd output;

  VectorXd dq;
  readJointVelocities(dq);

  MatrixXd const & jac = bodyJacobian(jointName);

  // computeBodyJacobianDot_(, static_cast<size_t>(jointNameToIndex(jointName)), dq, output);

  int idx = jointNameToIndex(jointName);
  // The number of non-zero coloumns
  int cols = static_cast<int>(idx) + 1;
  output = MatrixXd::Zero(6, cols);

  // Loop through the cols
  for(int ii = 0; ii < cols; ii++)
  {
    // output.col(static_cast<int>(ii)) = tc_prime_dot_(jacobian, ii);
    // Vector6d tc_dot;
    // tc_dagger_dot_(jac, ii, cols, dq, tc_dot);
    output.col(ii) = cross(link(ii)->bodyVelocity(), jac.col(ii));
  }

  return output;
}

Eigen::MatrixXd KinematicChain::computeBodyJacobianDerivative(std::string const & jointName) const
{
  Eigen::MatrixXd output;

  VectorXd dq;
  readJointVelocities(dq);

  computeBodyJacobianDot_(bodyJacobian(jointName), static_cast<size_t>(jointNameToIndex(jointName)), dq, output);

  return output;
}

Eigen::MatrixXd KinematicChain::computeBodyJacobianDerivative() const
{
  size_t idx = static_cast<size_t>(dof() - 1);

  Eigen::MatrixXd output;

  VectorXd dq;
  readJointVelocities(dq);
  // printMatrix("dq", dq);

  computeBodyJacobianDot_(bodyJacobian(), idx, dq, output);

  return output;
}
Eigen::MatrixXd KinematicChain::computeSpatialJacobianDerivative(std::string const & jointName) const
{
  Eigen::MatrixXd output;

  VectorXd dq;
  readJointVelocities(dq);

  computeSpatialJacobianDot_(spatialJacobian(), static_cast<size_t>(jointNameToIndex(jointName)), dq, output);

  return output;
}
Eigen::MatrixXd KinematicChain::computeSpatialJacobianDerivative() const
{
  size_t idx = static_cast<size_t>(dof() - 1);

  Eigen::MatrixXd output;

  VectorXd dq;
  readJointVelocities(dq);

  computeSpatialJacobianDot_(spatialJacobian(), idx, dq, output);

  return output;
}

Eigen::MatrixXd KinematicChain::computeSpatialJacobian() const
{
  size_t idx = static_cast<size_t>(dof() - 1);

  Eigen::MatrixXd output;

  computeSpatialJacobian_(idx, output);

  return output;
}

Vector6d KinematicChain::mBasis(std::string const & jointName) const
{
  Iso3d const & tf = transform(jointName);
  return metaAdgMatrix(tf) * joint(jointName)->awBasis();
}

Vector6d KinematicChain::awBasis(std::string const & jointName) const
{
  Iso3d const & tf = transform(jointName);
  return metaAdgTMatrix(tf.inverse()) * joint(jointName)->awBasis();
}

Eigen::Matrix<double, 6, 5> KinematicChain::cwBasis(std::string const & jointName) const
{
  Iso3d const & tf = transform(jointName);
  return metaAdgTMatrix(tf.inverse()) * joint(jointName)->cwBasis();
}

void KinematicChain::updateCOM()
{
  com_.setZero();
  double weightSum = 0;
  comJac_.setZero();
  NN_.setZero();
  Vp_ = 0;
  for(auto joint : joints())
  {
    auto link = joint->childLink();
    double const weight = link->urdfLink()->metaInertia->mass;
    weightSum += weight;
    Vector3d const & inertialCom = transform(joint->name()) * link->localCOM();
    com_ += weight * inertialCom;

    // const MatrixXd & bodyJac = bodyJacobian(joint->name());
    // auto linearJac = bodyJac.block(0, 0, 3, bodyJac.cols());
    // auto angularJac = bodyJac.block(3, 0, 3, bodyJac.cols());

    // Update the COM Jacobian
    // comJac_.block(0, 0, 3, bodyJac.cols()) += transform(joint->name()).rotation() * weight * (-skew(link->localCOM())
    // * angularJac + linearJac);
    MatrixXd const & bodyJac = linkComBodyJac(link->name());
    MatrixXd const & temp = transform(joint->name()).rotation() * bodyJac;
    auto dof = bodyJac.cols();

    comJac_.block(0, 0, 3, bodyJac.cols()).noalias() += weight * temp;

    size_t lIdx = static_cast<size_t>(linkNameToIndex(link->name()));
    linkComVels_[lIdx].noalias() = temp * dq_.head(dof);

    // Update N:
    NN_.head(dof) += link->mass() * (-GRAVITY) * temp.block(2, 0, 1, dof).transpose();

    // Update potential energy
    Vp_ += link->mass() * GRAVITY * computeLinkCom(link->name()).z();
  }

  com_ /= weightSum;
  comJac_ /= weightSum;
  // Update the CMM frame translation (w.r.t the root link frame).
  cmmtf_.translation() = com();

  comVelocity_.noalias() = comJac_ * dq_;
}

MatrixXd KinematicChain::computeLinkMomentumJacobian(std::string const & linkName, bool bodyFrame) const
{
  auto l = link(linkName);
  std::string const parentJointName = l->parentJointName();
  if(bodyFrame)
  {
    return l->localInertiaTensor() * bodyJacobian(parentJointName);
  }
  else
  {
    Iso3d const & tf = transform(parentJointName);
    return l->spatialInertia(tf) * spatialJacobian(parentJointName);
  }
}

MatrixXd KinematicChain::computeCMMTwo() const
{
  MatrixXd cmm = MatrixXd::Zero(6, dof());

  // CMM frame transform:
  for(auto const & l : links())
  {
    std::string const parentJointName = l->parentJointName();
    MatrixXd const lmJac = l->localInertiaTensor() * bodyJacobian(parentJointName);
    long const cols = lmJac.cols();
    cmm.block(0, 0, 6, cols) += wrenchTransform(parentJointName, "com") * lmJac;
  }
  return cmm;
}
MatrixXd KinematicChain::computeCMM() const
{
  MatrixXd cmm = MatrixXd::Zero(6, dof());

  // CMM frame transform:
  for(auto const & l : links())
  {
    std::string const & parentJointName = l->parentJointName();
    auto jtf = transform(parentJointName);
    // This is the transform from i->cmm frame: g_ci = g_sc ^ (-1) * g_si
    const Iso3d & tfInv = jtf.inverse() * cmmtf_;
    // Momentum transform from the parent joint frame to the CMM frame

    MatrixXd const & lmJac = computeLinkMomentumJacobian(l->name(), true);
    long const cols = lmJac.cols();
    cmm.block(0, 0, 6, cols) += metaAdgTMatrix(tfInv) * lmJac;
  }
  return cmm;
}

MatrixXd KinematicChain::computeCMMDerivative() const
{
  MatrixXd cmmd = MatrixXd::Zero(6, dof());

  for(auto const & l : links())
  {
    std::string const parentJointName = l->parentJointName();

    MatrixXd const jac = bodyJacobian(parentJointName);
    MatrixXd const jacd = computeBodyJacobianDerivative(parentJointName);

    MatrixXd const lmJac = l->localInertiaTensor() * jac;

    MatrixXd const lmJacd = l->localInertiaTensor() * jacd;

    long cols = lmJac.cols();

    cmmd.block(0, 0, 6, cols) +=
        wrenchTransformDerivative(parentJointName, "com") * lmJac + wrenchTransform(parentJointName, "com") * lmJacd;
  }
  return cmmd;
}

Matrix6d KinematicChain::computeCentroidalInertiaTwo() const
{
  Matrix6d centroidalInertia = Matrix6d::Zero();

  std::vector<Matrix6d> ci_vec;
  ci_vec.resize(dof());

  for(int ii = 0; ii < dof(); ii++)
  {
    ci_vec[ii] = link(ii)->localInertiaTensor();
  }

  for(int ii = dof() - 1; ii > 0; ii--)
  {
    ci_vec[ii - 1] += inertiaTransform(ci_vec[ii], link(ii)->parentJointName(), link(ii - 1)->parentJointName());
  }

  centroidalInertia = inertiaTransform(ci_vec[0], link(0)->parentJointName(), "com");
  return centroidalInertia;
}

Matrix6d KinematicChain::computeCentroidalInertia() const
{
  Matrix6d centroidalInertia = Matrix6d::Zero();

  for(auto link : links())
  {
    auto tf = transform(link->parentJointName());
    centroidalInertia += link->spatialInertia(cmmtf_.inverse() * tf);
  }

  return centroidalInertia;
}

Vector3d KinematicChain::computePointVel(std::string const & jointName, Vector3d const & t) const
{
  VectorXd dq;
  readJointVelocities(dq);
  MatrixXd const & bodyJac = bodyJacobian(jointName);
  MatrixXd const & linearJac = bodyJac.block(0, 0, 3, bodyJac.cols());
  MatrixXd const & angularJac = bodyJac.block(3, 0, 3, bodyJac.cols());
  return transform(jointName).rotation() * (-skew(t) * angularJac + linearJac) * dq_.head(bodyJac.cols());
}

Matrix6d KinematicChain::inertiaTransform(Matrix6d const & inertia,
                                          std::string const & currenFrame,
                                          std::string const & targetFrame) const
{
  Matrix6d const & velTF = velTransform(targetFrame, currenFrame);
  return velTF.transpose() * inertia * velTF;
}

Matrix6d KinematicChain::inertiaTransformDerivative(Matrix6d const & inertia,
                                                    std::string const & currenFrame,
                                                    std::string const & targetFrame) const
{
  Matrix6d const & velTF = velTransform(targetFrame, currenFrame);
  Matrix6d const & velTFd = velTransformDerivative(targetFrame, currenFrame);
  return velTFd.transpose() * inertia * velTF + velTF.transpose() * inertia * velTFd;
}

Matrix6d KinematicChain::velTransform(std::string const & a, std::string const & b) const
{
  if(a == b)
  {
    return metaAdgMatrix(Iso3d::Identity());
  }

  Iso3d const & g_ba = transform(b).inverse() * transform(a);

  return metaAdgMatrix(g_ba);
}

Matrix6d KinematicChain::velTransformDerivative(std::string const & a, std::string const & b) const
{

  if(a == b)
  {
    return Matrix6d::Zero();
  }

  Vector6d const & Vb_ba = computeBodyVel(b, a);

  // RoboticsUtils::quickPrint("Vel transform a -> b", velTransform(a, b));
  // RoboticsUtils::quickPrint("Vb_ba", Vb_ba);

  return velTransform(a, b) * crossMatrix(Vb_ba);
}

std::string KinematicChain::randomFrameName() const
{

  int const total_num = dof() + 1 + 1;
  // int const total_num = dof();

  int jIdx = randInt(total_num);

  if(jIdx == dof() + 1)
  {
    return "com";
  }
  else if(jIdx == dof())
  {
    return "inertial";
  }

  return joint(jIdx)->name();
}

Vector6d KinematicChain::inertialFrameBodyVel(std::string const & name) const
{
  if(name == "inertial")
  {
    return Vector6d::Zero();
  }
  int const & type = frameType(name);
  switch(type)
  {
    case 1: // joint
    {
      bool fullSize = false;
      MatrixXd const & Ja = bodyJacobian(name, fullSize);
      return Ja * jointVelocities().head(Ja.cols());
    }
    case 2: // end-effector
    {
      MatrixXd const & Ja = endEffector(name)->bodyJacobian();
      return Ja * jointVelocities().head(Ja.cols());
    }
    case 3: // com
    {
      MatrixXd const & Ja = comBodyJac();
      return Ja * jointVelocities();
    }
    default:
    {
      RoboticsUtils::throw_runtime_error("The requested inertialFrame body velocity: " + name + " is not found",
                                         __FILE__, __LINE__);
    }
  }
}

Vector6d KinematicChain::computeBodyVel(std::string const & referenceFrame, std::string const & bodyFrame) const
{
  // \f$ V^{b}_{ab}  = V^{b}_{Ob} - Adg^{-1}_{a,b}V^{b}_{Oa}\f$

  // (1) Computes  Adg^{-1}_{a,b} or Adg_{b,a}

  Iso3d const & t_b = transform(bodyFrame);
  Iso3d const & t_a = transform(referenceFrame);
  auto Adg_ba = metaAdgInvMatrix(t_a.inverse() * t_b); // Equivalent: Adg^{-1}_{ab}

  // (2) Computes the Jacobians
  // bool fullSize = false;
  // MatrixXd const & Ja = bodyJacobian(referenceFrame, fullSize);
  // Vector6d const & Vb_Oa = Ja * jointVelocities().head(Ja.cols());
  Vector6d const & Vb_Oa = inertialFrameBodyVel(referenceFrame);

  // RoboticsUtils::quickPrint("Joint velocities", jointVelocities());

  // RoboticsUtils::quickPrint("Vb_Oa", Vb_Oa);

  // MatrixXd const & Jb = bodyJacobian(bodyFrame, fullSize);
  // Vector6d const & Vb_Ob = Jb * jointVelocities().head(Jb.cols());
  Vector6d const & Vb_Ob = inertialFrameBodyVel(bodyFrame);

  // RoboticsUtils::quickPrint("Vb_Ob", Vb_Ob);

  // Compute V^{b}_{ab}
  return Vb_Ob - Adg_ba * Vb_Oa;
}

Matrix6d KinematicChain::wrenchTransform(std::string const & a, std::string const & b) const
{
  if(a == b)
  {
    return metaAdgTMatrix(Iso3d::Identity());
  }

  Iso3d const & g_ab = transform(a).inverse() * transform(b);

  return metaAdgTMatrix(g_ab);
}

Matrix6d KinematicChain::wrenchTransformDerivative(std::string const & a, std::string const & b) const
{
  if(a == b)
  {
    return Matrix6d::Zero();
  }

  Iso3d const & g_ab = transform(a).inverse() * transform(b);
  Vector6d const & Vb_ab = computeBodyVel(a, b);

  return crossMatrix(Vb_ab).transpose() * metaAdgTMatrix(g_ab);
}

void KinematicChain::MCalcCRB(MatrixXd & M)
{
  auto parentLink = [](int i) -> int { return i - 1; };
  //
  M = MatrixXd::Zero(dof(), dof());
  std::vector<Matrix6d> crb_links;

  for(int i = 0; i < dof(); i++)
  { // 1. Initialize the CRB inertia for all the links
    crb_links.push_back(link(i)->localInertiaTensor());
  }

  for(int i = (dof() - 1); i >= 0; i--)
  { // 2. Loop from the leaf to the root

    int pi = parentLink(i);
    if(pi >= 0) // If parent link is not empty
    {
      auto piFrameName = link(pi)->parentJointName();
      auto iFrameName = link(i)->parentJointName();
      crb_links[static_cast<size_t>(pi)] += inertiaTransform(
          crb_links[static_cast<size_t>(i)], iFrameName, piFrameName); // Add the child link CRB inertia to the parent
    }
    // Momentum type of variable
    auto si = joint(link(i)->parentJointName())->bodyTC();
    Vector6d h = crb_links[static_cast<size_t>(i)] * si;

    M(i, i) = si.transpose() * h; //

    int j = i;

    while(j > 0)
    {
      // Inner loop to root.
      int pj = parentLink(j);

      h = wrenchTransform(link(j)->parentJointName(), link(pj)->parentJointName()) * h;
      j = pj;
      auto sj = joint(link(j)->parentJointName())->bodyTC();
      M(i, j) = h.transpose() * sj;
      M(j, i) = M(i, j);
    }

  } // End of the loop from leaf to root

} // End of CRB algorithm

void KinematicChain::CoriolisAndCentrifugalCalc(VectorXd & cc)
{

  cc = VectorXd::Zero(dof());

  // Apply the ready-made link velocities.
  std::vector<Vector6d> linkAccs, linkForces;
  size_t chain_size = static_cast<size_t>(dof());
  linkAccs.resize(chain_size);
  linkForces.resize(chain_size);

  auto parentJoint = [](int i) -> int { return i - 1; };

  for(int i = 0; i < dof(); i++)
  { // Loop from root to leaf
    size_t ti = static_cast<size_t>(i);
    std::shared_ptr<GeoJoint> & j = joints_[ti];
    std::shared_ptr<GeoLink> const & l = j->childLink();

    int plIdx = parentJoint(i);
    // Vector6d link_vel = j->jointVelocity();
    Vector6d link_vel = l->bodyVelocity();
    Matrix6d velP2b;

    Vector6d link_acc = cross(link_vel, j->jointVelocity());

    if(plIdx >= 0)
    {
      // There is parent joint/link
      size_t t_plIdx = static_cast<size_t>(plIdx);
      // std::shared_ptr<GeoLink> & pl = links_[t_plIdx];
      std::shared_ptr<GeoJoint> & pj = joints_[t_plIdx];
      // velP2b = metaAdgInvMatrix(pj->relativeTransform());
      // velP2b = velTransform(pj->name(), j->name());
      velP2b = metaAdgInvMatrix(pj->transform().inverse() * j->transform());
      link_acc += velP2b * linkAccs[t_plIdx];
    }

    linkAccs[ti] = link_acc;
    linkForces[ti] =
        l->localInertiaTensor() * link_acc + dualCross(l->bodyVelocity(), l->localInertiaTensor() * l->bodyVelocity());
    ;

  } // End of the root to leaf loop

  for(int i = (dof() - 1); i >= 0; i--)
  { // Loop from leaf to root
    size_t ti = static_cast<size_t>(i);
    std::shared_ptr<GeoJoint> & j = joints_[ti];
    // std::shared_ptr<GeoLink> & l = links_[ti];

    cc(i) = j->bodyTC().transpose() * linkForces[ti];

    int plIdx = parentJoint(i);
    if(plIdx >= 0)
    {

      size_t t_plIdx = static_cast<size_t>(plIdx);
      // std::shared_ptr<GeoLink> & pl = links_[t_plIdx];
      std::shared_ptr<GeoJoint> & pj = joints_[t_plIdx];

      Matrix6d const & wrenchP2b = metaAdgTMatrix(j->transform().inverse() * pj->transform());
      // const Matrix6d & wrenchP2b = metaAdgTMatrix(pj->relativeTransform());
      // const Matrix6d & wrenchP2b = wrenchTransform(j->name(), pj->name());
      linkForces[t_plIdx] += wrenchP2b * linkForces[ti];
    }
  } // End of the torque loop
}

void KinematicChain::IdCalc(VectorXd const & q, VectorXd const & dq, VectorXd const & ddq)
{ // Adopt the body-coordinate computation

  // The implementation follows Table 5.1 of Featherstone's book.

  auto parentJoint = [](int i) -> int { return i - 1; };

  Vector6d iniAcc = Vector6d::Zero();
  iniAcc(2) = -GRAVITY;

  Iso3d tf = Iso3d::Identity();

  for(int i = 0; i < dof(); i++)
  { // Loop from root to leaf
    size_t ti = static_cast<size_t>(i);
    std::shared_ptr<GeoJoint> & j = joints_[ti];
    std::shared_ptr<GeoLink> const & l = j->childLink();

    j->update(q(i));
    j->updateDq(dq(i));
    j->updateDDq(ddq(i));

    // 1. Joint calculation
    tf = tf * j->incrementalTransform(); // The incremental transform has been updated when 'update(q)' was called.
    j->setTransform(tf * j->initialTransform()); // Set the transform g_Oi

    // Body velocity
    // Iso3d btf= j->incrementalTransform() * j->initialTransform();
    // const Vector6d & Si = metaAdgInv(btf, j->twistCoordinate()).vector();
    // j->setBodyTC(Si);
    j->setJointVel(j->bodyTC() * j->dq());

    // 2. Set the link velocity and acceleration
    int plIdx = parentJoint(i);
    size_t t_plIdx = static_cast<size_t>(plIdx);
    Vector6d link_vel = j->jointVelocity();

    Matrix6d velP2b;
    if(plIdx >= 0)
    {
      // There is parent joint/link
      std::shared_ptr<GeoLink> & pl = links_[t_plIdx];
      std::shared_ptr<GeoJoint> & pj = joints_[t_plIdx];
      velP2b = metaAdgInvMatrix(pj->transform().inverse() * j->transform());
      // velP2b = velTransform(pj->name(), j->name());
      link_vel += velP2b * pl->bodyVelocity();
    }
    l->setBodyVel(link_vel);

    Vector6d const & joint_acc = j->bodyTC() * j->ddq();
    Vector6d link_acc = joint_acc + cross(link_vel, j->jointVelocity());
    if(plIdx >= 0)
    {
      // There is parent joint/link
      std::shared_ptr<GeoLink> & pl = links_[t_plIdx];
      link_acc += velP2b * pl->bodyAcceleration();
    }
    if(plIdx == -1)
    { // Only applies to the root link. It should propagates to the leaf.
      link_acc += iniAcc;
    }

    l->setBodyAcc(link_acc);

    l->calcWrench();
  } // End of the loop from root to leaf

  for(int i = (dof() - 1); i >= 0; i--)
  { // Loop from leaf to root
    size_t ti = static_cast<size_t>(i);
    std::shared_ptr<GeoJoint> & j = joints_[ti];
    std::shared_ptr<GeoLink> & l = links_[ti];

    double tau = j->bodyTC().transpose() * l->wrench();
    // RoboticsUtils::quickPrint("tau: " + std::to_string(i), tau);
    j->updateTau(tau);

    int plIdx = parentJoint(i);
    if(plIdx >= 0)
    {

      size_t t_plIdx = static_cast<size_t>(plIdx);
      std::shared_ptr<GeoLink> & pl = links_[t_plIdx];
      std::shared_ptr<GeoJoint> & pj = joints_[t_plIdx];

      Matrix6d const & wrench2P = metaAdgTMatrix(j->transform().inverse() * pj->transform());
      // const Matrix6d & wrenchP2b = wrenchTransform(j->name(), pj->name());
      pl->setWrench(pl->wrench() + wrench2P * l->wrench());
    }
  } // End of the torque loop
}

EndEffector::EndEffector(std::string const & name,
                         std::string const & parentJointName,
                         Iso3d const & tf,
                         std::shared_ptr<KinematicChain> const robot)
: name_(name), jointName_(parentJointName), relativeTf_(tf), robot_(robot)
{

  dof_ = robot->spatialJacobian(parentJointName).cols();

  jac_.resize(6, dof());
  bodyJac_.resize(6, dof());

  twistTransform_ = metaAdgInvMatrix(tf);
  update();

  RoboticsUtils::quickInfo("Created end-effector: " + name_ + " with parent joint: " + jointName_);
}

void EndEffector::update()
{

  tf_ = robot()->transform(jointName_) * relativeTransform();

  jac_.noalias() = robot()->spatialJacobian(jointName_);

  bodyJac_.noalias() = twistTransform_ * robot()->bodyJacobian(jointName_);

  Vector6d const & bodyVel = bodyJacobian() * robot()->jointVelocities().head(dof());

  pVelBody_ = bodyVel.head(3);
  pVel_ = tf_.rotation() * pVelBody_;

  aVelBody_ = bodyVel.tail(3);
  aVel_ = tf_.rotation() * bodyVel.tail(3);
}

std::shared_ptr<GeoJoint> const & EndEffector::parentJoint() const
{
  return robot_->joint(jointName_);
}

std::shared_ptr<GeoLink> const & EndEffector::parentLink() const
{
  return robot_->joint(jointName_)->childLink();
}
} // namespace GeoRobotics
