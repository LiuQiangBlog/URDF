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
#include "GeometricRobotics/Kinematics/KinematicTree.h"

#include <RoboticsUtils/utils.h>
#include <memory>

namespace GeoRobotics
{

ComFrame::ComFrame(std::string const & frameName, std::shared_ptr<KinematicTree> const tree)
: KinematicTreeObserver(frameName), GeoFrame(frameName, tree)
{
  inertialTransform_.linear() = Matrix3d::Identity();
  inertialTransform_.translation() = robot()->com();

  RoboticsUtils::quickInfo("Created ComFrame: " + frameName);
}
void ComFrame::updateKTreeObserver()
{
  // Inherited from KinematicTreeObserver

  inertialTransform_.translation() = robot()->com();
  // inertialTransform_ = computeTransform();
}

MatrixXd ComFrame::computeBodyJacobian(bool const & fullJacobian) const
{
  // Inherited from GeoFrame
  MatrixXd jac = MatrixXd::Zero(6, robot()->dof());
  jac.topRows(3) = robot()->computeComJac();

  return jac;
}

Vector6d const ComFrame::inertialFrameBodyVel()
{
  return computeBodyJacobian(true) * robot()->jointVelocities();
}

MatrixXd FixedFrame::computeBodyJacobian(bool const & fullJacobian) const
{
  return MatrixXd::Zero(robot()->dof(), robot()->dof());
}

JointDependentFrame::JointDependentFrame(std::string const & name,
                                         std::string const & parentGeoJoint,
                                         Iso3d const & offset,
                                         std::shared_ptr<KinematicTree> const tree)
: JointStateObserver(name), GeoFrame(name, tree), parentJointName_(parentGeoJoint), offset_(offset)
{
  // Check if the parent frame already exists
  if(robot()->hasFrame(name))
  {
    // Set the set of articulated joints.
    RoboticsUtils::THROW_RUNTIME_ERROR("The frame: " + name + " already exists!");
  }

  if(!robot()->hasArticulatedJoint(parentGeoJoint))
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("The desired articulated joint: " + parentGeoJoint + " does not exists!");
  }

  twistTransform_ = metaAdgInvMatrix(offset_);

  auto pj = robot()->joint(parentJointName_);
  dof_ = 6 + static_cast<int>(pj->articulatedJoints().size());

  // auto l = robot()->joint(parentGeoJoint)->childLink();

  // Go through the dependent joints from root to leaf
  // for(auto l : robot()->graph()->odLinks(l->name()))
  // {
  //   // The link and joint has the same index.
  //   if(robot()->joint(l)->dof() == 1)
  //   {
  //     dependentJoints_.push_back(l);
  //   }
  // } // End of for loop
  // std::shared_ptr<JointStateObserver> jsObserverPtr = shared_from_this();
  // Attach to the parent joint

  // std::shared_ptr<JointStateObserver> jsObserverPtr = std::static_pointer_cast<JointStateObserver>(this);
  // robot->joint(parentGeoJoint)->attachObserver(jsObserverPtr);
  RoboticsUtils::quickInfo("Created JointDependentFrame: " + frameName() + " with parent joint: " + parentJointName_);
}

MatrixXd JointDependentFrame::computePointJacobian(Vector3d const & p, bool const fullSize) const
{
  MatrixXd const & bodyJac = computeBodyJacobian(fullSize);

  // RoboticsUtils::quickPrint("frame bodyJac", bodyJac);
  MatrixXd const & linearJac = bodyJac.block(0, 0, 3, bodyJac.cols());
  MatrixXd const & angularJac = bodyJac.block(3, 0, 3, bodyJac.cols());

  MatrixXd comJac;
  comJac.resize(3, bodyJac.cols());

  Iso3d const & tf = computeTransform();

  // RoboticsUtils::quickPrint("frame tf", tf.matrix());
  comJac.noalias() = tf.rotation() * (-skew(p) * angularJac + linearJac);
  // RoboticsUtils::quickPrint("com jac", comJac);

  return comJac;
}

Vector3d JointDependentFrame::computePointVelocity(Vector3d const & p) const
{
  VectorXd dq;
  robot()->readJointVelocities(dq, parentJointName_);

  // RoboticsUtils::quickPrint("dq", dq);

  bool fullSize = false;
  return computePointJacobian(p, fullSize) * dq;
}

Iso3d JointDependentFrame::computeTransform() const
{
  return robot()->joint(parentJointName_)->transform() * offset_;
}

MatrixXd JointDependentFrame::computeBodyJacobian(const bool & fullJacobian) const
{
  return twistTransform_ * robot()->computeBodyJacobian(parentJointName_, fullJacobian);
}

Vector6d const JointDependentFrame::inertialFrameBodyVel()
{
  bool fullSize = false;
  MatrixXd const & Ja = computeBodyJacobian(fullSize);
  // robot->joint(parentJointName_)->articulatedJoints()
  VectorXd dq;
  robot()->readJointVelocities(dq, parentJointName_);
  return Ja * dq;

  // bool fullSize = true;
  // MatrixXd const & Ja = computeBodyJacobian(fullSize);
  // return Ja * robot()->jointVelocities();
}

void JointDependentFrame::updateObserver()
{
  // RoboticsUtils::quickInfo("Updating JointDependentFrame's observer");
  inertialTransform_ = computeTransform();
}
Vector6d KinematicTree::computeBodyVel(std::string const & referenceFrame, std::string const & bodyFrame) const
{
  // \f$ V^{b}_{ab}  = V^{b}_{Ob} - Adg^{-1}_{a,b}V^{b}_{Oa}\f$

  // (1) Computes  Adg^{-1}_{a,b} or Adg_{b,a}

  auto rf = frame(referenceFrame);
  auto bf = frame(bodyFrame);

  Iso3d const & t_b = bf->transform();
  Iso3d const & t_a = rf->transform();
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

Vector6d KinematicTree::inertialFrameBodyVel(std::string const & name) const
{
  return frame(name)->inertialFrameBodyVel();
}

MatrixXd KinematicTree::computeBodyJacobianDerivative(std::string const & jointName, bool const & fullJacobian) const
{
  if(!hasArticulatedJoint(jointName))
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("The robot " + name() + " does not have the articulated joint: " + jointName);
  }

  auto j = joint(jointName);
  MatrixXd jacd;

  if(fullJacobian)
  {
    computeFullSizeBodyJacobianDerivative_(jointName, jacd);
    // RoboticsUtils::throw_runtime_error("Full-size Jacobian derivative is not implemented yet", __FILE__, __LINE__);
  }
  else
  {
    computeBodyJacobianDerivative_(jointName, jacd);
  }

  return jacd;
}
MatrixXd KinematicTree::computeBodyJacobian(const std::string & jointName, const bool & fullJacobian) const
{
  if(!hasArticulatedJoint(jointName))
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("The robot " + name() + " does not have the articulated joint: " + jointName);
  }

  auto j = joint(jointName);
  MatrixXd jac;

  computeBodyJacobian_(jointName, jac, fullJacobian);

  // Iso3d accumulated_tf = initialTransform(jointIndexToName(idx));

  // for(size_t ii = 0; ii < local_dof; ii++)
  //{
  //  // size_t jj = joints_.size() - ii - 1;
  //  size_t jj = local_dof - ii - 1;

  //  accumulated_tf = incrementalTransform(jointIndexToName(jj)) * accumulated_tf;

  //  output.col(static_cast<int>(jj)) = metaAdgInv(accumulated_tf, tcs_[jj]).vector();
  //}
  return jac;
}

void KinematicTree::computeFullSizeBodyJacobianDerivative_(const std::string & jointName, MatrixXd & jacd) const
{
  MatrixXd compact_jacd;
  computeBodyJacobianDerivative_(jointName, compact_jacd);

  jacd = MatrixXd::Zero(6, dof());

  auto j = joint(jointName);
  std::deque<int> jSet;
  odArticulatedJoints(jSet, j->childLink()->name());

  int startIdx = 0;
  int aDof = static_cast<int>(jSet.size());

  if(isFloatingBase())
  {
    aDof += 6;
    startIdx = 6;
  }

  if(isFloatingBase())
  {
    jacd.leftCols(6).noalias() = compact_jacd.leftCols(6);
  }

  size_t jj = 0;
  for(int ii = startIdx; ii < aDof; ii++)
  {
    int jMatrixIdx = joint(jSet[jj])->inertiaMatrixJointIndex();
    jacd.col(jMatrixIdx) = compact_jacd.col(ii);
    jj++;
  }
}
void KinematicTree::computeBodyJacobianDerivative_(const std::string & jointName, MatrixXd & jacd) const
{

  // VectorXd dq;
  // readJointVelocities(dq);

  MatrixXd jac;
  bool fullSize = false;
  computeBodyJacobian_(jointName, jac, fullSize);

  // RoboticsUtils::quickPrint("jacd bodyJac", jac);
  VectorXd local_dq;

  auto j = joint(jointName);
  // int aDof = static_cast<int>(j->articulatedJoints().size()) + 6;
  std::deque<int> jSet;
  odArticulatedJoints(jSet, j->childLink()->name());
  int ini_aDof = static_cast<int>(jSet.size());
  local_dq.resize(ini_aDof);

  // Read the dependent joint velocities.
  for(int ii = 0; ii < ini_aDof; ii++)
  {
    local_dq(ii) = joint(jSet[static_cast<size_t>(ii)])->dq();
  }

  // RoboticsUtils::quickInfo("There are ", ini_aDof, " dependent-actuated joints");
  // RoboticsUtils::quickPrint("Local dq ", local_dq);

  // int aDof = 0;
  size_t startIdx = 0;

  if(isFloatingBase())
  {
    // aDof = ini_aDof + 6;
    startIdx = 6;
  }
  // else
  //{
  //  aDof = ini_aDof;
  //}

  jacd = MatrixXd::Zero(6, jac.cols());

  if(isFloatingBase())
  {
    // How to compute the derivative?
    // const Matrix6d & w_tf = wrenchTransform(j->name(), joint(0)->name());

    Matrix6d const v_tf = velTransform(joint(0)->name(), j->name());
    // const Matrix6d & v_tf_inv = velTransform(j->name(), joint(0)->name());
    Vector6d const vel_bp = jac.rightCols(ini_aDof) * local_dq;
    Vector6d const vel_pb = -v_tf.colPivHouseholderQr().solve(vel_bp);
    jacd.leftCols(6).noalias() = v_tf * crossMatrix(vel_pb);
    // jacd.leftCols(6).noalias() = - crossMatrix(vel_bp);
  }

  //// RoboticsUtils::quickPrint("temp Jac: ", jac);

  // The number of non-zero coloumns
  size_t cols = jSet.size();

  if(cols + startIdx > static_cast<size_t>(jac.cols()))
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("Found more dofs than Jac columns");
  }
  // Loop through the cols
  for(size_t ii = startIdx; ii < startIdx + cols; ii++)
  {
    // output.col(static_cast<int>(ii)) = tc_prime_dot_(jacobian, ii);
    // int idx = joint(jSet[ii])->inertiaMatrixJointIndex();
    Vector6d tc_dot;
    // RoboticsUtils::quickInfo("Processing column: ", ii);
    tc_dagger_dot_(jac, ii, startIdx, cols, local_dq, tc_dot);
    jacd.col(static_cast<int>(ii)) = tc_dot;
  }
}

void KinematicTree::tc_dagger_dot_(const MatrixXd & jacobian,
                                   const size_t & ii,
                                   const size_t & startIdx,
                                   const size_t & localDof,
                                   const VectorXd & dq,
                                   Vector6d & output) const
{
  MatrixXd matrix = MatrixXd::Zero(6, static_cast<int>(localDof));

  // const VectorXd & tc = jacobian.col(static_cast<int>(jSet[ii]));
  const VectorXd & tc = jacobian.col(static_cast<int>(ii));
  // RoboticsUtils::quickPrint("tc", tc);
  // for(size_t j = idx + 1; j < startIdx + localDof; j++)
  //{
  //  RoboticsUtils::quickPrint("cross", jacobian.col(j));
  //  matrix.col(j - startIdx) = cross(tc, jacobian.col(j));
  //}
  // for(size_t j = jSet[ii + 1]; j < startIdx + localDof; j++)
  if(ii + 1 >= startIdx + localDof)
  {
    output.setZero();
    return;
  }
  for(int j = static_cast<int>(ii + 1); j < static_cast<int>(startIdx + localDof); j++)
  {
    // int new_idx = joint(jSet[j])->inertiaMatrixJointIndex();
    // RoboticsUtils::quickPrint("cross", jacobian.col(j));
    matrix.col(j - startIdx) = cross(tc, jacobian.col(j));
  }

  // RoboticsUtils::quickPrint("matrix ", matrix);
  // RoboticsUtils::quickPrint("local dq", dq.segment(0, localDof));
  // output = matrix * dq.segment(0, static_cast<int>(localDof));
  // output = matrix * dq.head(static_cast<int>(localDof));
  output = matrix * dq;
}

MatrixXd KinematicTree::computeBodyJacobianDerivative(const std::string & jointName, const std::string deltaJoint) const
{
  // In the frame of idx
  const MatrixXd & jac = computeBodyJacobian(jointName, false);

  int aSize = static_cast<int>(jac.cols());
  int aDof = aSize;
  int startIdx = 0;
  if(isFloatingBase())
  {
    startIdx = 6;
  }

  MatrixXd output = MatrixXd::Zero(6, aDof);

  auto j = joint(jointName);
  std::deque<int> jSet;
  odArticulatedJoints(jSet, j->childLink()->name());
  // printOdArticulatedJoints(j->childLink()->name());

  auto jIterator = std::find(jSet.begin(), jSet.end(), jointNameToIndex(deltaJoint));
  if(jIterator == jSet.end())
  {
    return output;
  }

  // Find the inertia-matrix index:
  // int iIdx = j->inertiaMatrixJointIndex();
  // int jIdx = joint(deltaJoint) ->inertiaMatrixJointIndex();
  // Iterate over the axes
  // for(int ii = 0; ii<jac.cols(); ii++)
  // RoboticsUtils::quickError("jSet size: ", jSet.size());
  // RoboticsUtils::quickError("index: ", jIterator - jSet.begin());
  int delta_idx = static_cast<int>(startIdx + jIterator - jSet.begin());
  for(int ii = startIdx; ii < delta_idx; ii++)
  {
    output.col(ii).noalias() = cross(jac.col(ii), jac.col(delta_idx));
  }
  return output;
}

void KinematicTree::computeBodyJacobian_(std::string const & jointName, MatrixXd & jac, bool const fullSize) const
{

  auto j = joint(jointName);
  // int aDof = static_cast<int>(j->articulatedJoints().size()) + 6;
  std::deque<int> jSet;
  odArticulatedJoints(jSet, j->childLink()->name());

  int aDof = static_cast<int>(jSet.size());
  int startIdx = 0;

  if(isFloatingBase())
  {
    aDof += 6;
    startIdx = 6;
  }

  if(fullSize)
  {
    jac = MatrixXd::Zero(6, dof());
  }
  else
  {
    jac = MatrixXd::Zero(6, aDof);
  }

  if(isFloatingBase())
  {
    // const Matrix6d & w_tf = wrenchTransform(j->name(), joint(0)->name());
    // jac.leftCols(6).noalias() = w_tf.transpose();
    const Matrix6d & v_tf = velTransform(joint(0)->name(), j->name());
    jac.leftCols(6).noalias() = v_tf;
  }

  // RoboticsUtils::quickPrint("temp Jac: ", jac);

  Iso3d accumulated_tf = j->initialTransform();

  for(int ii = startIdx; ii < aDof; ii++)
  {
    // RoboticsUtils::quickHL("ii: ", ii, " aDof: ", aDof);
    // Go through the dependent links
    // graph()->articulatedJoints()[ii]
    // int jIdx = j->jointIndex();
    size_t newIdx = static_cast<size_t>(aDof - ii - 1);
    auto jj = joint(jSet[newIdx]);
    accumulated_tf = jj->incrementalTransform() * accumulated_tf;

    int jMatrixIdx = static_cast<int>(startIdx + newIdx);
    if(fullSize)
    {
      jMatrixIdx = jj->inertiaMatrixJointIndex();
    }

    jac.col(jMatrixIdx) = metaAdgInv(accumulated_tf, jj->twistCoordinate()).vector();
    // RoboticsUtils::quickPrint("new col: ", jac.col(ii));
  }
}

void SpanningTree::createGeoLink(const std::shared_ptr<Link> newLink,
                                 const int & idx,
                                 std::vector<std::shared_ptr<GeoLink>> & links)
{
  // auto currentLink = model->getLink(newNode);
  // RoboticsUtils::quickInfo("Creating GeoLink: ", newLink->name, " with idx: ", idx);
  // links[static_cast<size_t>(idx)] = std::make_shared<GeoLink>(newLink);
  links[static_cast<size_t>(idx)] = GeoLink::createGeoLink(newLink);
}

void SpanningTree::createGeoJoint(const std::vector<std::shared_ptr<GeoLink>> & links,
                                  const int & idx,
                                  const std::shared_ptr<Joint> urdfJoint,
                                  std::vector<std::shared_ptr<GeoJoint>> & joints,
                                  std::vector<std::pair<int, int>> & articulatedJoints,
                                  std::vector<std::pair<int, int>> & temp_articulatedJoints,
                                  Iso3d & iniTF)
{

  int parentLinkIdx = nodes.find(urdfJoint->parentLinkName)->second;
  auto parentGeoLink = links[static_cast<size_t>(parentLinkIdx)];
  auto childGeoLink = links[static_cast<size_t>(idx)];

  // joints[static_cast<size_t>(idx)] =
  //     std::make_shared<GeoJoint>(urdfJoint, childGeoLink, parentGeoLink, articulatedJoints, iniTF);
  joints[static_cast<size_t>(idx)] =
      GeoJoint::createGeoJoint(urdfJoint, childGeoLink, parentGeoLink, temp_articulatedJoints, iniTF);

  auto j = joints[static_cast<size_t>(idx)];
  auto j0 = joints[0];

  if(j->dof() == 1) // If this is an articulated joint
  {

    // j->setTwistCoordinate(j->computeTwistCoordinate(iniTF)); // This is already done in the constructor.
    // articulatedJoints.push_back(idx);

    size_t addOn{0};

    if(j0->jointType() == JointType::FLOATING)
    {
      addOn += 6;
    }
    auto new_idx_pair = std::make_pair(idx, static_cast<int>(articulatedJoints.size() + addOn));
    articulatedJoints.push_back(new_idx_pair);
    temp_articulatedJoints.push_back(new_idx_pair);
    j->setIndices(articulatedJoints.back());
  }
  else
  {
    j->setIndices(std::make_pair(idx, -1));
  }
}

void Graph::DFSVisit_(const std::shared_ptr<Model> & model,
                      const std::string & currentNode,
                      std::vector<int> & odLinkVec,
                      Iso3d & iniTF,
                      int & idx,
                      SpanningTree & st,
                      std::vector<std::pair<int, int>> & temp_articulatedJoints)
{

  // RoboticsUtils::quickInfo("Handling link: ", currentNode, " with idx: ", idx);
  // 0. Push the current node index into the stack
  odLinkVec.push_back(idx);

  // 1. Push the currentNode and step into the spanning tree
  // auto lastLink = model->getLink(currentNode);

  // RoboticsUtils::quickInfo("Creating GeoLink: ", currentLink->name, " with idx: ",  idx);
  // links_[static_cast<size_t>(idx)] = std::make_shared<GeoLink>(currentLink);

  st.insertNode(currentNode, idx);

  // 1.1 Accumulate the initial transforms
  // auto currentLink = model->getLink(currentNode);
  // auto currentTF = currentLink->parentJoint()->transform.toMatrix();
  // iniTF = iniTF * currentTF;

  RoboticsUtils::quickError("Started Checking link: ", currentNode);
  // 2. Check the adjacent nodes of the currentNode
  for(auto const & step : adj_[currentNode])
  {
    std::string const & nextNode = step.first;
    // If not yet in the spanning tree
    if(!st.hasNode(nextNode))
    {
      // Insert the edge and the node
      sTree_.insertEdge(step, idx, model->getLink(step.first), model->getJoint(step.second), joints_,
                        articulatedJoints_, temp_articulatedJoints, links_, iniTF);
      DFSVisit_(model, nextNode, odLinkVec, iniTF, idx, st, temp_articulatedJoints);
    }
  }

  RoboticsUtils::quickError("Finished Checking link: ", currentNode);
  // 3. Write the dependent links and pop
  // Visited all the child node, if there is any.
  orderedDependentLinks_[currentNode] = odLinkVec;
  // 3.3 The transforms
  odLinkVec.pop_back();
  RoboticsUtils::quickInfo("Pop the odlinkvec with the size of: ", odLinkVec.size());
  // 3.4 Reverse the initial transforms, set of dependent links, joints,
  auto currentLink = model->getLink(currentNode);
  auto pj = currentLink->parentJoint();
  auto currentTF = pj->transform.toMatrix();
  iniTF = iniTF * currentTF.inverse();

  if(pj->type != JointType::FIXED)
  {
    RoboticsUtils::quickInfo("Pop the temp_articulatedJoints with the size of: ", temp_articulatedJoints.size());
    temp_articulatedJoints.pop_back();
  }
}

void Graph::initializeContainers_(const std::string & rootNode,
                                  const std::shared_ptr<Model> & model,
                                  const bool & floatingBase)
{
  // Initialize the rootNode link
  // links_[0] = std::make_shared<GeoLink>(model->getLink(rootNode));
  links_[0] = GeoLink::createGeoLink(model->getLink(rootNode));
  // Fill the parent joint of the root link:
  auto childGeoLink = links_[0];
  // joints_[0] = std::make_shared<GeoJoint>(childGeoLink, floatingBase);
  joints_[0] = GeoJoint::createGeoJoint(childGeoLink, floatingBase);
  links_[0]->urdfLink()->setParentJoint(joints_[0]->getURDFJoint());
  sTree_.edges.insert(std::make_pair(joints_[0]->name(), 0));
}
void Graph::makeTree(std::string const & rootNode, std::shared_ptr<Model> const & model, bool const & floatingBase)
{

  links_.resize(model->getLinks().size());
  joints_.resize(model->getJoints().size() + 1);

  sTree_.reset(rootNode);
  orderedDependentLinks_.clear();

  // The dependent links ordered from the root link.
  std::vector<int> odLinkVec;
  Iso3d iniTF;
  iniTF.setIdentity();
  odLinkVec.push_back(0); // Initialize with the root link index.

  initializeContainers_(rootNode, model, floatingBase);

  std::vector<std::pair<int, int>> temp_articulatedJoints; // Evaluate the "articulatedJoints" of each GeoJoint

  orderedDependentLinks_[rootNode] = odLinkVec;

  // orderedDependentLinks_[rootNode] = {};
  int nodeIdx = 1; // The root node has idx 0;

  for(auto const & step : adj_[rootNode])
  {

    std::string const & nextNode = step.first;
    if(!sTree_.hasNode(nextNode)) // Next node is not visited
    {
      // Insert the edge and the node
      sTree_.insertEdge(step, nodeIdx, model->getLink(step.first), model->getJoint(step.second), joints_,
                        articulatedJoints_, temp_articulatedJoints, links_, iniTF);
      DFSVisit_(model, nextNode, odLinkVec, iniTF, nodeIdx, sTree_, temp_articulatedJoints);
    }
    // DFSVisit_(n.first, nodeIdx, sTree_);
    // subgraphIdx++;
  }
}

Graph::Graph(const std::shared_ptr<Model> & model, const std::string & rootLinkName, const bool & floatingBase)
{
  // RoboticsUtils::quickHL("Graph constructor");
  RoboticsUtils::quickHL("The URDF model joint size is: ", model->getJoints().size());

  for(auto & joint : model->getJoints())
  {
    // std::cout << "Adding joint: " << joint.first << std::endl;
    addEdge(joint.second);
  }
  makeTree(rootLinkName, model, floatingBase);

  // spanningTree().print();

  dof_ = static_cast<int>(articulatedJoints().size());

  if(floatingBase)
  {
    dof_ += 6;
  }

  RoboticsUtils::quickHL(" The robot ", model->name(), " has ", articulatedJoints_.size(),
                         " articulated joints, and DOF: ", dof());
}

void Graph::addEdge(std::shared_ptr<Joint> const & joint)
{
  addEdge(joint->childLinkName, joint->parentLinkName, joint->name);
}

void Graph::addEdge(std::string const & nameOne, std::string const & nameTwo, std::string const & edgeName)
{
  adj_[nameOne].push_back({nameTwo, edgeName});
  adj_[nameTwo].push_back({nameOne, edgeName});
}

void Graph::addEdge(Link const & one, Link const & two, Joint const & joint)
{
  addEdge(one.name, two.name, joint.name);
}

bool Graph::hasNode_(std::string const & name) const
{

  if(adj_.find(name) == adj_.end())
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool Graph::findPath(std::string const & start, std::string const & goal, kPath & path)
{
  if(!hasNode_(start))
  {
    return false;
  }
  if(!hasNode_(goal))
  {
    return false;
  }

  if(start == goal) // returns an empty path
  {
    return true;
  }

  visited_.clear();

  for(auto pair : adj_)
  {
    visited_[pair.first] = false;
  }

  if(!adj_[start].empty())
  {
    visited_[start] = true;
    // DFS_(adj_[start].front(), goal, path);
    for(auto & j : adj_[start])
    {
      // std::cout<<"to Link: "<< j.first << ", via joint: "<< j.second <<std::endl;
      // If the node is not visited
      if(visited_[j.first] == false)
      {
        if(DFS_(j, goal, path))
        {
          return true;
        }
      }
    }
  }
  else
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("The starting node: " + start + "does not have an edge.");
  }

  if(path.back().first == goal)
  {
    return true;
  }

  return false;
}

bool Graph::DFS_(const kStep & thisStep, const std::string & goal, kPath & path)
{
  // RoboticsUtils::quickInfo("Visiting: " + start);
  // RoboticsUtils::quickInfo("goal: " + goal);
  // printPath(path);

  /*
  if(visited_[thisStep.first] == true)
  {
    RoboticsUtils::quickError(thisStep.first + " has been visited.");
    return false;
  }
  */

  path.push_back(thisStep);

  if(thisStep.first == goal)
  {

    // RoboticsUtils::quickHL("Found");
    return true;
  }

  visited_[thisStep.first] = true;

  // If it has adjacent nodes
  if(!adj_[thisStep.first].empty())
  {
    // std::cout<<"Link: "<< thisStep.first << " has "<< adj_[thisStep.first].size() << " non-empty edges"<<std::endl;
    // Loop over all the adjacent nodes
    for(auto & j : adj_[thisStep.first])
    {
      // std::cout<<"to Link: "<< j.first << ", via joint: "<< j.second <<std::endl;
      // If the node is not visited
      if(visited_[j.first] == false)
      {
        if(DFS_(j, goal, path))
        {
          return true;
        }
      }
    }
  }

  // If backtracking is taking place
  // std::cout<<RoboticsUtils::info<<"Pop back: "<<"link: "<< path.back().first<< ", joint: " <<path.back().second
  // <<RoboticsUtils::reset<<std::endl;
  path.pop_back();
  return false;
}

void Graph::printGraph() const
{
  RoboticsUtils::quickInfo("The undirected graph has the edges:");

  for(auto & link : adj_)
  {
    std::cout << RoboticsUtils::info << "Link: " << link.first << ", connects to: " << RoboticsUtils::reset
              << std::endl;
    for(auto & edge : link.second)
    {
      std::cout << "link: " << edge.first << " through joint: " << edge.second << std::endl;
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;
}
void Graph::printPath(const kPath & path) const
{
  RoboticsUtils::quickInfo("Found the path:");

  if(path.size() == 0)
  {
    RoboticsUtils::quickError("The path is empty!");
    return;
  }

  for(auto id = path.begin(); id != path.end(); id++)
  {
    std::cout << "Link: " << RoboticsUtils::info << id->first << ", joint: " << RoboticsUtils::hlight << id->second
              << " --> " << RoboticsUtils::reset;
  }

  std::cout << std::endl;
}

KinematicTree::KinematicTree(std::shared_ptr<Model> const modelPtr,
                             std::string const & rootLinkName,
                             std::string const & treeName,
                             bool const & floatingBase)
: modelPtr_(modelPtr), name_(treeName), floatingBase_(floatingBase)
{

  RoboticsUtils::quickError("Start the floating-base KinematicTree constructor of : " + treeName);

  kGraph_.reset(new Graph(model(), rootLinkName, floatingBase));

  // RoboticsUtils::quickError(" The articulatedJoints indices are: ");
  // for(auto ii : graph()->articulatedJoints())
  // {
  //   RoboticsUtils::quickHL("joint name: ", joint(ii.first)->name(), " idx-1: ", ii.first, ", idx-2: ", ii.second);
  // }

  q_ = VectorXd::Zero(dof());
  dq_ = VectorXd::Zero(dof());
  ddq_ = VectorXd::Zero(dof());

  JSIM_ = MatrixXd::Zero(dof(), dof());

  mass_ = 0.0;
  for(int i = 0; i < static_cast<int>(numberOfLinks()); i++)
  { // 1. Initialize the CRB inertia for all the links
    crb_links_.push_back(link(i)->localInertiaTensor());
    mass_ += link(i)->mass();
  }
}
void KinematicTree::initializeFrames()
{

  RoboticsUtils::quickHL("KinematicTree: " + model()->name() + ", is parsing the links.");
  // Create frames for the links, and the joints
  for(auto const & l : links())
  {
    Iso3d relativeTf = l->localCOMTransform(); // Place the origin of the frame at the CoM

    auto const j = joint(l->parentJointName());

    // if(j->dof() != 1)
    // {
    //   continue;
    // }

    RoboticsUtils::quickHL("linkFrame: " + l->name());
    RoboticsUtils::quickHL("parent joint name: " + j->name());
    std::shared_ptr<JointDependentFrame> linkFrame =
        std::make_shared<JointDependentFrame>(l->name(), j->name(), relativeTf, shared_from_this());

    j->attachObserver(linkFrame);

    RoboticsUtils::quickHL("joint: " + j->name() + " has attached the frame: ", linkFrame->name());
    frames_[l->name()] = linkFrame;
  }

  RoboticsUtils::quickHL("KinematicTree: " + model()->name() + ", is parsing the joints.");
  for(auto const & j : joints())
  {
    Iso3d relativeTf = Iso3d::Identity(); // Place the origin of the frame at the CoM

    // if(j->dof() != 1)
    // {
    //   continue;
    // }

    std::shared_ptr<JointDependentFrame> jointFrame =
        std::make_shared<JointDependentFrame>(j->name(), j->name(), relativeTf, shared_from_this());

    RoboticsUtils::quickHL("jointFrame: " + j->name());
    j->attachObserver(jointFrame);

    RoboticsUtils::quickHL("joint: " + j->name() + " has attached the frame: ", jointFrame->name());
    frames_[j->name()] = jointFrame;
  }

  RoboticsUtils::quickHL("KinematicTree: " + model()->name() + ", parsed the joints.");

  std::string comFrameName = "com";
  std::shared_ptr<ComFrame> cmmFrame = std::make_shared<ComFrame>(comFrameName, shared_from_this());
  attachObserver(cmmFrame); // Attach the COM frame to the kinematicTree
  frames_[comFrameName] = cmmFrame; // Register the COM frame

  std::string inertialFrameName = "inertial";
  std::shared_ptr<FixedFrame> inertialFrame = std::make_shared<FixedFrame>(inertialFrameName, shared_from_this());

  frames_[inertialFrameName] = inertialFrame; // Register the COM frame
  RoboticsUtils::quickHL("KinematicTree: " + model()->name() + ", has added the CoM and inertial frame.");
}

std::shared_ptr<KinematicChain> const KinematicTree::createKinematicChain(std::string const & chainName,
                                                                          std::string const & root,
                                                                          std::string const & leaf)
{
  kPath path;
  if(!graph()->findPath(root, leaf, path))
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("Unable to find the path from reference frame: " + root
                                       + " to target frame: " + leaf);
  }

  std::vector<std::shared_ptr<GeoJoint>> joints;
  std::vector<std::shared_ptr<GeoLink>> links;

  if(path.size() == 0)
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("The path from reference frame: " + root + " to target frame: " + leaf
                                       + " is empty!");
  }
  for(auto & step : path)
  {

    // 1. Insert the joint
    if((model()->getJoint(step.second))->type == JointType::FIXED)
    {
      // We should not encounter this, as by construction, the transform of the fixed joint has been merged.
      continue;
      /*
      RoboticsUtils::throw_runtime_error(
        "Unable to find the path from reference frame: " + root + " to target frame: " + leaf, __FILE__, __LINE__);
  */
    }
    else
    {
      auto joint = this->joint(step.second);
      // if((joint != nullptr)&&(link != nullptr))
      if(joint != nullptr)
      {
        // joints.insert(std::make_pair(joint->name, std::make_shared<GeoJoint>(joint)));
        joints.push_back(joint);
        // links.push_back(link);
      }
      else
      {
        RoboticsUtils::THROW_RUNTIME_ERROR("Found empty joint pointer: " + step.second);
      }
    }

    // 2. Insert the link

    auto link = this->link(step.first);

    if(link != nullptr)
    {
      links.push_back(link);
    }
    else
    {
      RoboticsUtils::THROW_RUNTIME_ERROR("Found empty link pointer: " + step.first);
    }
  }

  // std::shared_ptr<GeoLink> baseLink =  getLink(root);
  // chains_.insert(std::make_pair(chainName, std::make_shared<KinematicChain>(chainName, root, leaf, baseLink, joints,
  // links)));
  chains_.insert(std::make_pair(chainName, std::make_shared<KinematicChain>(chainName, root, leaf, joints, links)));

  return chains_[chainName];
}
/*
std::shared_ptr<GeoJoint> KinematicTree::metaGetJoint(const std::string & name) const
{
  RoboticsUtils::quickInfo("The number of joints is: ", joints_.size());
  RoboticsUtils::quickInfo("The number of joint indices is: ", jointIndices_.size());
  return joints_[static_cast<size_t>(jointNameToIndex(name))];
}
*/

/*
std::shared_ptr<GeoLink> KinematicTree::metaGetLink(const std::string & name) const
{
  return links_[static_cast<size_t>(linkNameToIndex(name))];
}
*/

Iso3d KinematicTree::transformTwo(std::string const & refrenceFrameName, std::string const & targetFrameName)
{

  // RoboticsUtils::quickPrint("transform of: " + refrenceFrameName, frame(refrenceFrameName)->transform().matrix());
  // RoboticsUtils::quickPrint("transform of: " + targetFrameName, frame(targetFrameName)->transform().matrix());
  return frame(refrenceFrameName)->transform().inverse() * frame(targetFrameName)->transform();
}

Iso3d KinematicTree::transform(std::string const & refrenceFrameName,
                               std::string const & targetFrameName,
                               bool const & initialTransform)
{
  Iso3d output = Iso3d::Identity();

  kPath path;

  // clang-format off
  if(!graph()->findPath(refrenceFrameName, targetFrameName, path))
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("Unable to find the path from reference frame: " + refrenceFrameName + " to target frame: " + targetFrameName);
  }
  // clang-format on 
  graph()->printPath(path);
  // See eq~3.1 of Murray et al. 1994
  if(path.size()!=0)
  {
    for(auto const & step : path)
    {
      if(this->joint(step.second) == nullptr)
      {
         std::cout<<"The pointer to: "<<step.second<<" is null"<<std::endl;
      }
      //printMatrix("step: ", joint(step.second)->initialRelativeTransform().matrix());
      // Multiply the relative transform along the path.

      // RoboticsUtils::quickPrint("step: " + joint(step.second)->name(), joint(step.second)->relativeTransform().matrix());
      if(initialTransform)
      {
        output = output * this->joint(step.second)->initialRelativeTransform();
      }
      else
      {
        // output = output * this->joint(step.second)->relativeTransform();
        output = output * this->joint(step.second)->incrementalTransform();
      }
    }

    output = output * this->joint(path.rbegin()->second)->initialTransform();
  }


  return output;
}



int KinematicTree::linkNameToIndex(std::string const & name) const
{
  auto const & link = linkIndices_().find(name);

  if(link == linkIndices_().end())
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("The joint: " + name + " is not found");
    return -1;
  }
  return link->second;
}

int KinematicTree::jointNameToIMIndex(std::string const & jointName) const
{
  return joint(jointNameToIndex(jointName))->inertiaMatrixJointIndex();
}

int KinematicTree::jointNameToIndex(std::string const & name) const
{
  auto const & joint = jointIndices_().find(name);
  if( joint == jointIndices_().end())
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("The joint: " + name + " is not found");
    return -1;
  }
  return joint->second;
}

std::string KinematicTree::jointIndexToName(size_t const & idx) const
{
  if(idx >= numberOfJoints())
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("idx: " + std::to_string(idx) + "is out of the range.");
  }
  return joints()[idx]->name();
}

std::string KinematicTree::linkIndexToName(size_t const & idx) const
{
  if(idx >= numberOfLinks())
  {
    RoboticsUtils::THROW_RUNTIME_ERROR("idx: " + std::to_string(idx) + "is out of the range.");
  }
  return links()[idx]->name();
}

void KinematicTree::printTree() const
{
  RoboticsUtils::quickHL("The Kinematic tree " + name() + " has ", links().size(), " links:");

  for(auto const & l : links())
  {
    RoboticsUtils::quickInfo("link: ",  l->name(), " has index: ",  linkNameToIndex(l->name())); 
  }
  RoboticsUtils::quickHL("The Kinematic tree " + name() + " has ", joints().size(), " joints:");

  for(auto const & j : joints())
  {
   if(j!=nullptr)
   {
     RoboticsUtils::quickInfo( "joint: ",  j->name(), " has index: ", jointNameToIndex(j->name()), " and degrees of freedom: ",  j->dof());
   }
  }

  graph()->printGraph();

  std::cout << std::endl;
}

//void KinematicTree::numberTheLinksAndJoints_()
//{
//  // Numbering according to Sec.~4.1.2 of Featherstone. 
//  int nL = static_cast<int>(model()->numLinks());
//  int nJ = static_cast<int>(model()->numJoints());
//
//  RoboticsUtils::quickError("nL: ", nL);
//  RoboticsUtils::quickError("nJ: ", nJ);
//
//  // 1. Apply the link indices of the spanning tree.
//  //linkIndices_ = graph()->spanningTree().nodes;
//  //assert(nL == linkIndices_().size());
//
//  // 2. Numbering the joints 
//  for (auto e:graph()->spanningTree().steps)
//  {
//   
//   auto childLink = linkIndices_().find(e.second.first);
//
//   if(childLink == linkIndices_().end())
//   {
//     RoboticsUtils::throw_runtime_error("The child link: " + e.second.first + " of joint: " + e.first + " is not found from the robot's link lists", __FILE__, __LINE__);
//   }
//   // Apply the child link's index to its parent joint.
//   jointIndices_[e.first] = childLink->second;
//   RoboticsUtils::quickHL("joint: ", e.first, " has index: ", childLink->second);
//  }
//
//
//  // 3. Numbering the remaining joints in any order
//  //if(nL >= nJ)
//  //{
//  //  diffNJNL_ = nJ - nL;
//  //}
//  //else
//  //{
//  //  diffNJNL_ = 0; 
//  //}
//  diffNJNL_ = nJ - nL;
//
//  RoboticsUtils::quickError("diffNJNL_ : ", diffNJNL_);
//}


void KinematicTree::readLinkNames_(std::vector<std::string> & names) const
{
  for(auto l:links())
  {
    auto name = l->name() + ", index: " + std::to_string(linkNameToIndex(l->name()));
    names.push_back(name); 
  }
}

    //  define a property writer to color the edges as required
  class color_writer {
  public:

      // constructor - needs reference to graph we are coloring
    color_writer( std::ofstream &f, BoostGraph & g ) : myGraph( g ), myFile(f) {}

      // functor that does the coloring
    template <class VertexOrEdge>
    void operator()(std::ofstream& out, const VertexOrEdge& e) const {

        // check if this is the edge we want to color red
        if( source( e, myGraph ) == 56 &&
            target( e, myGraph ) == 57  )
                out << "[color=red]";
    }
  private:
    BoostGraph & myGraph;
    std::ofstream & myFile;
  };

void KinematicTree::addEdges_(std::map<BoostGraph::edge_descriptor, std::string> & ename, BoostGraph & g) const
{
  for(auto e:graph()->spanningTree().steps)  
  {
    int nc = linkNameToIndex(e.second.first);
    auto j = joint(e.first);
    int np = linkNameToIndex(j->parentLink()->name());
    //BoostGraph::edge_descriptor eHandle;
    auto eHandle = add_edge(static_cast<size_t>(np), static_cast<size_t>(nc), g).first;
    std::string dofString; 
    
    std::string edgeLabel = e.first + ", index: " + std::to_string(jointNameToIndex(e.first)) + ", dof: " + std::to_string(j->dof());
    //ename[eHandle] = colorString + edgeLabel;
    ename[eHandle] = edgeLabel;

    //edges.push_back(std::make_pair(np, nc));
  }
}

/*
void KinematicTree::readEdges_(std::vector<std::pair<int, int>> & edges) const
{
  for(auto e:graph()->spanningTree().edges)  
  {
    int nc = linkNameToIndex(e.second.first);
    auto j = joint(e.first);
    int np = linkNameToIndex(j->parentLink()->name());
    edges.push_back(std::make_pair(np, nc));
  }
}
*/


int KinematicTree::visualize() const
{
  
  // Get the link (node) names
  std::vector<std::string> lNames;
  readLinkNames_(lNames);


  // Define the graph
  //using namespace boost;
  //std::vector<std::pair<int, int>> edges;
  //readEdges_(edges);

  //typedef adjacency_list< vecS, vecS, directedS
  //  > bGraph;


  //const int nedges = sizeof(edges)/sizeof(std::pair<int, int>);

  //int weights[nedges];
  //std::fill(weights, weights + nedges, 1);

  //bGraph g(edges.begin(), edges.end(), graph()->spanningTree().nodes.size());
  //bGraph g(edges.begin(), edges.end(), weights, graph()->spanningTree().nodes.size());
  BoostGraph g(graph()->spanningTree().nodes.size());

  std::map<BoostGraph::edge_descriptor, std::string> ename;

  KinematicTree::addEdges_(ename, g);

  std::string fileName = this->name() + "_kinematics_tree.dot";
  std::string pdfFileName = this->name() + "_kinematics_tree.pdf";
  //Write graph to file
  {
    std::ofstream f(fileName);

    boost::write_graphviz(f, g, boost::make_label_writer(&lNames[0]), make_label_writer(boost::make_assoc_property_map(ename)));

    f.close();
  }
  std::string command = std::string("dot -Tpdf ") + fileName.c_str() + std::string(" -o ") + pdfFileName.c_str();
  return std::system(command.c_str());
  // std::string visualizeCommand = std::string("evince ") + pdfFileName + std::string(" & ");
  // std::system(visualizeCommand.c_str());

}

void KinematicTree::printOdArticulatedJoints(std::string const & linkName) const
{
   std::deque<int> jSet;
   odArticulatedJoints(jSet, linkName);
   RoboticsUtils::quickHL("The dependent joints of link: ", linkName, " include: ");
   for(size_t ii = 0; ii < jSet.size(); ii++)
   {
     auto j = joint(jSet[ii]);
     RoboticsUtils::quickInfo("name:", j->name(), " index: ", j->jointIndex());
   }
}
void KinematicTree::odArticulatedJoints(std::deque<int> & set, const std::string & linkName) const
{
    int plIdx = linkNameToIndex(linkName);
    //int j = jointNameToIndex(link(linkName)->parentJointName());
    //set.push_front(j);
    // Loop until the root link
    while(plIdx != 0) 
    {
       int pjIdx = jointNameToIndex(link(plIdx)->parentJointName());
       if(isArticulatedJoint_(pjIdx))
       {
         set.push_front(pjIdx);
       }
       plIdx =  linkNameToIndex(parentLink(linkIndexToName(static_cast<size_t>(plIdx)))->name());
    }
}
void KinematicTree::odJoints(std::deque<int> & set, const std::string & linkName) const
{
    int plIdx = linkNameToIndex(linkName);
    //int j = jointNameToIndex(link(linkName)->parentJointName());
    //set.push_front(j);
    // Loop until the root link
    while(plIdx != 0) 
    {
       int pjIdx = jointNameToIndex(link(plIdx)->parentJointName());
       set.push_front(pjIdx);
       plIdx =  linkNameToIndex(parentLink(linkIndexToName(static_cast<size_t>(plIdx)))->name());
    }

    set.push_front(0); // Push the root link
}


// void KinematicTree::update()
// This API does not work. It leads to non issue if we also update the contact surface
// {
//   readJointPositions(q_);
//   readJointVelocities(dq_);
//   readJointAccelerations(ddq_);
//   
//   updateTree_();
// }

void KinematicTree::setRandom(size_t status, std::default_random_engine & re, double safety_factor)
{
  for(auto idx : graph()->articulatedJoints())
  {
    joint(idx.first)->setRandom(status, re, safety_factor);
  }

  // Set the floating-base joint
  
  if(isFloatingBase())
  {
    joint(0)->setRandom(status, re, safety_factor);
  }

  readJointPositions(q_);
  readJointVelocities(dq_);
  readJointAccelerations(ddq_);
  
  updateTree_();
}

void KinematicTree::readJointPositions(VectorXd & q) const
{

  q = VectorXd::Zero(dof());

  if(isFloatingBase())
  {
   auto tf = joint(0)->transform();
   q.head(3) = tf.translation();
   q.segment<3>(3) = joint(0)->tfAngleAxis();
  }

  for(auto idx: graph()->articulatedJoints())
  {
    auto j = joint(idx.first);
    q(idx.second) = j->q();
    //ii++;
  }
}

void KinematicTree::readJointVelocities(VectorXd & dq, std::string const & jointName) const
{
  int dof = 0, count = 0;
  auto pj = joint(jointName);

  if(isFloatingBase())
  {
   dof = 6;
   count = 6;
   // dq.head(6) = joint(0)->jointVelocity();
  }
  dof += static_cast<int>(pj->articulatedJoints().size()) + 1; // Plus 1 means including the joint velocity of the joint: "jointName" itself

  dq = VectorXd::Zero(dof);

  if(isFloatingBase())
  {
    dq.head(6) = joint(0)->jointVelocity();
  }

  for(auto idx: pj->articulatedJoints())
  {
    auto j = joint(idx.first);
    dq(count) = j->dq();
    count++;
  }
  dq(count) = pj->dq();
}

void KinematicTree::readJointVelocities(VectorXd & dq) const
{
  dq = VectorXd::Zero(dof());

  if(isFloatingBase())
  {
   dq.head(6) = joint(0)->jointVelocity();
  }

  for(auto idx: graph()->articulatedJoints())
  {
    auto j = joint(idx.first);
    dq(idx.second) = j->dq();
  }
}

void KinematicTree::readJointAccelerations(VectorXd & ddq) const
{

  ddq = VectorXd::Zero(dof());

  if(floatingBase_)
  {
   ddq.head(6).setZero();
  }

  for(auto idx: graph()->articulatedJoints())
  {
    auto j = joint(idx.first);
    ddq(idx.second) = j->ddq();
  }
  
}

void KinematicTree::readJointTorques(VectorXd & tau) const
{
  tau.resize(dof());

  if(floatingBase_)
  {
   tau.head(6).setZero();
  }

  for(auto idx: graph()->articulatedJoints())
  {
    auto j = joint(idx.first);
    tau(idx.second) = j->tau();
  }
 
}

// void KinematicTree::computeComJacobian(MatrixXd & comJac) const
// {
// 
//   comJac = MatrixXd::Zero(3, dof());
// 
//   for(auto l : links())
//   {
//     double weight = l->mass() / mass();
//     
//     MatrixXd bodyJac;
//     computeLinkComBodyJac_(bodyJac, l->name(), true);
// 
//     // const MatrixXd & temp = joint(l->parentJointName())->transform().rotation() * bodyJac;
// 
//     comJac.noalias() += weight * bodyJac;
//   }
// 
// }
// void KinematicTree::computeLinkComBodyJac_(MatrixXd & comJac, const std::string & linkName, const bool & fullJacobian ) const
// {
//   const std::shared_ptr<GeoLink> & l = link(linkName);
// 
//   MatrixXd bodyJac;
//  
//   computeBodyJacobian_(l->parentJointName(), bodyJac, fullJacobian);
// 
//   const MatrixXd & linearJac = bodyJac.block(0, 0, 3, bodyJac.cols());
//   const MatrixXd & angularJac = bodyJac.block(3, 0, 3, bodyJac.cols());
// 
//   auto & j = joint(l->parentJointName());
//   Iso3d const & tf = j->transform();
// 
//   comJac.resize(3, bodyJac.cols());
//   comJac.noalias() = tf.rotation() * (-skew(l->localCOM()) * angularJac + linearJac);
// }

void KinematicTree::MCalcCRB(MatrixXd & M)
{
  //auto parentJoint = [&](int i){ 
  //        if(i == 0)
  //          return -1;
  //        else
  //        {
  //          const std::string & pjName = joint(i)->parentLink()->parentJointName();
  //          return jointNameToIndex(pjName);
  //        }
  //};

  auto parentLink = [&](int i){ 
	  if(i == 0)
	    return -1;
	  else
	  {
	    const std::string & plName = joint(link(i)->parentJointName())->parentLink()->name();
	    return linkNameToIndex(plName);
	  }
  };


  //
  M = MatrixXd::Zero(dof(), dof());
  std::vector<Matrix6d> crb_links = get_crb_links();

  //for(int i = 0; i < static_cast<int>(numberOfLinks()); i++)
  //{ // 1. Initialize the CRB inertia for all the links
  //  crb_links.push_back(link(i)->localInertiaTensor());
  //}

  //int ajIdx = static_cast<int>(articulatedJoints().size());
  //int ajIdx = dof();
  // Note that size_t is never < 0.
  // RoboticsUtils::quickInfo("There are ", articulatedJoints().size(), " articulatedJoints.");
  //for(size_t i = (articulatedJoints().size() - 1); i > 0; i--)
  //for(int i = (static_cast<int>(articulatedJoints().size()) - 1); i >= 0; i--)
  //for(int i = (static_cast<int>(articulatedJoints().size()) - 1); i >= 0; i--)
  for(int i = (static_cast<int>(links().size()) - 1); i > 0; i--)
  //for(int i = (dof() - 6); i > 0; i--)
  { // 2. Loop from the leaf to the root

    auto current_J = joint(link(i)->parentJointName());
    // #1 joint index, 
    //int idx = articulatedJoints()[static_cast<size_t>(i)].first;
    int idx = current_J->jointIndex(); 
    // #2 index in the inertia matrix
    //int aidx = articulatedJoints()[static_cast<size_t>(i)].second;
    int aidx = current_J->inertiaMatrixJointIndex(); 

    //RoboticsUtils::quickInfo("joint: ", joint(idx)->name(), " joint index: ", idx, ", inertiamatrix index: ",aidx);

    int pidx = parentLink(idx);
    auto piFrameName = link(pidx)->parentJointName();
    auto iFrameName = link(idx)->parentJointName();
    // Add the child link CRB inertia to the parent
    crb_links[static_cast<size_t>(pidx)] += inertiaTransform(
        crb_links[static_cast<size_t>(idx)], iFrameName, piFrameName); 

    //RoboticsUtils::quickInfo("Joint: ", joint(link(idx)->parentJointName())->name(), " joint position: ", joint(link(idx)->parentJointName())->q());
    // Momentum type of variable
    //auto si = joint(link(idx)->parentJointName())->bodyTC();
    auto si = current_J->bodyTC();
    Vector6d h = crb_links[static_cast<size_t>(i)] * si;

    //RoboticsUtils::quickPrint("si", si);
    //RoboticsUtils::quickPrint("h", h);

    M(aidx, aidx) = si.transpose() * h; //

    //RoboticsUtils::quickInfo("Joint: ", joint(link(idx)->parentJointName())->name(), " is updating: M(", aidx, ", ", aidx,") = ",  M(aidx, aidx));
    int j = idx;

    while(parentLink(j)!= 0)
    {
       // Inner loop to root.
       int pj = parentLink(j);

       h = wrenchTransform(link(j)->parentJointName(), link(pj)->parentJointName()) * h;
       j = pj;
       auto sj = joint(link(j)->parentJointName())->bodyTC();
       int rIdx = aidx; 
       int cIdx = joint(pj)->inertiaMatrixJointIndex(); 
       M(rIdx, cIdx) = h.transpose() * sj;
       M(cIdx, rIdx) = M(rIdx, cIdx);
      
       //RoboticsUtils::quickInfo("Joint: ", joint(link(idx)->parentJointName())->name(), " is updating: M(", rIdx, ", ", cIdx,") = ",  M(rIdx, cIdx));
    }

    // RoboticsUtils::quickInfo("Transforming wrench from link: ", link(j)->name(), " to link: ", link(0)->name());
    h = wrenchTransform(link(j)->parentJointName(), link(0)->parentJointName()) * h;

    //RoboticsUtils::quickPrint("F_"+ std::to_string(aidx) +" is: ", h);
    //RoboticsUtils::quickInfo("Transforming wrench from link: ", link(j)->name(), " to link: ", link(0)->name());
    M.block<6,1>(0, aidx) = h;
    M.block<1,6>(aidx, 0) = h.transpose();
  } // End of the loop from leaf to root

  M.block<6,6>(0, 0) = crb_links[0];
} // End of CRB algorithm






//void KinematicTree::updateJointsAndTransform_()
//{
//  int ii = 0;
//  if(floatingBase_)
//  {
//   q_.head(6).setZero();
//   dq_.head(6).setZero();
//   ddq_.head(6).setZero();
//   ii += 6;
//  }
//  for(auto idx: graph()->articulatedJoints())
//  {
//    auto j = joint(static_cast<int>(idx));
//    q_(ii) = j->q();
//    dq_(ii) = j->dq();
//    ddq_(ii) = j->ddq();
//    ii++;
//  }
//
//  updateTransform_();
//}


void KinematicTree::updateTree_()
{
  //RoboticsUtils::quickHL("This is a void update of the kinematic tree.");

  // Read the joint values
  
  // 1. Update the Transforms via the resursive DFS search.
  updateTransform_();

  // 2. Compute the joint torque with "inverse dynamics algorithm". It also updates: the joint transforms, and the link velocity, acceleration, force.
  
  IdCalc(q_, dq_, ddq_);
  
  // 3. Compute the joint-space inertia matrix with "forward dynamics algorithm".
  MCalcCRB(JSIM_);

  // 4. Compute Coriolis and Centrifugal according to the Recursive Newton Eular Algorithm (RENA)
  
  // 5. Compute the potential energy and kinetic energy terms. 
 
  // 6. Centroidal dynamics
  
}
void KinematicTree::setJointPositions(VectorXd const & q)
{

  // RoboticsUtils::quickPrint("Setting joint positions", q);

  if(q.size() != dof())
  {
     RoboticsUtils::quickError("setJointPositions for robot: " + name() + ", the dq size: ", q.size(), " does not match the robot DOF: ", dof());
     RoboticsUtils::THROW_RUNTIME_ERROR("Fail to setJointPositions due to missmathcing input size.");
  }

  // If there is a floating-base joint
  if(isFloatingBase())
  {
    auto j = joint(0);
    Vector3d t = q.segment<3>(0);
    //j->update(q(0), q(1), q(2), q(3), t);
    j->update(q(3), q(4), q(5), t);
  }

  for(auto idx:articulatedJoints())
  {
    auto j = joint(idx.first);
    //auto & joint = joints_[static_cast<size_t>(idx)];
    // RoboticsUtils::quickInfo("joint: ", j->name(), " position value: ", q(idx.second), "idx 1: ", idx.first, ", 2: ", idx.second);
    j->update(q(idx.second));
  }

  // Update the current joint positions
  q_ = q;
  //readJointVelocities(dq_);
  // updateJointVelDependentComponents_();

  updateTransform_();
}

void KinematicTree::setJointVelocities(VectorXd const & dq)
{
  if(dq.size() != dof())
  {
     RoboticsUtils::quickError("setJointVelocities for robot: " + name() + ", the dq size: ", dq.size(), " does not match the robot DOF: ", dof());
     RoboticsUtils::THROW_RUNTIME_ERROR("Fail to setJointVelocities due to missmathcing input size.");
  }

  // Set the floating-base joint velocities
  if(isFloatingBase())
  {
    auto j = joint(0);
    j->updateDq(dq.segment<3>(0), dq.segment<3>(3));
  }
  for(auto idx:articulatedJoints())
  {
    auto j = joint(idx.first);
    //auto & joint = joints_[static_cast<size_t>(idx)];
    j->updateDq(dq(idx.second));
  }
  // Update the current joint velocities !!!
  dq_ = dq;
  //readJointVelocities(dq_);
  // updateJointVelDependentComponents_();

}

void KinematicTree::setJointAccelerations(VectorXd const & ddq)
{
  if(ddq.size() != dof())
  {
     RoboticsUtils::quickError("setJointAccelerations for robot: " + name() + ", the dq size: ", ddq.size(), " does not match the robot DOF: ", dof());
     RoboticsUtils::THROW_RUNTIME_ERROR("Fail to setJointAccelerations due to missmathcing input size.");
  }

  // Set the floating-base joint velocities
  //ddq_.head(6) = ddq.head(6);

  //for(size_t idx = 6; idx < static_cast<size_t>(ddq.size()); idx++)
  //{
  //  auto j = joint(articulatedJoints()[idx - 6]);
  //  //auto & joint = joints_[static_cast<size_t>(idx)];
  //  j->updateDDq(ddq(static_cast<int>(idx)));
  //}
  for(auto idx:articulatedJoints())
  {
    auto j = joint(idx.first);
    //auto & joint = joints_[static_cast<size_t>(idx)];
    j->updateDDq(ddq(idx.second));
  }
  // Update the current joint velocities !!!
  ddq_ = ddq;
  //readJointVelocities(dq_);
  // updateJointVelDependentComponents_();

}

bool KinematicTree::isArticulatedJoint_(const std::string & jName) const
{
  return isArticulatedJoint_(jointNameToIndex(jName));
}

bool KinematicTree::isArticulatedJoint_(const int & idx) const
{
  return (joint(idx)->dof() == 1);
}

void KinematicTree::IdCalc(const VectorXd & q, const VectorXd & dq, const VectorXd & ddq)
{ // Adopt the body-coordinate computation

  //RoboticsUtils::quickPrint("Joint positions ", q);
  //RoboticsUtils::quickPrint("Joint vel ", dq);
  //RoboticsUtils::quickPrint("Joint acc ", ddq);
  // The implementation follows Table 5.1 of Featherstone's book.

  //RoboticsUtils::quickInfo("IdCalc: the input q has size: ", q.size());
  //RoboticsUtils::quickInfo("IdCalc: the robot has dof: ", dof(), " and ", articulatedJoints().size(), " articulatedJoints.");

  setJointPositions(q); // It would update the transforms too. 
  setJointVelocities(dq);
  setJointAccelerations(ddq);

  auto parentLink = [&](int i){ 
	  if(i == 0)
	    return -1;
	  else
	  {
	    const std::string & plName = joint(link(i)->parentJointName())->parentLink()->name();
	    return linkNameToIndex(plName);
	  }
  };

  auto parentJoint = [&](int i){ 
	  if(i == 0)
	    return -1;
	  else
	  {
	    const std::string & pjName = joint(i)->parentLink()->parentJointName();
	    return jointNameToIndex(pjName);
	  }
  };

  Vector6d iniAcc = Vector6d::Zero();
  iniAcc(2) = -GRAVITY;

  //Iso3d tf = joint(articulatedJoints()[0])->initialTransform();

  // 1. ---------------- i = 1 to N  -----------------------
  //for(size_t idx= 0; idx < graph()->articulatedJoints().size(); idx++)
  for(size_t idx= 0; idx < graph()->articulatedJoints().size(); idx++)
  { // Loop from root to leaf

    // #1 joint index, 
    int ii = articulatedJoints()[static_cast<size_t>(idx)].first;
    // #2 index in the inertia matrix
    // int aidx = articulatedJoints()[static_cast<size_t>(idx)].second;
    //int ii = articulatedJoints()[idx];
    //RoboticsUtils::quickHL("KT IdCalc: idx: ", idx);
    //RoboticsUtils::quickHL("KT IdCalc: processing joint: ", joint(ii)->name()," with index: ", ii);
    //std::shared_ptr<GeoJoint> & j = joints_[ti];
    auto j = joint(ii);
    const std::shared_ptr<GeoLink> & l = j->childLink();

    // 1. Joint calculation
    j->setJointVel(j->bodyTC() * j->dq());

    // 2. Set the link velocity 
    int plIdx = parentJoint(ii);
    //size_t t_plIdx = static_cast<size_t>(plIdx);
    Vector6d link_vel = j->jointVelocity();

    //RoboticsUtils::quickHL("KT IdCalc: parent joint index: ", plIdx);

    // Account for the parent link velocity
    Matrix6d velP2b;
    if(plIdx >= 0)  
    {
      // There is parent joint/link
      const std::shared_ptr<GeoLink> & pl = link(plIdx);
      const std::shared_ptr<GeoJoint> & pj = joint(plIdx);
      velP2b = metaAdgInvMatrix(pj->transform().inverse() * j->transform());
      // velP2b = velTransform(pj->name(), j->name());
      link_vel += velP2b * pl->bodyVelocity();
    }
    l->setBodyVel(link_vel);

    // 3. Set the link acceleration
    const Vector6d & joint_acc = j->bodyTC() * j->ddq();
    Vector6d link_acc = joint_acc + cross(link_vel, j->jointVelocity());

    if(plIdx >= 0)
    {
      // There is parent joint/link
      const std::shared_ptr<GeoLink> & pl = link(plIdx);
      link_acc += velP2b * pl->bodyAcceleration();
    }else if(plIdx == -1)
    { // Only applies to the root link. It should propagates to the leaf.
      link_acc += iniAcc;
    }

    l->setBodyAcc(link_acc);

    l->calcWrench();
    //RoboticsUtils::quickHL("KT IdCalc: finished one inner iteration. ");
  } // End of the loop from root to leaf

  //RoboticsUtils::quickError("KT IdCalc: Strated the outer loops. ");



  link(0)->setBodyAcc(iniAcc);
  link(0)->calcWrench();

  std::vector<Matrix6d> crb_links = get_crb_links();
  // 2. ---------------- i = N to 1 -----------------------
  for(int i = (static_cast<int>(graph()->articulatedJoints().size()) - 1); i >= 0; i--)
  { // Loop from leaf to root
    int ii = articulatedJoints()[static_cast<size_t>(i)].first;

    //RoboticsUtils::quickHL("KT IdCalc outer loop: idx: ", i);
    //RoboticsUtils::quickHL("KT IdCalc outer loop: processing joint: ", joint(ii)->name()," with index: ", ii);

    int plIdx = parentLink(ii);
    auto piFrameName = link(plIdx)->parentJointName();
    auto iFrameName = link(ii)->parentJointName();
    // 2.1 Add the child link CRB inertia to the parent
    crb_links[static_cast<size_t>(plIdx)] += inertiaTransform(
        crb_links[static_cast<size_t>(ii)], iFrameName, piFrameName); 

    // 2.2 Add the child link wrench to the parent link 
    Vector6d pWrench = link(plIdx)->wrench();
    link(plIdx)->setWrench(pWrench + wrenchTransform(iFrameName, piFrameName) * link(ii)->wrench());

  } // End of the torque loop
 
  // 3. ---------------- i = 1 to N -----------------------
  // Evaluate the joint torques
  
  // 3.1 root link acceleration
  
  //Vector6d a00 = -crb_links[0].inverse() * link(0)->wrench();

  std::vector<Vector6d> base_accs;
  base_accs.resize(numberOfLinks());
  base_accs[0] = -crb_links[0].colPivHouseholderQr().solve(link(0)->wrench());

  // 3.2 acctuation torque of all the joints 
  for(size_t idx= 0; idx < graph()->articulatedJoints().size(); idx++)
  {
    // #1 joint index, 
    int ii = articulatedJoints()[idx].first;
    //RoboticsUtils::quickHL("KT IdCalc outer loop: idx: ", i);
    //RoboticsUtils::quickHL("KT IdCalc outer loop: processing joint: ", joint(ii)->name()," with index: ", ii);
    int plIdx = parentLink(ii);
    auto piFrameName = link(plIdx)->parentJointName();
    auto iFrameName = link(ii)->parentJointName();

    base_accs[static_cast<size_t>(ii)] =  velTransform(piFrameName, iFrameName) * base_accs[static_cast<size_t>(plIdx)];

    // 2. parent link acceleration to child link

    const std::shared_ptr<GeoJoint> & j = joint(ii);
    const std::shared_ptr<GeoLink> & l = link(ii);

    double tau = j->bodyTC().transpose() * ( crb_links[static_cast<size_t>(ii)]* base_accs[static_cast<size_t>(ii)]+ l->wrench());
    // RoboticsUtils::quickPrint("tau: " + std::to_string(i), tau);
    j->updateTau(tau);
  }

}

void KinematicTree::checkTransform_(std::unordered_set<std::string>& visited, const std::string & currentNode,  const std::string & currentJoint, Iso3d & iniTF)
  {
    visited.insert(currentNode);
    auto cj = joint(currentJoint);
    // Increment the initial transform
    
    if(cj->dof() == 1)
    {
      iniTF = iniTF * cj->incrementalTransform();
    }
  }
void KinematicTree::transformDFSUpdate_(
                      const std::string & currentNode,
                      const std::string & currentJoint,
                      Iso3d & iniTF,
                      std::unordered_set<std::string>& visited)
  {

  checkTransform_(visited, currentNode, currentJoint, iniTF);

  // 2. Check the adjacent nodes of the currentNode
  auto adjList = graph()->adjacencyList();
  for(auto & step : adjList[currentNode])
  {
    const std::string & nextNode = step.first;
    const std::string & nextJoint = step.second;
    // If not yet visited 
    if(visited.find(nextNode) == visited.end())
    {
      // Insert the edge and the node
      //sTree_.insertEdge(step, idx, model->getLink(step.first), model->getJoint(step.second), joints_,
                        //articulatedJoints_, links_, iniTF);
      transformDFSUpdate_( nextNode, nextJoint, iniTF, visited);
    }
  }

  // Reached the end of a chain:
  auto currentLink = link(currentNode);
  auto pj = joint(currentLink->parentJointName());

  pj->setTransform(iniTF * pj->initialTransform()); // Note that we multiplies the initial transform, see Eq.~3.9 of Murray's book 

  if(pj->dof() == 1)
  {
    iniTF = iniTF * pj->incrementalTransform().inverse();
  }

  double weight = currentLink->mass() / mass();
  const Vector3d & inertialCom = pj->transform() * currentLink->localCOM();
  //RoboticsUtils::quickPrint("Link: "+ currentLink->name() +"'s COM: ", inertialCom);
  com_ += weight * inertialCom;
}

void KinematicTree::updateTransform_()
{


   std::string rootNode = link(0)->name();
   std::string rootJoint = link(0)->parentJointName();

   com_.setZero();

   Iso3d iniTF = Iso3d::Identity();

   // Initialize with the floating-base joint transform
   if(isFloatingBase())
   {
     iniTF = joint(0)->transform(); 
   }

   std::unordered_set<std::string> visited;

   // Check all the joints with a DFS search
   transformDFSUpdate_(
		   rootNode,
		   rootJoint,
		   iniTF,
		   visited);


   cmmtf_.translation() = com();

   // Notify the KinematicTree observers: e.g., the COM frame
   notifyObservers();
}

std::string KinematicTree::randomLinkName() const
{
  auto lIdx = randInt(links().size());
  auto it = links().begin();
  std::advance(it, lIdx);

  return it->get()->urdfLink()->name; 
}


std::string KinematicTree::randomJointName() const
{
  auto jIdx = randInt(joints().size());
  auto it = joints().begin();
  std::advance(it, jIdx);

  return it->get()->name(); 
}


std::string KinematicTree::randomFrameName() const
{
  auto fIdx = randInt(frames_.size());
  auto it = frames_.begin();
  std::advance(it, fIdx);

  return it->first; 
}

Matrix6d KinematicTree::velTransform(std::string const & a, std::string const & b) const
{
  if(a == b)
  {
    return Matrix6d::Identity(); 
  }

  // Iso3d const g_ba = joint(b)->transform().inverse() * joint(a)->transform();
  Iso3d const g_ba = frame(b)->transform().inverse() * frame(a)->transform();

  return metaAdgMatrix(g_ba);
}

Matrix6d KinematicTree::velTransformDerivative(std::string const & a, std::string const & b) const
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




Matrix6d KinematicTree::inertiaTransform(Matrix6d const & inertia,
                                          std::string const & currenFrame,
                                          std::string const & targetFrame) const
{
  if(currenFrame == targetFrame)
  {
    return inertia;
  }

  Matrix6d const & velTF = velTransform(targetFrame, currenFrame);
  return velTF.transpose() * inertia * velTF;
}

Matrix6d KinematicTree::inertiaTransformDerivative(Matrix6d const & inertia,
                                                    std::string const & currenFrame,
                                                    std::string const & targetFrame) const
{
  if(currenFrame == targetFrame)
  {
    return inertia;
  }

  Matrix6d const & velTF = velTransform(targetFrame, currenFrame);
  Matrix6d const & velTFd = velTransformDerivative(targetFrame, currenFrame);
  return velTFd.transpose() * inertia * velTF + velTF.transpose() * inertia * velTFd;
}

Matrix6d KinematicTree::wrenchTransform(std::string const & a, std::string const & b) const
{
  if(a == b)
  {
    return Matrix6d::Identity(); 
  }

  Iso3d const & g_ab = frame(a)->transform().inverse() * frame(b)->transform();

  return metaAdgTMatrix(g_ab);
}

Matrix6d KinematicTree::wrenchTransformDerivative(std::string const & a, std::string const & b) const
{
  if(a == b)
  {
    return Matrix6d::Zero();
  }

  Iso3d const & g_ab = frame(a)->transform().inverse() * frame(b)->transform();
  Vector6d const & Vb_ab = computeBodyVel(a, b);

  return crossMatrix(Vb_ab).transpose() * metaAdgTMatrix(g_ab);
}


Matrix6d KinematicTree::computeCentroidalInertia() const
{
  Matrix6d centroidalInertia = Matrix6d::Zero();
  //centroidalInertia.setZero();

  //RoboticsUtils::quickInfo("The robot: ", name(), " has ", links().size(), " geoLinks.");
  //RoboticsUtils::quickInfo("The mass is: ", mass()); 
  //RoboticsUtils::quickPrint("COM: ", com()); 
  for(auto link : links())
  {
    auto tf = joint(link->parentJointName())->transform();
    centroidalInertia += link->spatialInertia(cmmtf_.inverse() * tf);
  }

  return centroidalInertia;
}


MatrixXd KinematicTree::computeCMMDerivative() const
{
  MatrixXd cmmd = MatrixXd::Zero(6, dof());

  for(auto const & l : links())
  {
    std::string const parentJointName = l->parentJointName();

    MatrixXd const jac = computeBodyJacobian(parentJointName, true);
    MatrixXd const jacd = computeBodyJacobianDerivative(parentJointName, true);

    MatrixXd const lmJac = l->localInertiaTensor() * jac;

    MatrixXd const lmJacd = l->localInertiaTensor() * jacd;

    // long cols = lmJac.cols();

    cmmd +=
        wrenchTransformDerivative(parentJointName, "com") * lmJac + wrenchTransform(parentJointName, "com") * lmJacd;
  }
  return cmmd;

}

MatrixXd KinematicTree::computeComJac() const
{
  MatrixXd comJac = MatrixXd::Zero(3, dof()); 

  for(auto const & link: links())
  {
 
    // auto link = joint->childLink();
    double const weight = link->mass() / mass();

    std::shared_ptr<JointDependentFrame> l = std::dynamic_pointer_cast<JointDependentFrame>(frame(link->name()));

    bool fullSize = true;
    MatrixXd const & bodyJac = l->computePointJacobian(Vector3d::Zero(), fullSize);

    comJac.noalias() += weight * bodyJac;
  }

  return comJac; 
}
	
MatrixXd KinematicTree::computeCMM() const
{
  MatrixXd cmm = MatrixXd::Zero(6, dof());

  // CMM frame transform:
  for(auto const & l : links())
  {
    std::string const parentJointName = l->parentJointName();
    MatrixXd const lmJac = l->localInertiaTensor() * computeBodyJacobian(parentJointName, true);
    cmm += wrenchTransform(parentJointName, "com") * lmJac;
  }
  return cmm;
}

// MatrixXd KinematicTree::computeCMM() const
// {
//   MatrixXd cmm = MatrixXd::Zero(6, dof());
// 
//   // CMM frame transform:
//   for(auto const & l : links())
//   {
//     std::string const & parentJointName = l->parentJointName();
//     auto jtf = joint(parentJointName)->transform();
//     // This is the transform from i->cmm frame: g_ci = g_sc ^ (-1) * g_si
//     Iso3d const & tfInv = jtf.inverse() * cmmtf_;
//     // Momentum transform from the parent joint frame to the CMM frame
// 
//     MatrixXd bodyJac;
//     bool fullSize = true;
//     computeBodyJacobian_(parentJointName, bodyJac, fullSize);
//     MatrixXd const & lmJac = l->localInertiaTensor() * bodyJac;
//     cmm += metaAdgTMatrix(tfInv) * lmJac;
//   }
// 
//   return cmm;
// }



void KinematicTree::CCNCalc(VectorXd & cc)
{
  Vector6d iniAcc = Vector6d::Zero();
  iniAcc(2) = -GRAVITY;

  cc = VectorXd::Zero(dof());

  // Apply the ready-made link velocities.
  std::vector<Vector6d> linkAccs, linkForces;
  size_t tree_size = graph()->articulatedJoints().size();
  // Add 1 to account for the floating-base link 
  linkAccs.resize(tree_size + 1);
  linkForces.resize(tree_size + 1);

  auto parentJoint = [&](int i){ 
	  if(i == 0)
	    return -1;
	  else
	  {
	    std::string const & pjName = joint(i)->parentLink()->parentJointName();
	    return jointNameToIndex(pjName);
	  }
  };

  for(size_t idx= 0; idx < graph()->articulatedJoints().size(); idx++)
  { // Loop from root to leaf

    // #1 joint index, 
    int ii = articulatedJoints()[static_cast<size_t>(idx)].first;
    // #2 index in the inertia matrix
    // int aidx = articulatedJoints()[static_cast<size_t>(idx)].second;

    auto j = joint(ii);
    std::shared_ptr<GeoLink> const & l = j->childLink();

    int plIdx = parentJoint(ii);
    Vector6d link_vel = l->bodyVelocity();
    Matrix6d velP2b;
    Vector6d link_acc = cross(link_vel, j->jointVelocity());

    if(plIdx >= 0)  
    {
      size_t t_plIdx = static_cast<size_t>(plIdx);
      // There is parent joint/link
      std::shared_ptr<GeoLink> const & pl = link(plIdx);
      std::shared_ptr<GeoJoint> const & pj = joint(plIdx);
      velP2b = metaAdgInvMatrix(pj->transform().inverse() * j->transform());
      // velP2b = velTransform(pj->name(), j->name());
      link_vel += velP2b * pl->bodyVelocity();
      link_acc += velP2b * linkAccs[t_plIdx];
    }else if(plIdx == -1)
    { // Only applies to the root link. It should propagates to the leaf.
      link_acc += iniAcc;
    }
    linkAccs[idx] = link_acc;
    linkForces[idx] =
        l->localInertiaTensor() * link_acc + dualCross(l->bodyVelocity(), l->localInertiaTensor() * l->bodyVelocity());
  } // End of the root to leaf loop

  auto l0 = joint(0)->childLink();
  linkForces[0] =
        l0->localInertiaTensor() * iniAcc + dualCross(l0->bodyVelocity(), l0->localInertiaTensor() * l0->bodyVelocity());

  for(int idx = (static_cast<int>(graph()->articulatedJoints().size()) - 1); idx >= 0; idx--)
  {
    // #1 joint index, 
    int ii = articulatedJoints()[static_cast<size_t>(idx)].first;
    // #2 index in the inertia matrix
    int aidx = articulatedJoints()[static_cast<size_t>(idx)].second;

    auto j = joint(ii);

   // 1. Evaluate C_i
    cc(aidx) = j->bodyTC().transpose() * linkForces[ii];

   // 2. Evaluate f_parent_of_i
    int plIdx = parentJoint(ii);
    if(plIdx >= 0)
    {
	auto piFrameName = link(plIdx)->parentJointName();
	auto iFrameName = link(ii)->parentJointName();
	size_t t_plIdx = static_cast<size_t>(plIdx);
      // std::shared_ptr<GeoLink> & pl = links_[t_plIdx];
      linkForces[t_plIdx] += wrenchTransform(iFrameName, piFrameName) * linkForces[ii];
    }
    
  } // End of leaf to root loop.

  // Evaluate C_0: 
  cc.head(6) =  linkForces[0];

}

Surface::Surface(std::string const & name,
                         std::string const & parentJointName,
                         Iso3d const & tf,
                         std::shared_ptr<KinematicTree const> const r)
: name_(name), parentJointName_(parentJointName), relativeTf_(tf), robot_(r)
{

  robot()->odArticulatedJoints(joints_, robot()->joint(parentJointName)->childLink()->name());

  dof_ = static_cast<int>(joints_.size());
  aDof_ = dof_;

  if(robot()->isFloatingBase())
  {
    dof_ += 6; 
    joints_.push_front(0); // Count the floating-base joint.
  }

  cmmtf_ = Iso3d::Identity();

  ci_.setZero();
  cm_.setZero();

  cmm_ = MatrixXd::Zero(6, dof());

  dq_ = VectorXd::Zero(dof());

  bodyJac_ = MatrixXd::Zero(6, dof());

  twistTransform_ = metaAdgInvMatrix(tf);

  // RoboticsUtils::quickHL("Initialize surface chain: ", name_);
  for (auto i:joints())
  {
    auto l = robot()->link(i);
    // RoboticsUtils::quickError("link ", l->name(), " has mass: ", l->mass());
    weight_ += l->mass();
  }

  update();

  RoboticsUtils::quickInfo("Created Surface: " + name_ + " with parent joint: " + parentJointName_ + " and mass of ", mass(), " Kgs");
}

void Surface::updateCentroidalFrame_()
{

   // 1. The centroidal frame transform and read the joint velocities
   com_.setZero();
   int count_ = 0;
   for (auto i:joints())
   {
     auto currentLink = robot()->link(i);
     double weight = currentLink->mass() / mass();

     auto pj = robot()->joint(currentLink->parentJointName());
     const Vector3d & inertialCom = pj->transform() * currentLink->localCOM();
     // RoboticsUtils::quickPrint("Link: "+ currentLink->name() +"'s COM: ", inertialCom);
     com_ += weight * inertialCom;

     if(pj->jointType() == JointType::FLOATING)
     {
       dq_.segment<6>(count_) = pj->jointVelocity();
     }
     else
     {
       dq_(count_) = pj->dq(); 
     }
     count_ += pj->dof();
   }

   cmmtf_.translation() = com();
   // 2. centroidal inertia
  //RoboticsUtils::quickInfo("The robot: ", name(), " has ", links().size(), " geoLinks.");
  //RoboticsUtils::quickInfo("The mass is: ", mass()); 
  //RoboticsUtils::quickPrint("COM: ", com()); 

  ci_.setZero();
  for (auto i:joints())
   {
     auto pj = robot()->joint(i);
     auto l = robot()->link(i);
     auto tf = pj->transform();
     ci_ += l->spatialInertia(cmmtf_.inverse() * tf);
   }

  // 3. CMM
  cmm_.setZero();

  // CMM frame transform:
  for (auto i:joints())
  {
    auto pj = robot()->joint(i);
    auto l = robot()->link(i);
    //const std::string & parentJointName = l->parentJointName();
    auto jtf = pj->transform();
    // This is the transform from i->cmm frame: g_ci = g_sc ^ (-1) * g_si
    Iso3d const & tfInv = jtf.inverse() * cmmtf_;

    // Momentum transform from the parent joint frame to the CMM frame

    MatrixXd bodyJac;
    bodyJac = robot()->computeBodyJacobian(pj->name(), false);

    MatrixXd const & lmJac = l->localInertiaTensor() * bodyJac;
    long cols = lmJac.cols();
    cmm_.block(0, 0, 6, cols) += metaAdgTMatrix(tfInv) * lmJac;

  }
  // 4. AV
  cm_ = cmm_ * jointVelocities();
  av_ = ci_.inverse() * cm_; 
  
}

void Surface::update()
{

  tf_ = robot()->joint(parentJointName_)->transform() *relativeTransform(); 

  // RoboticsUtils::quickPrint("tf: ", tf_.matrix());

  bodyJac_.noalias() = twistTransform_ * robot()->computeBodyJacobian(parentJointName_, false);
  // RoboticsUtils::quickPrint("twistTransform: ", twistTransform_.matrix());

  // RoboticsUtils::quickPrint("bodyJacobian: ", bodyJac_);

  //const Vector6d & bodyVel = bodyJacobian() * robot()->jointVelocities().head(dof());
  Vector6d const & bodyVel = bodyJacobian() * robot()->jointVelocities();

  pVelBody_ = bodyVel.head(3);
  pVel_ = tf_.rotation() * pVelBody_;

  aVelBody_ = bodyVel.tail(3);
  aVel_ = tf_.rotation() * bodyVel.tail(3);

  updateCentroidalFrame_();
}

std::shared_ptr<GeoJoint> const Surface::parentJoint() const
{
  return robot()->joint(parentJointName_);
}

std::shared_ptr<GeoLink> const Surface::parentLink() const
{
  //return robot()->joint(parentJointName_)->childLink();
  return parentJoint()->childLink();
}
} // namespace GeoRobotics
