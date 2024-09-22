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
#include "GeometricRobotics/urdf/Model.h"

namespace GeoRobotics
{

void Model::initTree(std::unordered_map<std::string, std::string> & parent_link_tree)
{
  // Loop through all joints. The parent/child link names are already filled.
  // For every link, assign the pointers: children links and children joints
  for(auto & joint : joints_)
  {
    std::string parent_link_name = joint.second->parentLinkName;
    std::string child_link_name = joint.second->childLinkName;

    if(parent_link_name.empty() || child_link_name.empty())
    {
      RoboticsUtils::throw_runtime_error("Joint [" + joint.second->name
                                             + "] is missing a parent and/or child link specification.",
                                         __FILE__, __LINE__);
    }
    else
    {
      // find child and parent links
      std::shared_ptr<Link> child_link, parent_link;
      this->getLink(child_link_name, child_link);
      if(!child_link)
      {
        RoboticsUtils::throw_runtime_error(
            "child link [" + child_link_name + "] of joint [" + joint.first + "] not found", __FILE__, __LINE__);
      }
      this->getLink(parent_link_name, parent_link);
      if(!parent_link)
      {
        RoboticsUtils::throw_runtime_error(
            "parent link [" + parent_link_name + "] of joint [" + joint.first
                + "] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint "
                  "needs to be explicitly defined in the robot description. To fix this problem you can either remove "
                  "this joint ["
                + joint.first + "] from your urdf file, or add \"<link name=\"" + parent_link_name
                + "\" />\" to your urdf file.",
            __FILE__, __LINE__);
      }

      // set parent link for child link
      child_link->setParentLink(parent_link);

      // set parent joint for child link
      child_link->setParentJoint(joint.second);

      // set child joint for parent link
      parent_link->addChildJoint(joint.second);

      // set child link for parent link
      parent_link->addChildLink(child_link);

      // fill in child/parent string map
      parent_link_tree[child_link->name] = parent_link_name;
    }
  }
}

void Model::initRoot(const std::unordered_map<std::string, std::string> & parent_link_tree)
{
  rootLink_.reset();

  // find the links that have no parent in the tree
  for(auto & l : links_)
  {
    std::unordered_map<std::string, std::string>::const_iterator parent = parent_link_tree.find(l.first);
    if(parent == parent_link_tree.end())
    {
      // store root link
      if(!rootLink_)
      {
        getLink(l.first, rootLink_);
      }
      // we already found a root link
      else
      {
        RoboticsUtils::throw_runtime_error("Two root links found: [" + rootLink_->name + "] and [" + l.first + "]",
                                           __FILE__, __LINE__);
      }
    }
  }
  if(!rootLink_)
  {
    RoboticsUtils::throw_runtime_error("No root link found. The robot xml is not a valid tree.", __FILE__, __LINE__);
  }
}

void Model::merge()
{

  std::string rootNode = rootLink_->name;
  // std::string rootJoint = link(0)->parentJointName();

  std::unordered_set<std::string> visited;
  // Check all the joints with a DFS search
  visitDFS_(rootNode,
            // rootJoint,
            visited);
}
void Model::visitDFS_(const std::string & currentNode,
                      // const std::string & currentJoint,
                      std::unordered_set<std::string> & visited)
{
  // 1. Merge the link
  visited.insert(currentNode);

  // 2. Check the adjacent nodes of the currentNode
  auto adjList = getLink(currentNode)->childrenJoints();
  for(auto j : adjList)
  {

    // If not yet visited
    if(visited.find(j->childLinkName) == visited.end())
    {
      visitDFS_(j->childLinkName, visited);
    }
  }
  // No child any more.
  // Merge the child link into the parent link if the parent joint is a fixed joint.

  RoboticsUtils::quickHL("Processing the link: ", currentNode);

  if(currentNode != rootLink_->name)
  {
    mergeLink_(currentNode);
  }
}

void Model::mergeLink_(const std::string & linkName)
{

  if(getLink(linkName)->parentJoint()->type != JointType::FIXED)
  {
    return;
  }
  else
  {
    RoboticsUtils::quickHL("Merging the fixed joint: ", getLink(linkName)->parentJoint()->name);
    mergeFixedJoint_(getLink(linkName)->parentJoint());
  }
}

void Model::mergeFixedJoint_(std::shared_ptr<Joint> joint)
{
  // 1. Merge the parent and child link

  // Parent link
  std::shared_ptr<Link> pl = getLink(joint->parentLinkName);
  // Child link
  std::shared_ptr<Link> cl = getLink(joint->childLinkName);

  // a. Keep the parent joint of the parent link

  // Nothing

  // b. Add the child link joints to the parent link
  for(auto clj : cl->childrenJoints())
  {
    // b1. Update the parent link of each joint
    clj->parentLinkName = pl->name;

    // b2. Update the transform of each joint: g_j = g_p * g_j.
    Eigen::Isometry3d newTransform = joint->transform.toMatrix() * clj->transform.toMatrix();
    clj->transform.position = newTransform.translation();
    Eigen::Quaterniond q(newTransform.linear());
    clj->transform.rotation.setFromQuaternion(q.x(), q.y(), q.z(), q.w());

    // b3. Keep the axis
    // Nothing
  }

  // c. Add the child link inertia matrix

  if(cl->metaInertia->nonZeroMass())
  {
    mergeInertiaMatrix_(pl->metaInertia, cl->metaInertia, joint);
  }

  // Matrix6d plIM = pl->metaInertia->constructInertiaTensor();
  // Matrix6d clIM = cl->metaInertia->constructInertiaTensor();

  // Vel transform from parent COM to child COM.
  // const Matrix6d & velTF = metaAdgMatrix(g_pc_cc);

  //// Add the transformed inertia matrix to the parent inertia matrix
  // plIM += velTF.transpose() * clIM* velTF;

  // d. Remove the current joint from the parent link

  // pl->metaInertia += cl->metaInertia;
  pl->removeChildJoint(joint);

  // e. Remove the current joint and the merged child link from the Model.
  removeLink_(cl->name);
  removeJoint_(joint->name);
}

void Model::mergeInertiaMatrix_(std::shared_ptr<MetaInertia> pIM,
                                std::shared_ptr<MetaInertia> cIM,
                                const std::shared_ptr<Joint> & j)
{
  Iso3d g_pc, g_j, g_pcc;
  // COM frame in the parent link
  g_pc = pIM->origin.toMatrix();
  // Joint frame in the parent link
  g_j = j->transform.toMatrix();
  // Child COM frame in the parent link
  g_pcc = j->transform.toMatrix() * cIM->origin.toMatrix();
  // Child COM frame in the parent COM frame
  // g_pc_cc =  g_pc.inverse() * g_pcc;

  // 1. Compute the New COM

  Iso3d new_com_tf = Iso3d::Identity();

  double total_mass = pIM->mass + cIM->mass;

  new_com_tf.translation() =
      pIM->origin.position * (pIM->mass / total_mass) + g_pcc.translation() * (cIM->mass / total_mass);

  // 2. Compute the Centroidal Inertia matrix
  Matrix6d centroidalInertia = Matrix6d::Zero();

  centroidalInertia += pIM->spatialInertia(new_com_tf.inverse() * g_pc);
  centroidalInertia += cIM->spatialInertia(new_com_tf.inverse() * g_pcc);

  // 3. Set the inertia according to centroidal inertia.
  // centroidalInertia should be block-diagonal.

  MatrixXd diff = centroidalInertia.block<3, 3>(0, 0) - total_mass * Matrix3d::Identity();

  double threshold = 0.001;

  if(diff.norm() > threshold)
  {
    RoboticsUtils::throw_runtime_error("The mass matrix is not an identity matrix!", __FILE__, __LINE__);
  }

  MatrixXd rdiff1 = centroidalInertia.block<3, 3>(3, 0);
  MatrixXd rdiff2 = centroidalInertia.block<3, 3>(0, 3);

  if(rdiff1.norm() > threshold)
  {
    RoboticsUtils::throw_runtime_error("The off-diagonal matrix one is not all-zero!", __FILE__, __LINE__);
  }
  if(rdiff2.norm() > threshold)
  {
    RoboticsUtils::throw_runtime_error("The off-diagonal matrix two is not all-zero!", __FILE__, __LINE__);
  }

  MatrixXd rdiff = centroidalInertia.block<3, 3>(3, 3) - centroidalInertia.block<3, 3>(3, 3).transpose();
  if(rdiff.norm() > threshold)
  {
    RoboticsUtils::throw_runtime_error("The rotational inertia is not symmetric!", __FILE__, __LINE__);
  }

  pIM->origin.position = new_com_tf.translation();
  pIM->mass = total_mass;
  pIM->ixx = centroidalInertia(3, 3);
  pIM->ixy = centroidalInertia(3, 4);
  pIM->ixz = centroidalInertia(3, 5);
  pIM->iyy = centroidalInertia(4, 4);
  pIM->iyz = centroidalInertia(4, 5);
  pIM->izz = centroidalInertia(5, 5);
}

void Model::removeJoint_(const std::string & name)
{
  auto j = joints_.find(name);

  if(j != joints_.end())
  {
    joints_.erase(j);
  }
  else
  {
    RoboticsUtils::throw_runtime_error("Failed to find the link: " + name + " from the Model: " + name_, __FILE__,
                                       __LINE__);
  }
}

void Model::removeLink_(const std::string & name)
{
  auto l = links_.find(name);

  if(l != links_.end())
  {
    links_.erase(l);
  }
  else
  {
    RoboticsUtils::throw_runtime_error("Failed to find the link: " + name + " from the Model: " + name_, __FILE__,
                                       __LINE__);
  }
}

} // namespace GeoRobotics
