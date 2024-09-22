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
#include <GeometricRobotics/Robot.h>

namespace GeoRobotics
{

Robot::Robot(std::shared_ptr<Model> model) : modelPtr_(model)
{
  RoboticsUtils::quickHL("The URDF file of the Robot: '" + model->name() + "' has been parsed.");
  RoboticsUtils::quickInfo("There are ", getModel()->getJoints().size(), " joints.");
  RoboticsUtils::quickHL("There are ", getModel()->getLinks().size(), " links.");

  // for(auto link : getModel()->getLinks())
  // {
  //   std::cout << "link: " << link.first << ", " << link.second->name << std::endl;
  // }

  RoboticsUtils::quickError("Created the Robot: " + model->name());
}

std::shared_ptr<KinematicTree> const & Robot::createFloatingBaseKinematicTree(std::string const & treeName,
                                                                              std::string const & rootLinkName,
                                                                              Iso3d const & initialTf)
{
  RoboticsUtils::quickInfo("Start the function: createFloatingBaseKinematicTree, the root link is: " + rootLinkName);

  // auto rootJoint = floatingBaseJoint(rootLinkName, initialTf);

  RoboticsUtils::quickInfo("Prepared the containers for creating a kinematic tree");

  RoboticsUtils::quickInfo("The model has joints: ", getModel()->getJoints().size());
  RoboticsUtils::quickInfo("The model has links: ", getModel()->getLinks().size());

  trees_.insert(std::make_pair(treeName, std::make_shared<KinematicTree>(getModel(), rootLinkName, treeName, true)));

  RoboticsUtils::quickInfo("Inserted the kinematicTree: " + treeName, ", whose has DOF: ", trees_[treeName]->dof());
  RoboticsUtils::quickInfo("There are ", trees_.size(), " trees.");
  return trees_[treeName];

} // End of createKinematicTree

/*
std::shared_ptr<Joint> Robot::floatingBaseJoint(const std::string & rootLinkName, const Iso3d & initialTf)
{
  std::shared_ptr<Joint> fbJ;
  fbJ = std::make_shared<Joint>();
  fbJ->type = JointType::FLOATING;
  fbJ->name = "Floating-base joint";
  fbJ->childLinkName = rootLinkName;
  fbJ->transform.position << initialTf.translation().x(), initialTf.translation().y(), initialTf.translation().z();
  auto q = Eigen::Quaterniond(initialTf.rotation());
  fbJ->transform.rotation.setFromQuaternion(q.x(), q.y(), q.z(), q.w());

  return fbJ;
}
*/

} // namespace GeoRobotics
