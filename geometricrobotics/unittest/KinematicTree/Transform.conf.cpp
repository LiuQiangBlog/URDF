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
#include "GeometricRobotics/Robot.h"
#include "GeometricRobotics/urdf/Parser.h"
#include "gtest/gtest.h"
#include <RoboticsUtils/utils.h>
#include <limits>

namespace GeoRobotics
{

class Transform : public testing::Test
{
protected:
  Parser parser;

  std::shared_ptr<Model> hrpModelPtr;
  std::shared_ptr<Robot> hrp;
  std::shared_ptr<KinematicTree> hTree;

  std::shared_ptr<Model> hrpMergeModelPtr;
  std::shared_ptr<Robot> hrpMerge;
  std::shared_ptr<KinematicTree> hTreeMerge;

  std::shared_ptr<Model> animalModelPtr;
  std::shared_ptr<Robot> animal;
  std::shared_ptr<KinematicTree> aTree;

  int num_iterations = 1000;
  double tf_error = 1e-3;

  double jd_step_size = 0.0002; // Amounts to maximum 11.4594 degree per second
  double dt = 0.001;

  std::vector<std::shared_ptr<KinematicTree>> robots;

  void SetUp() override
  {
    // hrpModelPtr = parser.parseURDFFile("/home/yuquan/catkin_ws/src/hrp4/hrp4_description/urdf/hrp4.urdf");
    // hrp = std::make_shared<Robot>(hrpModelPtr);
    // hTree = hrp->createFloatingBaseKinematicTree("hrp", "base_link");
    // robots.push_back(hTree);

    hrpMergeModelPtr = parser.parseURDFFile("/home/yuquan/catkin_ws/src/hrp4/hrp4_description/urdf/hrp4.urdf");
    hrpMergeModelPtr->merge();
    hrpMerge = std::make_shared<Robot>(hrpMergeModelPtr);
    hTreeMerge = hrpMerge->createFloatingBaseKinematicTree("hrp", "base_link");
    hTreeMerge->initializeFrames();
    robots.push_back(hTreeMerge);

    animalModelPtr = parser.parseURDFFile("@RobotExamplesPath@Animal.urdf");
    animalModelPtr->merge();
    animal = std::make_shared<Robot>(animalModelPtr);
    aTree = animal->createFloatingBaseKinematicTree("animal", "base");
    aTree->initializeFrames();
    robots.push_back(aTree);
  }

  void TearDown() override {}

  void checkTransform(const std::shared_ptr<KinematicTree> & robot,
                      std::unordered_set<std::string> & visited,
                      const std::string & currentNode,
                      const std::string & currentJoint,
                      Iso3d & iniTF)
  {
    visited.insert(currentNode);
    auto cj = robot->joint(currentJoint);
    // Increment the initial transform

    if(cj->dof() == 1)
    {
      iniTF = iniTF * cj->incrementalTransform();
      // iniTF = iniTF * cj->relativeTransform();
    }
    // auto currentTF = cj->relativeTransform();
  }

  void transformDFSVisit(const std::shared_ptr<KinematicTree> & robot,
                         const std::string & currentNode,
                         const std::string & currentJoint,
                         Iso3d & iniTF,
                         std::unordered_set<std::string> & visited)
  {

    checkTransform(robot, visited, currentNode, currentJoint, iniTF);

    // 2. Check the adjacent nodes of the currentNode
    auto adjList = robot->graph()->adjacencyList();
    for(auto & step : adjList[currentNode])
    {
      const std::string & nextNode = step.first;
      const std::string & nextJoint = step.second;
      // If not yet visited
      if(visited.find(nextNode) == visited.end())
      {
        // Insert the edge and the node
        transformDFSVisit(robot, nextNode, nextJoint, iniTF, visited);
      }
    }

    // Reached the end of a chain:
    auto currentLink = robot->link(currentNode);
    auto pj = robot->joint(currentLink->parentJointName());

    if(pj->dof() == 1)
    {
      // RoboticsUtils::quickError("Checking articulated joint: ", pj->name(),
      //                          " idx: ", robot->jointNameToIndex(pj->name()));
      // RoboticsUtils::quickInfo("Updated transform: ", pj->transform().matrix());
      // RoboticsUtils::quickInfo("Manual transform: ", iniTF.matrix());

      const MatrixXd & diff = pj->transform().matrix() - (iniTF * pj->initialTransform()).matrix();
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);

      // auto currentTF = pj->incrementalTransform();
      iniTF = iniTF * pj->incrementalTransform().inverse();
      // iniTF = iniTF * pj->relativeTransform().inverse();
    }
  }
  void test_comJac(std::shared_ptr<KinematicTree> & robot)
  {
    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {

      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);

      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      // std::string jName = robot->randomJointName();

      // RoboticsUtils::quickHL("Checking joint: ", jName);

      Vector3d com = robot->com();

      Vector3d com_frame_vec = robot->frame("com")->transform().translation();

      ASSERT_NEAR((com - com_frame_vec).norm(), 0.0, tf_error);

      Vector3d comd = robot->computeComJac() * robot->jointVelocities();
      Vector3d comd_two = robot->frame("com")->computeBodyJacobian(true) * robot->jointVelocities();

      // RoboticsUtils::quickPrint("com: ", com);
      // RoboticsUtils::quickPrint("comd: ", comd);

      robot->setJointPositions(qNext);

      Vector3d delta_com = robot->com();

      ASSERT_NEAR((delta_com - robot->frame("com")->transform().translation()).norm(), 0.0, tf_error);

      // RoboticsUtils::quickPrint("next com: ", delta_com);

      MatrixXd numerical_comd = (delta_com - com) / dt;

      // RoboticsUtils::quickPrint("numerical cmmd: ", numerical_comd);

      auto diff = numerical_comd - comd;

      ASSERT_NEAR(diff.norm(), 0.0, 0.06);

      auto diff_two = numerical_comd - comd_two;

      ASSERT_NEAR(diff_two.norm(), 0.0, 0.06);
    }

  } // end of test_comJac

  void test_computeBodyVel(std::shared_ptr<KinematicTree> & robot)
  {
    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    // for(int i = 0; i < 20; i++)
    {
      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);

      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      // std::string refName = robot->randomFrameName();
      std::string refName = robot->randomJointName();
      std::string bodyName = robot->randomFrameName();

      Iso3d const tf = robot->transformTwo(refName, bodyName);

      // RoboticsUtils::quickHL("Reference name: ", refName);
      // RoboticsUtils::quickHL("body name: ", bodyName);

      Vector6d bv = robot->computeBodyVel(refName, bodyName);

      // RoboticsUtils::quickPrint("body v: ", bv.head(3));

      robot->setJointPositions(qNext);

      Iso3d const delta_tf = robot->transformTwo(refName, bodyName);

      MatrixXd numerical_tfd = (delta_tf.matrix() - tf.matrix()) / dt;

      Vector3d numerical_v = tf.linear().transpose().matrix() * numerical_tfd.block<3, 1>(0, 3);

      // RoboticsUtils::quickPrint("numerical v: ", numerical_v);

      Vector3d diff = numerical_v - bv.head(3);

      ASSERT_NEAR(diff.norm(), 0.0, 0.03);

      // RoboticsUtils::quickError("difference: ", diff.norm());
    }

  } // end of test_computeBodyVel

  void test_cmmd(std::shared_ptr<KinematicTree> & robot)
  {
    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {

      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);

      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      MatrixXd cmm = robot->computeCMM();
      // MatrixXd cmm_two = robot->computeCMMTwo();

      MatrixXd cmmd = robot->computeCMMDerivative();

      Vector3d comd_cmm = cmm.block(0, 0, 3, robot->dof()) * robot->jointVelocities() / robot->mass();
      Vector3d comd = robot->computeComJac() * robot->jointVelocities();

      auto diff = comd_cmm - comd;

      ASSERT_NEAR(diff.norm(), 0.0, tf_error);

      const Matrix6d & ci = robot->computeCentroidalInertia();
      const Vector6d & av = ci.colPivHouseholderQr().solve(cmm * dq);

      ASSERT_NEAR((comd - av.head(3)).norm(), 0.0, tf_error);

      robot->setJointPositions(qNext);

      MatrixXd delta_cmm = robot->computeCMM();

      MatrixXd numerical_cmmd = (delta_cmm - cmm) / dt;

      auto diff_two = cmmd - numerical_cmmd;
      if(diff_two.norm() > tf_error)
      {
        RoboticsUtils::quickPrint("cmmd: ", cmmd);

        RoboticsUtils::quickPrint("numerical cmmd: ", numerical_cmmd);
      }
      // RoboticsUtils::quickError("difference: ", (cmmd - numerical_cmmd).norm());
      ASSERT_NEAR(diff_two.norm(), 0.0, tf_error);
    }

  } // end of test_cmmd

  void test_fullsize_jacobian(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    for(int i = 0; i < num_iterations; i++)
    {

      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(1, re);

      std::string jName = robot->randomJointName();

      // RoboticsUtils::quickHL("Checking joint: ", jName);

      auto j = robot->joint(jName);

      VectorXd qd_compact;
      robot->readJointVelocities(qd_compact, jName);

      Vector6d vel_compact = robot->computeBodyJacobian(jName, false) * qd_compact;

      Vector6d vel = robot->computeBodyJacobian(jName, true) * robot->jointVelocities();

      auto diff = vel_compact - vel;
      // RoboticsUtils::quickPrint("numerical w: ", numerical_w);
      // RoboticsUtils::quickPrint("jac w: ", skew(bodyVel.tail(3)));

      ASSERT_NEAR(diff.norm(), 0.0, tf_error);
    }

  } // end of test_transform

  void test_jacobiand(std::shared_ptr<KinematicTree> & robot)
  {
    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 200; i++)
    {

      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);

      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      std::string jName = robot->randomJointName();

      // RoboticsUtils::quickHL("Checking joint: ", jName);

      auto j = robot->joint(jName);
      // Iso3d tf = j->transform();
      MatrixXd jac = robot->computeBodyJacobian(jName, false);
      MatrixXd full_jac = robot->computeBodyJacobian(jName, true);

      MatrixXd jacd = robot->computeBodyJacobianDerivative(jName, false);
      MatrixXd full_jacd = robot->computeBodyJacobianDerivative(jName, true);

      // RoboticsUtils::quickPrint("Jacd: ", jacd);

      robot->setJointPositions(qNext);

      MatrixXd delta_jac = robot->computeBodyJacobian(jName, false);
      MatrixXd full_delta_jac = robot->computeBodyJacobian(jName, true);

      MatrixXd numerical_jacd = (delta_jac.matrix() - jac.matrix()) / dt;
      MatrixXd full_numerical_jacd = (full_delta_jac.matrix() - full_jac.matrix()) / dt;

      // RoboticsUtils::quickPrint("numerical Jacd: ", numerical_jacd);
      // RoboticsUtils::quickPrint("numerical full Jacd: ", full_numerical_jacd);

      auto diff = numerical_jacd - jacd;
      auto full_diff = full_numerical_jacd - full_jacd;
      // RoboticsUtils::quickPrint("numerical w: ", numerical_w);
      // RoboticsUtils::quickPrint("jac w: ", skew(bodyVel.tail(3)));

      ASSERT_NEAR(diff.norm(), 0.0, tf_error);
      ASSERT_NEAR(full_diff.norm(), 0.0, tf_error);
    }

  } // end of test_transform

  void test_pointvel(std::shared_ptr<KinematicTree> & robot)
  {
    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {

      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);

      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      std::string lName = robot->randomLinkName();

      // RoboticsUtils::quickHL("Checking link: ", lName);

      Vector3d point = grVec3d(0.0, 0.2, false);
      Iso3d p_tf = Iso3d::Identity();
      p_tf.translation() = point;

      std::shared_ptr<JointDependentFrame> l = std::dynamic_pointer_cast<JointDependentFrame>(robot->frame(lName));

      Iso3d tf = l->transform() * p_tf;
      // RoboticsUtils::quickPrint("Point", point);
      // RoboticsUtils::quickPrint("Point tf", p_tf.matrix());

      Vector3d bodyVel = l->computePointVelocity(point);

      // RoboticsUtils::quickPrint("Jac vel: ", bodyVel);

      robot->setJointPositions(qNext);

      Iso3d delta_tf = l->transform() * p_tf;

      MatrixXd numerical_tfd = (delta_tf.matrix() - tf.matrix()) / dt;

      Vector3d numerical_vel = numerical_tfd.block<3, 1>(0, 3);

      // RoboticsUtils::quickPrint("numerical vel: ", numerical_vel);
      // RoboticsUtils::quickPrint("bodyVel: ", bodyVel);

      ASSERT_NEAR((numerical_vel - bodyVel).norm(), 0.0, tf_error);
    }

  } // end of test_transform

  void test_jacobian(std::shared_ptr<KinematicTree> & robot)
  {
    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 300; i++)
    {

      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);

      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      std::string jName = robot->randomJointName();
      if(jName == "R_F42")
      {
        continue;
      }
      if(jName == "R_F32")
      {
        continue;
      }

      if(jName == "R_KNEE_P")
      {
        continue;
      }
      if(jName == "L_HAND_J0")
      {
        continue;
      }
      if(jName == "R_HAND_J0")
      {
        continue;
      }

      // RoboticsUtils::quickHL("Checking joint: ", jName);

      auto j = robot->joint(jName);
      Iso3d tf = j->transform();
      Vector6d bodyVel = robot->computeBodyJacobian(jName, true) * robot->jointVelocities();

      // RoboticsUtils::quickPrint("Jac vel: ", bodyVel);

      robot->setJointPositions(qNext);

      Iso3d delta_tf = j->transform();

      MatrixXd numerical_tfd = (delta_tf.matrix() - tf.matrix()) / dt;

      auto numerical_v = tf.linear().transpose().matrix() * numerical_tfd.block<3, 1>(0, 3);

      // RoboticsUtils::quickPrint("numerical v: ", numerical_v);

      auto numerical_Rd = numerical_tfd.block<3, 3>(0, 0);

      auto numerical_w =
          tf.linear().inverse().matrix() * numerical_Rd; // body w = R^-1 * Rd, see eq.~(2.49) of Murray's book

      // RoboticsUtils::quickPrint("numerical w: ", numerical_w);
      // RoboticsUtils::quickPrint("jac w: ", skew(bodyVel.tail(3)));

      auto diff = (numerical_v - bodyVel.head(3));
      if(diff.norm() > 0.04)
      {
        RoboticsUtils::quickPrint("numerical v: ", numerical_v);
        RoboticsUtils::quickPrint("body linear vel: ", bodyVel.head(3));

        RoboticsUtils::quickPrint("Body Jacobian: ", robot->computeBodyJacobian(jName, false));
        VectorXd qd_compact;
        robot->readJointVelocities(qd_compact, jName);
        RoboticsUtils::quickPrint("Dependent joint velocity: ", qd_compact);

        RoboticsUtils::quickPrint("compact Body Jacobian vel: ", robot->computeBodyJacobian(jName, false) * qd_compact);
      }

      ASSERT_NEAR(diff.norm(), 0.0, 0.04);
      ASSERT_NEAR((numerical_w - skew(bodyVel.tail(3))).norm(), 0.0, 0.04);
    }

  } // end of test_transform

  void test_transform(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    for(int i = 0; i < num_iterations; i++)
    {

      robot->setRandom(2, re);

      std::string rootNode = robot->link(0)->name();
      std::string rootJoint = robot->link(0)->parentJointName();

      Iso3d iniTF = Iso3d::Identity();
      if(robot->isFloatingBase())
      {
        iniTF = robot->joint(0)->transform();
      }
      std::unordered_set<std::string> visited;
      // Check all the joints with a DFS search
      transformDFSVisit(robot, rootNode, rootJoint, iniTF, visited);
    }

  } // end of test_transform

  void testRENA(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    // for(int i = 0; i < num_iterations; i++)
    // for(int i = 0; i < 100; i++)
    {

      robot->setRandom(2, re);

      robot->IdCalc(robot->jointPositions(), robot->jointVelocities(), robot->jointAccelerations());
    }

  } // end of testRENA_transform

  // Test GeoFrame
  // void test_GeoFrame(std::shared_ptr<KinematicTree> & robot)
  // {
  //   std::default_random_engine re;
  //   std::vector<std::shared_ptr<GeoFrame>> frames;
  //   RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), "
  //   joints");
  //   // Initialize the JointDependentFrames
  //   for(auto & j : robot->joints())
  //   {
  //     // RoboticsUtils::quickPrint("Joint name: ", j->name());

  //     if(j->dof() != 1)
  //     {
  //       continue;
  //     }

  //     Iso3d relativeTf = grTF(-0.2, 0.2);
  //     Iso3d relativeTf_two = grTF(-0.2, 0.2);
  //     Iso3d relativeTf_three = grTF(-0.2, 0.2);

  //     // RoboticsUtils::quickPrint("Relative transform: ", relativeTf.matrix());

  //     std::shared_ptr<JointDependentFrame> newFrame =
  //         std::make_shared<JointDependentFrame>(j->name() + "_test_frame", j->name(), relativeTf, robot);
  //     std::shared_ptr<JointDependentFrame> newFrame_two =
  //         std::make_shared<JointDependentFrame>(j->name() + "_test_frame_two", j->name(), relativeTf_two, robot);
  //     std::shared_ptr<JointDependentFrame> newFrame_three =
  //         std::make_shared<JointDependentFrame>(j->name() + "_test_frame_three", j->name(), relativeTf_three, robot);
  //     j->attachObserver(newFrame);
  //     j->attachObserver(newFrame_two);
  //     j->attachObserver(newFrame_three);
  //     j->printObservers();
  //     frames.push_back(newFrame);
  //     frames.push_back(newFrame_two);
  //     frames.push_back(newFrame_three);

  //     frames.push_back(robot->frame(j->name()));
  //   }
  //   for (auto const & l : robot->links())
  //   {
  //     frames.push_back(robot->frame(l->name()));
  //   }

  //   RoboticsUtils::quickInfo("----------Created ", frames.size(), " frames-----------------");
  //   for(int i = 0; i < num_iterations; i++)
  //   {

  //     // RoboticsUtils::quickHL("----------iteration: ", i);
  //     // Set random joint position, velocity, and acceleration
  //     robot->setRandom(2, re);

  //     for(auto & f : frames)
  //     {
  //       // RoboticsUtils::quickError("----------JointFrame name: ", f->name());
  //       Eigen::MatrixXd diff =
  //           f->transform().matrix() - (robot->joint(f->parentJointName())->transform() * f->offset()).matrix();
  //       ASSERT_NEAR(diff.norm(), 0.0, tf_error);
  //     }
  //   }

  // } // end of test_GeoFrame

  // Test ComFrame and FixedFrame transform
  void test_Frames(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;
    RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), " joints");

    for(int i = 0; i < num_iterations; i++)
    {

      // RoboticsUtils::quickHL("----------iteration: ", i);
      // Set random joint position, velocity, and acceleration
      robot->setRandom(2, re);

      // RoboticsUtils::quickError("----------JointFrame name: ", f->name());
      Eigen::MatrixXd diff = robot->frame("com")->transform().matrix() - robot->centroidalTransform().matrix();
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);

      Eigen::MatrixXd diff_fixed = robot->frame("inertial")->transform().matrix() - Iso3d::Identity().matrix();
      ASSERT_NEAR(diff_fixed.norm(), 0.0, tf_error);
    }

  } // end of test_Frames

  void test_bodyvel_calc(std::shared_ptr<KinematicTree> & robot)
  {
    // Verify if the KinematicTree's transform() can match the joint's transform()

    std::default_random_engine re;
    std::vector<std::shared_ptr<JointDependentFrame>> frames;
    // RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), " joints");

    // Test link transforms
    for(int i = 0; i < num_iterations; i++)
    // for(int i = 0; i < 1; i++)
    {

      // RoboticsUtils::quickHL("----------iteration: ", i);
      // Set random joint position, velocity, and acceleration
      robot->setRandom(1, re);

      // auto jSize = robot->articulatedJoints().size();
      // std::string const & jName1 = robot->jointIndexToName(1 + randInt(jSize + 1));
      auto j = robot->joint(robot->randomJointName());
      // auto j = robot->joint("R_WRIST_R");

      auto full_jac = robot->computeBodyJacobian(j->name(), true);
      // RoboticsUtils::quickPrint("Full-size Jacobian", full_jac);
      // RoboticsUtils::quickPrint("Full-size qd", robot->jointVelocities());

      // robot->printOdArticulatedJoints(j->childLink()->name());
      auto compact_jac = robot->computeBodyJacobian(j->name(), false);
      // RoboticsUtils::quickPrint("Compact Jacobian", compact_jac);

      // robot->joint(parentJointName_)->articulatedJoints()
      VectorXd dq;
      robot->readJointVelocities(dq, j->name());

      auto vel1 = compact_jac * dq;

      // RoboticsUtils::quickPrint("compact  qd", dq);

      // RoboticsUtils::quickPrint("compact Jac vel", vel1);

      auto vel2 = full_jac * robot->jointVelocities();

      // RoboticsUtils::quickPrint("fullsize Jac vel", vel2);

      auto diff = vel1 - vel2;
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);
    }
  }

  // void test_bodyvel_calc(std::shared_ptr<KinematicTree> & robot)
  // {
  //   // Verify if the KinematicTree's transform() can match the joint's transform()

  //   std::default_random_engine re;
  //   std::vector<std::shared_ptr<JointDependentFrame>> frames;
  //   // RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), "
  //   joints");

  //   // Test link transforms
  //   for(int i = 0; i < num_iterations; i++)
  //   {

  //     RoboticsUtils::quickHL("----------iteration: ", i);
  //     // Set random joint position, velocity, and acceleration
  //     robot->setRandom(2, re);

  //     // auto jSize = robot->articulatedJoints().size();
  //     // std::string const & jName1 = robot->jointIndexToName(1 + randInt(jSize + 1));
  //     auto f = robot->frame(robot->randomFrameName());

  //     auto vel1 = f->inertialFrameBodyVel();
  //     bool fullSize = false;
  //     MatrixXd const & Ja = robot->computeBodyJacobian(fullSize);
  //     // robot->joint(parentJointName_)->articulatedJoints()
  //     VectorXd dq;
  //     robot->readJointVelocities(dq, parentJointName_);

  //     RoboticsUtils::quickPrint("compact Jac vel", vel1);

  //     bool fullSize = true;
  //     MatrixXd const & Ja = f->computeBodyJacobian(fullSize);

  //     auto vel2 = Ja * robot->jointVelocities();

  //     RoboticsUtils::quickPrint("fullsize Jac vel", vel2);

  //     auto diff = vel1 - vel2;
  //     ASSERT_NEAR(diff.norm(), 0.0, tf_error);
  //   }
  // }

  void test_fb_joint_transform(std::shared_ptr<KinematicTree> & robot)
  {
    // Verify if the KinematicTree's transform() can match the joint's transform()

    std::default_random_engine re;
    std::vector<std::shared_ptr<JointDependentFrame>> frames;
    // RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), " joints");

    // Test link transforms
    for(int i = 0; i < num_iterations; i++)
    {

      // RoboticsUtils::quickHL("----------iteration: ", i);
      // Set random joint position, velocity, and acceleration
      robot->setRandom(2, re);

      // auto jSize = robot->articulatedJoints().size();
      // std::string const & jName1 = robot->jointIndexToName(1 + randInt(jSize + 1));
      std::string const & n1 = robot->joints()[0]->childLink()->name();
      std::string const & jn = robot->randomJointName();
      auto const & j = robot->joint(jn);
      std::string const & n2 = j->childLink()->name();

      // RoboticsUtils::quickError("Checking----------Frame 1: ", n1, " and Frame 2: ", n2);

      auto const & root_tf = robot->joints()[0]->transform();
      // RoboticsUtils::quickPrint("root transform", root_tf.matrix());

      auto tf1 = robot->transform(n1, n2); // * robot->link(n2)->localCOMTransform();
      // RoboticsUtils::quickPrint("transform", tf1.matrix());

      tf1 = root_tf * tf1;
      // RoboticsUtils::quickPrint("root_tf * transform", tf1.matrix());

      // RoboticsUtils::quickError("Checking----------joint: ", jn);
      auto const & tf2 = j->transform();

      // RoboticsUtils::quickPrint("transformTwo", tf2.matrix());
      // robot->joint
      // RoboticsUtils::quickError("----------JointFrame name: ", f->name());
      Eigen::MatrixXd diff = tf1.matrix() - tf2.matrix();
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);
    }
  }

  void test_joint_transform(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;
    std::vector<std::shared_ptr<JointDependentFrame>> frames;
    RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), " joints");

    // Test link transforms
    // for(int i = 0; i < num_iterations; i++)
    for(int i = 0; i < 1; i++)
    {

      // RoboticsUtils::quickHL("----------iteration: ", i);
      // Set random joint position, velocity, and acceleration
      robot->setRandom(2, re);

      // auto jSize = robot->articulatedJoints().size();
      // std::string const & jName1 = robot->jointIndexToName(1 + randInt(jSize + 1));
      std::string const & l1n = robot->randomLinkName();
      auto const & j1 = robot->joint(robot->link(l1n)->parentJointName());
      // std::string const & n1 = robot->joints()[0]->childLink()->name();
      std::string const & j2n = robot->randomJointName();
      auto const & j2 = robot->joint(j2n);
      std::string const & l2n = j2->childLink()->name();

      RoboticsUtils::quickError("Checking----------Frame 1: ", l1n, " and Frame 2: ", l2n);

      // auto const & root_tf = robot->joints()[0]->transform();
      // RoboticsUtils::quickPrint("root transform", root_tf.matrix());

      auto tf1 = robot->transform(l1n, l2n); // * robot->link(n2)->localCOMTransform();
      RoboticsUtils::quickPrint("transform", tf1.matrix());

      // tf1 = root_tf * tf1;
      // RoboticsUtils::quickPrint("root_tf * transform", tf1.matrix());

      RoboticsUtils::quickError("Checking----------joint: ", jn);
      auto const & tf2 = j1->transform().inverse() * j2->transform();

      RoboticsUtils::quickPrint("transformTwo", tf2.matrix());
      // robot->joint
      // RoboticsUtils::quickError("----------JointFrame name: ", f->name());
      Eigen::MatrixXd diff = tf1.matrix() - tf2.matrix();
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);
    }
  }
  // Test different transform approaches
  void test_transform_calculation(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;
    std::vector<std::shared_ptr<JointDependentFrame>> frames;
    RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), " joints");

    // Test link transforms
    for(int i = 0; i < num_iterations; i++)
    {

      // RoboticsUtils::quickHL("----------iteration: ", i);
      // Set random joint position, velocity, and acceleration
      robot->setRandom(2, re);

      // auto jSize = robot->articulatedJoints().size();
      // std::string const & jName1 = robot->jointIndexToName(1 + randInt(jSize + 1));
      // std::string const & n1 = robot->randomLinkName();
      std::string const & n1 = robot->links()[0]->urdfLink()->name;
      std::string const & n2 = robot->randomLinkName();

      RoboticsUtils::quickError("Checking----------Frame 1: ", n1, " and Frame 2: ", n2);

      auto const & tf1 = robot->transform(n1, n2); // * robot->link(n2)->localCOMTransform();

      auto const & j1 = robot->joint(robot->link(n1)->parentJointName())->name();
      auto const & j2 = robot->joint(robot->link(n2)->parentJointName())->name();

      RoboticsUtils::quickError("Checking----------joint 1: ", j1, " and joint 2: ", j2);
      auto const & tf2 = robot->transformTwo(j1, j2);
      RoboticsUtils::quickPrint("transform", tf1.matrix());

      RoboticsUtils::quickPrint("transformTwo", robot->transformTwo(n1, n2).matrix());
      // robot->joint
      // RoboticsUtils::quickError("----------JointFrame name: ", f->name());
      Eigen::MatrixXd diff = tf1.matrix() - tf2.matrix();
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);
    }

  } // end of test_transform_calculation

  // Test JointDependentFrame transform
  void test_JointDependentFrame(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;
    std::vector<std::shared_ptr<JointDependentFrame>> frames;
    RoboticsUtils::quickInfo("----------Testing robot: ", robot->name(), " with ", robot->joints().size(), " joints");
    // Initialize the JointDependentFrames
    for(auto & j : robot->joints())
    {
      // RoboticsUtils::quickPrint("Joint name: ", j->name());

      if(j->dof() != 1)
      {
        continue;
      }

      Iso3d relativeTf = grTF(-0.2, 0.2);
      Iso3d relativeTf_two = grTF(-0.2, 0.2);
      Iso3d relativeTf_three = grTF(-0.2, 0.2);

      // RoboticsUtils::quickPrint("Relative transform: ", relativeTf.matrix());

      std::shared_ptr<JointDependentFrame> newFrame =
          std::make_shared<JointDependentFrame>(j->name() + "_test_frame", j->name(), relativeTf, robot);
      std::shared_ptr<JointDependentFrame> newFrame_two =
          std::make_shared<JointDependentFrame>(j->name() + "_test_frame_two", j->name(), relativeTf_two, robot);
      std::shared_ptr<JointDependentFrame> newFrame_three =
          std::make_shared<JointDependentFrame>(j->name() + "_test_frame_three", j->name(), relativeTf_three, robot);
      j->attachObserver(newFrame);
      j->attachObserver(newFrame_two);
      j->attachObserver(newFrame_three);
      j->printObservers();
      frames.push_back(newFrame);
      frames.push_back(newFrame_two);
      frames.push_back(newFrame_three);
    }

    RoboticsUtils::quickInfo("----------Created ", frames.size(), " frames-----------------");
    for(int i = 0; i < num_iterations; i++)
    {

      // RoboticsUtils::quickHL("----------iteration: ", i);
      // Set random joint position, velocity, and acceleration
      robot->setRandom(2, re);

      for(auto & f : frames)
      {
        // RoboticsUtils::quickError("----------JointFrame name: ", f->name());
        Eigen::MatrixXd diff =
            f->transform().matrix() - (robot->joint(f->parentJointName())->transform() * f->offset()).matrix();
        ASSERT_NEAR(diff.norm(), 0.0, tf_error);
      }
    }

  } // end of test_JointFrameTransform

  void randomJointVel(const VectorXd & q, std::default_random_engine & re, VectorXd & dq, VectorXd & qNext)
  {

    int dof = static_cast<int>(q.size());

    dq = VectorXd::Zero(dof);
    qNext = VectorXd::Zero(dof);
    qNext.head(6) = q.head(6);

    for(int ii = 6; ii < dof; ii++)
    {
      double delta_q = grNumber(re, -1.0 * jd_step_size, 1.0 * jd_step_size);
      qNext(ii) = q(ii) + delta_q;
      dq(ii) = delta_q / dt;
    }
  }

  void testAdjointTransformDerivative(std::shared_ptr<KinematicTree> & robot)
  {
    // Test the derivative of adjoint transform:
    // d(adg_ab) = - adg_ab * (crossMatrix(V^s_ba)) = adg_ab * (crossMatrix(V^b_ab))
    // See Lemma~3 of RobotDynamics

    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 300; i++)
    {
      // std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      // printMatrix("old q", q);

      // VectorXd q = robot->jointPositions();
      VectorXd dq, qNext;
      // printMatrix("joint position", q);
      // const Iso3d & tf = robot->transform();

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      // robot->readJointVelocities(dq);

      // robot->readJointVelocities(dq);
      // RoboticsUtils::quickPrint("joint positions", q);
      // RoboticsUtils::quickPrint("joint velocities", dq);
      // RoboticsUtils::quickPrint("next joint positions", qNext);
      // RoboticsUtils::quickPrint("net joint positions diff", qNext - q);
      // printMatrix("new q", qNext);

      // auto j = robot->joint(robot->randomJointName());
      // auto j_tf = j->transform();

      // auto f = robot->frame(robot->randomLinkName());
      // auto f = robot->frame("com");
      auto f = robot->frame(robot->randomFrameName());
      // auto f = robot->frame(robot->randomJointName());
      // auto f = robot->frame(robot->randomJointName()); // Does not work yet for a random frame
      // RoboticsUtils::quickError("working on frame:", f->frameName());

      auto j_tf = f->transform();

      // RoboticsUtils::quickPrint("tf", j_tf.matrix());
      const Matrix6d & adgM = metaAdgMatrix(j_tf);
      const Matrix6d & adgMT = metaAdgTMatrix(j_tf);

      const Vector6d & bodyVel = f->computeBodyJacobian(true) * dq;
      // const Vector6d & bodyVel = f->inertialFrameBodyVel();

      const Matrix6d & adgMd = adgM * (crossMatrix(bodyVel));
      const Matrix6d & adgMTd = crossMatrix(bodyVel).transpose() * adgM.transpose();

      // Perform the integration
      // robot->velIntegration(dt);

      robot->setJointPositions(qNext);

      // Read the new adgM
      auto delta_j_tf = f->transform();
      // RoboticsUtils::quickPrint("new tf", delta_j_tf.matrix());
      const Matrix6d & delta_adgM = metaAdgMatrix(delta_j_tf);
      const Matrix6d & delta_adgMT = metaAdgTMatrix(delta_j_tf);

      const MatrixXd & numerical_adgMd = (delta_adgM - adgM) / dt;
      const MatrixXd & numerical_adgMTd = (delta_adgMT - adgMT) / dt;

      // RoboticsUtils::quickPrint("numerical adgMd", numerical_adgMd);
      // RoboticsUtils::quickPrint("computed adgMd", adgMd);
      // RoboticsUtils::quickPrint("numerical_adgMd - adgMd", numerical_adgMd - adgMd);
      // std::cout << "Error is: " << (numerical_adgMd - adgMd).norm() << std::endl;
      ASSERT_NEAR((numerical_adgMd - adgMd).norm(), 0.0, tf_error);
      ASSERT_NEAR((numerical_adgMTd - adgMTd).norm(), 0.0, tf_error);
    }
  }
  void testTransformDerivative(std::shared_ptr<KinematicTree> & robot)
  {

    VectorXd q, dq;

    q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      std::cout << "Iteration: -----------" << i << "------" << std::endl;
      robot->setRandom(0, re);
      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      // std::string const a = robot->randomFrameName();
      std::string const a = "com";
      std::string const b = robot->randomJointName();

      // if(b == "RF_KFE")
      // {
      //   continue;
      // }

      // std::string const a = robot->randomLinkName();
      // std::string const b = robot->randomLinkName();

      // std::string const a = robot->randomJointName();
      // std::string const b = robot->randomJointName();

      RoboticsUtils::quickHL("Body velocity with Reference frame: ", a, " and body frame: ", b);

      Matrix6d const velTf = robot->velTransform(a, b);
      Matrix6d const & velTfd = robot->velTransformDerivative(a, b);
      RoboticsUtils::quickPrint("velTfd", velTfd);

      Matrix6d const wrenchTf = robot->wrenchTransform(a, b);
      Matrix6d const wrenchTfd = robot->wrenchTransformDerivative(a, b);

      // Matrix6d const inertia = robot->joint(a)->childLink()->localInertiaTensor();
      MetaInertia mi;
      mi.randomize();
      Matrix6d const & inertia = mi.constructInertiaTensor();

      Matrix6d const inertiaTf = robot->inertiaTransform(inertia, a, b);
      Matrix6d const inertiaTfd = robot->inertiaTransformDerivative(inertia, a, b);

      robot->setJointPositions(qNext);

      const Matrix6d & delta_velTf = robot->velTransform(a, b);

      const Matrix6d & delta_wrenchTf = robot->wrenchTransform(a, b);
      const Matrix6d & delta_inertiaTf = robot->inertiaTransform(inertia, a, b);

      const MatrixXd & numerical_velTfd = (delta_velTf - velTf) / dt;
      const MatrixXd & numerical_wrenchTfd = (delta_wrenchTf - wrenchTf) / dt;
      const MatrixXd & numerical_inertiaTfd = (delta_inertiaTf - inertiaTf) / dt;

      RoboticsUtils::quickPrint("numerical velTfd", numerical_velTfd);

      RoboticsUtils::quickError("velTfd diff ", (numerical_velTfd - velTfd).norm());
      RoboticsUtils::quickError("wrenchTfd diff ", (numerical_wrenchTfd - wrenchTfd).norm());
      RoboticsUtils::quickError("inertiaTfd diff ", (numerical_inertiaTfd - inertiaTfd).norm());
      // ASSERT_NEAR((numerical_velTfd - velTfd).norm(), 0.0,  10 * tf_error);
      // ASSERT_NEAR((numerical_wrenchTfd - wrenchTfd).norm(), 0.0, 2 * tf_error);
      // ASSERT_NEAR((numerical_inertiaTfd - inertiaTfd).norm(), 0.0, 2 * tf_error);
    }
  }
};

TEST_F(Transform, case1)
{

  for(auto robot : robots)
  {
    test_transform(robot);
  }
} // End of case1

// TEST_F(Transform, case2)
// {
//
//   // Test transforms of the merged tree
//
//   std::default_random_engine re;
//   for(int i = 0; i < num_iterations; i++)
//   {
//     hTree->setRandom(2, re);
//     hTreeMerge->setJointPositions(hTree->jointPositions());
//     hTreeMerge->setJointVelocities(hTree->jointVelocities());
//     hTreeMerge->setJointAccelerations(hTree->jointAccelerations());
//
//     for(auto j : hTree->articulatedJoints())
//     {
//
//       auto joint = hTree->joint(j.first);
//       const MatrixXd & diff = joint->transform().matrix() - hTreeMerge->joint(joint->name())->transform().matrix();
//
//       ASSERT_NEAR(diff.norm(), 0.0, tf_error);
//     }
//   }
//
// } // End of case2

TEST_F(Transform, case3)
{

  // RoboticsUtils::quickVec("articulated joints: ", robots[1]->articulatedJoints());
  // testRENA(robots[1]);

  for(auto robot : robots)
  {
    testRENA(robot);
  }

} // End of case3

// TEST_F(Transform, case3)
//{
//
//  RoboticsUtils::quickVec("articulated joints: ", robots[1]->articulatedJoints());
//  //testRENA(robots[1]);
//
//
//  //for(auto robot : robots)
//  //{
//  //  testRENA(robot);
//  //}
//
//  int rid = 1;
//  std::default_random_engine re;
//  robots[rid]->setRandom(2, re);
//  RoboticsUtils::quickPrint("joint positions", robots[rid]->jointPositions());
//  RoboticsUtils::quickPrint("joint velocities", robots[rid]->jointVelocities());
//  RoboticsUtils::quickPrint("joint accelerations", robots[rid]->jointAccelerations());
//} // End of case3

TEST_F(Transform, case5)
{

  // RoboticsUtils::quickVec("articulated joints: ", robots[1]->articulatedJoints());
  // testRENA(robots[1]);

  // test_JointFrame(hTreeMerge);
  // test_JointFrame(aTree);
  for(auto robot : robots)
  {
    test_JointDependentFrame(robot);
    test_Frames(robot);
  }

} // End of case5

TEST_F(Transform, case6_1)
{

  for(auto robot : robots)
  {
    test_fb_joint_transform(robot);
  }

} // End of case6_1

// TEST_F(Transform, case6)
// {
//
//   // test_joint_transform(hTreeMerge);
//   // for(auto robot : robots)
//   // {
//   //   test_joint_transform(robot);
//   //   // test_transform_calculation(robot);
//   // }
//
// } // End of case6

TEST_F(Transform, case7_1)
{

  // for(auto robot : robots)
  // {
  //   testAdjointTransformDerivative(robot);
  // }
  testAdjointTransformDerivative(aTree);
}

TEST_F(Transform, case7_2)
{

  for(auto robot : robots)
  {
    test_bodyvel_calc(robot);
  }
}

TEST_F(Transform, case7_3)
{

  for(auto robot : robots)
  {
    test_fullsize_jacobian(robot);
  }
}

TEST_F(Transform, case7_4)
{

  test_jacobiand(aTree);
  // test_jacobiand(hTreeMerge);
  // for(auto robot : robots)
  // {
  //   test_jacobiand(robot);
  // }
}

TEST_F(Transform, case7_6)
{

  test_pointvel(aTree);
  // for(auto robot : robots)
  // {
  //   test_pointvel(robot);
  // }
}

TEST_F(Transform, case7_7)
{

  for(auto robot : robots)
  {
    test_comJac(robot);
  }
}

TEST_F(Transform, case7)
{

  for(auto robot : robots)
  {
    testTransformDerivative(robot);
  }

} // End of case7

TEST_F(Transform, case8)
{

  for(auto robot : robots)
  {
    test_computeBodyVel(robot);
  }

  // test_computeBodyVel(aTree);

} // End of case8

TEST_F(Transform, case10)
{

  for(auto robot : robots)
  {
    test_jacobian(robot);
  }
}

TEST_F(Transform, case9)
{

  test_cmmd(aTree); // Fixed bug: notify the kinematicTreeObservers after updating transforms

  // test_cmmd(hTreeMerge); // Can not pass the test due to the angular part of the com frame transform derivative
}

} // namespace GeoRobotics
