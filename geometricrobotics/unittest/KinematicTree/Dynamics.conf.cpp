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

namespace GeoRobotics
{
struct ScaraParams
{
  double l1 = 0.2;
  double l2 = 0.3;
  double l0 = 0.4;
};

struct ElbowParams
{
  double l1 = 0.5;
  double l2 = 0.5;
  double l0 = 0.6;
};

class Dynamics : public testing::Test
{
protected:
  Parser parser;

  std::shared_ptr<Model> scaraPtr;
  std::shared_ptr<KinematicChain> scaraChain;
  std::shared_ptr<KinematicTree> sKPtr;
  std::shared_ptr<Robot> scaraRobot;

  std::shared_ptr<Model> elbowPtr;
  std::shared_ptr<KinematicChain> elbowChain;
  std::shared_ptr<KinematicTree> eKPtr;
  std::shared_ptr<Robot> elbowRobot;

  std::shared_ptr<Model> modelPtr;
  std::shared_ptr<KinematicChain> pandaChain;
  std::shared_ptr<KinematicTree> pKPtr;
  std::shared_ptr<Robot> pandaRobot;

  std::shared_ptr<Model> hrpModelPtr;
  std::shared_ptr<Robot> hrp;
  std::shared_ptr<KinematicTree> hTree;

  std::shared_ptr<Model> animalModelPtr;
  std::shared_ptr<Robot> animal;
  std::shared_ptr<KinematicTree> aTree;

  int num_iterations = 1000;
  double tf_error = 1e-3;

  std::vector<std::shared_ptr<KinematicTree>> robots;

  void SetUp() override
  {
    // Create the 3DOF SCARA robot shown in Fig.3.3 of Murray et al. 1994
    scaraPtr = parser.parseURDFFile("@RobotExamplesPath@SCARA.urdf");
    scaraPtr->merge();
    scaraRobot = std::make_shared<Robot>(scaraPtr);
    sKPtr = scaraRobot->createFloatingBaseKinematicTree("scara", "world");
    // The base_link was merged.
    scaraChain = sKPtr->createKinematicChain("scara", "world", "link_4");

    // scaraChain->printKinematicChain();

    // Create the ELBOW Manipulator shown in Fig.3.4 of Murray et al. 1994
    elbowPtr = parser.parseURDFFile("@RobotExamplesPath@ELBOW.urdf");
    elbowPtr->merge();
    elbowRobot = std::make_shared<Robot>(elbowPtr);

    eKPtr = elbowRobot->createFloatingBaseKinematicTree("elbow", "world");
    // The base_link was merged.
    elbowChain = eKPtr->createKinematicChain("elbow", "world", "link_6");
    elbowChain->printKinematicChain();

    // pKPtr = pandaRobot->createFloatingBaseKinematicTree("panda", "world");
    //// The base_link was merged.
    // pandaChain = pKPtr->createKinematicChain("panda", "world", "panda_link7");
    // pandaChain->printKinematicChain();

    hrpModelPtr = parser.parseURDFFile("/home/yuquan/catkin_ws/src/hrp4/hrp4_description/urdf/hrp4.urdf");
    hrpModelPtr->merge();

    hrp = std::make_shared<Robot>(hrpModelPtr);
    hTree = hrp->createFloatingBaseKinematicTree("hrp", "base_link");
    hTree->initializeFrames();
    robots.push_back(hTree);

    robots[0]->printTree();
    robots[0]->visualize();

    animalModelPtr = parser.parseURDFFile("@RobotExamplesPath@Animal.urdf");
    animalModelPtr->merge();

    animal = std::make_shared<Robot>(animalModelPtr);
    aTree = animal->createFloatingBaseKinematicTree("animal", "base");
    aTree->initializeFrames();
    robots.push_back(aTree);
    robots[1]->printTree();
    robots[1]->visualize();
  }

  void TearDown() override {}

  void checkDynamics(const std::shared_ptr<KinematicTree> & robot,
                     std::unordered_set<std::string> & visited,
                     const std::string & currentNode,
                     const std::string & currentJoint,
                     Iso3d & iniTF)
  {
    visited.insert(currentNode);
    auto cj = robot->joint(currentJoint);
    // Increment the initial Dynamics

    if(cj->dof() == 1)
    {
      iniTF = iniTF * cj->incrementalTransform();
    }
    // auto currentTF = cj->relativeDynamics();
  }

  void transformDFSVisit(const std::shared_ptr<KinematicTree> & robot,
                         const std::string & currentNode,
                         const std::string & currentJoint,
                         Iso3d & iniTF,
                         std::unordered_set<std::string> & visited)
  {

    checkDynamics(robot, visited, currentNode, currentJoint, iniTF);

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
      // RoboticsUtils::quickInfo("Updated Dynamics: ", pj->Dynamics().matrix());
      // RoboticsUtils::quickInfo("Manual Dynamics: ", iniTF.matrix());

      const MatrixXd & diff = pj->transform().matrix() - (iniTF * pj->initialTransform()).matrix();
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);

      auto currentTF = pj->incrementalTransform();
      iniTF = iniTF * currentTF.inverse();
    }
  }
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

  void test_CRB(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    for(int i = 0; i < num_iterations; i++)
    // for(int i = 0; i < 100; i++)
    {

      robot->setRandom(2, re);

      MatrixXd jsim;
      robot->MCalcCRB(jsim);
      // 1. Test symmetry
      // RoboticsUtils::quickPrint("JSIM", jsim);
      ASSERT_NEAR((jsim - jsim.transpose()).norm(), 0.0, 0.01);

      // 2. Test positive definiteness.
      Eigen::LLT<Eigen::MatrixXd> lltM(jsim);
      ASSERT_TRUE(lltM.info() != Eigen::NumericalIssue);
    }

  } // end of test_CRB

  void test_chains(std::shared_ptr<KinematicChain> & chain, std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    RoboticsUtils::quickHL("Testing the kinematicChain of robot: ", robot->name());
    // for(int i = 0; i < num_iterations; i++)
    // for(int i = 0; i < 100; i++)
    {
      robot->setRandom(2, re);

      VectorXd q = robot->jointPositions();

      RoboticsUtils::quickPrint("Tree q", q);
      VectorXd dq = robot->jointVelocities();
      RoboticsUtils::quickPrint("Tree dq", dq);
      chain->setJointPositions(q.tail(chain->dof()));
      RoboticsUtils::quickPrint("chain q", chain->jointPositions());
      chain->setJointVelocities(dq.tail(chain->dof()));
      RoboticsUtils::quickPrint("chain dq", chain->jointVelocities());

      // MatrixXd jsim;
      // robot->MCalcCRB(jsim);
      // RoboticsUtils::quickPrint("KinematicTree JSIM", jsim);
      // RoboticsUtils::quickPrint("KinematicChain M", chain->M());

      // dq.head(6).setZero();
      // double ke = dq.transpose() * jsim * dq;
      // RoboticsUtils::quickPrint("KinematicTree's JSIM kinetic energy", ke);
      // RoboticsUtils::quickPrint("KinematicChain's kinetic energy", chain->kineticEnergy());

      // Compare the transform
      for(auto j : chain->joints())
      {
        RoboticsUtils::quickError("Checking the transform of joint: ", j->name());

        RoboticsUtils::quickPrint("Tree transform ", robot->joint(j->name())->transform().matrix());
        RoboticsUtils::quickPrint("Chain transform ", chain->transform(j->name()).matrix());

        const Eigen::Matrix4d & m1 = robot->joint(j->name())->transform().matrix();

        const Eigen::Matrix4d & m2 = chain->transform(j->name()).matrix();

        Matrix4d diff = m1 - m2;

        // ASSERT_NEAR((diff).norm(), 0.0, tf_error);
      }
    }
  }

  void test_ci(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    double threshold = 0.01;
    for(int i = 0; i < num_iterations; i++)
    // for(int i = 0; i < 100; i++)
    {

      robot->setRandom(2, re);

      Matrix6d ci;
      ci = robot->computeCentroidalInertia();
      // RoboticsUtils::quickPrint("CI", ci);
      const Matrix3d & urCorner = ci.block<3, 3>(0, 3);
      const Matrix3d & llCorner = ci.block<3, 3>(3, 0);
      const Matrix3d & ulCorner = ci.block<3, 3>(0, 0);

      ASSERT_NEAR((urCorner).norm(), 0.0, threshold);
      ASSERT_NEAR((llCorner).norm(), 0.0, threshold);
      ASSERT_NEAR((ulCorner - robot->mass() * Matrix3d::Identity()).norm(), 0.0, threshold);

      ASSERT_NEAR((ci.block<3, 3>(0, 0) - robot->mass() * Matrix3d::Identity()).norm(), 0.0, threshold);
    }

  } // end of testRENA_Dynamics
  void test_smart_pointer(std::shared_ptr<KinematicTree> & robot)
  {
    auto count = robot.use_count();
    std::shared_ptr<KinematicTree> r_1 = robot->getPtr();
    ASSERT_EQ(robot.use_count(), count + 1);
    ASSERT_EQ(r_1.use_count(), count + 1);
  }
};

TEST_F(Dynamics, case0)
{

  for(auto robot : robots)
  {
    test_smart_pointer(robot);
  }

} // End of case3

TEST_F(Dynamics, case1)
{

  for(auto robot : robots)
  {
    test_transform(robot);
  }
  for(auto robot : robots)
  {
    robot->printTree();
    robot->visualize();
  }

} // End of case1

TEST_F(Dynamics, case2)
{

  for(auto robot : robots)
  {
    test_CRB(robot);
  }

} // End of case2

TEST_F(Dynamics, case3)
{

  for(auto robot : robots)
  {
    test_ci(robot);
  }

} // End of case3

// TEST_F(Dynamics, case4)
//{
//
//  // Not applicable anymore. All the chains are initialized with a floating-base joint
//  test_chains(scaraChain, sKPtr);
//  //test_chains(elbowChain, eKPtr);
//
//} // End of case4

// TEST_F(Dynamics, case3)
//{
//
//  // RoboticsUtils::quickVec("articulated joints: ", robots[1]->articulatedJoints());
//  // testRENA(robots[1]);
//
//  for(auto robot : robots)
//  {
//    testRENA(robot);
//  }
//
//} // End of case3

// TEST_F(Dynamics, case3)
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

} // namespace GeoRobotics
