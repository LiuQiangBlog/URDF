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

namespace GeoRobotics
{

class FbRobotExample : public testing::Test
{
protected:
  Parser parser;

  std::shared_ptr<Model> hrpModelPtr;
  std::shared_ptr<Robot> hrp;
  std::shared_ptr<KinematicTree> hTree;

  std::shared_ptr<Model> jvrcModelPtr;
  std::shared_ptr<Robot> jvrc;
  std::shared_ptr<KinematicTree> jTree;

  std::shared_ptr<Model> hrpMergeModelPtr;
  std::shared_ptr<Robot> hrpMerge;
  std::shared_ptr<KinematicTree> hTreeMerge;

  std::shared_ptr<Model> animalModelPtr;
  std::shared_ptr<Robot> animal;
  std::shared_ptr<KinematicTree> aTree;

  std::shared_ptr<Model> cassieModelPtr;
  std::shared_ptr<Robot> cassie;
  std::shared_ptr<KinematicTree> cassieTree;

  // std::shared_ptr<Model> centauroModelPtr;
  // std::shared_ptr<Robot> centauro;

  int num_iterations = 1000;
  double tf_error = 1e-3;

  std::vector<std::shared_ptr<KinematicTree>> robots;

  void SetUp() override
  {

    // hrpMergeModelPtr = parser.parseURDFFile("/home/yuquan/catkin_ws/src/hrp4/hrp4_description/urdf/hrp4.urdf");
    // hrpMergeModelPtr->merge();
    // hrpMerge = std::make_shared<Robot>(hrpMergeModelPtr);
    // hTreeMerge = hrpMerge->createFloatingBaseKinematicTree("hrp", "base_link");
    // hTreeMerge->initializeFrames();

    // robots.push_back(hTreeMerge);

    hrpModelPtr = parser.parseURDFFile("/home/yuquan/catkin_ws/src/hrp4/hrp4_description/urdf/hrp4.urdf");
    hrpModelPtr->merge();
    hrp = std::make_shared<Robot>(hrpModelPtr);
    hTree = hrp->createFloatingBaseKinematicTree("hrp", "base_link");
    hTree->initializeFrames();

    robots.push_back(hTree);

    animalModelPtr = parser.parseURDFFile("@RobotExamplesPath@Animal.urdf");

    animal = std::make_shared<Robot>(animalModelPtr);
    aTree = animal->createFloatingBaseKinematicTree("animal", "base");

    aTree->initializeFrames();
    robots.push_back(aTree);

    jvrcModelPtr = parser.parseURDFFile("/usr/share/mc_rtc/jvrc_description/urdf/jvrc1.urdf");
    jvrc = std::make_shared<Robot>(jvrcModelPtr);
    jTree = jvrc->createFloatingBaseKinematicTree("jvrc", "base_link");
    jTree->initializeFrames();

    cassieModelPtr = parser.parseURDFFile("@RobotExamplesPath@cassie_v4.urdf");
    cassie = std::make_shared<Robot>(cassieModelPtr);
    cassieTree = cassie->createFloatingBaseKinematicTree("cassie", "pelvis");
    cassieTree->initializeFrames();

    // Comment out centauro for now, due to the joint axis issue:
    // Axis: 0, -0.984808, 0.173648
    // centauroModelPtr = parser.parseURDFFile("@RobotExamplesPath@Centauro.urdf");

    // centauro = std::make_shared<Robot>(centauroModelPtr);
  }

  void TearDown() override {}

  bool equalJoint(const kStep & one, const kStep & two)
  {

    if(one.second != two.second)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  bool equalStep(const kStep & one, const kStep & two)
  {

    if(one.first != two.first)
    {
      return false;
    }
    else if(one.second != two.second)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  bool equalPath(const kPath & path, const kPath & answer)
  {
    auto a = answer.begin();
    for(auto const & n : path)
    {
      if(!equalStep(n, *a))
      {
        return false;
      }
      a++;
    }
    return true;
  }

  bool reverseEqualPath(const kPath & p1, const kPath & p2)
  {

    if(p1.size() != p2.size())
    {
      RoboticsUtils::quickError("path 1 has size: ", p1.size());
      RoboticsUtils::quickError("path 2 has size: ", p2.size());
      return false;
    }

    auto a = p1.begin();
    auto b = p2.rbegin();

    for(size_t i = 0; i < p1.size(); i++)
    {
      if(!equalJoint(*a, *b))
      {
        RoboticsUtils::quickError("Step a from: ", a->first, " to: ", a->second);
        RoboticsUtils::quickError("Step b from: ", b->first, " to: ", b->second);
        return false;
      }
      a++;
      b++;
    }
    return true;
  }

  void test_articulated_joints(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    // RoboticsUtils::quickInfo("Test joint transforms of robot: ", robot->name());

    for(int i = 0; i < num_iterations; i++)
    // for(int i = 0; i < 1; i++)
    {

      // robot->setRandom(0, re);

      // RoboticsUtils::quickInfo("Iteration:-------- ", i, " ---------");

      auto j = robot->randomJointName();
      if(j == "Floating-base joint")
      {
        continue;
      }
      RoboticsUtils::quickHL("joint: ", j);
      // RoboticsUtils::quickHL(" Analyzing the initial transform of joint: ", j->name());

      auto const & jj = robot->joint(j);
      auto djSet = jj->articulatedJoints();

      std::deque<int> jSet;
      robot->odArticulatedJoints(jSet, jj->childLink()->name());

      size_t n = djSet.size();
      ASSERT_EQ(n + 1, jSet.size());

      for(size_t ii = 0; ii < n; ii++)
      {
        RoboticsUtils::quickInfo("djSet[ii]: ", djSet[ii].first, " jSet[ii]: ", jSet[ii]);
        ASSERT_EQ(djSet[ii].first, jSet[ii]);
      }

      ASSERT_EQ(jj->jointIndex(), jSet[n]);
    }
  }

  void test_joint_transform(std::shared_ptr<KinematicTree> & robot)
  {
    std::default_random_engine re;

    RoboticsUtils::quickInfo("Test joint transforms of robot: ", robot->name());
    for(int i = 0; i < num_iterations; i++)
    // for(int i = 0; i < 1; i++)
    {

      robot->setRandom(0, re);

      // RoboticsUtils::quickInfo("Iteration:-------- ", i, " ---------");

      for(auto j = robot->joints().begin() + 1; j != robot->joints().end(); j++)
      {
        // RoboticsUtils::quickHL("joint: ", j->get()->name());
        // RoboticsUtils::quickHL(" Analyzing the initial transform of joint: ", j->name());

        auto const & jj = j->get();
        auto djSet = jj->articulatedJoints();

        // RoboticsUtils::quickHL(" Its child link name is: ", jj->childLink()->name());
        // RoboticsUtils::quickPrint("transform: ", jj->transform().matrix());
        // RoboticsUtils::quickHL(" Its dependent-joint number is: ", djSet.size());

        GeoRobotics::Iso3d iniTF = GeoRobotics::Iso3d::Identity();

        auto const & root_tf = robot->joints()[0]->transform();

        if(robot->isFloatingBase())
        {
          iniTF = root_tf;
        }

        // RoboticsUtils::quickInfo("Root TF: ", iniTF.matrix());

        if(djSet.size() != 0)
        {
          for(auto const & dj : djSet)
          {

            auto const & djp = robot->joint(dj.first);
            // RoboticsUtils::quickInfo("Dependent joint: ", djp->name());
            // RoboticsUtils::quickPrint("with incremental transform: ", djp->incrementalTransform().matrix());
            iniTF = iniTF * djp->incrementalTransform();

            // RoboticsUtils::quickPrint("iniTF: ", iniTF.matrix());
          }
        }

        // RoboticsUtils::quickPrint(jj->name() + " incrementalTransform: ", jj->incrementalTransform().matrix());
        iniTF = iniTF * jj->incrementalTransform();
        // RoboticsUtils::quickPrint("iniTF: ", iniTF.matrix());

        // RoboticsUtils::quickPrint("Multiply the initial transform: ", jj->initialTransform().matrix());
        iniTF = iniTF * jj->initialTransform();

        // RoboticsUtils::quickPrint(" Computed transform: ", iniTF.matrix());
        // RoboticsUtils::quickPrint("Joint transform", j->get()->transform().matrix());
        auto diff = iniTF.matrix() - j->get()->transform().matrix();
        // RoboticsUtils::quickError("Diff", diff.norm());

        ASSERT_NEAR(diff.norm(), 0.0, 0.01);
      }
    }
  }

  void test_path_finding(std::shared_ptr<KinematicTree> & robot)
  {
    // -------------------------Test "path finding":
    // (1) If a path exists
    // (2) By swaping the start and the goal, we can find the reverse path.
    // auto const & lNameOne = robot->randomLinkName();
    for(int i = 0; i < num_iterations; i++)
    {
      // RoboticsUtils::quickInfo("Evaluating link: ", l->name());
      kPath p1, p2;

      auto const & l1 = robot->randomLinkName();
      auto const & l2 = robot->randomLinkName();

      robot->graph()->findPath(l1, l2, p1);
      robot->graph()->findPath(l2, l1, p2);

      if(!reverseEqualPath(p1, p2))
      {
        RoboticsUtils::quickError("Checking link: ", l1, " and link: ", l2);
        robot->graph()->printPath(p1);
        robot->graph()->printPath(p2);
      }
      ASSERT_EQ(reverseEqualPath(p1, p2), true);
    }
  }
};

TEST_F(FbRobotExample, case0)
{

  // 1. create a kinematic tree from the root link
  std::string treeName = "jvrc";
  std::string rootLinkName = "base_link";

  const std::shared_ptr<KinematicTree> & jvrcTree = jvrc->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the jvrc whole-body kinematic tree.");

  jvrcTree->printTree();
  jvrcTree->visualize();

} // End of case1

TEST_F(FbRobotExample, case1)
{

  // 1. create a kinematic tree from the root link
  std::string treeName = "hrp";
  std::string rootLinkName = "base_link";

  // Create a floating-base joint
  // std::shared_ptr<GeoJoint> fbJoint;

  // fbJoint = hrp->createFloatingBaseJoint(rootLinkName);
  //
  // RoboticsUtils::quickHL("Created the floating-base joint");

  // fbJoint->print();

  const std::shared_ptr<KinematicTree> & hTree = hrp->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the hrp whole-body kinematic tree.");

  hTree->printTree();
  hTree->visualize();

} // End of case1

TEST_F(FbRobotExample, case_cassie)
{

  // 1. create a kinematic tree from the root link
  std::string treeName = "cassie";
  std::string rootLinkName = "pelvis";

  // Create a floating-base joint
  // std::shared_ptr<GeoJoint> fbJoint;

  // fbJoint = hrp->createFloatingBaseJoint(rootLinkName);
  //
  // RoboticsUtils::quickHL("Created the floating-base joint");

  // fbJoint->print();

  const std::shared_ptr<KinematicTree> & cTree = cassie->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the cassie whole-body kinematic tree.");

  cTree->printTree();
  cTree->visualize();

} // End of case_cassie

TEST_F(FbRobotExample, case2)
{

  // -------------------------Test the dependent joints of a link

  // 1. create a kinematic tree from the root link
  std::string treeName = "hrp";
  std::string rootLinkName = "base_link";

  const std::shared_ptr<KinematicTree> & hTree = hrp->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the hrp whole-body kinematic tree.");

  for(auto l : hTree->links())
  {
    // RoboticsUtils::quickInfo("Evaluating link: ", l->name());
    kPath path2root;
    hTree->graph()->findPath(rootLinkName, l->name(), path2root);

    // RoboticsUtils::quickInfo("Found path between: ", l->name(), " and ", rootLinkName);
    std::deque<int> jSet;

    hTree->odJoints(jSet, l->name());

    // RoboticsUtils::quickInfo("Found the set of dependent joints.");

    // RoboticsUtils::quickInfo("The dependent joints are: ");
    for(size_t ii = 0; ii < jSet.size() - 1; ii++)
    {
      assert(jSet[ii] == (hTree->jointNameToIndex(path2root.front().second)));
      path2root.pop_front();
    }
  }
} // End of case2

TEST_F(FbRobotExample, case3)
{

  // -------------------------Test the dependent links of a link

  // 1. create a kinematic tree from the root link
  std::string treeName = "hrp";
  std::string rootLinkName = "base_link";

  const std::shared_ptr<KinematicTree> & hTree = hrp->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the hrp whole-body kinematic tree.");

  for(auto l : hTree->links())
  {
    // RoboticsUtils::quickInfo("Evaluating link: ", l->name());
    kPath path2root;
    hTree->graph()->findPath(rootLinkName, l->name(), path2root);

    // RoboticsUtils::quickInfo("Found path between: ", l->name(), " and ", rootLinkName);
    std::vector<int> lSet;

    lSet = hTree->graph()->odLinks(l->name());

    // RoboticsUtils::quickInfo("Found the set of dependent links.");

    // RoboticsUtils::quickInfo("The dependent links are: ");
    for(size_t ii = 1; ii < lSet.size(); ii++)
    {
      // RoboticsUtils::quickInfo("The path size is: ", path2root.size());
      // RoboticsUtils::quickHL("link: ", hTree->linkIndexToName(lSet[ii]), ", index: ", lSet[ii]);
      // RoboticsUtils::quickHL(path2root.front().first);
      assert(lSet[ii] == (hTree->linkNameToIndex(path2root.front().first)));
      path2root.pop_front();
    }
  }
} // End of case3

TEST_F(FbRobotExample, case4)
{

  // -------------------------Test the initial transforms.

  // 1. create a kinematic tree from the root link
  std::string treeName = "hrp";
  std::string rootLinkName = "base_link";

  const std::shared_ptr<KinematicTree> & hTree = hrp->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  // RoboticsUtils::quickHL("Created the hrp whole-body kinematic tree.");

  for(auto j : hTree->joints())
  {
    // Manually compute the Create the initial transform:

    // RoboticsUtils::quickHL(" Analyzing the initial transform of joint: ", j->name());

    std::deque<int> jSet;

    hTree->odJoints(jSet, j->childLink()->name());

    GeoRobotics::Iso3d iniTF = GeoRobotics::Iso3d::Identity();
    for(auto dj : jSet)
    {
      iniTF = iniTF * hTree->joint(dj)->initialRelativeTransform();
    }

    // RoboticsUtils::quickPrint(" The computed initial transform: ", iniTF.matrix());
    // RoboticsUtils::quickPrint(" The initial transform: ", j->initialTransform().matrix());
    ASSERT_NEAR((j->initialTransform().matrix() - iniTF.matrix()).norm(), 0.0, 0.01);
  }

  RoboticsUtils::quickHL(" Double-checked the initial transforms of all the joints");
} // End of case4

TEST_F(FbRobotExample, case5)
{

  // 1. create a kinematic tree from the root link
  std::string treeName = "animal";
  std::string rootLinkName = "base";

  // Create a floating-base joint
  // std::shared_ptr<GeoJoint> fbJoint;

  // fbJoint = hrp->createFloatingBaseJoint(rootLinkName);
  //
  // RoboticsUtils::quickHL("Created the floating-base joint");

  // fbJoint->print();

  const std::shared_ptr<KinematicTree> & aTree = animal->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the 'animal robot' whole-body kinematic tree.");

  aTree->printTree();
  aTree->visualize();

} // End of case5
TEST_F(FbRobotExample, case6)
{

  // -------------------------Test the dependent joints of a link

  // 1. create a kinematic tree from the root link
  std::string treeName = "animal";
  std::string rootLinkName = "base";

  const std::shared_ptr<KinematicTree> & aTree = animal->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the animal whole-body kinematic tree.");

  for(auto l : aTree->links())
  {
    // RoboticsUtils::quickInfo("Evaluating link: ", l->name());
    kPath path2root;
    aTree->graph()->findPath(rootLinkName, l->name(), path2root);

    // RoboticsUtils::quickInfo("Found path between: ", l->name(), " and ", rootLinkName);
    std::deque<int> jSet;

    aTree->odJoints(jSet, l->name());

    // RoboticsUtils::quickInfo("Found the set of dependent joints.");

    // RoboticsUtils::quickInfo("The dependent joints are: ");
    for(size_t ii = 0; ii < jSet.size() - 1; ii++)
    {
      assert(jSet[ii] == (aTree->jointNameToIndex(path2root.front().second)));
      path2root.pop_front();
    }
  }
} // End of case6

TEST_F(FbRobotExample, case7)
{

  // -------------------------Test the dependent links of a link

  // 1. create a kinematic tree from the root link
  std::string treeName = "animal";
  std::string rootLinkName = "base";

  const std::shared_ptr<KinematicTree> & aTree = animal->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  RoboticsUtils::quickHL("Created the animal whole-body kinematic tree.");

  for(auto l : aTree->links())
  {
    // RoboticsUtils::quickInfo("Evaluating link: ", l->name());
    kPath path2root;
    aTree->graph()->findPath(rootLinkName, l->name(), path2root);

    // RoboticsUtils::quickInfo("Found path between: ", l->name(), " and ", rootLinkName);
    std::vector<int> lSet;

    lSet = aTree->graph()->odLinks(l->name());

    // RoboticsUtils::quickInfo("Found the set of dependent links.");

    // RoboticsUtils::quickInfo("The dependent links are: ");
    for(size_t ii = 1; ii < lSet.size(); ii++)
    {
      // RoboticsUtils::quickInfo("The path size is: ", path2root.size());
      // RoboticsUtils::quickHL("link: ", hTree->linkIndexToName(lSet[ii]), ", index: ", lSet[ii]);
      // RoboticsUtils::quickHL(path2root.front().first);
      assert(lSet[ii] == (aTree->linkNameToIndex(path2root.front().first)));
      path2root.pop_front();
    }
  }
} // End of case7

TEST_F(FbRobotExample, case8)
{

  // -------------------------Test the initial transforms.

  // 1. create a kinematic tree from the root link
  std::string treeName = "animal";
  std::string rootLinkName = "base";

  const std::shared_ptr<KinematicTree> & aTree = animal->createFloatingBaseKinematicTree(treeName, rootLinkName);

  // hTree->graph()->makeTree("base_link");

  // RoboticsUtils::quickHL("Created the hrp whole-body kinematic tree.");

  for(auto j : aTree->joints())
  {
    // Manually compute the Create the initial transform:

    // RoboticsUtils::quickHL(" Analyzing the initial transform of joint: ", j->name());

    std::deque<int> jSet;

    aTree->odJoints(jSet, j->childLink()->name());

    GeoRobotics::Iso3d iniTF = GeoRobotics::Iso3d::Identity();
    for(auto dj : jSet)
    {
      iniTF = iniTF * aTree->joint(dj)->initialRelativeTransform();
    }

    // RoboticsUtils::quickPrint(" The computed initial transform: ", iniTF.matrix());
    // RoboticsUtils::quickPrint(" The initial transform: ", j->initialTransform().matrix());
    ASSERT_NEAR((j->initialTransform().matrix() - iniTF.matrix()).norm(), 0.0, 0.01);
  }

  RoboticsUtils::quickHL(" Double-checked the initial transforms of all the joints");

} // End of case8

// TEST_F(FbRobotExample, case9)
// {
//
//   // 1. create a kinematic tree from the root link
//   std::string treeName = "centauro";
//   std::string rootLinkName = "world";
//
//   const std::shared_ptr<KinematicTree> & cTree = centauro->createFloatingBaseKinematicTree(treeName, rootLinkName);
//
//   RoboticsUtils::quickHL("Created the centauro whole-body kinematic tree.");
//
//   cTree->printTree();
//   cTree->visualize();
//
// } // End of case9

// TEST_F(FbRobotExample, case10)
// {
//
//   // 1. create a kinematic tree from the root link
//   std::string treeName = "hrp";
//   std::string rootLinkName = "base_link";
//
//   // Create a floating-base joint
//   // std::shared_ptr<GeoJoint> fbJoint;
//
//   // fbJoint = hrp->createFloatingBaseJoint(rootLinkName);
//   //
//   // RoboticsUtils::quickHL("Created the floating-base joint");
//
//   // fbJoint->print();
//
//   hrpModelPtr->merge();
//
//   std::shared_ptr<Robot> hrpMerged = std::make_shared<Robot>(hrpModelPtr);
//
//   const std::shared_ptr<KinematicTree> & hTree = hrpMerged->createFloatingBaseKinematicTree(treeName, rootLinkName);
//
//   // hTree->graph()->makeTree("base_link");
//
//   RoboticsUtils::quickHL("Created the hrp whole-body kinematic tree.");
//
//   hTree->printTree();
//   hTree->visualize();
//
// } // End of case10
//
TEST_F(FbRobotExample, case11)
{

  // -------------------------Test "path finding"
  for(auto & robot : robots)
  {
    test_path_finding(robot);
  }
} // End of case11

TEST_F(FbRobotExample, case12)
{

  // -------------------------Test "path finding"
  test_joint_transform(hTree);

  // BUG: segmentation fault for the animal robot
} // End of case12

TEST_F(FbRobotExample, case13)
{

  // -------------------------Test "articulated joints"

  test_articulated_joints(hTree);

  // BUG: Does not work for the animal robot yet
} // End of case13

} // namespace GeoRobotics
