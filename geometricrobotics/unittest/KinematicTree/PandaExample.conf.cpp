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
#include "GeometricRobotics/urdf/Parser.h"
#include "gtest/gtest.h"
#include <limits>

namespace GeoRobotics
{

class PandaExample : public testing::Test
{
protected:
  Parser parser;
  std::shared_ptr<Model> modelPtr;

  void SetUp() override
  {
    modelPtr = parser.parseURDFFile("@RobotExamplesPath@panda_foot.urdf");
  }

  void TearDown() override {}

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
    for(auto & n : path)
    {
      if(!equalStep(n, *a))
      {
        return false;
      }
      a++;
    }
    return true;
  }
};

TEST_F(PandaExample, caseOne)
{

  Graph g;

  g.addEdge("1", "2", "a");
  g.addEdge("1", "3", "b");
  g.addEdge("2", "4", "c");
  g.addEdge("2", "5", "d");
  g.addEdge("2", "6", "e");
  g.addEdge("3", "7", "f");
  g.addEdge("3", "8", "g");
  g.addEdge("3", "9", "h");

  kPath answer;
  answer.push_back({"2", "c"});
  answer.push_back({"1", "a"});
  answer.push_back({"3", "b"});
  answer.push_back({"8", "g"});
  kPath path;
  bool found = g.findPath("4", "8", path);

  g.printPath(path);
  ASSERT_EQ(found, true);
  ASSERT_EQ(equalPath(path, answer), true);

  kPath answer_two;
  answer_two.push_back({"2", "c"});
  answer_two.push_back({"5", "d"});

  kPath path_two;
  bool found_two = g.findPath("4", "5", path_two);

  g.printPath(path_two);
  ASSERT_EQ(found_two, true);
  ASSERT_EQ(equalPath(path_two, answer_two), true);

  kPath answer_three;
  answer_three.push_back({"1", "b"});
  answer_three.push_back({"2", "a"});
  answer_three.push_back({"5", "d"});

  kPath path_three;
  bool found_three = g.findPath("3", "5", path_three);

  g.printPath(path_three);
  ASSERT_EQ(found_three, true);
  ASSERT_EQ(equalPath(path_three, answer_three), true);

  kPath answer_four;
  answer_four.push_back({"3", "g"});
  answer_four.push_back({"1", "b"});
  answer_four.push_back({"2", "a"});
  answer_four.push_back({"5", "d"});

  kPath path_four;
  bool found_four = g.findPath("8", "5", path_four);

  g.printPath(path_four);
  ASSERT_EQ(found_four, true);
  ASSERT_EQ(equalPath(path_four, answer_four), true);

} // End of caseOne

// TEST_F(PandaExample, caseTwo)
//{
//
//  // graph of the robot kinematics:
//  Graph rg(modelPtr);
//  // rg.printGraph();
//
//  kPath path, pathTwo, pathThree;
//
//  bool found = rg.findPath("panda_link0", "panda_link7", path);
//  rg.printPath(path);
//
//  ASSERT_EQ(found, true);
//
//  bool foundTwo = rg.findPath("main_link", "panda_foot", pathTwo);
//  rg.printPath(pathTwo);
//
//  ASSERT_EQ(foundTwo, true);
//
//  bool foundThree = rg.findPath("panda_foot", "main_link", pathThree);
//  rg.printPath(pathThree);
//
//  ASSERT_EQ(foundThree, true);
//
//} // End of caseTwo

} // namespace GeoRobotics
