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
#include <chrono>
#include <limits>

namespace GeoRobotics
{
struct ScaraParams
{
  double l1 = 0.2;
  double l2 = 0.3;
  double l0 = 0.4;
};

struct ThreeLinkParams
{
  double l1 = 0.5;
  double l2 = 0.5;
  double l0 = 0.6;
};

struct ElbowParams
{
  double l1 = 0.5;
  double l2 = 0.5;
  double l0 = 0.6;
};

struct StanfordParams
{
  double l1 = 0.4;
  double l0 = 0.6;
};

class InverseDynamicsExample : public testing::Test
{
protected:
  Parser parser;
  std::shared_ptr<Model> modelPtr;
  std::shared_ptr<KinematicChain> pandaRobot;
  std::shared_ptr<KinematicTree> pKPtr;

  std::shared_ptr<Model> scaraPtr;
  std::shared_ptr<KinematicChain> scaraRobot;
  std::shared_ptr<KinematicTree> sKPtr;

  std::shared_ptr<Model> elbowPtr;
  std::shared_ptr<KinematicChain> elbowRobot;
  std::shared_ptr<KinematicTree> eKPtr;

  std::shared_ptr<Model> sdPtr;
  std::shared_ptr<KinematicChain> sdRobot;
  std::shared_ptr<KinematicTree> sdKPtr;

  std::shared_ptr<Model> krPtr;
  std::shared_ptr<KinematicChain> krRobot;
  std::shared_ptr<KinematicTree> krKPtr;

  std::shared_ptr<Model> tPtr;
  std::shared_ptr<KinematicChain> tRobot;
  std::shared_ptr<KinematicTree> tKPtr;
  std::vector<std::shared_ptr<KinematicChain>> robots;

  ScaraParams sParams;
  StanfordParams sdParams;
  ElbowParams eParams;
  ThreeLinkParams tParams;

  Matrix3d scara_rotaiton;
  Vector3d scara_translation;

  double abs_error = 1e-5;
  double tf_error = 1e-3;
  double adg_error = 3e-3;

  // double jd_step_size = 0.0001;
  // double dt = 0.001;

  double jd_step_size = 0.0002;
  double dt = 0.001;

  // double md_step_size = 0.1;
  // double mdt = 1;

  void SetUp() override
  {
    modelPtr = parser.parseURDFFile("@RobotExamplesPath@panda_foot.urdf");
    pKPtr = std::make_shared<KinematicTree>(modelPtr);

    pandaRobot = pKPtr->createKinematicChain("panda", "panda_link0", "panda_link8");
    robots.push_back(pandaRobot);

    // Create the 3DOF SCARA robot shown in Fig.3.3 of Murray et al. 1994
    scaraPtr = parser.parseURDFFile("@RobotExamplesPath@SCARA.urdf");
    sKPtr = std::make_shared<KinematicTree>(scaraPtr);

    scaraRobot = sKPtr->createKinematicChain("scara", "base_link", "EndEffector");

    scaraRobot->printKinematicChain();

    robots.push_back(scaraRobot);

    // Create the ELBOW Manipulator shown in Fig.3.4 of Murray et al. 1994
    elbowPtr = parser.parseURDFFile("@RobotExamplesPath@ELBOW.urdf");

    eKPtr = std::make_shared<KinematicTree>(elbowPtr);

    elbowRobot = eKPtr->createKinematicChain("elbow", "base_link", "EndEffector");
    elbowRobot->printKinematicChain();

    robots.push_back(elbowRobot);

    // Create the KR5
    krPtr = parser.parseURDFFile("@RobotExamplesPath@KR5/KR5.urdf");
    krKPtr = std::make_shared<KinematicTree>(krPtr);

    krRobot = krKPtr->createKinematicChain("krRobot", "base_link", "palm");

    robots.push_back(krRobot);

    sParams.l0 = 0.4;
    sParams.l1 = 0.5;
    sParams.l2 = 0.5;

    eParams.l0 = 0.6;
    eParams.l1 = 0.5;
    eParams.l2 = 0.5;

    // Create the STANFORD Manipulator shown in Fig.3.14 of Murray et al. 1994
    sdPtr = parser.parseURDFFile("@RobotExamplesPath@STANFORD.urdf");

    sdKPtr = std::make_shared<KinematicTree>(sdPtr);

    sdRobot = sdKPtr->createKinematicChain("stanford", "base_link", "EndEffector");
    sdRobot->printKinematicChain();

    robots.push_back(sdRobot);

    // Create the STANFORD Manipulator shown in Fig.4.5 of Murray et al. 1994
    tPtr = parser.parseURDFFile("@RobotExamplesPath@ThreeLinks.urdf");

    tKPtr = std::make_shared<KinematicTree>(tPtr);

    tRobot = tKPtr->createKinematicChain("ThreeLinkRobot", "base_link", "EndEffector");
    tRobot->printKinematicChain();
    robots.push_back(tRobot);
  }

  double c(const double & q)
  {
    return cos(q);
  }
  double s(const double & q)
  {
    return sin(q);
  }

  void randomJointVel(const VectorXd & q, std::default_random_engine & re, VectorXd & dq, VectorXd & qNext)
  {

    int dof = static_cast<int>(q.size());

    dq.resize(dof);
    qNext.resize(dof);
    for(int ii = 0; ii < dof; ii++)
    {
      double delta_q = grNumber(re, -1.0 * jd_step_size, 1.0 * jd_step_size);
      qNext(ii) = q(ii) + delta_q;
      dq(ii) = delta_q / dt;
    }
  }

  void randomPosition(const int & iters,
                      std::shared_ptr<KinematicChain> & robot,
                      void func(std::shared_ptr<KinematicChain> & robot, const double & threshold))
  {
    std::default_random_engine re;

    for(int i = 0; i < iters; i++)
    {
      robot->setRandom(0, re);
      // Run the actual function here
      func(robot, tf_error);
    }
  }

  void testSpeed(std::shared_ptr<KinematicChain> & robot)
  {
    std::default_random_engine re;

    std::vector<double> v;
    for(int i = 0; i < 5000; i++)
    {

      robot->setRandom(2, re);
      auto startUpdate = std::chrono::high_resolution_clock::now();

      robot->IdCalc(robot->jointPositions(), robot->jointVelocities(), robot->jointAccelerations());
      // VectorXd cc;
      // robot->CoriolisAndCentrifugalCalc(cc);

      auto stopModelUpdate = std::chrono::high_resolution_clock::now();

      auto durationModelUpdate = std::chrono::duration_cast<std::chrono::microseconds>(stopModelUpdate - startUpdate);

      v.push_back(static_cast<double>(durationModelUpdate.count()));
    }

    double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
    double m = sum / v.size();

    double accum = 0.0;
    std::for_each(std::begin(v), std::end(v), [&](const double d) { accum += (d - m) * (d - m); });

    double stdev = sqrt(accum / (v.size() - 1));

    RoboticsUtils::quickHL("For robot: " + robot->name() + ", the mean update time is: " + std::to_string(m)
                           + " microseconds, and the standard deviation is: " + std::to_string(stdev)
                           + " microseconds.");
  }
  void testRENA_jacobian(std::shared_ptr<KinematicChain> & robot)
  {
    std::default_random_engine re;

    for(int i = 0; i < 5000; i++)
    {

      robot->setRandom(2, re);

      robot->IdCalc(robot->jointPositions(), robot->jointVelocities(), robot->jointAccelerations());

      // Check the transform of each joint frame
      for(int ii = 0; ii < robot->dof(); ii++)
      {
        const std::shared_ptr<GeoLink> & l = robot->link(ii);
        const VectorXd & dq = robot->jointVelocities().head(ii + 1);
        const MatrixXd & diff = robot->bodyJacobian(l->parentJointName()) * dq - l->bodyVelocity();
        ASSERT_NEAR(diff.norm(), 0.0, tf_error);
      }
    }

  } // end of testRENA_jacobian

  void testRENA_transform(std::shared_ptr<KinematicChain> & robot)
  {
    std::default_random_engine re;

    for(int i = 0; i < 5000; i++)
    {

      robot->setRandom(2, re);

      robot->IdCalc(robot->jointPositions(), robot->jointVelocities(), robot->jointAccelerations());

      // Check the transform of each joint frame
      for(int ii = 0; ii < robot->dof(); ii++)
      {
        const MatrixXd & diff =
            robot->transform(robot->joint(ii)->name()).matrix() - robot->joint(ii)->transform().matrix();
        ASSERT_NEAR(diff.norm(), 0.0, tf_error);
      }
    }

  } // end of testRENA_transform

  void testRENA_tau(std::shared_ptr<KinematicChain> & robot)
  {
    std::default_random_engine re;

    VectorXd q, dq, ddq;
    q.resize(robot->dof());
    dq.resize(robot->dof());
    ddq.resize(robot->dof());

    for(int i = 0; i < 5000; i++)
    {

      robot->setRandom(2, re);
      robot->readJointPositions(q);
      robot->readJointVelocities(dq);
      robot->readJointAccelerations(ddq);

      robot->IdCalc(q, dq, ddq);

      VectorXd test_tau_0 = robot->M() * ddq + robot->C() * dq;

      // printMatrix("book tau 0", test_tau_0);
      VectorXd test_tau = robot->M() * ddq + robot->C() * dq + robot->N();

      // printMatrix("book tau", test_tau);
      VectorXd tau_RNEA;
      robot->readJointTorques(tau_RNEA);
      // printMatrix("RNEA tau", tau_RNEA);

      ASSERT_NEAR((tau_RNEA - test_tau).norm(), 0.0, tf_error);
    }

  } // end of testRENA_tau
};

TEST_F(InverseDynamicsExample, case_RNEA_1)
{

  for(auto robot : robots)
  {
    testRENA_transform(robot);
  }

} // End of case_last

TEST_F(InverseDynamicsExample, case_RNEA_2)
{

  for(auto robot : robots)
  {
    testRENA_jacobian(robot);
  }

} // End of case_last

TEST_F(InverseDynamicsExample, case_RNEA_3)
{

  for(auto robot : robots)
  {
    testRENA_tau(robot);
  }

  // testRENA_tau(scaraRobot);
} // End of case_last

TEST_F(InverseDynamicsExample, case_last)
{

  for(auto robot : robots)
  {
    testSpeed(robot);
  }

} // End of case_last

} // namespace GeoRobotics
