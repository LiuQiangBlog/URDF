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
#include "GeometricRobotics/Kinematics/KinematicTree.h"
#include "GeometricRobotics/urdf/Parser.h"
#include "gtest/gtest.h"
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
  double r0 = 0.4;
  double r1 = 0.2;
  double r2 = 0.2;
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

class CoriolisAndCentrifugalExample : public testing::Test
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
  Eigen::MatrixXd stanford_jacobian(const std::vector<double> & q)
  {
    // Example 3.9 of Murray et al. 1994

    size_t dof = q.size();
    Eigen::MatrixXd output;
    output.resize(6, static_cast<int>(dof));
    output.setZero();

    double s0 = s(q[0]);
    double s1 = s(q[1]);
    // double s2 = s(q[2]);
    double s3 = s(q[3]);
    double s4 = s(q[4]);

    double c0 = c(q[0]);
    double c1 = c(q[1]);
    // double c2 = c(q[2]);
    double c3 = c(q[3]);
    double c4 = c(q[4]);

    Vector3d w_1_prime, w_2_prime;
    w_1_prime << 0, 0, 1;
    w_2_prime << -c0, -s0, 0;

    Vector3d qw, q1;
    // clang-format-off
    qw << -(sdParams.l1 + q[2]) * s0 * c1, (sdParams.l1 + q[2]) * c0 * c1, sdParams.l0 - (sdParams.l1 + q[2]) * s1;
    q1 << 0, 0, sdParams.l0;
    // clang-format-on

    /*
    output(5,0) = 1.0;


    output(0,1) = sdParams.l0 * s0;
    output(1,1) = -sParams.l0 * c0;

    output(3,1) = - c0;
    output(4,1) = - s0;
    */
    output.col(0) << -skew(w_1_prime) * q1, w_1_prime;
    output.col(1) << -skew(w_2_prime) * q1, w_2_prime;

    output(0, 2) = -s0 * c1;
    output(1, 2) = c0 * c1;
    output(2, 2) = -s1;

    Vector3d w_4_prime, w_5_prime, w_6_prime;

    w_4_prime << -s0 * s1, c0 * s1, c1;

    w_5_prime << -c0 * c3 + s0 * c1 * s3, -s0 * c3 - c0 * c1 * s3, s1 * s3;

    w_6_prime << -c4 * (s0 * c1 * c3 + c0 * s3) + s0 * s1 * s4, c4 * (c0 * c1 * c3 - s0 * s3) - c0 * s1 * s4,
        -s1 * c3 * c4 - c1 * s4;

    output.col(3) << -skew(w_4_prime) * qw, w_4_prime;
    output.col(4) << -skew(w_5_prime) * qw, w_5_prime;
    output.col(5) << -skew(w_6_prime) * qw, w_6_prime;

    return output;
  }
  Eigen::MatrixXd scara_jacobian(const std::vector<double> & q)
  {
    // Example 3.8 of Murray et al. 1994

    size_t dof = q.size();
    Eigen::MatrixXd output;
    output.resize(6, static_cast<int>(dof));
    output.setZero();

    output(5, 0) = 1.0;

    output(0, 1) = sParams.l1 * c(q[0]);
    output(1, 1) = sParams.l1 * s(q[0]);
    output(5, 1) = 1.0;

    output(0, 2) = sParams.l2 * c(q[0]) + sParams.l2 * c(q[0] + q[1]);
    output(1, 2) = sParams.l1 * s(q[0]) + sParams.l2 * s(q[0] + q[1]);
    ;
    output(5, 2) = 1.0;

    output(2, 3) = 1.0;

    return output;
  }

  VectorXd scara_N()
  {
    // See example 4.4 on Page 196 of Murray et al. 1994.
    VectorXd N;
    N.resize(4);
    N.setZero();
    auto ls = scaraRobot->links();
    double m4 = ls[3]->mass();

    N(3) = -m4 * GRAVITY;

    return N;
  }
  MatrixXd scara_cc(const VectorXd & q, const VectorXd & dq)
  {
    // See example 4.4 on Page 196 of Murray et al. 1994.
    MatrixXd C;
    C.resize(4, 4);
    C.setZero();

    // double r1 = sParams.l1/2;
    double r2 = sParams.l2 / 2;

    auto ls = scaraRobot->links();
    double m2 = ls[1]->mass();
    double m3 = ls[2]->mass();
    double m4 = ls[3]->mass();

    double gama = sParams.l1 * sParams.l2 * m3 + sParams.l1 * sParams.l2 * m4 + sParams.l1 * m2 * r2;

    C(0, 0) = -gama * s(q(1)) * dq(1);
    C(0, 1) = -gama * s(q(1)) * (dq(0) + dq(1));
    C(1, 0) = gama * s(q(1)) * dq(0);

    return C;
  }

  VectorXd tRobot_N(const VectorXd & q)
  {
    // See example 4.3 on Page 190 of Murray et al. 1994.
    VectorXd N;
    N.resize(3);
    N.setZero();

    double l1 = tParams.l1;
    double r1 = tParams.r1;
    double r2 = tParams.r2;

    auto ls = tRobot->links();

    double m2 = ls[1]->mass();
    double m3 = ls[2]->mass();

    // double gama = sParams.l1 * sParams.l2 * m3 + sParams.l1 * sParams.l2 * m4 + sParams.l1 * m2 * r2;

    double g = -GRAVITY;
    const double & temp = m3 * r2 * cos(q(1) + q(2));
    N(1) = -g * (m2 * r1 + m3 * l1) * cos(q(1)) - g * temp;
    N(2) = -g * temp;

    return N;
  }
  Iso3d scara_transform(const std::vector<double> & q)
  {
    Iso3d output;
    output.setIdentity();

    Matrix3d R;
    R.setIdentity();
    R << cos(q[0] + q[1] + q[2]), -sin(q[0] + q[1] + q[2]), 0, sin(q[0] + q[1] + q[2]), cos(q[0] + q[1] + q[2]), 0, 0,
        0, 1;

    output.linear() = R;

    output.translation() << -sParams.l1 * sin(q[0]) - sParams.l2 * sin(q[0] + q[1]),
        sParams.l1 * cos(q[0]) + sParams.l2 * cos(q[0] + q[1]), sParams.l0 + q[3];

    return output;
  }

  void TearDown() override {}
  void testBodyJacobianDerivativeForEachJoint(std::shared_ptr<KinematicChain> & robot)
  {
    VectorXd q, new_q, dq;

    q.resize(robot->dof());
    new_q.resize(robot->dof());
    dq.resize(robot->dof());

    q.setZero();
    new_q.setZero();
    dq.setZero();

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      new_q = q;

      for(auto joint : robot->joints())
      {
        MatrixXd jacobian = robot->bodyJacobian(joint->name());
        // Iterate over the joints
        for(int ii = 0; ii < jacobian.cols(); ii++)
        {
          new_q(ii) += jd_step_size;
          dq = (new_q - q) / dt;

          robot->setJointPositions(new_q);
          robot->setJointVelocities(dq);

          MatrixXd jacobiand = robot->computeBodyJacobianDerivative(robot->jointNameToIndex(joint->name()), ii);

          // MatrixXd delta_jacobian = robot->computeBodyJacobian(joint->name());
          MatrixXd delta_jacobian = robot->bodyJacobian(joint->name());

          MatrixXd numerical_jacobiand = (delta_jacobian - jacobian) / dt;

          // std::cout<<"Jacobian: "<<robot->jointNameToIndex(joint->name())<<", joint------------------: "<<ii<<" dq
          // is: "<<dq(ii)<<std::endl; printMatrix("computed Jacobian derivative:", jacobiand * dq(ii));
          // printMatrix("numerical Jacobian derivative", numerical_jacobiand);
          ASSERT_NEAR((numerical_jacobiand - jacobiand * dq(ii)).norm(), 0.0, tf_error);

          robot->setJointPositions(q);
          new_q(ii) -= jd_step_size;
        }
      }
    }
  }
  void testSpatialJacobianDerivativeForEachJoint(std::shared_ptr<KinematicChain> & robot)
  {
    VectorXd q, new_q, dq;

    q.resize(robot->dof());
    new_q.resize(robot->dof());
    dq.resize(robot->dof());

    q.setZero();
    new_q.setZero();
    dq.setZero();

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      new_q = q;

      for(auto joint : robot->joints())
      {
        MatrixXd jacobian = robot->spatialJacobian(joint->name());
        for(int ii = 0; ii < robot->dof(); ii++)
        {
          new_q(ii) += jd_step_size;
          dq = (new_q - q) / dt;

          robot->setJointPositions(new_q);
          robot->setJointVelocities(dq);

          const MatrixXd & jacobiand =
              robot->computeSpatialJacobianDerivative(robot->jointNameToIndex(joint->name()), ii);

          // MatrixXd delta_jacobian = robot->computeBodyJacobian(joint->name());
          const MatrixXd & delta_jacobian = robot->spatialJacobian(joint->name());

          const MatrixXd & numerical_jacobiand = (delta_jacobian - jacobian) / dt;

          // std::cout<<"joint: "<<ii<<" dq is: "<<dq(ii)<<std::endl;
          // printMatrix("computed Jacobian derivative:", jacobiand * dq(ii));
          // printMatrix("numerical Jacobian derivative", numerical_jacobiand);
          ASSERT_NEAR((numerical_jacobiand - jacobiand * dq(ii)).norm(), 0.0, tf_error);

          robot->setJointPositions(q);
          new_q(ii) -= jd_step_size;
        }
      }
    }
  }
  void testTRobot()
  {
    VectorXd q; // new_q,
    std::shared_ptr<KinematicChain> & robot = tRobot;
    q.resize(robot->dof());
    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      // std::cout<<"iteration: " << i <<"--------"<<std::endl;
      // std::cout<<"potential energy: " << robot->potentialEnergy() <<"--------"<<std::endl;
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      // new_q = q;
      // printMatrix("joint velocities", dq);

      // robot->updateCoriolisAndCentrifugalForces_();
      const VectorXd & n = tRobot_N(q);
      const VectorXd & N = robot->N();

      // std::cout<<"urdf Robot r1: "<<robot->link(0)->localCOM()<<std::endl;
      // std::cout<<"params Robot r1: "<<tParams.r1<<std::endl;
      // printMatrix("N", N);
      // printMatrix("book N", n);
      // printMatrix("q", q);
      ASSERT_NEAR((N - n).norm(), 0.0, tf_error);
    }
  }
  void testScara()
  {
    VectorXd q, // new_q,
        dq, ddq;
    std::shared_ptr<KinematicChain> & robot = scaraRobot;
    q.resize(robot->dof());
    // new_q.resize(robot->dof());
    dq.resize(robot->dof());
    ddq.resize(robot->dof());

    // q.setZero();
    // new_q.setZero();
    // dq.setZero();

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {

      // std::cout << "iteration: " << i << "--------" << std::endl;
      robot->setRandom(2, re);
      robot->readJointPositions(q);
      robot->readJointVelocities(dq);
      robot->readJointAccelerations(ddq);
      // new_q = q;
      // printMatrix("joint velocities", dq);

      // robot->updateCoriolisAndCentrifugalForces_();
      const MatrixXd & cc = scara_cc(q, dq);
      const MatrixXd & C = robot->C();
      // printMatrix("C", C);
      // printMatrix("book C", cc);
      // printMatrix("book C * dq", cc * dq);
      // printMatrix("dq", dq);
      // printMatrix("Md", robot->Md());
      // printMatrix("C + C.T", robot->Md());

      // const MatrixXd & md = robot->Md();
      ASSERT_NEAR((C - cc).norm(), 0.0, tf_error);
      const VectorXd & n = scara_N();
      const VectorXd & N = robot->N();

      // printMatrix("N", N);
      // printMatrix("book N", n);

      ASSERT_NEAR((N - n).norm(), 0.0, tf_error);

      robot->IdCalc(q, dq, ddq);
      // const MatrixXd test = md - 2 * C;
      VectorXd test_tau = robot->M() * ddq + robot->C() * dq + robot->N();

      // printMatrix("book tau", test_tau);
      VectorXd tau_RNEA;
      robot->readJointTorques(tau_RNEA);

      ASSERT_NEAR((tau_RNEA - test_tau).norm(), 0.0, tf_error);
      // printMatrix("RNEA tau", tau_RNEA);

      VectorXd cc_RNEA;
      robot->CoriolisAndCentrifugalCalc(cc_RNEA);
      // printMatrix("cc_RNEA", cc_RNEA);

      ASSERT_NEAR((cc_RNEA - C * dq).norm(), 0.0, tf_error);
    }
  }
  void compareCC(std::shared_ptr<KinematicChain> & robot)
  {
    // VectorXd q, new_q, dq;

    // q.resize(robot->dof());
    // new_q.resize(robot->dof());
    // dq.resize(robot->dof());

    // q.setZero();
    // new_q.setZero();
    // dq.setZero();

    std::default_random_engine re;
    VectorXd q, dq, ddq;
    q.resize(robot->dof());
    dq.resize(robot->dof());
    ddq.resize(robot->dof());

    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(2, re);
      robot->readJointPositions(q);
      robot->readJointVelocities(dq);
      robot->readJointAccelerations(ddq);

      const MatrixXd & C = robot->C();

      robot->IdCalc(q, dq, ddq);
      VectorXd cc_RNEA;
      robot->CoriolisAndCentrifugalCalc(cc_RNEA);
      printMatrix("cc_RNEA", cc_RNEA);

      ASSERT_NEAR((cc_RNEA - C * dq).norm(), 0.0, tf_error);
    }
  }
  void testCC(std::shared_ptr<KinematicChain> & robot)
  {
    // VectorXd q, new_q, dq;

    // q.resize(robot->dof());
    // new_q.resize(robot->dof());
    // dq.resize(robot->dof());

    // q.setZero();
    // new_q.setZero();
    // dq.setZero();

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(1, re);
      // robot->readJointPositions(q);
      // robot->readJointVelocities(dq);
      // new_q = q;
      // printMatrix("joint velocities", dq);

      // robot->updateCoriolisAndCentrifugalForces_();
      const MatrixXd & C = robot->C();
      // printMatrix("C", C);
      // printMatrix("dq", dq);
      // printMatrix("Md", robot->Md());
      // printMatrix("C + C.T", robot->Md());

      const MatrixXd & md = robot->Md();
      ASSERT_NEAR((md - (C + C.transpose())).norm(), 0.0, tf_error);

      const MatrixXd & test = md - 2 * C;
      ASSERT_NEAR((test + test.transpose()).norm(), 0.0, tf_error);
    }
  }

  // Test skew-symmetry
  void testInertiaMatrix(std::shared_ptr<KinematicChain> & robot)
  {
    // std::vector<double> tq, tdq;
    VectorXd tdq;
    std::default_random_engine re;
    for(int i = 0; i < 100; i++)
    {

      robot->setRandom(1, re);
      // robot->readJointPositions(tq);
      robot->readJointVelocities(tdq);
      // Eigen::VectorXd dq = Eigen::Map<Eigen::VectorXd>(tdq.data(), static_cast<int>(tdq.size()));

      // printMatrix("joint velocities", dq);

      // double es = (tdq * robot->M() * tdq.transpose())(0);
      // double bs = (tdq * robot->Mb() * tdq.transpose())(0);

      // ASSERT_NEAR(es - bs, 0.0, tf_error);

      // 1. Symmetry
      const MatrixXd & mb = robot->M();
      const MatrixXd & m = robot->computeMs();

      ASSERT_NEAR((mb - mb.transpose()).norm(), 0.0, tf_error);
      ASSERT_NEAR((m - m.transpose()).norm(), 0.0, tf_error);

      // clang-format off
    // 2. positive definite 
    // Check the Stack-overflow post: https://stackoverflow.com/questions/35227131/eigen-check-if-matrix-is-positive-semi-definite
      // clang-format on

      Eigen::LLT<Eigen::MatrixXd> lltM(m);
      Eigen::LLT<Eigen::MatrixXd> lltMb(mb);
      ASSERT_TRUE(lltM.info() != Eigen::NumericalIssue);
      ASSERT_TRUE(lltMb.info() != Eigen::NumericalIssue);
    }
  }
  void testMDot(std::shared_ptr<KinematicChain> & robot)
  {
    VectorXd q, new_q, dq;

    q.resize(robot->dof());
    new_q.resize(robot->dof());
    dq.resize(robot->dof());

    q.setZero();
    new_q.setZero();
    dq.setZero();

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      new_q = q;

      MatrixXd m = robot->M();
      for(int ii = 0; ii < robot->dof(); ii++)
      {
        // std::cout<<"joint: "<<ii <<"-----------------"<<std::endl;

        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt;

        robot->setJointPositions(new_q);
        robot->setJointVelocities(dq);

        // Only the iith joint velocity is non-zero:
        // MatrixXd mbd = robot->Md();
        MatrixXd delta_m = robot->M();

        MatrixXd numerical_md = (delta_m - m) / dt;

        MatrixXd c_md = robot->computeMd(ii) * dq(ii);

        // printMatrix("Numerical Matirx derivative", numerical_md );
        // printMatrix("dq", dq);
        // std::cout<<"dq_i is: "<<dq(ii)<<std::endl;
        // printMatrix("New matirx derivative", c_mbd);

        ASSERT_NEAR((numerical_md - c_md).norm(), 0.0, tf_error);

        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      }
    }
  }

  void testBodyJacobianDot(std::shared_ptr<KinematicChain> & robot)
  {
    VectorXd q, new_q, dq;

    q.resize(robot->dof());
    new_q.resize(robot->dof());
    dq.resize(robot->dof());

    q.setZero();
    new_q.setZero();
    dq.setZero();

    std::default_random_engine re;

    for(int i = 0; i < 1; i++)
    {
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      new_q = q;

      // robot->updateCoriolisAndCentrifugalForces_();
      MatrixXd C = robot->C();
      // printMatrix("C", C);
      // MatrixXd jacobian = robot->spatialJacobian(joint->name());
      for(int ii = 0; ii < robot->dof(); ii++)
      {
        // std::cout<<"joint: "<<ii <<"-----------------"<<std::endl;

        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt;

        robot->setJointPositions(new_q);
        robot->setJointVelocities(dq);

        // Only the iith joint is non-zero:

        // Iso3d tf_i = robot->transform(robot->jointIndexToName(ii));
        // for (int jj = 0; jj <robot->dof(); jj++)
        for(int jj = 0; jj < robot->dof(); jj++)
        {
          MatrixXd pd_ji_i = robot->computeBodyJacobianDerivative(robot->jointIndexToName(jj));
          // std::cout << "The " << jj << " jacobian w.r.t. " << ii << " joint derivative:" << std::endl;
          // printMatrix("jacobian derivative", pd_ji_i);

          long size = pd_ji_i.cols();

          // Transform from frame jj->ii: Adg^{-1}_ji = Adg_ij

          // Iso3d tf_j = robot->transform(robot->jointIndexToName(jj));

          MatrixXd cjd = // metaAdgMatrix( tf_j.inverse() * tf_i ) *
              C.block(ii * 6, 0, 6, size) * dq(ii);

          // printMatrix("block", C.block(ii*6, 0, 6, size));
          // printMatrix("dq", dq);
          // std::cout << "dq_i is: " << dq(ii) << std::endl;

          // printMatrix("composed jacobian derivative", cjd);

          // ASSERT_NEAR((cjd - pd_ji_i).norm(), 0.0, tf_error);
        }

        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      }
    }
  }
};

TEST_F(CoriolisAndCentrifugalExample, case2)
{

  testMDot(krRobot);
  testMDot(pandaRobot);
  testMDot(scaraRobot);
  testMDot(elbowRobot);
}

TEST_F(CoriolisAndCentrifugalExample, case3)
{

  for(auto robot : robots)
  {
    testSpatialJacobianDerivativeForEachJoint(robot);
    testBodyJacobianDerivativeForEachJoint(robot);
  }
}

TEST_F(CoriolisAndCentrifugalExample, case1)
{

  for(auto robot : robots)
  {
    testCC(robot);
  }
}

TEST_F(CoriolisAndCentrifugalExample, case5)
{
  for(auto robot : robots)
  {
    compareCC(robot);
  }
}
TEST_F(CoriolisAndCentrifugalExample, case4)
{
  testScara();
  testTRobot();
}

} // namespace GeoRobotics
