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
#include <RoboticsUtils/utils.h>
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

class DynamicsExample : public testing::Test
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

  double jd_step_size = 0.0002; // Amounts to maximum 11.4594 degree per second
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

    scaraRobot = sKPtr->createKinematicChain("scara", "world", "EndEffector");

    scaraRobot->printKinematicChain();

    robots.push_back(scaraRobot);

    // Create the ELBOW Manipulator shown in Fig.3.4 of Murray et al. 1994
    elbowPtr = parser.parseURDFFile("@RobotExamplesPath@ELBOW.urdf");

    eKPtr = std::make_shared<KinematicTree>(elbowPtr);

    elbowRobot = eKPtr->createKinematicChain("elbow", "world", "EndEffector");
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

  void testLinkCOMVel(std::shared_ptr<KinematicChain> & robot)
  {
    // Test the derivative of adjoint transform:
    // d(adg_ab) = - adg_ab * (crossMatrix(V_ba))
    // See Lemma~3 of RobotDynamics

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
      // robot->readJointVelocities(dq);
      new_q = q;
      // printMatrix("joint velocities", dq);

      // auto com = robot->com();
      std::vector<MatrixXd> jbJacs;
      std::vector<MatrixXd> lcomJacs;
      std::vector<Iso3d> tfs;
      std::vector<Vector3d> linkComs;
      for(auto link : robot->links())
      {
        linkComs.push_back(robot->computeLinkCom(link->name()));
        jbJacs.push_back(robot->bodyJacobian(link->parentJointName()));
        lcomJacs.push_back(robot->linkComBodyJac(link->name()));
        tfs.push_back(robot->transform(link->parentJointName()));
      }
      // printMatrix("com ", com);
      // std::cout << "Iteration: =========" << i << " ========================================" << std::endl;
      for(int ii = 0; ii < robot->dof(); ii++)
      {

        // std::cout << "section: =========" << ii << " ========================================" << std::endl;
        // Set the new vel
        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt;
        robot->setJointPositions(new_q);
        robot->setJointVelocities(dq);

        std::vector<Vector3d> newLinkComs;
        // Check the link com vel of each link
        for(auto link : robot->links())
        {
          newLinkComs.push_back(robot->computeLinkCom(link->name()));
        }
        for(size_t ii = 0; ii < static_cast<size_t>(robot->dof()); ii++)
        {

          // const MatrixXd & bodyJac = jbJacs[ii];
          // auto linearJac = bodyJac.block(0, 0, 3, bodyJac.cols());
          // auto angularJac = bodyJac.block(3, 0, 3, bodyJac.cols());

          auto link = robot->links()[ii];

          // std::cout << "link: =========" << link->name() << " ========================================" << std::endl;
          const Vector3d & linkVel = (newLinkComs[ii] - linkComs[ii]) / dt;
          // printMatrix("Numerical link vel", linkVel);
          // const Vector3d & computedVel = tfs[ii].rotation() * ((-skew(link->localCOM()) * angularJac + linearJac) *
          // dq);
          // const Vector6d & bodyVel = bodyJac * dq;
          // const Vector3d & computedVel = tfs[ii].rotation() * (skew(bodyVel.tail(3)) * link->localCOM() +
          // bodyVel.head(3)); const Vector3d & computedVel = tfs[ii].rotation() * ( - skew(link->localCOM()) *
          // bodyVel.tail(3) + bodyVel.head(3));
          const Vector3d & computedVel = tfs[ii].rotation() *
                                         //(-skew(link->localCOM()) * angularJac + linearJac)
                                         lcomJacs[ii] * dq.head(lcomJacs[ii].cols());

          // printMatrix("Computed link vel", computedVel);

          ASSERT_NEAR((linkVel - computedVel).norm(), 0.0, tf_error);
          ASSERT_NEAR((linkVel - robot->linkComVelocity(link->name())).norm(), 0.0, tf_error);
          ASSERT_NEAR((computedVel - robot->linkComVelocity(link->name())).norm(), 0.0, tf_error);
        }

        // Set the joint position back
        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      }
    }
  }
  void testCMMTwo(std::shared_ptr<KinematicChain> & robot)
  {
    // Compute two ways of computing the centroidal inertia:
    //   -- Orin's approach
    //   -- Direct inertia transform

    VectorXd q, new_q, dq;

    q = VectorXd::Zero(robot->dof());
    new_q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(2, re);
      robot->readJointPositions(q);
      // RoboticsUtils::quickPrint(robot->name() + " joint positions: ", q);
      // robot->readJointVelocities(dq);
      new_q = q;
      // printMatrix("joint velocities", dq);

      const MatrixXd & cbJac = robot->comBodyJac();
      auto com = robot->com();
      const MatrixXd & cmm = robot->computeCMM();
      const MatrixXd & cmmTwo = robot->computeCMMTwo();
      const Matrix6d & ci = robot->computeCentroidalInertia();
      const Matrix6d & ci2 = robot->computeCentroidalInertiaTwo();

      // RoboticsUtils::quickPrint("ci", ci);
      // RoboticsUtils::quickPrint("ci2", ci2);

      ASSERT_NEAR((ci - ci2).norm(), 0.0, tf_error);
      ASSERT_NEAR((cmm - cmmTwo).norm(), 0.0, tf_error);
      ASSERT_NEAR((cmm.topRows(3) - robot->mass() * cbJac).norm(), 0.0, tf_error);
      //// printMatrix("com ", com);
      // for(int ii = 0; ii < robot->dof(); ii++)
      //{
      //  new_q(ii) += jd_step_size;
      //  dq = (new_q - q) / dt;

      //  robot->setJointPositions(new_q);
      //  robot->setJointVelocities(dq);

      //  auto new_com = robot->com();

      //  // printMatrix("new com ", new_com);
      //  // printMatrix("dq", dq);
      //  const Vector3d & numerical_comd = (new_com - com) / dt;
      //  const Vector3d comd = cbJac * dq;
      //  // printMatrix("numerical comd", numerical_comd);
      //  // printMatrix("computed comd", comd);
      //  ASSERT_NEAR((numerical_comd - comd).norm(), 0.0, tf_error);

      //  const Vector6d & h = cmm * dq;

      //  // printMatrix("cmm", cmm);
      //  // printMatrix("numerical comd", numerical_comd);
      //  // printMatrix("comd", comd);
      //  // std::cout<<"Mass is: "<<robot->mass()<<std::endl;
      //  // printMatrix("h/m", h.head(3)/robot->mass());

      //  ASSERT_NEAR((comd - h.head(3) / robot->mass()).norm(), 0.0, tf_error);

      //  const Vector6d & av = ci.colPivHouseholderQr().solve(cmm * dq);
      //  // robot->computeAverageVelocity(ci, cmm);

      //  ASSERT_NEAR((comd - av.head(3)).norm(), 0.0, tf_error);

      //  robot->setJointPositions(q);
      //  new_q(ii) -= jd_step_size;
      //}
    }
  }
  void testCMM(std::shared_ptr<KinematicChain> & robot)
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
      // robot->readJointVelocities(dq);
      new_q = q;
      // printMatrix("joint velocities", dq);

      const MatrixXd & cbJac = robot->comBodyJac();
      auto com = robot->com();
      const MatrixXd & cmm = robot->computeCMM();
      const Matrix6d & ci = robot->computeCentroidalInertia();

      // printMatrix("com ", com);
      for(int ii = 0; ii < robot->dof(); ii++)
      {
        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt;

        robot->setJointPositions(new_q);
        robot->setJointVelocities(dq);

        auto new_com = robot->com();

        // printMatrix("new com ", new_com);
        // printMatrix("dq", dq);
        const Vector3d & numerical_comd = (new_com - com) / dt;
        const Vector3d comd = cbJac * dq;
        // printMatrix("numerical comd", numerical_comd);
        // printMatrix("computed comd", comd);
        ASSERT_NEAR((numerical_comd - comd).norm(), 0.0, tf_error);

        const Vector6d & h = cmm * dq;

        // printMatrix("cmm", cmm);
        // printMatrix("numerical comd", numerical_comd);
        // printMatrix("comd", comd);
        // std::cout<<"Mass is: "<<robot->mass()<<std::endl;
        // printMatrix("h/m", h.head(3)/robot->mass());

        ASSERT_NEAR((comd - h.head(3) / robot->mass()).norm(), 0.0, tf_error);

        const Vector6d & av = ci.colPivHouseholderQr().solve(cmm * dq);
        // robot->computeAverageVelocity(ci, cmm);

        ASSERT_NEAR((comd - av.head(3)).norm(), 0.0, tf_error);

        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      }
    }
  }
  void testEndEffector(std::shared_ptr<KinematicChain> & robot)
  {
    VectorXd q, new_q, dq;

    q.resize(robot->dof());
    new_q.resize(robot->dof());
    dq.resize(robot->dof());

    q.setZero();
    new_q.setZero();
    dq.setZero();
    std::default_random_engine re;

    // Assign an endEffector at the com of each link, then test the transform and the bodyJacobian.
    for(auto l : robot->links())
    {
      robot->createEndEffector(l->name() + "_endEffector", l->parentJointName(), l->localCOMTransform());
    }

    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      // robot->readJointVelocities(dq);
      new_q = q;
      // printMatrix("joint velocities", dq);

      for(auto e : robot->endEffectors())
      {
        auto ee = e.second;

        auto lName = ee->parentLink()->name();

        const Vector3d & diff = robot->computeLinkCom(lName) - ee->transform().translation();
        ASSERT_NEAR(diff.norm(), 0.0, tf_error);

        long dof = ee->bodyJacobian().cols();
        const MatrixXd & jacDiff = robot->linkComBodyJac(lName) - ee->bodyJacobian().block(0, 0, 3, dof);
        ASSERT_NEAR(jacDiff.norm(), 0.0, tf_error);
      }
    }
    for(auto l : robot->links())
    {
      robot->removeEndEffector(l->name() + "_endEffector");
    }
  }

  void testCOMJac(std::shared_ptr<KinematicChain> & robot)
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
      // robot->readJointVelocities(dq);
      new_q = q;
      // printMatrix("joint velocities", dq);

      const MatrixXd & cbJac = robot->comBodyJac();
      auto com = robot->com();

      // printMatrix("com ", com);
      for(int ii = 0; ii < robot->dof(); ii++)
      {
        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt;

        robot->setJointVelocities(dq);

        robot->setJointPositions(new_q);
        auto new_com = robot->com();

        // printMatrix("new com ", new_com);
        // printMatrix("dq", dq);
        const Vector3d & numerical_comd = (new_com - com) / dt;
        const Vector3d & comd = cbJac * dq;
        // printMatrix("numerical comd", numerical_comd);
        // printMatrix("computed comd", comd);
        ASSERT_NEAR((numerical_comd - comd).norm(), 0.0, tf_error);

        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      }
    }
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
  /*
   * I should ask the robot to integrate with a random velocity
void randomSingleJointVel(const int& iters, std::shared_ptr<KinematicChain> & robot,  void
func(std::shared_ptr<KinematicChain> & robot, const VectorXd & dq, const double & dt, const double & threshold) )
  {

    VectorXd q, new_q, dq;
    q.resize(robot->dof());
    new_q.resize(robot->dof());
    dq.resize(robot->dof());

    q.setZero();
    new_q.setZero();
    dq.setZero();


    std::default_random_engine re;

    for(int i = 0; i < iters; i++)
    {
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      new_q = q;
      for(int ii = 0; ii < robot->dof(); ii++)
      {
        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt;

        robot->setJointPositions(new_q);
        robot->setJointVelocities(dq);


        // Run the actual function here
        func(robot, dq, dt, tf_error);

  // Set back the joint position
        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      } // End of joints
    } // End of iterations.
  }
  */
  static void testMomentumTransform(std::shared_ptr<KinematicChain> & robot, const double & threshold)
  {

    for(auto link : robot->links())
    {
      const MatrixXd & bodyMomentaJac = robot->computeLinkMomentumJacobian(link->name(), true);

      // printMatrix("BodyMomentaJac", bodyMomentaJac);
      const MatrixXd & spatialMomentaJac = robot->computeLinkMomentumJacobian(link->name(), false);

      // printMatrix("SpatialMomentaJac", bodyMomentaJac);
      // const Iso3d & tf = robot->transform(link->parentJointName());
      // This is the momentum transform from body to inertial frame
      // Vector6d diff = spatialMomentaJac - metaAdgTMatrix(tf.inverse()) * bodyMomentaJac;
      Vector6d diff = spatialMomentaJac
                      - robot->wrenchTransform(link->parentJointName(), "inertial")
                            * bodyMomentaJac; ///< Momentum transform is equivalent to wrench transform
      // printMatrix("diff", diff);
      ASSERT_NEAR(diff.norm(), 0.0, threshold);
      // std::cout<<"-----------------------------------"<<std::endl;
    }
  }
  static void testCOM(std::shared_ptr<KinematicChain> & robot, const double & threshold)
  {

    Matrix6d centroidalInertia;
    // Set up the centroidal frame transform
    Iso3d comtf;
    comtf.setIdentity();
    comtf.translation() = robot->com();
    centroidalInertia.setZero();

    for(auto joint : robot->joints())
    {
      auto tf = robot->transform(joint->name());
      centroidalInertia += joint->childLink()->spatialInertia(comtf.inverse() * tf);
    }

    // printMatrix("centroidal inertia tensor", centroidalInertia);

    const Matrix3d & urCorner = centroidalInertia.block<3, 3>(0, 3);
    const Matrix3d & llCorner = centroidalInertia.block<3, 3>(3, 0);
    const Matrix3d & ulCorner = centroidalInertia.block<3, 3>(0, 0);

    ASSERT_NEAR((urCorner).norm(), 0.0, threshold);
    ASSERT_NEAR((llCorner).norm(), 0.0, threshold);
    ASSERT_NEAR((ulCorner - robot->mass() * Matrix3d::Identity()).norm(), 0.0, threshold);

    const Matrix6d & ci = robot->computeCentroidalInertia();
    ASSERT_NEAR((centroidalInertia - ci).norm(), 0.0, threshold);
    ASSERT_NEAR((ci.block(0, 0, 3, 3) - robot->mass() * Matrix3d::Identity()).norm(), 0.0, threshold);
  }

  /*
  static void metaTestAdjointTransformDerivative(std::shared_ptr<KinematicChain> & robot, const VectorXd & dq, const
  double & dt, const double & threshold)
  {

      auto joint = robot->joints().back();

      Matrix6d adgM = metaAdgMatrix(robot->transform(joint->name()));
  Vector6d bodyVel = robot->bodyJacobian(joint->name()) * dq;
        Matrix6d adgMd = adgM * (crossMatrix(bodyVel));

        Matrix6d delta_adgM = metaAdgMatrix(robot->transform(joint->name()));

        MatrixXd numerical_adgMd = (delta_adgM - adgM) / dt;

        ASSERT_NEAR((numerical_adgMd - adgMd).norm(), 0.0, threshold);


  }
  */
  void testAdjointTransformDerivativeSimple(std::shared_ptr<KinematicChain> & robot)
  {
    // Test the derivative of adjoint transform:
    // d(adg_ab) = adg_ab * (crossMatrix(V_ba))
    // See Lemma~3 of RobotDynamics

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
      robot->setRandom(1, re);
      robot->readJointPositions(q);
      printMatrix("old q", q);

      robot->readJointVelocities(dq);

      auto joint = robot->joints().back();

      const Matrix6d & adgM = metaAdgMatrix(robot->transform(joint->name()));
      const Vector6d & bodyVel = robot->bodyJacobian(joint->name()) * dq;
      const Matrix6d & adgMd = adgM * (crossMatrix(bodyVel));

      // Perform the integration
      robot->velIntegration(dt);

      robot->readJointPositions(new_q);

      printMatrix("new q", new_q);
      // Read the new adgM
      const Matrix6d & delta_adgM = metaAdgMatrix(robot->transform(joint->name()));

      const MatrixXd & numerical_adgMd = (delta_adgM - adgM) / dt;

      // printMatrix ("numerical adgMd", numerical_adgMd);
      // printMatrix ("computed adgMd", adgMd);
      // printMatrix ("numerical_adgMd - adgMd", numerical_adgMd - adgMd);

      ASSERT_NEAR((numerical_adgMd - adgMd).norm(), 0.0, tf_error);
    }
  }

  void testCMMDerivative(std::shared_ptr<KinematicChain> & robot)
  {

    VectorXd q, new_q, dq;

    q = VectorXd::Zero(robot->dof());
    new_q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      // std::cout<<"Iteration: -----------" << i << "------" <<std::endl;
      robot->setRandom(0, re);
      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      Matrix6d const cmm = robot->computeCMMTwo();
      Matrix6d const cmmd = robot->computeCMMDerivative();

      robot->setJointPositions(qNext);

      Matrix6d const delta_cmm = robot->computeCMMTwo();

      MatrixXd const numerical_cmmd = (delta_cmm - cmm) / dt;

      // printMatrix("numerical cmmd", numerical_cmmd);
      // printMatrix("computed cmmd", cmmd);
      ASSERT_NEAR((numerical_cmmd - cmmd).norm(), 0.0, tf_error);
    }
  }
  void testTransformDerivative(std::shared_ptr<KinematicChain> & robot)
  {

    VectorXd q, new_q, dq;

    q = VectorXd::Zero(robot->dof());
    new_q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      // std::cout<<"Iteration: -----------" << i << "------" <<std::endl;
      robot->setRandom(0, re);
      robot->readJointPositions(q);

      VectorXd dq, qNext;

      randomJointVel(q, re, dq, qNext);
      robot->setJointVelocities(dq);

      std::string const a = robot->randomFrameName();
      std::string const b = robot->randomFrameName();

      // RoboticsUtils::quickHL("Body velocity with Reference frame: ", a, " and body frame: ", b);

      Matrix6d const velTf = robot->velTransform(a, b);
      Matrix6d const & velTfd = robot->velTransformDerivative(a, b);

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

      // printMatrix("numerical velTfd", numerical_velTfd);
      // printMatrix("computed velTfd", velTfd);
      ASSERT_NEAR((numerical_velTfd - velTfd).norm(), 0.0, tf_error);
      ASSERT_NEAR((numerical_wrenchTfd - wrenchTfd).norm(), 0.0, tf_error);
      ASSERT_NEAR((numerical_inertiaTfd - inertiaTfd).norm(), 0.0, tf_error);
    }
  }

  void testAdjointTransformDerivative(std::shared_ptr<KinematicChain> & robot)
  {
    // Test the derivative of adjoint transform:
    // d(adg_ab) = - adg_ab * (crossMatrix(V^s_ba)) = adg_ab * (crossMatrix(V^b_ab))
    // See Lemma~3 of RobotDynamics

    VectorXd q, new_q, dq;

    q = VectorXd::Zero(robot->dof());
    new_q = VectorXd::Zero(robot->dof());
    dq = VectorXd::Zero(robot->dof());

    std::default_random_engine re;

    for(int i = 0; i < 100; i++)
    {
      // std::cout<<"Iteration: -----------" << i << "------" <<std::endl;
      robot->setRandom(0, re);
      robot->readJointPositions(q);
      // printMatrix("old q", q);

      // VectorXd q = robot->jointPositions();
      VectorXd dq, qNext;
      // printMatrix("joint position", q);
      // const Iso3d & tf = robot->transform();

      randomJointVel(q, re, dq, qNext);

      // robot->readJointVelocities(dq);

      // robot->readJointVelocities(dq);
      // new_q = q;
      // printMatrix("joint velocities", dq);
      // printMatrix("new q", qNext);

      auto joint = robot->joints().back();

      const Matrix6d & adgM = metaAdgMatrix(robot->transform(joint->name()));
      const Matrix6d & adgMT = metaAdgTMatrix(robot->transform(joint->name()));

      const Vector6d & bodyVel = robot->bodyJacobian(joint->name()) * dq;
      const Matrix6d & adgMd = adgM * (crossMatrix(bodyVel));
      const Matrix6d & adgMTd = crossMatrix(bodyVel).transpose() * adgM.transpose();

      // Perform the integration
      // robot->velIntegration(dt);

      robot->setJointPositions(qNext);
      // robot->readJointPositions(new_q);

      // Read the new adgM
      const Matrix6d & delta_adgM = metaAdgMatrix(robot->transform(joint->name()));
      const Matrix6d & delta_adgMT = metaAdgTMatrix(robot->transform(joint->name()));

      const MatrixXd & numerical_adgMd = (delta_adgM - adgM) / dt;
      const MatrixXd & numerical_adgMTd = (delta_adgMT - adgMT) / dt;

      // printMatrix ("numerical adgMd", numerical_adgMd);
      // printMatrix ("computed adgMd", adgMd);
      // printMatrix ("numerical_adgMd - adgMd", numerical_adgMd - adgMd);
      // std::cout<<"Error is: "<< (numerical_adgMd - adgMd).norm() <<std::endl;
      ASSERT_NEAR((numerical_adgMd - adgMd).norm(), 0.0, tf_error);
      ASSERT_NEAR((numerical_adgMTd - adgMTd).norm(), 0.0, tf_error);
    }
  }

  Iso3d elbow_transform(const std::vector<double> & q)
  {
    Iso3d output;
    output.setIdentity();

    Matrix3d R;
    R.setIdentity();

    // clang-format off
    R(0, 0) = c(q[5]) * (c(q[0]) * c(q[3]) - s(q[0]) * c(q[1] + q[2]) * s(q[3]))
              + s(q[5])
                    * (s(q[0]) * s(q[1] + q[2]) * c(q[4]) 
		       + s(q[0]) * c(q[1] + q[2]) * c(q[3]) * s(q[4])
                       + c(q[0]) * s(q[3]) * s(q[4]));

    R(0, 1) = -c(q[4]) * (s(q[0]) * c(q[1] + q[2]) * c(q[3]) + c(q[0]) * s(q[3])) 
	    + s(q[0]) * s(q[1] + q[2]) * s(q[4]);
    R(0, 2) =
        c(q[5])
            * (
              -c(q[4]) * s(q[0]) * s(q[1] + q[2]) - (
		      c(q[1] + q[2]) * c(q[3]) * s(q[0]) + c(q[0]) * s(q[3])
		      ) * s(q[4])
	      )
        + (
	  c(q[0]) * c(q[3]) - c(q[1] + q[2]) * s(q[0]) * s(q[3])
	  ) * s(q[5]);

    R(1, 0) =
        c(q[5]) * (c(q[3]) * s(q[0]) + c(q[0]) * c(q[1] + q[2]) * s(q[3]))
        - (c(q[0]) * c(q[4]) * s(q[1] + q[2]) 
	   + (
	     c(q[0]) * c(q[1] + q[2]) * c(q[3]) - s(q[0]) * s(q[3])
	     ) * s(q[4])
	   )
           * s(q[5]);

    R(1, 1) = c(q[4]) * (c(q[0]) * c(q[1] + q[2]) * c(q[3]) - s(q[0]) * s(q[3])) 
	    - c(q[0]) * s(q[1] + q[2]) * s(q[4]);

    R(1, 2) =
        c(q[5])
            * (c(q[0]) * c(q[4]) * s(q[1] + q[2]) 
               + (c(q[0]) * c(q[1] + q[2]) * c(q[3]) 
	           - s(q[0]) * s(q[3])
		   ) * s(q[4])
	       )
        + (c(q[3]) * s(q[0]) + c(q[0]) * c(q[1] + q[2]) * s(q[3])) * s(q[5]);

    R(2, 0) =
        -c(q[5]) * s(q[1] + q[2]) * s(q[3]) 
	- (
	   c(q[1] + q[2]) * c(q[4]) - c(q[3]) * s(q[1] + q[2]) * s(q[4])
	  ) * s(q[5]);

    R(2, 1) = -c(q[3]) * c(q[4]) * s(q[1] + q[2]) - c(q[1] + q[2]) * s(q[4]);

    R(2, 2) =
        c(q[5]) * (c(q[1] + q[2]) * c(q[4]) - c(q[3]) * s(q[1] + q[2]) * s(q[4])) 
	- s(q[1] + q[2]) * s(q[3]) * s(q[5]);

    // clang-format on 

    output.linear() = R;

    output.translation() << -sin(q[0]) * (eParams.l1 * cos(q[1]) + eParams.l2 * cos(q[1] + q[2])),
        cos(q[0]) * (eParams.l1 * cos(q[1]) + eParams.l2 * cos(q[1] + q[2])),
        eParams.l0 - eParams.l1 * sin(q[1]) - eParams.l2 * sin(q[1] + q[2]);

    return output;
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
    double s2 = s(q[2]);
    double s3 = s(q[3]);
    double s4 = s(q[4]);

    double c0 = c(q[0]);
    double c1 = c(q[1]);
    double c2 = c(q[2]);
    double c3 = c(q[3]);
    double c4 = c(q[4]);

    Vector3d w_1_prime, w_2_prime;
    w_1_prime << 0, 0, 1;
    w_2_prime << -c0, -s0, 0; 

    Vector3d qw, q1;
    // clang-format-off
    qw << - (sdParams.l1  + q[2]) * s0 * c1,
       (sdParams.l1  + q[2]) * c0 * c1,
       sdParams.l0 - (sdParams.l1  + q[2]) * s1;
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


    output(0,2) = - s0 * c1;
    output(1,2) = c0 * c1;
    output(2,2) = - s1;


    Vector3d w_4_prime, w_5_prime, w_6_prime;

    w_4_prime << - s0 * s1, 
	      c0 * s1, 
	      c1;

    w_5_prime << - c0 * c3 + s0 * c1 * s3, 
	      - s0 * c3 - c0 * c1 * s3, 
	      s1 * s3;

    w_6_prime << - c4 * (s0 * c1 * c3 + c0 * s3)  + s0 * s1 * s4, 
	       c4 * (c0 * c1 * c3 - s0 * s3)  - c0 * s1 * s4, 
	       - s1 * c3 * c4  - c1 * s4;

    output.col(3) << - skew(w_4_prime) * qw, w_4_prime;
    output.col(4) << - skew(w_5_prime) * qw, w_5_prime;
    output.col(5) << - skew(w_6_prime) * qw, w_6_prime;


    return output;
  }
  Eigen::MatrixXd scara_jacobian(const std::vector<double> & q)
  {
    // Example 3.8 of Murray et al. 1994
    
    size_t dof = q.size();
    Eigen::MatrixXd output;
    output.resize(6, static_cast<int>(dof));
    output.setZero();

    output(5,0) = 1.0;
  

    output(0,1) = sParams.l1 * c(q[0]);
    output(1,1) = sParams.l1 * s(q[0]);
    output(5,1) = 1.0;

    output(0,2) = sParams.l2 * c(q[0]) +  sParams.l2 * c(q[0] + q[1]);
    output(1,2) = sParams.l1 * s(q[0]) +  sParams.l2 * s(q[0] + q[1]);;
    output(5,2) = 1.0;

    output(2,3) = 1.0;

    return output;
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

  void testSubspaces(std::shared_ptr<KinematicChain> & robot)
  {
    for(auto & joint:robot->joints() )
    {
      joint->printBasis();
    }
  }
  void testSpeed(std::shared_ptr<KinematicChain> & robot)
  {
    std::default_random_engine re;  

     std::vector<double> v;
     for (int i = 0; i< 5000; i++)
     {

       auto startUpdate = std::chrono::high_resolution_clock::now();
       robot->setRandom(1, re);

       auto stopModelUpdate = std::chrono::high_resolution_clock::now();
       

       auto durationModelUpdate = std::chrono::duration_cast<std::chrono::microseconds>(stopModelUpdate - startUpdate);

      v.push_back(static_cast<double>(durationModelUpdate.count()));
     }


     double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
     double m =  sum / static_cast<double>(v.size());
     
     double accum = 0.0;
     std::for_each (std::begin(v), std::end(v), [&](const double d) {
         accum += (d - m) * (d - m);
     });
     
     double stdev = sqrt(accum / static_cast<double>(v.size()-1));

     RoboticsUtils::quickHL("For robot: " + robot->name() + ", the mean update time is: " + std::to_string(m) + " microseconds, and the standard deviation is: "+ std::to_string(stdev) + " microseconds.");
  }




  void testInertiaMatrixDerivative(std::shared_ptr<KinematicChain> & robot)
  {
   VectorXd q, new_q, dq;
   
   q.resize(robot->dof());
   new_q.resize(robot->dof());
   dq.resize(robot->dof());

   q.setZero();
   new_q.setZero();
   dq.setZero();

  std::default_random_engine re;  

  for (int i = 0; i< 100; i++)
  {
    robot->setRandom(0, re);
    robot->readJointPositions(q);
    //robot->readJointVelocities(dq);
    new_q = q;
    //printMatrix("joint velocities", dq);

    MatrixXd mb = robot->M();
    MatrixXd m = robot->computeMs();

    for (int ii = 0; ii < robot->dof(); ii++)
    {
      new_q(ii) += jd_step_size;
      dq = (new_q - q) / dt; 

      robot->setJointPositions(new_q);
      robot->setJointVelocities(dq);

      MatrixXd mbd = robot->Md();

      MatrixXd delta_mb = robot->M();
      MatrixXd delta_m = robot->computeMs();
      
      MatrixXd numerical_mbd = (delta_mb - mb)/dt;
      MatrixXd numerical_md = (delta_m - m)/dt;

      
      //std::cout<<"At iteration: "<<i<<", and joint: "<<ii<<std::endl;
      //printMatrix("q:", q);
      //printMatrix("new q:", new_q);
      //printMatrix("read q:", robot->jointPositions());
      //printMatrix("dq:", dq);
      //printMatrix("read dq:", robot->jointVelocities());

     
      
      //printMatrix("numerical inertia tensor derivative:", numerical_mbd);
      //printMatrix("tensor derivative:", mbd);
      MatrixXd diff = numerical_mbd - mbd;
      //if(diff.norm()>tf_error)
      //{
        //printMatrix("diff", diff);
      //}
      ASSERT_NEAR(diff.norm(), 0.0, tf_error);
      ASSERT_NEAR((numerical_md - numerical_mbd).norm(), 0.0, tf_error);

      //robot->setJointPositions(q);
      new_q(ii) -= jd_step_size;

    }

  }
  }
  void testBodyJacobianDerivative(std::shared_ptr<KinematicChain> & robot)
  {
   VectorXd q, new_q, dq;
   
   q.resize(robot->dof());
   new_q.resize(robot->dof());
   dq.resize(robot->dof());

   q.setZero();
   new_q.setZero();
   dq.setZero();

  std::default_random_engine re;  

  for (int i = 0; i< 100; i++)
  {
    robot->setRandom(0, re);
    robot->readJointPositions(q);
    //robot->readJointVelocities(dq);
    new_q = q;
    //printMatrix("joint velocities", dq);

    MatrixXd jacobian = robot->bodyJacobian();
    //MatrixXd mb = robot->Mb();

    for (int ii = 0; ii < robot->dof(); ii++)
    {
      new_q(ii) += jd_step_size;
      dq = (new_q - q) / dt; 

      robot->setJointPositions(new_q);
      robot->setJointVelocities(dq);

      const MatrixXd & jacobiand = robot->computeBodyJacobianDerivative();
      //MatrixXd mbd = robot->Md();

      //MatrixXd delta_jacobian = robot->computeBodyJacobian();
      const MatrixXd & delta_jacobian = robot->bodyJacobian();
      //MatrixXd delta_mb = robot->Mb();
      
      const MatrixXd & numerical_jacobiand = (delta_jacobian - jacobian)/dt;
      //MatrixXd numerical_mbd = (delta_mb - mb)/dt;

      /*
      printMatrix("q:", q);
      printMatrix("new q:", new_q);
      printMatrix("dq:", dq);

      printMatrix("Jacobian:", jacobian);
      printMatrix("New Jacobian:", delta_jacobian);

      */
      //printMatrix("numerical Jacobian derivative:", numerical_jacobiand);
      //printMatrix("Jacobian derivative:", jacobiand);
      
      //printMatrix("Jacobian diff:", jacobian - delta_jacobian);
      //printMatrix("Numerical Jacobian derivative:", numerical_jacobiand);
      //printMatrix("Jacobian derivative:", jacobiand);
      //printMatrix("Mb diff:", mb - delta_mb);
      //printMatrix("numerical mbd:", numerical_mbd);
      //printMatrix("mbd diff:", numerical_mbd - mbd);
      //printMatrix("mbd:", mbd);

      ASSERT_NEAR((numerical_jacobiand - jacobiand).norm(), 0.0, tf_error);

      //robot->setJointPositions(q);
      new_q(ii) -= jd_step_size;
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


  for (int i = 0; i< 100; i++)
  {
    robot->setRandom(0, re);
    robot->readJointPositions(q);
    new_q = q;

    for (auto joint : robot->joints())
    {
      MatrixXd jacobian = robot->spatialJacobian(joint->name());
      for (int ii = 0; ii < robot->dof(); ii++)
      {
        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt; 

        robot->setJointPositions(new_q);
        robot->setJointVelocities(dq);

        MatrixXd jacobiand = robot->computeSpatialJacobianDerivative(joint->name());

        //MatrixXd delta_jacobian = robot->computeBodyJacobian(joint->name());
        MatrixXd delta_jacobian = robot->spatialJacobian(joint->name());
        
        MatrixXd numerical_jacobiand = (delta_jacobian - jacobian)/dt;
        
	ASSERT_NEAR((numerical_jacobiand - jacobiand).norm(), 0.0, tf_error);

        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      }

    }
   }
  }
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


  for (int i = 0; i< 100; i++)
  {
    robot->setRandom(0, re);
    robot->readJointPositions(q);
    //robot->readJointVelocities(dq);
    new_q = q;
    //printMatrix("joint velocities", dq);

    for (auto joint : robot->joints())
    {
      MatrixXd jacobian = robot->bodyJacobian(joint->name());
      for (int ii = 0; ii < robot->dof(); ii++)
      {
        new_q(ii) += jd_step_size;
        dq = (new_q - q) / dt; 

        robot->setJointPositions(new_q);
        robot->setJointVelocities(dq);

        MatrixXd jacobiand = robot->computeBodyJacobianDerivative(joint->name());
	printMatrix("Jacobiand", jacobiand);
        MatrixXd delta_jacobian = robot->computeBodyJacobian(joint->name());
        
        MatrixXd numerical_jacobiand = (delta_jacobian - jacobian)/dt;
	printMatrix("Numerical Jacobiand", numerical_jacobiand);

        MatrixXd jacobiandRF = robot->computeBodyJacobianDerivativeRF(joint->name());
	printMatrix("JacobiandRF", jacobiandRF);
	RoboticsUtils::quickPrint("Diff RF approach", jacobiandRF - jacobiand);
	ASSERT_NEAR((jacobiandRF - jacobiand).norm(), 0.0, tf_error);
	ASSERT_NEAR((numerical_jacobiand - jacobiand).norm(), 0.0, tf_error);

        robot->setJointPositions(q);
        new_q(ii) -= jd_step_size;
      }

    }
   }
  }
  void testSpatialJacobianDerivative(std::shared_ptr<KinematicChain> & robot)
  {
   VectorXd q, new_q, dq;
   
   q.resize(robot->dof());
   new_q.resize(robot->dof());
   dq.resize(robot->dof());

   q.setZero();
   new_q.setZero();
   dq.setZero();

  std::default_random_engine re;  

  // Assuming 200 Hz:
  for (int i = 0; i< 100; i++)
  {
    robot->setRandom(0, re);
    robot->readJointPositions(q);
    //robot->readJointVelocities(dq);
    new_q = q;
    //printMatrix("joint velocities", dq);

    MatrixXd jacobian = robot->spatialJacobian();

    for (int ii = 0; ii < robot->dof(); ii++)
    {
      new_q(ii) += jd_step_size;
      dq = (new_q - q) / dt; 

      robot->setJointPositions(new_q);
      robot->setJointVelocities(dq);

      MatrixXd jacobiand = robot->computeSpatialJacobianDerivative();

      //MatrixXd delta_jacobian = robot->computeSpatialJacobian();
      MatrixXd delta_jacobian = robot->spatialJacobian();
      
      MatrixXd numerical_jacobiand = (delta_jacobian - jacobian)/dt;

      /*
      printMatrix("q:", q);
      printMatrix("new q:", new_q);
      printMatrix("dq:", dq);

      printMatrix("Jacobian:", jacobian);
      printMatrix("New Jacobian:", delta_jacobian);

      */
      //printMatrix("numerical Jacobian derivative:", numerical_jacobiand);
      //printMatrix("Jacobian derivative:", jacobiand);
      ASSERT_NEAR((numerical_jacobiand - jacobiand).norm(), 0.0, tf_error);

      robot->setJointPositions(q);
      new_q(ii) -= jd_step_size;
    }

  }

  }
  void testCentroidalDynamics(std::shared_ptr<KinematicChain> & robot)
  {
   //std::vector<double> tq, tdq;
   VectorXd tdq;
   std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    
    //std::cout<<"cenoitrdal dynamics iteration: " <<i <<std::endl;
    robot->setRandom(1, re);
    //robot->readJointPositions(tq);
    robot->readJointVelocities(tdq);
    //Eigen::VectorXd dq = Eigen::Map<Eigen::VectorXd>(tdq.data(), static_cast<int>(tdq.size()));

    Vector6d agg_m = Vector6d::Zero();
    Vector6d agg_av_m = Vector6d::Zero();
    Vector6d diff_m = Vector6d::Zero();
    
    //MatrixXd diff_m; 
    //diff_m.resize(6, robot->dof());
    //diff_m.setZero();

    Vector6d rv_m= Vector6d::Zero();
    //Vector6d rv_m_sj= Vector6d::Zero();
    MatrixXd rv_m_sj;
    rv_m_sj.resize(6, robot->dof());
    rv_m_sj.setZero();
    Vector6d cm = robot->centroidalInertia() * robot->averageVelocity();
    for (auto & j:robot->joints())
    {
       // auto jtf = robot->transform(j->name());
       // This is the transform from i->cmm frame: g_ci = g_sc ^ (-1) * g_si
       //const Iso3d & g_ic = jtf.inverse() * robot->centroidalTransform();
       //const Matrix6d & geoFT_ic = metaAdgMatrix(g_ic).transpose(); // TwistTF: c -> i
       //const Matrix6d & geoFT_ic = metaAdgMatrix(g_ic).transpose(); // TwistTF: c -> i
       const Matrix6d & geoFT_ic = robot->wrenchTransform(j->name(), "com");
       const Matrix6d & geoVel_ci = robot->velTransform("com", j->name());
       //const Matrix6d & twistTF_ci = metaAdgMatrix(g_ic); // TwistTF: c -> i
       // Momentum of a link in its local frame
       const Vector6d & local_m = j->childLink()->localInertiaTensor() * (robot->bodyJacobian(j->name()) * tdq);
       // Momentum induced by the average velocity
       //const Vector6d & local_av_m = j->childLink()->localInertiaTensor() * geoFT_ic.transpose() * robot->averageVelocity();
       //const Vector6d & local_av_m = j->childLink()->localInertiaTensor() * geoFT_ic.transpose() * robot->averageVelocity();
       const Vector6d & local_av_m = j->childLink()->localInertiaTensor() * geoFT_ic.transpose() * robot->avJacobian() * robot->jointVelocities();
       agg_m += geoFT_ic * local_m; 
       agg_av_m += geoFT_ic * local_av_m; 
       //rv_m += geoFT_ic * j->childLink()->localInertiaTensor() * (robot->bodyJacobian(j->name()) * robot->jointVelocities() - metaAdgMatrix(g_ic) * robot->avJacobian() * robot->jointVelocities()); 
//       rv_m += geoFT_ic * j->childLink()->localInertiaTensor() * robot->bodyJacobian(j->name()) * robot->jointVelocities() 
//	       - geoFT_ic * j->childLink()->localInertiaTensor() *  metaAdgMatrix(g_ic) * robot->avJacobian() * robot->jointVelocities(); 

       MatrixXd tempJacobian;
       tempJacobian.resize(6,robot->dof());
       tempJacobian.setZero();

       //const MatrixXd & bj = robot->bodyJacobian(j->name());
       //long cols = bj.cols();
       //tempJacobian.block(0, 0, 6, cols) = bj; 

       rv_m += geoFT_ic * j->childLink()->localInertiaTensor() * ( 
	       //robot->bodyJacobian(j->name(), true)-  metaAdgMatrix(g_ic) * robot->avJacobian()) 
	       robot->bodyJacobian(j->name(), true)-  geoVel_ci * robot->avJacobian()) 
	       * robot->jointVelocities(); 


       //diff_m += geoFT_ic * j->childLink()->localInertiaTensor() * (robot->bodyJacobian(j->name()) - metaAdgMatrix(g_ic)* robot->avJacobian() ) * robot->jointVelocities(); 
       //diff_m += geoFT_ic * j->childLink()->localInertiaTensor() * (robot->bodyJacobian(j->name()) * robot->jointVelocities() - metaAdgMatrix(g_ic)* robot->averageVelocity() ) ; 
       diff_m += geoFT_ic * j->childLink()->localInertiaTensor() * (robot->bodyJacobian(j->name()) * robot->jointVelocities() - geoVel_ci * robot->averageVelocity() ) ; 
       //diff_m += geoFT_ic * j->childLink()->localInertiaTensor() * (robot->bodyJacobian(j->name())  - metaAdgMatrix(g_ic) * robot->avJacobian() ); 
       //rv_m += geoFT_ic * j->childLink()->localInertiaTensor() * robot->rVelJacobian(j->name())*tdq; 
//       rv_m_sj += geoFT_ic * j->childLink()->localInertiaTensor() *
//	       metaAdgInvMatrix(jtf) * ((
//	robot->spatialJacobian(j->name()) - metaAdgMatrix(robot->centroidalTransform())* robot->avJacobian()
//		)); 
//       rv_m_sj += geoFT_ic * j->childLink()->localInertiaTensor() *
//	       ((
//	robot->bodyJacobian(j->name()) - twistTF_ci * robot->avJacobian()
//		)); 

    }

    ASSERT_NEAR((cm - agg_m).norm(), 0.0, tf_error);
    ASSERT_NEAR((cm - agg_av_m).norm(), 0.0, tf_error);
    ASSERT_NEAR((agg_m - agg_av_m).norm(), 0.0, tf_error);
    ASSERT_NEAR(diff_m.norm(), 0.0, tf_error);
    //ASSERT_NEAR((diff_m*robot->jointVelocities()).norm(), 0.0, tf_error);
    ASSERT_NEAR(rv_m.norm(), 0.0, tf_error);
    //ASSERT_NEAR((rv_m_sj*tdq).norm(), 0.0, tf_error);
  }
  }
  void testRelativeVelocity(std::shared_ptr<KinematicChain> & robot)
  {
   //std::vector<double> tq, tdq;
   VectorXd tdq;
   std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    
    //std::cout<<"rv iteration: "<<i <<std::endl;
    robot->setRandom(1, re);
    //robot->readJointPositions(tq);
    robot->readJointVelocities(tdq);
    //Eigen::VectorXd dq = Eigen::Map<Eigen::VectorXd>(tdq.data(), static_cast<int>(tdq.size()));

    //printMatrix("joint velocities", dq);

    //double es = (tdq * robot->M() * tdq.transpose())(0);
    //double bs = (tdq * robot->Mb() * tdq.transpose())(0);
   
    //ASSERT_NEAR(es - bs, 0.0, tf_error);
    // 0. Average velocity Jacobian

    ASSERT_NEAR((robot->averageVelocity() - robot->avJacobian() * tdq).norm(), 0.0, tf_error);

    // 1. Momentum conservation
    
    double rEk = 0.0;
    double rEk_links = 0.0;
    double aEk_links = 0.0;
    double ek_compo = 0.0;
    for (auto j:robot->joints())
    {
       // auto jtf = robot->transform(j->name());
       // This is the transform from i->cmm frame: g_ci = g_sc ^ (-1) * g_si
       //const Iso3d & g_ic = jtf.inverse() * robot->centroidalTransform();
       //const Matrix6d & twistTF_ci = metaAdgMatrix(g_ic); // TwistTF: c -> i
       const Matrix6d & twistTF_ci = robot->velTransform("com", j->name()); // TwistTF: c -> i

       const MatrixXd & rVelJac = robot->rVelJacobian(j->name());
       double diff = ((robot->bodyJacobian(j->name(), true) - ( twistTF_ci * robot->avJacobian() + rVelJac )) * tdq).norm();


       const Vector6d & aVel = twistTF_ci * robot->avJacobian() * tdq;
       const Vector6d & rVel = robot->bodyJacobian(j->name()) * tdq - twistTF_ci * robot->averageVelocity();
       const Vector6d & rVel_jac = robot->rVelJacobian(j->name()) * tdq; 
	      
	//const Vector6d & rVel = rVelJac * tdq;

       rEk_links += 0.5 * rVel.transpose() * j->childLink()->localInertiaTensor() * rVel; 
       rEk += 0.5 * rVel_jac.transpose() * j->childLink()->localInertiaTensor() * rVel_jac; 
       aEk_links += 0.5 * aVel.transpose() * j->childLink()->localInertiaTensor() * aVel; 
       const Vector6d & compoVel = ( twistTF_ci * robot->avJacobian() + rVelJac ) * tdq;
       ek_compo += 0.5 * compoVel.transpose() * j->childLink()->localInertiaTensor() * compoVel; 

       ASSERT_NEAR(diff, 0.0, tf_error);
    
    }
    
    ASSERT_NEAR(robot->kineticEnergy(), ek_compo, tf_error);
    // 2. Kinetic energy conservation
    
    double aEk = 0.0;
    const Vector6d & av = robot->averageVelocity();
    const Matrix6d & ci = robot->centroidalInertia();

    aEk = 0.5 * av.transpose() * ci * av;

    ASSERT_NEAR(aEk, aEk_links, tf_error);
    //std::cout<<"Ek is: "<<robot->kineticEnergy()<<std::endl;
    //std::cout<<"aEk is: "<<aEk<<std::endl;
    //std::cout<<"rEk_links is: "<<rEk_links<<std::endl;
    //std::cout<<"rEk is: "<<rEk<<std::endl;

    double rEk_ci = 0.5* tdq.transpose() * 
	   (robot->M() - robot->cmm().transpose() * ci.inverse().transpose() * robot->cmm()) 
	    * tdq;
    //std::cout<<"rEk_ci is: "<<rEk_ci<<std::endl;

    ASSERT_NEAR(rEk_ci, rEk_links, tf_error);
    ASSERT_NEAR(robot->kineticEnergy(), aEk + rEk_links, tf_error);
    //rEk = 0.5 * dq.transpose() * robot->relativeVelocityInertia() * dq;
    ASSERT_NEAR(robot->kineticEnergy(), aEk + rEk, tf_error);

    // 3. transpose(XG) * systemInertia * relativeVel == 0
    // We show that the relative velocity momentum in the centroidal frame is zero! 
    
    Vector6d crm = Vector6d::Zero();
    // The system inertia 
    MatrixXd si, bJacs; 

    int dof = robot->dof();
    int sd = 6 * dof;
    si.resize(sd, sd);
    si.setZero();

    bJacs.resize(sd, dof);
    bJacs.setZero();

    // The relative velocity Jacobian: 
    MatrixXd rVelJac; 
    rVelJac.resize(sd, dof);
    rVelJac.setZero();
    // XG
    MatrixXd XG; 
    XG.resize(sd, 6);
    XG.setZero();


    for (auto & j:robot->joints())
    {
       int idx = robot->jointNameToIndex(j->name());
       // auto jtf = robot->transform(j->name());
       // This is the transform from i->cmm frame: g_ci = g_sc ^ (-1) * g_si
       //const Iso3d & g_ic = jtf.inverse() * robot->centroidalTransform();
       //const Matrix6d & geoFT_ic = metaAdgMatrix(g_ic).transpose(); // wrench TF: i -> c 
       const Matrix6d & geoFT_ic = robot->wrenchTransform(j->name(), "com"); //  wrench TF: i -> c 

       //const Matrix6d & twistTF_ci = metaAdgMatrix(g_ic); // TwistTF: c -> i
       const Matrix6d & twistTF_ci = robot->velTransform("com", j->name()); // TwistTF: c -> i

       //const Matrix6d & geoFT_ic = metaAdgInvMatrix(g_ic.inverse()).transpose(); // TwistTF: c -> i
       //const Matrix6d & twistTF_ci = metaAdgInvMatrix(g_ic.inverse); // TwistTF: c -> i

       //Vector6d rm = j->childLink()->localInertiaTensor() * (robot->bodyJacobian(j->name())*tdq - twistTF_ci * robot->avJacobian() * tdq);
       Vector6d rm = j->childLink()->localInertiaTensor() * robot->rVelJacobian(j->name()) * tdq;
       //Vector6d rh = j->childLink()->localInertiaTensor() * (robot->avJacobian() * tdq);
       crm += geoFT_ic * rm;

       si.block<6,6>(6*idx, 6*idx) = j->childLink()->localInertiaTensor();
       XG.block<6,6>(6*idx, 0) = twistTF_ci;
       bJacs.block(6*idx, 0, 6, dof) = robot->bodyJacobian(j->name(), true);

    }

    rVelJac = bJacs - XG * robot->avJacobian();
    MatrixXd rInertia  = rVelJac.transpose() * si * rVelJac;

    double new_rEk = 0.5 * tdq.transpose() * rInertia * tdq;
    //std::cout<<"new rEk is: "<<new_rEk<<std::endl;

    ASSERT_NEAR(crm.norm(), 0.0, tf_error);
    ASSERT_NEAR(new_rEk, rEk_links, tf_error);

    //const Iso3d & tf_cp = robot->centroidalTransform().inverse() * robot->transform();
    //const Matrix6d & adgInvCP = metaAdgInvMatrix(tf_cp);
    const Matrix6d & vel_cn = robot->velTransform("com", robot->lastJoint()); 

    VectorXd dq1, dq2, dq;
    const MatrixXd & bj = robot->bodyJacobian();

    //const MatrixXd & c_rJac = bj - adgInvCP * robot->avJacobian();
    const MatrixXd & c_rJac = bj - vel_cn * robot->avJacobian();

    Vector6d wJump;
    wJump.setZero();

    double lb = -30;
    double ub = 30;

    wJump.head(3) = grVec3d(re, lb, ub, false);
    //wJump.tail(3) = grVec3d(re, lb, ub, false);

    //printMatrix("wJump", wJump);

    dq = robot->M().colPivHouseholderQr().solve(bj.transpose() * wJump);

    dq2 = robot->relativeVelocityInertia().colPivHouseholderQr().solve(c_rJac.transpose() * wJump);
    //dq2 = robot->relativeVelocityInertia().colPivHouseholderQr().solve(bj.transpose() * wJump);

    const MatrixXd & av_M = robot->avJacobian().transpose() * robot->cmm();

    //dq1 = av_M.colPivHouseholderQr().solve( robot->avJacobian().transpose() * adgInvCP.transpose() * wJump);
    //dq1 = av_M.colPivHouseholderQr().solve( (adgInvCP * robot->avJacobian()).transpose() * wJump);
    dq1 = av_M.colPivHouseholderQr().solve( (vel_cn * robot->avJacobian()).transpose() * wJump);
    //dq1 = av_M.colPivHouseholderQr().solve(bj.transpose() * wJump);
    
    //const Vector6d d_av = ci.colPivHouseholderQr().solve(adgInvCP.transpose() * wJump);

    // This test shows that we can not solve dq separately.
    //printMatrix("d_av", d_av.transpose());
    //printMatrix("d_av_q", (robot->avJacobian() * dq).transpose());
    //printMatrix("d_av_q1", (robot->avJacobian() * dq1).transpose());
    //printMatrix("d_av_q2", (robot->avJacobian() * dq2).transpose());

    //printMatrix("dq", dq.transpose());
    //printMatrix("dq1", dq1.transpose());
    //printMatrix("dq2", dq2.transpose());

    //ASSERT_NEAR((dq1 + dq2).norm(), dq.norm(), tf_error);
    ASSERT_NEAR((av_M + robot->relativeVelocityInertia()).norm(), robot->M().norm(), tf_error);

    //double e1 = dq1.transpose() * av_M * dq1;
    //double e2 = dq2.transpose() * robot->relativeVelocityInertia() * dq2;
    //double e = dq.transpose() * robot->M() * dq;

    //ASSERT_NEAR(e1 + e2, e, tf_error);


  }
  }

  void testCRBSpeed(std::shared_ptr<KinematicChain> & robot)
  {
     std::default_random_engine re;  

     std::vector<double> v;
     std::vector<double> crb_v;
     for (int i = 0; i< 5000; i++)
     {

       robot->setRandom(1, re);

       auto startUpdate = std::chrono::high_resolution_clock::now();
       const MatrixXd & m = robot->computeMs();
       auto stopModelUpdate = std::chrono::high_resolution_clock::now();
       
       auto durationModelUpdate = std::chrono::duration_cast<std::chrono::microseconds>(stopModelUpdate - startUpdate);
       v.push_back(static_cast<double>(durationModelUpdate.count()));

       auto crb_startUpdate = std::chrono::high_resolution_clock::now();
       MatrixXd m_crb; 
       robot->MCalcCRB(m_crb);
       auto crb_stopModelUpdate = std::chrono::high_resolution_clock::now();
       
       auto crb_durationModelUpdate = std::chrono::duration_cast<std::chrono::microseconds>(crb_stopModelUpdate - crb_startUpdate);
       crb_v.push_back(static_cast<double>(crb_durationModelUpdate.count()));
     }


     double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
     double m =  sum / static_cast<double>(v.size());

     double crb_sum = std::accumulate(std::begin(crb_v), std::end(crb_v), 0.0);
     double crb_m =  crb_sum / static_cast<double>(crb_v.size());
     

     double accum = 0.0, crb_accum = 0.0;
     std::for_each (std::begin(v), std::end(v), [&](const double d) {
         accum += (d - m) * (d - m);
     });

     std::for_each (std::begin(crb_v), std::end(crb_v), [&](const double d) {
         crb_accum += (d - crb_m) * (d - crb_m);
     });
     

     double stdev = sqrt(accum / static_cast<double>((v.size()-1)));
     double crb_stdev = sqrt(crb_accum / (static_cast<double>(crb_v.size()-1)));

     RoboticsUtils::quickHL("For robot: " + robot->name() + ", the Classical-M-calculation mean update time is: " + std::to_string(m) + " microseconds, and the standard deviation is: "+ std::to_string(stdev) + " microseconds.");

     RoboticsUtils::quickHL("For robot: " + robot->name() + ", the CRB-M-calculation mean update time is: " + std::to_string(crb_m) + " microseconds, and the standard deviation is: "+ std::to_string(crb_stdev) + " microseconds.");

  }
  void testJointTransforms(std::shared_ptr<KinematicChain> & robot)
  {
   // Check the transform according to Eq.~3.3 of Murray's book.
   //std::vector<double> tq, tdq;
   VectorXd tdq;
   std::default_random_engine re;
   for (int i = 0; i< 100; i++)
   //for (int i = 0; i< 1; i++)
   {
     
     robot->setRandom(2, re);
     Iso3d tf = Iso3d::Identity();

     for(int ii = 0; ii < robot->dof(); ii++)
     {
        auto j = robot->joint(ii);
        tf = tf * j->incrementalTransform();
        auto diff =  (tf.matrix() * j->initialTransform().matrix() - robot->transform(ii).matrix());

	// The following is wrong! 
	//tf = tf * j->incrementalTransform() * j->initialRelativeTransform();
        //auto diff =  (tf.matrix() - robot->transform(ii).matrix());


        //RoboticsUtils::quickError("Robot: ", robot->name(), " joint: ", j->name() );
        ASSERT_NEAR(diff.norm(), 0.0, tf_error);
     }
   }
  }
  void testCRBInertiaMatrix(std::shared_ptr<KinematicChain> & robot)
  {
   //std::vector<double> tq, tdq;
   VectorXd tdq;
  std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    
    robot->setRandom(1, re);
    //robot->readJointPositions(tq);
    robot->readJointVelocities(tdq);
    //Eigen::VectorXd dq = Eigen::Map<Eigen::VectorXd>(tdq.data(), static_cast<int>(tdq.size()));

    //printMatrix("joint velocities", dq);

    //double es = (tdq * robot->M() * tdq.transpose())(0);
    //double bs = (tdq * robot->Mb() * tdq.transpose())(0);
   
    //ASSERT_NEAR(es - bs, 0.0, tf_error);

    // 1. Symmetry
    const MatrixXd & mb = robot->M();
    MatrixXd m_crb, m_crb_new; 
    robot->MCalcCRB(m_crb); 
    //RoboticsUtils::quickPrint("M", mb);
    //RoboticsUtils::quickPrint("M CRB", m_crb);
   
    ASSERT_NEAR((mb - m_crb).norm(), 0.0, tf_error);
    // Symmetric
    ASSERT_NEAR((mb - mb.transpose()).norm(), 0.0, tf_error);
    ASSERT_NEAR((m_crb - mb.transpose()).norm(), 0.0, tf_error);
    ASSERT_NEAR((m_crb - m_crb.transpose()).norm(), 0.0, tf_error);

    // clang-format off
    // 2. positive definite 
    // Check the Stack-overflow post: https://stackoverflow.com/questions/35227131/eigen-check-if-matrix-is-positive-semi-definite
      // clang-format on

      Eigen::LLT<Eigen::MatrixXd> lltM(m_crb);
      Eigen::LLT<Eigen::MatrixXd> lltMb(mb);
      ASSERT_TRUE(lltM.info() != Eigen::NumericalIssue);
      ASSERT_TRUE(lltMb.info() != Eigen::NumericalIssue);
    }
  }
  void testInertiaMatrix(std::shared_ptr<KinematicChain> & robot)
  {
    // std::vector<double> tq, tdq;
    VectorXd tdq;
    std::default_random_engine re;
    // for (int i = 0; i< 100; i++)
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
      ASSERT_NEAR((mb - m.transpose()).norm(), 0.0, tf_error);

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
  void testKineticEnergy(std::shared_ptr<KinematicChain> & robot)
  {
    // std::vector<double> tq, tdq;
    VectorXd tdq;
    std::default_random_engine re;
    for(int i = 0; i < 100; i++)
    {
      robot->setRandom(1, re);
      // robot->readJointPositions(tq);
      robot->readJointVelocities(tdq);
      // printMatrix("joint velocities", dq);

      const VectorXd & es = 0.5 * tdq.transpose() * robot->computeMs() * tdq;
      const VectorXd & bs = 0.5 * tdq.transpose() * robot->M() * tdq;

      double diff = (es - bs)(0);
      ASSERT_NEAR(diff, 0.0, tf_error);

      ASSERT_NEAR(es(0), robot->kineticEnergy(), tf_error);

      double ke = 0.0;
      for(auto j : robot->joints())
      {
        const Vector6d & vel = robot->bodyJacobian(j->name()) * tdq;
        ke += 0.5 * vel.transpose() * j->childLink()->localInertiaTensor() * vel;
      }

      ASSERT_NEAR(robot->kineticEnergy(), ke, tf_error);
    }
  }
};

TEST_F(DynamicsExample, caseOne)
{
  for(auto robot : robots)
  {
    testKineticEnergy(robot);
  }
} // End of caseOne

TEST_F(DynamicsExample, caseTwo)
{
  for(auto robot : robots)
  {
    testBodyJacobianDerivative(robot);
    testSpatialJacobianDerivative(robot);
  }
} // End of caseTwo

TEST_F(DynamicsExample, caseFour)
{

  for(auto robot : robots)
  {
    testAdjointTransformDerivative(robot);
    // randomSingleJointVel(100, robot, metaTestAdjointTransformDerivative);
  }

} // End of caseFour

/*
TEST_F(DynamicsExample, caseFive)
{

  for(auto robot : robots)
  {
    testSubspaces(robot);
  }

} // End of caseFive
*/

TEST_F(DynamicsExample, caseSix)
{

  for(auto robot : robots)
  {
    // testCOMJacobian(robot);
    randomPosition(100, robot, testCOM);
    randomPosition(100, robot, testMomentumTransform);
  }

} // End of caseSix

TEST_F(DynamicsExample, caseSeven)
{

  for(auto robot : robots)
  {
    testCOMJac(robot);
  }

  for(auto robot : robots)
  {
    testLinkCOMVel(robot);
  }

} // End of caseSeven

TEST_F(DynamicsExample, caseEight)
{

  for(auto robot : robots)
  {
    testCMM(robot);
  }

} // End of caseEight

TEST_F(DynamicsExample, caseNine)
{

  for(auto robot : robots)
  {
    testEndEffector(robot);
  }

} // End of caseNine

TEST_F(DynamicsExample, caseTen)
{

  for(auto robot : robots)
  {
    testInertiaMatrix(robot);
  }

} // End of caseTen

TEST_F(DynamicsExample, case11)
{

  // std::shared_ptr<KinematicChain> newRobot = krRobot;
  // for(auto robot:robots)
  {
    testInertiaMatrixDerivative(pandaRobot);
    testInertiaMatrixDerivative(krRobot);
    testInertiaMatrixDerivative(scaraRobot);
    testInertiaMatrixDerivative(elbowRobot);
    // testInertiaMatrixDerivative(sdRobot);
  }

} // End of case11

TEST_F(DynamicsExample, case12)
{

  for(auto robot : robots)
  {
    testRelativeVelocity(robot);
  }

  // testRelativeVelocity(elbowRobot);
  // testRelativeVelocity(pandaRobot);
} // End of case12

TEST_F(DynamicsExample, case13)
{

  for(auto robot : robots)
  {
    testCentroidalDynamics(robot);
  }

} // End of case13

/*
TEST_F(DynamicsExample, caseEight) {

 testAdjointTransformDerivativeSimple(elbowRobot);

} // End of caseEight
*/

TEST_F(DynamicsExample, crb_case1)
{

  for(auto robot : robots)
  {
    testCRBInertiaMatrix(robot);
  }

} // End of caseTen

TEST_F(DynamicsExample, joint_transforms)
{

  for(auto robot : robots)
  {
    testJointTransforms(robot);
  }

} // End of joint_transforms

TEST_F(DynamicsExample, crb_case2)
{

  for(auto robot : robots)
  {
    testCRBSpeed(robot);
  }

} // End of caseTen

TEST_F(DynamicsExample, case_last)
{

  for(auto robot : robots)
  {
    testSpeed(robot);
  }

} // End of case_last

TEST_F(DynamicsExample, case10)
{

  // testCMMTwo(pandaRobot);
  for(auto robot : robots)
  {
    testCMMTwo(robot);
  }

} // End of case10

TEST_F(DynamicsExample, case3)
{
  // for(auto robot : robots)
  //{
  //  testBodyJacobianDerivativeForEachJoint(robot);
  //  testSpatialJacobianDerivativeForEachJoint(robot);
  //}

  // testBodyJacobianDerivativeForEachJoint(pandaRobot);

} // End of caseThree

TEST_F(DynamicsExample, case14)
{

  for(auto robot : robots)
  {
    testTransformDerivative(robot);
  }

} // End of case14

TEST_F(DynamicsExample, case15)
{

  RoboticsUtils::quickHL("------------------Testing CMM derivative for robot: ", krRobot->name());
  testCMMDerivative(krRobot);
  RoboticsUtils::quickHL("------------------Testing CMM derivative for robot: ", pandaRobot->name());
  testCMMDerivative(pandaRobot);

} // End of case15

} // namespace GeoRobotics
