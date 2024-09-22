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
#include "GeometricRobotics/ImpactDynamics/IDMInterface.h"
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

class ImpactDynamics : public testing::Test
{
protected:
  Parser parser;
  std::shared_ptr<Model> modelPtr;
  std::shared_ptr<KinematicChain> pandaRobot;
  std::shared_ptr<KinematicTree> pKPtr;

  std::shared_ptr<Model> krPtr;
  std::shared_ptr<KinematicChain> krRobot;
  std::shared_ptr<KinematicTree> krKPtr;

  std::shared_ptr<Model> scaraPtr;
  std::shared_ptr<KinematicChain> scaraRobot;
  std::shared_ptr<KinematicTree> sKPtr;

  std::shared_ptr<Model> elbowPtr;
  std::shared_ptr<KinematicChain> elbowRobot;
  std::shared_ptr<KinematicTree> eKPtr;

  std::shared_ptr<Model> sdPtr;
  std::shared_ptr<KinematicChain> sdRobot;
  std::shared_ptr<KinematicTree> sdKPtr;

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

  int iterations = 100;
  double abs_error = 1e-5;
  double tf_error = 1e-3;
  double adg_error = 3e-3;
  double jd_step_size = 0.0001;
  double dt = 0.001;

  constexpr static double lambda_threshold = 0.012;
  constexpr static double jv_threshold = 0.02;

  constexpr static double dq_threshold = 0.005;
  constexpr static double panda_dq_threshold = 0.05;
  constexpr static double sd_dq_threshold = 0.05;

  constexpr static double panda_sblv_threshold = 0.02;
  constexpr static double kr_sblv_threshold = 0.5;
  constexpr static double sblv_threshold = 0.01;

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

    krPtr = parser.parseURDFFile("@RobotExamplesPath@KR5/KR5.urdf");
    krKPtr = std::make_shared<KinematicTree>(krPtr);

    krRobot = krKPtr->createKinematicChain("krRobot", "base_link", "palm");
    krRobot->printKinematicChain();

    krRobot = krKPtr->createKinematicChain("krRobot", "base_link", "palm");
    krRobot->printKinematicChain();

    robots.push_back(krRobot);

    // Create the impacting end-effector
    for(auto r : robots)
    {
      Iso3d eeOffset;
      eeOffset.setIdentity();
      // eeOffset.translation().z() = 0.1;
      std::string lastJointName = r->joint(r->dof() - 1)->name();
      r->createEndEffector(r->name() + "_impactBody", lastJointName, eeOffset);
    }
  }

  void TearDown() override {}

  void randomPositionVelocity(const int & iters,
                              std::shared_ptr<KinematicChain> & robot,
                              std::map<std::string, std::shared_ptr<IDMInterface>> & idms,
                              void func(std::default_random_engine & re,
                                        std::shared_ptr<KinematicChain> & robot,
                                        std::map<std::string, std::shared_ptr<IDMInterface>> & idms))
  {
    std::default_random_engine re;

    for(int i = 0; i < iters; i++)
    // for(int i = 0; i < 2; i++)
    {
      // Set the joint position and velocity
      robot->setRandom(1, re);
      // Run the actual function here
      func(re, robot, idms);
    }
  }

  static void testMomentumConservation(std::default_random_engine & re,
                                       std::shared_ptr<KinematicChain> & robot,
                                       std::map<std::string, std::shared_ptr<IDMInterface>> & idms)
  {

    for(auto idm : idms)
    {
      // RoboticsUtils::quickInfo("Testing the impact dynamics model: " + idm.first + " for robot: " + robot->name());
      std::shared_ptr<IDMInterface> id = idm.second;
      id->update();
      // printMatrix("impulse", id->robotPostImpactStates().impulse);
      printMatrix("Equivalent mass", id->eMass());
      printMatrix("end-effector velocity", (id->robot()->bodyJacobian() * id->robot()->jointVelocities()).transpose());
      printMatrix("contact point velocity", id->impactDynamicsModel()->robotContactPointVelocity().transpose());

      const Vector3d & impulse = id->robotPostImpactStates().impulse;

      printMatrix("impulse", impulse);
      // * impulse \leq momentum
      const Vector3d & hLocal =
          id->impactDynamicsModel()->endEffector()->transform().rotation() * (robot->comVelocity() * robot->mass());
      // const Vector3d & h = robot->comVelocity() * robot->mass();
      printMatrix("pre-impact robot momentum ", hLocal);
      const Vector3d & h_two = id->eMass().block<3, 3>(0, 0) * id->impactDynamicsModel()->robotContactPointVelocity();
      printMatrix("pre-impact equivalent robot momentum", h_two);

      ASSERT_TRUE(impulse.norm() <= h_two.norm());

      // idm.second->saveData();
    }
  }

  static void testKineticEnergy(std::default_random_engine & re,
                                std::shared_ptr<KinematicChain> & robot,
                                std::map<std::string, std::shared_ptr<IDMInterface>> & idms)
  {

    for(auto idm : idms)
    {
      // RoboticsUtils::quickInfo("Testing the impact dynamics model: " + idm.first + " for robot: " + robot->name());
      std::shared_ptr<IDMInterface> id = idm.second;
      id->update();
      // printMatrix("impulse", id->robotPostImpactStates().impulse);
      // printMatrix("end-effector velocity", (id->robot()->bodyJacobian() *
      // id->robot()->jointVelocities()).transpose()); printMatrix("contact point velocity",
      // id->impactDynamicsModel()->robotContactPointVelocity().transpose());

      // const Vector3d impulse = id->robotPostImpactStates().K_ * grVec4d(re);
      // printMatrix("fake impulse", impulse);

      const VectorXd & dq = robot->jointVelocities();
      double e = dq.transpose() * robot->M() * dq;

      // const Vector3d & contactVel = id->impactDynamicsModel()->robotContactPointVelocity();
      const Vector6d & contactVel = robot->bodyJacobian() * dq;

      double e2 = contactVel.transpose() * id->eMass() * contactVel;

      ASSERT_NEAR(e - e2, 0.0, 0.1);
    }
  }
  static void testEM(std::default_random_engine & re,
                     std::shared_ptr<KinematicChain> & robot,
                     std::map<std::string, std::shared_ptr<IDMInterface>> & idms)
  {

    std::shared_ptr<EndEffector> impactEE = robot->endEffector(robot->name() + "_impactBody");
    // Compute the CRB in the contact frame

    Matrix6d crbC;
    crbC.setZero();

    Iso3d tf_oe_inv = impactEE->transform().inverse();

    for(auto l : robot->links())
    {
      Iso3d tf_ei = tf_oe_inv * robot->transform(l->parentJointName());
      crbC += l->spatialInertia(tf_ei);
    }
    printMatrix("crbC", crbC);

    printMatrix("crbCom", robot->computeCentroidalInertia());
  }

  static void testFrictionCone(std::default_random_engine & re,
                               std::shared_ptr<KinematicChain> & robot,
                               std::map<std::string, std::shared_ptr<IDMInterface>> & idms)
  {

    for(auto idm : idms)
    {
      // RoboticsUtils::quickInfo("Testing the impact dynamics model: " + idm.first + " for robot: " + robot->name());
      std::shared_ptr<IDMInterface> id = idm.second;
      id->update();
      // printMatrix("impulse", id->robotPostImpactStates().impulse);
      // printMatrix("end-effector velocity", (id->robot()->bodyJacobian() *
      // id->robot()->jointVelocities()).transpose()); printMatrix("contact point velocity",
      // id->impactDynamicsModel()->robotContactPointVelocity().transpose());

      const Vector3d & impulse = id->robotPostImpactStates().impulse;

      // const Vector3d impulse = id->robotPostImpactStates().K_ * grVec4d(re);
      // printMatrix("fake impulse", impulse);

      // 1. At the contact point frame, the impulse fulfills the friction cone
      double zmiu = fabs(impulse.z()) * id->params().idParams.coeF;
      double x = fabs(impulse.x());
      double y = fabs(impulse.y());

      ASSERT_TRUE(x <= zmiu);
      ASSERT_TRUE(y <= zmiu);
    }
  }
}; // end of ImpactDynamics

TEST_F(ImpactDynamics, caseOne)
{

  std::shared_ptr<KinematicChain> robot = pandaRobot;
  // for(auto robot : robots)
  {

    std::shared_ptr<EndEffector> impactEE = robot->endEffector(robot->name() + "_impactBody");

    std::map<std::string, std::shared_ptr<IDMInterface>> idms;

    ImpactModelParams idParams;
    idParams.bodyCoordinates = true;
    idParams.timeStep = 1.0 / 200;
    idParams.coeF = 0.7;
    idParams.coeR = 0.2;

    IDMInterfaceParams paramsK, paramsKCRB, paramsFK, paramsFKCRB;
    IDMInterfaceParams paramsPlanarFID;

    paramsK.emChoice = EMChoice::Projection;
    paramsK.idModel = IDModel::Kinematic;
    paramsK.idParams = idParams;

    paramsKCRB = paramsK;
    paramsKCRB.emChoice = EMChoice::CRB;

    paramsFK.emChoice = EMChoice::Projection;
    paramsFK.idModel = IDModel::FrictionalKinematic;
    paramsFK.idParams = idParams;

    paramsFKCRB = paramsFK;
    paramsFKCRB.emChoice = EMChoice::CRB;

    paramsPlanarFID.emChoice = EMChoice::CRB;
    paramsPlanarFID.idModel = IDModel::FrictionalPlanar;
    paramsPlanarFID.idParams = idParams;

    std::shared_ptr<IDMInterface> iK = std::make_shared<IDMInterface>(robot, impactEE, paramsK);
    idms["KinematicModel"] = iK;
    std::shared_ptr<IDMInterface> iKCRB = std::make_shared<IDMInterface>(robot, impactEE, paramsKCRB);
    // idms["KinematicModelCRB"] = iKCRB;
    std::shared_ptr<IDMInterface> iFK = std::make_shared<IDMInterface>(robot, impactEE, paramsFK);
    // idms["FrictionalKinematicModel"] = iFK;
    std::shared_ptr<IDMInterface> iFKCRB = std::make_shared<IDMInterface>(robot, impactEE, paramsFKCRB);
    // idms["FrictionalKinematicModelCRB"] = iFKCRB;

    std::shared_ptr<IDMInterface> iFPlanar = std::make_shared<IDMInterface>(robot, impactEE, paramsPlanarFID);
    // idms["PlanarFrictionalModel"] = iFPlanar;

    // 1. Check the T and Ta complement each other in the local body frame.
    // randomPositionVelocity(iterations, robot, idms, testMomentumConservation);
    // randomPositionVelocity(iterations, robot, idms, testFrictionCone);
    // randomPositionVelocity(iterations, robot, idms, testKineticEnergy);
    randomPositionVelocity(iterations, robot, idms, testEM);
  }

  /*
  std::shared_ptr<KinematicChain> newRobot = elbowRobot;
  PredictorParams params;
  params.reset();

  std::shared_ptr<PostImpactPredictor> pip = std::make_shared<PostImpactPredictor>(newRobot, params);
  //randomPosition(1, newRobot, pip, testPip);
  randomPosition(iterations, newRobot, pip, testPip);
  */

} // End of caseOne

} // namespace GeoRobotics
