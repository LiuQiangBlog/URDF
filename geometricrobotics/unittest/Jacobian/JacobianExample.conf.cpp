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

struct StanfordParams
{
  double l1 = 0.4;
  double l0 = 0.6;
};

class JacobianExample : public testing::Test
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

  std::vector<std::shared_ptr<KinematicChain>> robots;
  ScaraParams sParams;
  StanfordParams sdParams;
  ElbowParams eParams;

  Matrix3d scara_rotaiton;
  Vector3d scara_translation;

  double abs_error = 1e-5;
  double tf_error = 1e-3;
  double jd_step_size = 0.0001;

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

  double dt = 0.001;
  void SetUp() override
  {
    // Create the 3DOF SCARA robot shown in Fig.3.3 of Murray et al. 1994
    scaraPtr = parser.parseURDFFile("@RobotExamplesPath@SCARA.urdf");
    sKPtr = std::make_shared<KinematicTree>(scaraPtr);

    scaraRobot = sKPtr->createKinematicChain("scara", "base_link", "EndEffector");

    scaraRobot->printKinematicChain();

    // Create the ELBOW Manipulator shown in Fig.3.4 of Murray et al. 1994
    elbowPtr = parser.parseURDFFile("@RobotExamplesPath@ELBOW.urdf");

    eKPtr = std::make_shared<KinematicTree>(elbowPtr);

    elbowRobot = eKPtr->createKinematicChain("elbow", "base_link", "EndEffector");
    elbowRobot->printKinematicChain();

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

    // Create the KR5
    krPtr = parser.parseURDFFile("@RobotExamplesPath@KR5/KR5.urdf");
    krKPtr = std::make_shared<KinematicTree>(krPtr);

    krRobot = krKPtr->createKinematicChain("krRobot", "base_link", "palm");
    krRobot->printKinematicChain();

    modelPtr = parser.parseURDFFile("@RobotExamplesPath@panda_foot.urdf");
    pKPtr = std::make_shared<KinematicTree>(modelPtr, "world");

    pandaRobot = pKPtr->createKinematicChain("panda", "panda_link0", "panda_link8");

    robots.push_back(scaraRobot);
    robots.push_back(elbowRobot);
    robots.push_back(sdRobot);
    robots.push_back(krRobot);
    robots.push_back(pandaRobot);
  }

  double c(const double & q)
  {
    return cos(q);
  }
  double s(const double & q)
  {
    return sin(q);
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

  void testPd(std::shared_ptr<KinematicChain> & robot)
  {
     // Test: dot{p} =  R*bodyTV 
    std::default_random_engine re;
    for (int i = 0; i< 100; i++)
    {  
      robot->setRandom(0, re);

      VectorXd q = robot->jointPositions();
      VectorXd dq, qNext;
      //printMatrix("joint position", q);
      const Iso3d & tf = robot->transform();

      randomJointVel(q, re, dq, qNext);

      //printMatrix("new joint position", qNext);

      Vector6d bodyVel = robot->bodyJacobian() * dq;
      Vector6d spatialVel = robot->spatialJacobian() * dq;

      //printMatrix("joint velocities", dq);
      //robot->velIntegration(dt);

      robot->setJointPositions(qNext);
      //printMatrix("new joint position", robot->jointPositions());
      const Iso3d & tf_new = robot->transform();

      //printMatrix("translation", tf.translation());
      //printMatrix("new translation", tf_new.translation());
      Vector3d pd = (tf_new.translation() - tf.translation() ) /dt; 
      //printMatrix("point velocity", pd);

      Vector3d computed_pd_body = tf.rotation() * bodyVel.head(3);
      Vector3d computed_pd_spatial = skew(spatialVel.tail(3)) * tf.translation() + spatialVel.head(3);

      //printMatrix("computed point velocity (bodyJacobian)", computed_pd_body);
      //printMatrix("computed point velocity (spatialJacobian)", computed_pd_spatial);

      ASSERT_NEAR((computed_pd_body - computed_pd_spatial).norm(), 0.0, tf_error);
      // Issue the PD is close to zero.
      //ASSERT_NEAR((pd - computed_pd).norm(), 0.0, tf_error);

    }
  
  }
  void testBodySpatialJacobian(std::shared_ptr<KinematicChain> & robot)
  {
  std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    //RoboticsUtils::quickInfo("Iteration: " + std::to_string(i));
    // Set a random joint configuration 
    
    robot->setRandom(0, re);

    Eigen::MatrixXd spatial_jac = robot->computeSpatialJacobian(); 
    Eigen::MatrixXd body_jac = robot->computeBodyJacobian();


    //ASSERT_NEAR((spatial_jac - metaAdgMatrix(robot->transform()) * body_jac).norm(), 0.0, tf_error);
    ASSERT_NEAR((spatial_jac - robot->velTransform(robot->lastJoint(), "inertial") * body_jac).norm(), 0.0, tf_error);
    //ASSERT_NEAR((body_jac- metaAdgInvMatrix(robot->transform()) * spatial_jac).norm(), 0.0, tf_error);
    ASSERT_NEAR((body_jac- robot->velTransform("inertial", robot->lastJoint()) * spatial_jac).norm(), 0.0, tf_error);


    for (auto j:robot->joints())
    {
      Eigen::MatrixXd j_spatial_jac = robot->computeSpatialJacobian(j->name(), true); 
      Eigen::MatrixXd j_spatial_jac_read = robot->spatialJacobian(j->name()); 
      Eigen::MatrixXd j_body_jac = robot->computeBodyJacobian(j->name(), true);
      Eigen::MatrixXd j_body_jac_read = robot->bodyJacobian(j->name());

     /* 
      printMatrix("read spatialJacobian", j_spatial_jac_read);
      printMatrix("computed spatialJacobian", j_spatial_jac);

      printMatrix("read bodyJacobian", j_body_jac_read);
      printMatrix("computed bodyJacobian", j_body_jac);
      */

      Iso3d j_tf = robot->transform(j->name());
      ASSERT_NEAR((j_spatial_jac - j_spatial_jac_read).norm(), 0.0, tf_error);
      ASSERT_NEAR((j_body_jac - j_body_jac_read).norm(), 0.0, tf_error);
      //ASSERT_NEAR((j_spatial_jac - metaAdgMatrix(j_tf) * j_body_jac).norm(), 0.0, tf_error);
      ASSERT_NEAR((j_spatial_jac - robot->velTransform(j->name(), "inertial") * j_body_jac).norm(), 0.0, tf_error);
      //ASSERT_NEAR((j_body_jac- metaAdgInvMatrix(j_tf) * j_spatial_jac).norm(), 0.0, tf_error);
      ASSERT_NEAR((j_body_jac-  robot->velTransform("inertial", j->name()) * j_spatial_jac).norm(), 0.0, tf_error);
    }
  }
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

  void test_jacobian(std::shared_ptr<KinematicChain> & robot)
  {
    VectorXd q, new_q, dq;

    q = VectorXd::Zero(robot->dof());
    new_q = VectorXd::Zero(robot->dof());
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

      std::string jName = robot->jointIndexToName(static_cast<size_t>(randInt(robot->dof())));

      // RoboticsUtils::quickHL("Checking joint: ", jName);

      auto j = robot->joint(jName);
      Iso3d tf = robot->transform(jName);
      Vector6d bodyVel = robot->computeBodyJacobian(jName, true) * robot->jointVelocities();

      // RoboticsUtils::quickPrint("Jac vel: ", bodyVel);

      robot->setJointPositions(qNext);

      Iso3d delta_tf = robot->transform(jName);

      MatrixXd numerical_tfd = (delta_tf.matrix() - tf.matrix()) / dt;

      auto numerical_v = tf.linear().transpose().matrix() * numerical_tfd.block<3, 1>(0, 3);

      // RoboticsUtils::quickPrint("numerical v: ", numerical_v);

      auto numerical_Rd = numerical_tfd.block<3, 3>(0, 0);

      auto numerical_w =
          tf.linear().inverse().matrix() * numerical_Rd; // body w = R^-1 * Rd, see eq.~(2.49) of Murray's book

      // RoboticsUtils::quickPrint("numerical w: ", numerical_w);
      // RoboticsUtils::quickPrint("jac w: ", skew(bodyVel.tail(3)));

      ASSERT_NEAR((numerical_v - bodyVel.head(3)).norm(), 0.0, 0.03);
      ASSERT_NEAR((numerical_w - skew(bodyVel.tail(3))).norm(), 0.0, tf_error);
    }

  } // end of test_transform

};

TEST_F(JacobianExample, caseOne)
{
  // Test the transform of the SCARA robot
  // 1. Construct the solution
  std::vector<double> sq;
  for (int idx = 0; idx < scaraRobot->dof(); idx++  )
  {
    sq.push_back(0.0);
  }

  Iso3d stf_ini = scara_transform(sq);
  //RoboticsUtils::quickInfo("The initial transform of the scara robot is: ");

  //std::cout<<"Rotation Matrix: "<<std::endl<< stf_ini.linear().matrix()<<std::endl;
  //std::cout<<"Translation: "<<std::endl<< stf_ini.translation()<<std::endl;


  Iso3d stf_ini_robot = scaraRobot->initialTransform();
  //RoboticsUtils::quickInfo("The initial transform of the scara robot is: ");
  //std::cout<<"Rotation Matrix: "<<std::endl<< stf_ini_robot.linear().matrix()<<std::endl;
  //std::cout<<"Translation: "<<std::endl<< stf_ini_robot.translation()<<std::endl;

  //scaraRobot->printSpatialTwists();

  ASSERT_NEAR((stf_ini_robot.linear() - stf_ini.linear()).norm(), 0.0, tf_error);
  ASSERT_NEAR((stf_ini_robot.translation() - stf_ini.translation()).norm(), 0.0, tf_error);




  
  std::vector<double> eq;
  for (int idx = 0; idx < elbowRobot->dof(); idx++  )
  {
    eq.push_back(0.0);
  }
  Iso3d etf_ini = elbow_transform(eq);
  // RoboticsUtils::quickInfo("The initial transform of the elbow robot is: ");

  // std::cout<<"Rotation Matrix: "<<std::endl<< etf_ini.linear().matrix()<<std::endl;
  // std::cout<<"Translation: "<<std::endl<< etf_ini.translation()<<std::endl;

  elbowRobot->setJointPositions(eq);
  Iso3d etf_ini_robot = elbowRobot->initialTransform();

  elbowRobot->printSpatialTwists();
  // RoboticsUtils::quickInfo("The initial transform of the elbow robot is: ");
  // std::cout<<"Rotation Matrix: "<<std::endl<< etf_ini_robot.linear().matrix()<<std::endl;
  // std::cout<<"Translation: "<<std::endl<< etf_ini_robot.translation()<<std::endl;

  
  // RoboticsUtils::quickInfo("The initial transforms of each elbow robot joint are: ");
  for(auto joint: elbowRobot->joints())
  {
    std::string jointName = joint->getURDFJoint()->name;
      
    // RoboticsUtils::quickInfo("The joint: " + jointName + " has the initial transform: ");

    std::cout<<RoboticsUtils::info<< elbowRobot->initialTransform(jointName).matrix()<<std::endl;
  }

    ASSERT_NEAR((etf_ini_robot.linear() - etf_ini.linear()).norm(), 0.0, tf_error);
    ASSERT_NEAR((etf_ini_robot.translation() - etf_ini.translation()).norm(), 0.0, tf_error);



} // End of caseOne


TEST_F(JacobianExample, caseTwo)
{
  // Test the transform of the SCARA robot
  // 1. Construct the solution
  
  std::vector<double> sq;
  std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    //RoboticsUtils::quickInfo("Iteration: " + std::to_string(i));
    // Set a random joint configuration 
    
    scaraRobot->setRandom(0, re);
    scaraRobot->readJointPositions(sq);

    Iso3d stf = scara_transform(sq);
    Iso3d stf_robot = scaraRobot->transform();
   // std::cout<<"The computed transform is: "<<std::endl;
   // std::cout<<RoboticsUtils::hlight<< stf.matrix()<<RoboticsUtils::reset<<std::endl;


   // std::cout<<"The current robot transform is: " <<std::endl;
   // std::cout<<RoboticsUtils::error<< stf_robot.matrix()<<RoboticsUtils::reset<<std::endl;
    


    ASSERT_NEAR((stf.linear() - stf_robot.linear()).norm(), 0.0, abs_error);
    ASSERT_NEAR((stf.translation() - stf_robot.translation()).norm(), 0.0, abs_error);

  }

} // End of caseTwo





TEST_F(JacobianExample, caseThree)
{
  // Test the transform of the ELBOW robot
  // 1. Construct the solution
  
  std::vector<double> eq;
  std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    //RoboticsUtils::quickInfo("Iteration: " + std::to_string(i));
    // Set a random joint configuration 
    
    elbowRobot->setRandom(0, re);
    elbowRobot->readJointPositions(eq);

    Iso3d etf = elbow_transform(eq);
    Iso3d etf_robot = elbowRobot->transform();

    //std::cout<<"The computed transform is: "<<std::endl;
    //std::cout<<RoboticsUtils::hlight<< etf.matrix()<<RoboticsUtils::reset<<std::endl;


    //std::cout<<"The current robot transform is: " <<std::endl;
    //std::cout<<RoboticsUtils::error<< etf_robot.matrix()<<RoboticsUtils::reset<<std::endl;
    
    ASSERT_NEAR((etf.linear() - etf_robot.linear()).norm(), 0.0, tf_error);
    ASSERT_NEAR((etf.translation() - etf_robot.translation()).norm(), 0.0, tf_error);

  }
} // End of caseThree

TEST_F(JacobianExample, caseFour)
{
  // Test the Jacobian of the SCARA robot
  // 1. Construct the solution
  
  std::vector<double> sq;
  std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    //RoboticsUtils::quickInfo("Iteration: " + std::to_string(i));
    // Set a random joint configuration 
    
    scaraRobot->setRandom(0, re);
    scaraRobot->readJointPositions(sq);

    Eigen::MatrixXd jac = scara_jacobian(sq);
    Eigen::MatrixXd jac_robot = scaraRobot->computeSpatialJacobian();

    // std::cout<<"The computed Jacobian is: "<<std::endl;
    // std::cout<<RoboticsUtils::hlight<< jac<<RoboticsUtils::reset<<std::endl;


    // std::cout<<"The current robot Jacobian is: " <<std::endl;
    // std::cout<<RoboticsUtils::error<< jac_robot <<RoboticsUtils::reset<<std::endl;
    
    ASSERT_NEAR((jac - jac_robot).norm(), 0.0, tf_error);

  }
} // End of caseFour



TEST_F(JacobianExample, caseFive)
{
  // Test the spatial Jacobian of the Stanford robot
  
  std::vector<double> sq;
  std::default_random_engine re;
  for (int i = 0; i< 100; i++)
  {
    //RoboticsUtils::quickInfo("Iteration: " + std::to_string(i));
    // Set a random joint configuration 
    
    sdRobot->setRandom(0, re);
    sdRobot->readJointPositions(sq);

    Eigen::MatrixXd jac = stanford_jacobian(sq);
    Eigen::MatrixXd jac_robot = sdRobot->computeSpatialJacobian();

    // std::cout<<"The computed Jacobian is: "<<std::endl;
    // std::cout<<RoboticsUtils::hlight<< jac<<RoboticsUtils::reset<<std::endl;


    // std::cout<<"The current robot Jacobian is: " <<std::endl;
    // std::cout<<RoboticsUtils::error<< jac_robot <<RoboticsUtils::reset<<std::endl;
    
    ASSERT_NEAR((jac - jac_robot).norm(), 0.0, tf_error);

  }
} // End of caseFive


TEST_F(JacobianExample, caseSix)
{
  // Test the body-spatial Jacobian transform 
  
   for (auto robot:robots)
   {
     testBodySpatialJacobian(robot);
     testPd(robot);
   }
} // End of caseSix

TEST_F(JacobianExample, case1)
{
  // Test the body-spatial Jacobian transform 
  
   for (auto robot:robots)
   {
     test_jacobian(robot);
   }
} // End of caseSix


} // namespace GeoRobotics
