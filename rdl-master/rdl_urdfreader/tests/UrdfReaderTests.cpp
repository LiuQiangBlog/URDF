// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <fstream>
#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rdl_dynamics/Dynamics.hpp>
#include <rdl_dynamics/Kinematics.hpp>
#include <rdl_dynamics/Model.hpp>
#include <rdl_dynamics/rdl_utils.hpp>
#include <rdl_urdfreader/urdfreader.hpp>

const double E_MINUS_14 = 1.0e-14;

class UrdfReaderTests : public testing::Test
{
  public:
    UrdfReaderTests()
    {
        srand(time(nullptr));
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }

    std::string getFileContents(const std::string& filename)
    {
        std::ifstream model_file(filename);
        if (!model_file)
        {
            std::cerr << "Error opening file '" << filename << "'." << std::endl;
            abort();
        }

        // reserve memory for the contents of the file
        std::string model_xml_string;
        model_file.seekg(0, std::ios::end);
        model_xml_string.reserve(model_file.tellg());
        model_file.seekg(0, std::ios::beg);
        model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

        model_file.close();
        return model_xml_string;
    }
};

TEST_F(UrdfReaderTests, q_index_two_chains)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/simple_two_chain_bot.urdf";

    std::string contents = getFileContents(path_to_urdf);
    std::vector<unsigned int> q_indices;
    std::vector<std::string> joint_names, body_names;
    EXPECT_TRUE(RobotDynamics::Urdf::parseJointAndBodyNamesFromString(contents, joint_names, body_names));
    EXPECT_TRUE(RobotDynamics::Urdf::parseJointAndQIndex(contents, q_indices));

    EXPECT_EQ(q_indices.size(), (unsigned int)3);
    EXPECT_STREQ(joint_names[0].c_str(), "test_robot_shoulder_lift_joint");
    EXPECT_STREQ(body_names[0].c_str(), "test_robot_upper_arm_link");
    EXPECT_EQ(q_indices[std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), "test_robot_shoulder_lift_joint"))], (unsigned int)0);

    EXPECT_STREQ(joint_names[1].c_str(), "test_robot_elbow_joint");
    EXPECT_STREQ(body_names[1].c_str(), "test_robot_forearm_link");
    EXPECT_EQ(q_indices[std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), "test_robot_elbow_joint"))], (unsigned int)1);

    EXPECT_STREQ(joint_names[2].c_str(), "test_robot_shoulder_pan_joint");
    EXPECT_STREQ(body_names[2].c_str(), "test_robot_shoulder_link");
    EXPECT_EQ(q_indices[std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), "test_robot_shoulder_pan_joint"))], (unsigned int)2);
}

TEST_F(UrdfReaderTests, testFirstJointFixedNonTrivialTransform)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/first_joint_fixed_non_trivial_xform.urdf";
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());

    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, model));

    RobotDynamics::Math::VectorNd q = RobotDynamics::Math::VectorNd::Zero(model->q_size);
    RobotDynamics::Math::VectorNd qdot = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    RobotDynamics::Math::VectorNd qddot = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);

    ASSERT_EQ(model->q_size, (unsigned int)1);
    ASSERT_EQ(model->qdot_size, (unsigned int)1);

    RobotDynamics::updateKinematics(*model, q, qdot, qddot);
    RobotDynamics::ReferenceFramePtr frame = model->referenceFrameMap["j1_link"];

    EXPECT_TRUE(model->referenceFrameMap["j1_link"]->getInverseTransformToRoot().r.isApprox(RobotDynamics::Math::Vector3d(0.0, 0.16, 0.8377), 1e-4));
}

TEST_F(UrdfReaderTests, testFixedArm)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_robot_arm.urdf";
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, model));

    // First body in URDf is a fixed joint body, so it'll get mashed together with the root body
    EXPECT_EQ(model->mBodies[0].mMass, 4.);
    EXPECT_TRUE(model->mBodies[0].mCenterOfMass.isApprox(RobotDynamics::Math::Vector3d(0., 0., 0.), 1e-14));

    EXPECT_EQ(model->Ib_c[0].Ixx, 0.0061063308908);
    EXPECT_EQ(model->Ib_c[0].Iyx, 0.0);
    EXPECT_EQ(model->Ib_c[0].Izx, 0.0);
    EXPECT_EQ(model->Ib_c[0].Iyy, 0.0061063308908);
    EXPECT_EQ(model->Ib_c[0].Izy, 0.0);
    EXPECT_EQ(model->Ib_c[0].Izz, 0.01125);

    unsigned int id = model->GetBodyId("test_robot_shoulder_link");
    EXPECT_EQ(model->mBodies[id].mMass, 7.778);
    EXPECT_TRUE(model->mBodies[id].mCenterOfMass.isApprox(RobotDynamics::Math::Vector3d(0., 0.01, 0.), 1e-14));

    EXPECT_EQ(model->Ib_c[id].Ixx, 0.0314743125769);
    EXPECT_EQ(model->Ib_c[id].Iyx, 0.);
    EXPECT_EQ(model->Ib_c[id].Izx, 0.);
    EXPECT_EQ(model->Ib_c[id].Iyy, 0.0314743125769);
    EXPECT_EQ(model->Ib_c[id].Izy, 0.);
    EXPECT_EQ(model->Ib_c[id].Izz, 0.021875625);

    id = model->GetParentBodyId(model->GetBodyId("gripper_right_finger_link"));
    EXPECT_EQ(model->GetBodyId("gripper_right_knuckle_link"), id);

    EXPECT_EQ(model->mJoints[1].mJointType, RobotDynamics::JointTypePrismatic);
    EXPECT_EQ(model->GetBodyId("test_robot_shoulder_link"), model->lambda[model->GetBodyId("test_robot_upper_arm_link")]);
}

TEST_F(UrdfReaderTests, testNegative1DofJointAxes)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_robot_arm.urdf";
    std::string file_path_2 = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_robot_arm_neg_jnt_axes.urdf";
    RobotDynamics::ModelPtr model(new RobotDynamics::Model()), model_neg_axes(new RobotDynamics::Model());
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, model));
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path_2, model_neg_axes));

    RobotDynamics::Math::VectorNd q = RobotDynamics::Math::VectorNd::Zero(model->q_size);
    RobotDynamics::Math::VectorNd q_neg = RobotDynamics::Math::VectorNd::Zero(model->q_size);
    RobotDynamics::Math::VectorNd qd = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    RobotDynamics::Math::VectorNd qd_neg = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    RobotDynamics::Math::VectorNd qdd = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    RobotDynamics::Math::VectorNd qdd_neg = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    RobotDynamics::Math::VectorNd tau = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);
    RobotDynamics::Math::VectorNd tau_neg = RobotDynamics::Math::VectorNd::Zero(model->qdot_size);

    for (int i = 0; i < q.size(); i++)
    {
        q[i] = 0.4 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        qd[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        tau[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        qdd[i] = 0.5 * M_PI * static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    }

    q_neg = -q;
    qd_neg = -qd;
    qdd_neg = -qdd;
    tau_neg = -tau;

    RobotDynamics::updateKinematics(*model, q, qd, qdd);
    RobotDynamics::updateKinematics(*model_neg_axes, q_neg, qd_neg, qdd_neg);

    for (unsigned int i = 0; i < model->mBodies.size(); i++)
    {
        EXPECT_TRUE(model->v[i].isApprox(model_neg_axes->v[i], 1e-14));
        EXPECT_TRUE(model->a[i].isApprox(model_neg_axes->a[i], 1e-14));
    }

    RobotDynamics::forwardDynamics(*model, q, qd, tau, qdd);
    RobotDynamics::forwardDynamics(*model_neg_axes, q_neg, qd_neg, tau_neg, qdd_neg);

    EXPECT_TRUE(qdd.isApprox(-qdd_neg, 1e-14));

    RobotDynamics::inverseDynamics(*model, q, qd, qdd, tau);
    RobotDynamics::inverseDynamics(*model_neg_axes, q_neg, qd_neg, qdd_neg, tau_neg);

    EXPECT_TRUE(tau.isApprox(-tau_neg, 1e-14));
}

TEST_F(UrdfReaderTests, testJointBodyMap)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_robot_arm.urdf";
    std::map<std::string, std::string> jointBodyMap;

    ASSERT_TRUE(RobotDynamics::Urdf::parseJointBodyNameMapFromFile(file_path, jointBodyMap));

    EXPECT_STREQ(jointBodyMap["test_robot_elbow_joint"].c_str(), "test_robot_forearm_link");
    EXPECT_TRUE(jointBodyMap.find("test_robot_ee_fixed_joint") == jointBodyMap.end());  // It's fixed, so shouldn't be here
    EXPECT_STREQ(jointBodyMap["test_robot_shoulder_pan_joint"].c_str(), "test_robot_shoulder_link");

    std::string contents = getFileContents(file_path);
    std::vector<unsigned int> q_indices;
    RobotDynamics::Urdf::parseJointAndQIndex(contents, q_indices);
    EXPECT_EQ(q_indices.size(), (unsigned int)12);
    // EXPECT_TRUE(q_indices[0] == 0);
}

TEST_F(UrdfReaderTests, testFloatingBaseRobot)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/floating_base_robot.urdf";
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(path_to_urdf, model));

    EXPECT_NEAR(model->mBodies[2].mMass, 4., 1e-14);
    EXPECT_TRUE(model->mBodies[2].mCenterOfMass.isApprox(RobotDynamics::Math::Vector3d(0.3, 0.2, 0.1), 1e-14));

    EXPECT_NEAR(model->Ib_c[2].Ixx, 0.1, 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Iyx, 0., 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Izx, 0., 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Iyy, 0.2, 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Izy, 0., 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Izz, 0.3, 1e-14);

    EXPECT_EQ(model->mJoints[1].mJointType, RobotDynamics::JointTypeTranslationXYZ);
    EXPECT_EQ(model->mJoints[2].mJointType, RobotDynamics::JointTypeSpherical);
}

TEST_F(UrdfReaderTests, testFloatingBaseRobotDeduceFloatingBase)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/floating_base_robot.urdf";
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(path_to_urdf, model));

    EXPECT_NEAR(model->mBodies[2].mMass, 4., 1e-14);
    EXPECT_TRUE(model->mBodies[2].mCenterOfMass.isApprox(RobotDynamics::Math::Vector3d(0.3, 0.2, 0.1), 1e-14));

    EXPECT_NEAR(model->Ib_c[2].Ixx, 0.1, 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Iyx, 0., 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Izx, 0., 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Iyy, 0.2, 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Izy, 0., 1e-14);
    EXPECT_NEAR(model->Ib_c[2].Izz, 0.3, 1e-14);

    EXPECT_EQ(model->mJoints[1].mJointType, RobotDynamics::JointTypeTranslationXYZ);
    EXPECT_EQ(model->mJoints[2].mJointType, RobotDynamics::JointTypeSpherical);
}

TEST_F(UrdfReaderTests, testRobotWithFloatingJoint)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/floating_joint_robot.urdf";
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(path_to_urdf, model));

    EXPECT_EQ(model->mJoints[1].mJointType, RobotDynamics::JointTypeTranslationXYZ);
    EXPECT_EQ(model->mJoints[2].mJointType, RobotDynamics::JointTypeEulerXYZ);

    std::string contents = getFileContents(path_to_urdf);
    std::vector<unsigned int> q_indices;
    EXPECT_TRUE(RobotDynamics::Urdf::parseJointAndQIndex(contents, q_indices));

    EXPECT_EQ(q_indices.size(), (unsigned int)1);
    EXPECT_EQ(q_indices[0], (unsigned int)6);
}

TEST_F(UrdfReaderTests, testRobotSingleBodyFloatingBase)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/single_body_floating_base.urdf";
    ASSERT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(path_to_urdf, model));

    std::string contents = getFileContents(path_to_urdf);
    std::vector<unsigned int> q_indices;
    EXPECT_TRUE(RobotDynamics::Urdf::parseJointAndQIndex(contents, q_indices));

    EXPECT_EQ(q_indices.size(), (unsigned int)0);
}

TEST_F(UrdfReaderTests, rotated_inertial)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/rotated_inertial.urdf";
    ASSERT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(path_to_urdf, model));

    RobotDynamics::Math::Matrix3d I1 = RobotDynamics::Math::Matrix3dIdentity;
    I1(0, 0) = 0.03;
    I1(1, 1) = 0.03;
    I1(2, 2) = 0.02;
    EXPECT_TRUE(I1.isApprox(model->mBodies[1].mInertia, 1.e-10));

    RobotDynamics::Math::Matrix3d I2 = RobotDynamics::Math::Matrix3dIdentity;
    I2(0, 0) = 0.03;
    I2(1, 1) = 0.02;
    I2(2, 2) = 0.03;
    EXPECT_TRUE(I2.isApprox(model->mBodies[2].mInertia, 1.e-10));

    RobotDynamics::Math::Matrix3d I3 = RobotDynamics::Math::Matrix3dIdentity;
    I3(0, 0) = 0.02;
    I3(1, 1) = 0.03;
    I3(2, 2) = 0.03;
    EXPECT_TRUE(I3.isApprox(model->mBodies[3].mInertia, 1.e-10));
}

TEST_F(UrdfReaderTests, rotated_inertial_floating_base)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/floating_base_rotated_inertial.urdf";
    ASSERT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(path_to_urdf, model));

    RobotDynamics::Math::Matrix3d I1 = RobotDynamics::Math::Matrix3dIdentity;
    I1(0, 0) = 0.03;
    I1(1, 1) = 0.03;
    I1(2, 2) = 0.02;
    EXPECT_TRUE(I1.isApprox(model->mBodies[2].mInertia, 1.e-10));

    RobotDynamics::Math::Matrix3d I2 = RobotDynamics::Math::Matrix3dIdentity;
    I2(0, 0) = 0.03;
    I2(1, 1) = 0.02;
    I2(2, 2) = 0.03;
    EXPECT_TRUE(I2.isApprox(model->mBodies[3].mInertia, 1.e-10));

    RobotDynamics::Math::Matrix3d I3 = RobotDynamics::Math::Matrix3dIdentity;
    I3(0, 0) = 0.02;
    I3(1, 1) = 0.03;
    I3(2, 2) = 0.03;
    EXPECT_TRUE(I3.isApprox(model->mBodies[4].mInertia, 1.e-10));
}

RobotDynamics::Math::SpatialMatrix reorgAddedInertia(const RobotDynamics::Math::SpatialMatrix& in)
{
    RobotDynamics::Math::SpatialMatrix out;
    out.block<3, 3>(0, 0) = in.block<3, 3>(3, 3);
    out.block<3, 3>(3, 0) = in.block<3, 3>(0, 3);
    out.block<3, 3>(0, 3) = in.block<3, 3>(3, 0);
    out.block<3, 3>(3, 3) = in.block<3, 3>(0, 0);
    return out;
}

TEST_F(UrdfReaderTests, parseHydrodynamicsParameters)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_robot_arm_hydro.urdf";
    RobotDynamics::Model model;
    std::map<std::string, RobotDynamics::Urdf::hydro::BodyHydroData> data;
    EXPECT_TRUE(RobotDynamics::Urdf::hydro::parseHydrodynamicsParameters(getFileContents(file_path).c_str(), data, false));

    EXPECT_FALSE(data["gripper_right_finger_link"].hasHydro);

    RobotDynamics::Math::SpatialMatrix I(1., 0., 0., 1.03, 0., 0., 0., 183., 0., 0., 0., 0., 0., 0., 184., 0., 0., 11.133, 0., 0., 0., 2., 0., 0., 0., 2.033, 0., 0., 44.,
                                         0., 0., 1.94, 0., 0., 0., 45.311);

    EXPECT_TRUE(data["gripper_left_finger_tip_link"].hasHydro);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].volume, 0.003);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].centerOfBuoyancy.x(), 0.01);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].centerOfBuoyancy.y(), 0.21);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].centerOfBuoyancy.z(), -0.03);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].linearDrag[3], 1.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].linearDrag[4], 2.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].linearDrag[5], 3.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].linearDrag[0], 4.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].linearDrag[1], 5.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].linearDrag[2], 6.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].quadraticDrag[3], -1.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].quadraticDrag[4], -2.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].quadraticDrag[5], -3.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].quadraticDrag[0], -4.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].quadraticDrag[1], -5.);
    EXPECT_EQ(data["gripper_left_finger_tip_link"].quadraticDrag[2], -6.);
    EXPECT_TRUE(reorgAddedInertia(I).isApprox(data["gripper_left_finger_tip_link"].addedMassMatrix, E_MINUS_14));

    I = RobotDynamics::Math::SpatialMatrix(1., 0., 0., 0., 0., 0., 0., 183., 0., 0., 0., 0., 0., 0., 184., 0., 0., 0.01, 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 44., 0.,
                                           11., 0., 0., 0., 0., 45.);
    EXPECT_TRUE(data["gripper_right_finger_tip_link"].hasHydro);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].volume, 0.00005);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].centerOfBuoyancy.x(), 0.11);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].centerOfBuoyancy.y(), 1.21);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].centerOfBuoyancy.z(), -0.13);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].linearDrag[3], 10.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].linearDrag[4], 20.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].linearDrag[5], 30.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].linearDrag[0], 40.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].linearDrag[1], 50.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].linearDrag[2], 60.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].quadraticDrag[3], -10.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].quadraticDrag[4], -20.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].quadraticDrag[5], -30.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].quadraticDrag[0], -40.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].quadraticDrag[1], -50.);
    EXPECT_EQ(data["gripper_right_finger_tip_link"].quadraticDrag[2], -60.);
    EXPECT_TRUE(reorgAddedInertia(I).isApprox(data["gripper_right_finger_tip_link"].addedMassMatrix, E_MINUS_14));

    I = RobotDynamics::Math::SpatialMatrix(1., 0., 0., 0., 0., 0., 0., 183., 0., 33.1, 0., 0., 0., 0., 184., 0., 0., 0., 0., 0., 0., 2., 3.11, 0., 0., 0., 0., 0., 44.,
                                           0., 0., 0., 0., 0., 0., 45.);

    EXPECT_TRUE(data["gripper_right_inner_knuckle_link"].hasHydro);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].volume, 0.1);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].centerOfBuoyancy.x(), 0.01);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].centerOfBuoyancy.y(), 0.21);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].centerOfBuoyancy.z(), -0.03);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].linearDrag[0], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].linearDrag[1], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].linearDrag[2], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].linearDrag[3], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].linearDrag[4], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].linearDrag[5], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].quadraticDrag[0], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].quadraticDrag[1], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].quadraticDrag[2], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].quadraticDrag[3], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].quadraticDrag[4], 0.);
    EXPECT_EQ(data["gripper_right_inner_knuckle_link"].quadraticDrag[5], 0.);
    EXPECT_TRUE(reorgAddedInertia(I).isApprox(data["gripper_right_inner_knuckle_link"].addedMassMatrix, E_MINUS_14));

    I.setZero();
    EXPECT_TRUE(data["gripper_left_finger_link"].hasHydro);
    EXPECT_EQ(data["gripper_left_finger_link"].volume, 0.1);
    EXPECT_EQ(data["gripper_left_finger_link"].centerOfBuoyancy.x(), 0.01);
    EXPECT_EQ(data["gripper_left_finger_link"].centerOfBuoyancy.y(), 0.21);
    EXPECT_EQ(data["gripper_left_finger_link"].centerOfBuoyancy.z(), -0.03);
    EXPECT_EQ(data["gripper_left_finger_link"].linearDrag[0], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].linearDrag[1], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].linearDrag[2], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].linearDrag[3], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].linearDrag[4], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].linearDrag[5], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].quadraticDrag[0], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].quadraticDrag[1], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].quadraticDrag[2], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].quadraticDrag[3], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].quadraticDrag[4], 0.);
    EXPECT_EQ(data["gripper_left_finger_link"].quadraticDrag[5], 0.);
    EXPECT_TRUE(data["gripper_left_finger_link"].addedMassMatrix.isZero(E_MINUS_14));

    I = RobotDynamics::Math::SpatialMatrix(1., 0., 0., 0., 0., 0., 0., 183., 0., 0., 0., 0., 0., 0., 184., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 44., 0.,
                                           0., 0., 0., 0., 0., 45.);

    EXPECT_TRUE(data["gripper_left_knuckle_link"].hasHydro);
    EXPECT_EQ(data["gripper_left_knuckle_link"].volume, 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].centerOfBuoyancy.x(), 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].centerOfBuoyancy.y(), 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].centerOfBuoyancy.z(), 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].linearDrag[0], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].linearDrag[1], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].linearDrag[2], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].linearDrag[3], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].linearDrag[4], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].linearDrag[5], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].quadraticDrag[0], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].quadraticDrag[1], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].quadraticDrag[2], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].quadraticDrag[3], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].quadraticDrag[4], 0.);
    EXPECT_EQ(data["gripper_left_knuckle_link"].quadraticDrag[5], 0.);
    EXPECT_TRUE(reorgAddedInertia(I).isApprox(data["gripper_left_knuckle_link"].addedMassMatrix, E_MINUS_14));
}

TEST_F(UrdfReaderTests, parseHydrodynamicsParametersFailureCases)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/invalid_hydro_added_mass.urdf";
    std::map<std::string, RobotDynamics::Urdf::hydro::BodyHydroData> data;
    EXPECT_FALSE(RobotDynamics::Urdf::hydro::parseHydrodynamicsParameters(getFileContents(file_path).c_str(), data, false));

    file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/invalid_drag_block.urdf";
    EXPECT_FALSE(RobotDynamics::Urdf::hydro::parseHydrodynamicsParameters(getFileContents(file_path).c_str(), data, false));

    file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/invalid_xml.urdf";
    EXPECT_FALSE(RobotDynamics::Urdf::hydro::parseHydrodynamicsParameters(getFileContents(file_path).c_str(), data, false));
}

TEST_F(UrdfReaderTests, checkHydroBodiesHaveCorrectHydroInfo)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_robot_arm_hydro.urdf";
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, model));

    RobotDynamics::Math::SpatialMatrix I(2., 0., 0., 0., 0., 0., 0., 44., 0., 0., 0., 0., 0., 0., 45., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 183.0, 0., 0.,
                                         0., 0., 0., 0., 184.);

    EXPECT_EQ(model->mBodies[model->GetBodyId("test_robot_shoulder_link")].volume, 0.1);
    EXPECT_EQ(model->mBodies[model->GetBodyId("test_robot_shoulder_link")].mCenterOfBuoyancy.x(), 0.1101);
    EXPECT_EQ(model->mBodies[model->GetBodyId("test_robot_shoulder_link")].mCenterOfBuoyancy.y(), 0.121);
    EXPECT_EQ(model->mBodies[model->GetBodyId("test_robot_shoulder_link")].mCenterOfBuoyancy.z(), -0.013);

    EXPECT_TRUE(I.isApprox(model->mBodies[model->GetBodyId("gripper_left_knuckle_link")].addedMassMatrix, E_MINUS_14));
}

/**
 * The following unit test comes from running a simulation and comparing Adot with a numerically differentiated A matrix. The error
 * between the two was never really greater than about 1.e-4 which seemed sufficient to me so I copied the matrices over here and
 * made this test which until a better test method is found will have to do.
 */
TEST_F(UrdfReaderTests, centroidalMomentumMatrixDot)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_centroidal_momentum_robot.urdf";
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, model));
    model->gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -9.81);

    RobotDynamics::Math::VectorNd q(model->q_size), qdot(model->qdot_size);

    q << 2843548.033249, -209909.31618248, -28.737112471808, 0.013866084594564, -0.018922989600583, -0.27902392880053, 46.988201887017, 167.71435783402,
        0.012894144231981, 0.003644780816261, 0.023437157143762, 0.17174982556876, -272.04399437984, -607.14717812687, 330.80048156122, -356.5405073459, 389.03904152552,
        0.0014068227420712, -177.3187382107, 0.95999755172569;
    qdot << 0.72356305088634, -0.27922136265996, -0.074898891065314, 0.057378004355399, 0.027501228787648, -0.11882428506134, 1.5988661473426e-05, 8.1292436215699e-06,
        0.0004935993483221, -1.2482539961087e-05, 4.493479337845e-06, 2.1550289263633e-05, -5.527300395089e-06, 2.0919225428219e-05, 2.0919225428732e-05,
        1.500606255915e-05, 7.1466266552134e-06, -0.0050623463209877, -1.1308026111878e-06;
    RobotDynamics::updateKinematicsCustom(*model, &q, &qdot, nullptr);

    RobotDynamics::Math::MatrixNd adot(6, model->qdot_size), a(6, model->qdot_size), adot2(6, model->qdot_size), a2(6, model->qdot_size);
    RobotDynamics::Math::SpatialVector adotv(81.38892596978, 134.45726399483, 41.428640871662, 75.132338829693, -60.321498882245, 47.201591640815);
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model, q, a, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(*model, q, qdot, adot, false);

    EXPECT_TRUE(adotv.isApprox(adot * qdot, 1.e-7));

    RobotDynamics::Utils::calcCentroidalMomentumMatrixAndMatrixDot(*model, q, qdot, a2, adot2, false);
    EXPECT_TRUE(a.isApprox(a2, 1.e-14));
    EXPECT_TRUE(adot.isApprox(adot2, 1.e-14));

    q << 2843555.7426166, -209906.42505932, -31.984289527845, -0.019318359864448, 0.084947029667317, 0.38843943516417, 261.52886515801, -370.40405071333, 0.5104959350258,
        1.1892981440042, -0.74378863055244, 0.88420467075226, 265.39312387749, -419.37422448213, 507.66603672344, -672.60712542416, 608.34632442873, 7.2151029691512e-06,
        241.8163217398, 0.91734704901263;
    qdot << 0.90780498518421, 0.37795991108503, -0.12286976779791, 0.016129512038977, 0.011259829695686, 0.10567671184028, 1.7140545707062e-05, 1.0042944296275e-05,
        0.06211524064157, 0.12779617820982, -0.088022453340217, 0.096940404958886, -4.9915274408924e-06, 2.3314482437527e-05, 2.3314482437333e-05, 1.5741349596001e-05,
        8.6437310570578e-06, -0.0021085140258945, -1.6102261528733e-06;
    RobotDynamics::updateKinematicsCustom(*model, &q, &qdot, nullptr);

    adotv << 29.80644161631, -26.852723901131, 4.1278484858571, 47.672589300394, 48.762586239518, -22.769718014228;
    RobotDynamics::Utils::calcCentroidalMomentumMatrix(*model, q, a, false);
    RobotDynamics::Utils::calcCentroidalMomentumMatrixDot(*model, q, qdot, adot, false);

    EXPECT_TRUE(adotv.isApprox(adot * qdot, 1.e-7));
    RobotDynamics::Utils::calcCentroidalMomentumMatrixAndMatrixDot(*model, q, qdot, a2, adot2, false);
    EXPECT_TRUE(a.isApprox(a2, 1.e-14));
    EXPECT_TRUE(adot.isApprox(adot2, 1.e-14));
}

TEST_F(UrdfReaderTests, eulerzyx_coordinates_for_floating_base)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_centroidal_momentum_robot.urdf";
    RobotDynamics::ModelPtr euler_model(new RobotDynamics::Model()), fb_model(new RobotDynamics::Model());
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, euler_model, RobotDynamics::JointTypeEulerZYX));
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, fb_model));

    RobotDynamics::Math::VectorNd q_eul(euler_model->q_size), qdot_eul(euler_model->qdot_size);
    RobotDynamics::Math::VectorNd q_fb(fb_model->q_size), qdot_fb(fb_model->qdot_size);

    // randomize the fb state
    for (int i = 0; i < q_eul.size(); ++i)
    {
        q_eul[i] = M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2;
        qdot_eul[i] = M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2;
        q_fb[i] = q_eul[i];
        qdot_fb[i] = qdot_eul[i];
    }

    unsigned int euler_base_link_id = euler_model->GetBodyId("base_link");
    unsigned int fb_base_link_id = fb_model->GetBodyId("base_link");

    // now fix the base link orientation bits since they have different orientation coordinates for the base link
    RobotDynamics::Math::Quaternion o(RobotDynamics::Math::XeulerZYX(q_eul[3], q_eul[4], q_eul[5]).E);
    fb_model->SetQuaternion(fb_base_link_id, o, q_fb);

    RobotDynamics::Math::Vector3d w = RobotDynamics::Math::angular_velocity_from_zyx_angle_rates(RobotDynamics::Math::Vector3d(q_eul[3], q_eul[4], q_eul[5]),
                                                                                                 RobotDynamics::Math::Vector3d(qdot_eul[3], qdot_eul[4], qdot_eul[5]));

    qdot_fb[3] = w[0];
    qdot_fb[4] = w[1];
    qdot_fb[5] = w[2];
    // now update both kinematics and check that they are the same
    RobotDynamics::updateKinematicsCustom(*euler_model, &q_eul, &qdot_eul);
    RobotDynamics::updateKinematicsCustom(*fb_model, &q_fb, &qdot_fb);

    for (size_t i = 0; i < fb_model->bodyFrames.size(); ++i)
    {
        EXPECT_TRUE(fb_model->bodyFrames[i]->getTransformToRoot().toMatrix().isApprox(euler_model->bodyFrames[i]->getTransformToRoot().toMatrix(), 1.e-14));
    }

    for (size_t i = 0; i < fb_model->bodyCenteredFrames.size(); ++i)
    {
        EXPECT_TRUE(
            fb_model->bodyCenteredFrames[i]->getTransformToRoot().toMatrix().isApprox(euler_model->bodyCenteredFrames[i]->getTransformToRoot().toMatrix(), 1.e-14));
    }
    EXPECT_TRUE(fb_model->v[fb_base_link_id].isApprox(euler_model->v[euler_base_link_id], 1.e-14));
}

TEST_F(UrdfReaderTests, eulerxyz_coordinates_for_floating_base)
{
    std::string file_path = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/test_centroidal_momentum_robot.urdf";
    RobotDynamics::ModelPtr euler_model(new RobotDynamics::Model()), fb_model(new RobotDynamics::Model());
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, euler_model, RobotDynamics::JointTypeEulerXYZ));
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(file_path, fb_model));

    RobotDynamics::Math::VectorNd q_eul(euler_model->q_size), qdot_eul(euler_model->qdot_size);
    RobotDynamics::Math::VectorNd q_fb(fb_model->q_size), qdot_fb(fb_model->qdot_size);

    // randomize the fb state
    for (int i = 0; i < q_eul.size(); ++i)
    {
        q_eul[i] = M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2;
        qdot_eul[i] = M_PI * (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) - M_PI_2;
        q_fb[i] = q_eul[i];
        qdot_fb[i] = qdot_eul[i];
    }

    unsigned int euler_base_link_id = euler_model->GetBodyId("base_link");
    unsigned int fb_base_link_id = fb_model->GetBodyId("base_link");

    // now fix the base link orientation bits since they have different orientation coordinates for the base link
    RobotDynamics::Math::Quaternion o(RobotDynamics::Math::XeulerXYZ(q_eul[3], q_eul[4], q_eul[5]).E);
    fb_model->SetQuaternion(fb_base_link_id, o, q_fb);

    RobotDynamics::Math::Vector3d w = RobotDynamics::Math::angular_velocity_from_xyz_angle_rates(RobotDynamics::Math::Vector3d(q_eul[3], q_eul[4], q_eul[5]),
                                                                                                 RobotDynamics::Math::Vector3d(qdot_eul[3], qdot_eul[4], qdot_eul[5]));

    qdot_fb[3] = w[0];
    qdot_fb[4] = w[1];
    qdot_fb[5] = w[2];
    // now update both kinematics and check that they are the same
    RobotDynamics::updateKinematicsCustom(*euler_model, &q_eul, &qdot_eul);
    RobotDynamics::updateKinematicsCustom(*fb_model, &q_fb, &qdot_fb);

    for (size_t i = 0; i < fb_model->bodyFrames.size(); ++i)
    {
        EXPECT_TRUE(fb_model->bodyFrames[i]->getTransformToRoot().toMatrix().isApprox(euler_model->bodyFrames[i]->getTransformToRoot().toMatrix(), 1.e-14));
    }

    for (size_t i = 0; i < fb_model->bodyCenteredFrames.size(); ++i)
    {
        EXPECT_TRUE(
            fb_model->bodyCenteredFrames[i]->getTransformToRoot().toMatrix().isApprox(euler_model->bodyCenteredFrames[i]->getTransformToRoot().toMatrix(), 1.e-14));
    }
    EXPECT_TRUE(fb_model->v[fb_base_link_id].isApprox(euler_model->v[euler_base_link_id], 1.e-14));
}

TEST_F(UrdfReaderTests, load_ros_params_from_file)
{
    std::string filepath = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/config/load_ros_params_from_file.yaml";
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "--params-file", filepath.c_str() });

    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("load_ros_params_from_file", options);
    RobotDynamics::ModelPtr model(new RobotDynamics::Model(n));
    std::string path_to_urdf = ament_index_cpp::get_package_share_directory("rdl_urdfreader") + "/tests/urdf/floating_base_robot.urdf";
    EXPECT_TRUE(RobotDynamics::Urdf::urdfReadFromFile(path_to_urdf, model));
    model->setupRosParameters();

    EXPECT_NEAR(model->mass, 17.778, 1.e-14);
    EXPECT_NEAR(model->volume, 1.1, 1.e-14);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    rclcpp::init(argc, argv);
    return RUN_ALL_TESTS();
}
