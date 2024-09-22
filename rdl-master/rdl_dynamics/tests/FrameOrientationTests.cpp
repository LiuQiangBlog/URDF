// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "rdl_dynamics/FrameOrientation.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "UnitTestUtils.hpp"

using namespace RobotDynamics;
using namespace RobotDynamics::Math;

class FrameOrientationTest : public ::testing::Test
{
  public:
    FrameOrientationTest()
    {
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(FrameOrientationTest, setters)
{
    ReferenceFramePtr f1(new ReferenceFrame("f1", ReferenceFrame::getWorldFrame(), SpatialTransform(), true, 0));

    Quaternion q(1., 2., 3., 4.);

    FrameOrientation orientation(f1, q);
    EXPECT_EQ(orientation.x(), 1.);
    EXPECT_EQ(orientation.y(), 2.);
    EXPECT_EQ(orientation.z(), 3.);
    EXPECT_EQ(orientation.w(), 4.);

    SpatialTransform X = Xrotz(0.1);
    Quaternion q2(X.E);

    orientation = FrameOrientation(f1, X.E);

    EXPECT_EQ(orientation.x(), Quaternion(X.E).x());
    EXPECT_EQ(orientation.y(), Quaternion(X.E).y());
    EXPECT_EQ(orientation.z(), Quaternion(X.E).z());
    EXPECT_EQ(orientation.w(), Quaternion(X.E).w());

    RobotDynamics::Math::FrameOrientation q3(f1, orientation.toAxisAngle());
    RobotDynamics::Math::FrameOrientation q4(f1, X.E);
    EXPECT_TRUE(q3.isApprox(q4, unit_test_utils::E_MINUS_14));

    RobotDynamics::Math::Vector3d axis(-0.3, 0.1, 1.2);
    axis.normalize();

    RobotDynamics::Math::AxisAngle axis_angle(-3.1, axis);
    RobotDynamics::Math::Matrix3d E(axis_angle);

    q3.setOrientation(axis_angle);
    q4.setOrientation(E);
    EXPECT_TRUE(q3.isApprox(q4, unit_test_utils::E_MINUS_14));
}

TEST_F(FrameOrientationTest, changeFrame)
{
    SpatialTransform X1 = Xrotz(M_PI_2);
    ReferenceFramePtr f1(new ReferenceFrame("f1", ReferenceFrame::getWorldFrame(), X1, true, 0));
    SpatialTransform X2 = Xrotz(-M_PI_2);
    ReferenceFramePtr f2(new ReferenceFrame("f2", f1, X2, true, 1));
    SpatialTransform X3 = Xrotz(-M_PI_2);
    ReferenceFramePtr f3(new ReferenceFrame("f3", f2, X3, true, 1));

    FrameOrientation orientation(f2, Quaternion());
    orientation.changeFrame(f1);

    Quaternion expected(X2.E);
    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(orientation, expected, unit_test_utils::E_MINUS_14));
    EXPECT_STREQ(orientation.getReferenceFrame()->getName().c_str(), "f1");

    orientation.changeFrame(ReferenceFrame::getWorldFrame());

    expected = Quaternion(0., 0., 0., 1.);
    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(orientation, expected, unit_test_utils::E_MINUS_14));
    EXPECT_STREQ(orientation.getReferenceFrame()->getName().c_str(), "world");

    orientation.changeFrame(f3);
    expected = Quaternion(Xrotz(M_PI_2).E);
    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(orientation, expected, unit_test_utils::E_MINUS_14));
    EXPECT_STREQ(orientation.getReferenceFrame()->getName().c_str(), "f3");
}

TEST_F(FrameOrientationTest, changeFrameAndCopy)
{
    SpatialTransform X1 = Xrotz(M_PI_2);
    std::shared_ptr<ReferenceFrame> f1(new ReferenceFrame("f1", ReferenceFrame::getWorldFrame(), X1, true, 0));
    SpatialTransform X2 = Xrotz(-M_PI_2);
    std::shared_ptr<ReferenceFrame> f2(new ReferenceFrame("f2", f1, X2, true, 1));
    SpatialTransform X3 = Xrotz(-M_PI_2);
    std::shared_ptr<ReferenceFrame> f3(new ReferenceFrame("f3", f2, X3, true, 1));

    FrameOrientation orientation(f2, Quaternion());
    FrameOrientation orientation_copy = orientation.changeFrameAndCopy(f1);

    Quaternion expected(Quaternion(X2.E));
    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(orientation_copy, expected, unit_test_utils::E_MINUS_14));
    EXPECT_STREQ(orientation.getReferenceFrame()->getName().c_str(), "f2");
    EXPECT_STREQ(orientation_copy.getReferenceFrame()->getName().c_str(), "f1");

    orientation = orientation_copy;
    orientation_copy = orientation.changeFrameAndCopy(ReferenceFrame::getWorldFrame());

    expected = Quaternion(0., 0., 0., 1.);
    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(orientation_copy, expected, unit_test_utils::E_MINUS_14));
    EXPECT_STREQ(orientation.getReferenceFrame()->getName().c_str(), "f1");
    EXPECT_STREQ(orientation_copy.getReferenceFrame()->getName().c_str(), "world");

    orientation = orientation_copy;
    orientation_copy = orientation.changeFrameAndCopy(f3);
    expected = Quaternion(Xrotz(M_PI_2).E);
    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(orientation_copy, expected, unit_test_utils::E_MINUS_14));
    EXPECT_STREQ(orientation.getReferenceFrame()->getName().c_str(), "world");
    EXPECT_STREQ(orientation_copy.getReferenceFrame()->getName().c_str(), "f3");
}

TEST_F(FrameOrientationTest, model)
{
    RobotDynamics::Body b(1., RobotDynamics::Math::Vector3d(1., 1., 1.), RobotDynamics::Math::Vector3d(1., 1., 1.));
    RobotDynamics::Joint floating_base(RobotDynamics::JointType::JointTypeFloatingBase);

    RobotDynamics::Model model;
    model.gravity = RobotDynamics::Math::MotionVector(0., 0., 0., 0., 0., -9.81);

    unsigned int floating_body_id = model.appendBody(RobotDynamics::Math::SpatialTransform(), floating_base, b, "b1");
    unsigned int fb_id =
        model.addBody(floating_body_id, RobotDynamics::Math::Xrotz(M_PI_2), RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), b, "fixed_body");
    model.addBody(fb_id, RobotDynamics::Math::Xroty(M_PI_2), RobotDynamics::Joint(RobotDynamics::JointType::JointTypeFixed), b, "fixed_body_2");

    RobotDynamics::Math::VectorNd q_ = RobotDynamics::Math::VectorNd::Zero(model.q_size);
    RobotDynamics::Math::VectorNd qdot_ = RobotDynamics::Math::VectorNd::Zero(model.qdot_size);
    RobotDynamics::Math::VectorNd tau_ = RobotDynamics::Math::VectorNd::Zero(model.qdot_size);

    RobotDynamics::ReferenceFramePtr b1_frame = model.getReferenceFrame("b1");
    RobotDynamics::ReferenceFramePtr fixed_body_frame = model.getReferenceFrame("fixed_body");
    RobotDynamics::ReferenceFramePtr fixed_body2_frame = model.getReferenceFrame("fixed_body_2");

    model.SetQuaternion(floating_body_id, RobotDynamics::Math::Quaternion(0., 0., 0., 1.), q_);
    RobotDynamics::updateKinematicsCustom(model, &q_, &qdot_, nullptr);

    FrameOrientation f(fixed_body_frame, RobotDynamics::Math::Xroty(M_PI_2).E);
    f.changeFrame(model.worldFrame);

    RobotDynamics::Math::Quaternion q(model.worldFrame->getTransformToDesiredFrame(fixed_body2_frame).E);
    EXPECT_TRUE(f.isApprox(q, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS();
}
