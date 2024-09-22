// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "UnitTestUtils.hpp"
#include "Fixtures.hpp"
#include "rdl_dynamics/Kinematics.hpp"

using namespace RobotDynamics::Math;

class SpatialMotionTests : public testing::Test
{
  public:
    SpatialMotionTests()
    {
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(FixedBase3DoFPlanar, get_frame_vectors)
{
    SpatialMotion v1(model->getReferenceFrame("body_a"), ReferenceFrame::getWorldFrame(), model->getReferenceFrame("body_a"), 1., 2., 3., 4., 5., 6.);

    FrameVector fvl = v1.getFramedLinearPart();
    FrameVector fva = v1.getFramedAngularPart();

    EXPECT_STREQ(fvl.getReferenceFrame()->getName().c_str(), v1.getReferenceFrame()->getName().c_str());

    EXPECT_EQ(fva.x(), v1.wx());
    EXPECT_EQ(fva.y(), v1.wy());
    EXPECT_EQ(fva.z(), v1.wz());

    EXPECT_EQ(fvl.x(), v1.vx());
    EXPECT_EQ(fvl.y(), v1.vy());
    EXPECT_EQ(fvl.z(), v1.vz());

    v1.setIncludingFrame(model->getReferenceFrame("body_b"), MotionVector(0.1, -0.1, 0.2, -0.2, 0.3, -0.3));

    EXPECT_STREQ(v1.getReferenceFrame()->getName().c_str(), "body_b");
    EXPECT_EQ(v1.wx(), 0.1);
    EXPECT_EQ(v1.wy(), -0.1);
    EXPECT_EQ(v1.wz(), 0.2);

    EXPECT_EQ(v1.vx(), -0.2);
    EXPECT_EQ(v1.vy(), 0.3);
    EXPECT_EQ(v1.vz(), -0.3);

    v1.setIncludingFrame(model->getReferenceFrame("body_c"), 3., 4., 5., 6., 7., 8.);

    EXPECT_STREQ(v1.getReferenceFrame()->getName().c_str(), "body_c");
    EXPECT_EQ(v1.wx(), 3.);
    EXPECT_EQ(v1.wy(), 4.);
    EXPECT_EQ(v1.wz(), 5.);

    EXPECT_EQ(v1.vx(), 6.);
    EXPECT_EQ(v1.vy(), 7.);
    EXPECT_EQ(v1.vz(), 8.);
}

TEST_F(FixedBase3DoFPlanar, changeFrameAndCopy)
{
    SpatialMotion v1(model->getReferenceFrame("body_b"), ReferenceFrame::getWorldFrame(), model->getReferenceFrame("body_b"), 1., 2., 3., 4., 5., 6.);
    SpatialMotion v2;

    randomizeStates();
    updateKinematicsCustom(*model, &Q, nullptr, nullptr);

    v2 = v1.changeFrameAndCopy(ReferenceFrame::getWorldFrame());

    EXPECT_FALSE(v2.wx() == v1.wx());
    EXPECT_TRUE(v2.wy() == v1.wy());  // its a rot about y axis, so this one isn't supposed to change
    EXPECT_FALSE(v2.wz() == v1.wz());
    EXPECT_FALSE(v2.vx() == v1.vx());
    EXPECT_FALSE(v2.vy() == v1.vy());
    EXPECT_FALSE(v2.vz() == v1.vz());

    v1.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(v2.wx() == v1.wx());
    EXPECT_TRUE(v2.wy() == v1.wy());
    EXPECT_TRUE(v2.wz() == v1.wz());
    EXPECT_TRUE(v2.vx() == v1.vx());
    EXPECT_TRUE(v2.vy() == v1.vy());
    EXPECT_TRUE(v2.vz() == v1.vz());

    SpatialMotion v3(model->getReferenceFrame("body_b"), ReferenceFrame::getWorldFrame(), model->getReferenceFrame("body_b"), 1., 2., 3., 4., 5., 6.);
    SpatialMotion v4;

    v4 = v3.changeFrameAndCopy(ReferenceFrame::getWorldFrame());

    EXPECT_FALSE(v3.wx() == v4.wx());
    EXPECT_TRUE(v3.wy() == v4.wy());  // its a rot about y axis, so this one isn't supposed to change
    EXPECT_FALSE(v3.wz() == v4.wz());
    EXPECT_FALSE(v3.vx() == v4.vx());
    EXPECT_FALSE(v3.vy() == v4.vy());
    EXPECT_FALSE(v3.vz() == v4.vz());

    v3.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(v3.wx() == v4.wx());
    EXPECT_TRUE(v3.wy() == v4.wy());
    EXPECT_TRUE(v3.wz() == v4.wz());
    EXPECT_TRUE(v3.vx() == v4.vx());
    EXPECT_TRUE(v3.vy() == v4.vy());
    EXPECT_TRUE(v3.vz() == v4.vz());
}

TEST_F(FixedBase3DoFPlanar, testAddAndSubtract)
{
    SpatialMotion v1(model->getReferenceFrame("body_a"), ReferenceFrame::getWorldFrame(), model->getReferenceFrame("body_a"), 0., 1., 0., 0., 0., 0.);

    SpatialMotion v2(model->getReferenceFrame("body_b"), ReferenceFrame::getWorldFrame(), model->getReferenceFrame("body_b"), 0., 1., 0., 0., 0., 0.);

    try
    {
        // Frame mismatches, should throw
        SpatialMotion v3 = v1 + v2;
        FAIL();
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ("Reference frames do not match!", e.what());
    }

    SpatialMotion v3(model->getReferenceFrame("body_c"), model->getReferenceFrame("body_b"), ReferenceFrame::getWorldFrame(), 0., 1., 0., 0., 0., 0.);

    try
    {
        // Experessed in frames aren't the same, should throw
        v3 += v2;
        FAIL();
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ("Reference frames do not match!", e.what());
    }

    SpatialMotion v4(model->getReferenceFrame("body_c"), model->getReferenceFrame("body_b"), model->getReferenceFrame("body_b"), 0., 1., 0., 0., 0., 0.);

    SpatialMotion v5 = v2 + v4;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v5, SpatialVector(0., 2., 0., 0., 0., 0.), unit_test_utils::E_MINUS_14));

    SpatialMotion v6 = v5;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v5, SpatialVector(0., 2., 0., 0., 0., 0.), unit_test_utils::E_MINUS_14));

    SpatialMotion v7(model->getReferenceFrame("body_c"), model->getReferenceFrame("body_b"), model->getReferenceFrame("body_b"), -0.2, 0.3, -0.4, 0.5, -0.6, 0.7);
    SpatialMotion v8(model->getReferenceFrame("body_a"), model->getReferenceFrame("body_c"), model->getReferenceFrame("body_b"), -0.5, -0.3, 0.1, 0.1, -4.6, 1.7);

    EXPECT_EQ(v7.wx(), -0.2);
    EXPECT_EQ(v7.wy(), 0.3);
    EXPECT_EQ(v7.wz(), -0.4);
    EXPECT_EQ(v7.vx(), 0.5);
    EXPECT_EQ(v7.vy(), -0.6);
    EXPECT_EQ(v7.vz(), 0.7);

    SpatialMotion v9 = v7 + v8;

    EXPECT_STREQ(v9.getReferenceFrame()->getName().c_str(), v8.getReferenceFrame()->getName().c_str());
    EXPECT_STREQ(v9.getBaseFrame()->getName().c_str(), v7.getBaseFrame()->getName().c_str());
    EXPECT_STREQ(v9.getReferenceFrame()->getName().c_str(), v7.getReferenceFrame()->getName().c_str());

    EXPECT_NEAR(v9.wx(), -0.7, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(v9.wy(), 0, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(v9.wz(), -0.3, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(v9.vx(), 0.6, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(v9.vy(), -5.2, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(v9.vz(), 2.4, unit_test_utils::E_MINUS_14);

    SpatialMotion v10 = v9 - v8;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v10, v7, unit_test_utils::E_MINUS_14));

    v10 = v9 - v7;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(v10, v8, unit_test_utils::E_MINUS_14));

    try
    {
        v8 -= v7;
        FAIL();
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Cannot perform -= operation on spatial motion vectors due to a reference frame mismatch!");
    }

    v7.setBodyFrame(model->getReferenceFrame("body_a"));
    v7.setBaseFrame(model->getReferenceFrame("body_b"));
    EXPECT_TRUE(v7.getBodyFrame() == model->getReferenceFrame("body_a"));
    EXPECT_TRUE(v7.getBaseFrame() == model->getReferenceFrame("body_b"));
    EXPECT_TRUE(v7.getReferenceFrame() == model->getReferenceFrame("body_b"));
}

TEST_F(SpatialMotionTests, testTransformToSpatialVec)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialTransform X_st;
    X_st.r = trans;

    SpatialMatrix X_66_matrix(SpatialMatrix::Zero(6, 6));
    X_66_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_st.E = X_66_matrix.block<3, 3>(0, 0);

    std::shared_ptr<ReferenceFrame> frameA(new ReferenceFrame("frameA", ReferenceFrame::getWorldFrame(), X_st, 0, 0));

    MotionVector v(1.1, 2.1, 3.1, 4.1, 5.1, 6.1);

    SpatialMotion m(frameA, ReferenceFrame::getWorldFrame(), frameA, v);

    v.transform(X_st);
    SpatialVector vec_out = m.transform_copy(X_st);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(vec_out, v, unit_test_utils::E_MINUS_14));
}

TEST_F(SpatialMotionTests, testChangeFrame)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialTransform X_st;
    X_st.r = trans;

    SpatialMatrix X_66_matrix(SpatialMatrix::Zero(6, 6));
    X_66_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_st.E = X_66_matrix.block<3, 3>(0, 0);

    std::shared_ptr<ReferenceFrame> frameA(new ReferenceFrame("frameA", ReferenceFrame::getWorldFrame(), X_st, 0, 0));

    MotionVector v(1.1, 2.1, 3.1, 4.1, 5.1, 6.1);

    SpatialMotion m(frameA, ReferenceFrame::getWorldFrame(), frameA, v);

    v.transform(X_st.inverse());

    m.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(m.isApprox(v, unit_test_utils::E_MINUS_14));

    try
    {
        m.changeFrame(nullptr);
        FAIL();
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Either this reference frame or desired reference frame is nullptr!");
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
