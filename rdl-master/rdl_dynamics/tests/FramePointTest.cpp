// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "rdl_dynamics/FramePoint.hpp"
#include "UnitTestUtils.hpp"

using namespace RobotDynamics;

class FramePointTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
        std::srand(time(NULL));
    }

    virtual void TearDown()
    {
    }

    ReferenceFramePtr root1 = ReferenceFrame::createARootFrame("root1");

    int nTests = 1000;

  private:
};

TEST_F(FramePointTest, testChangeFrame)
{
    RobotDynamics::Math::SpatialTransform transform1;

    transform1.E = RobotDynamics::Math::Xrotx(M_PI_2).E;
    transform1.r.set(6., 5., 3.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1, false, 1));

    transform1.E = RobotDynamics::Math::Xroty(M_PI_2).E;
    transform1.r.set(3., -6., 4.);

    ReferenceFramePtr frameB(new ReferenceFrame("B", frameA, transform1, false, 2));

    transform1.E = RobotDynamics::Math::Xrotz(M_PI_2).E;
    transform1.r = RobotDynamics::Math::Vector3d(0., -5., 0.);
    ReferenceFramePtr frameC(new ReferenceFrame("C", frameB, transform1, false, 3));

    frameA->update();
    frameB->update();
    frameC->update();

    double x = 3.;
    double y = -1.;
    double z = 2.;

    FramePoint framePoint(root1, x, y, z);

    EXPECT_NEAR(framePoint.x(), x, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), y, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), z, unit_test_utils::E_MINUS_14);

    framePoint.changeFrame(root1);

    EXPECT_NEAR(framePoint.x(), x, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), y, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), z, unit_test_utils::E_MINUS_14);

    framePoint.changeFrame(frameA);

    EXPECT_NEAR(framePoint.x(), -3., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), -1., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), 6., unit_test_utils::E_MINUS_14);

    framePoint.changeFrame(frameB);

    EXPECT_NEAR(framePoint.x(), -2., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), 5., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), -6., unit_test_utils::E_MINUS_14);

    framePoint.changeFrame(frameC);

    EXPECT_NEAR(framePoint.x(), 10., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), 2., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), -6., unit_test_utils::E_MINUS_14);

    framePoint.changeFrame(frameB);

    EXPECT_NEAR(framePoint.x(), -2., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), 5., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), -6., unit_test_utils::E_MINUS_14);

    framePoint.changeFrame(frameA);

    EXPECT_NEAR(framePoint.x(), -3., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), -1., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), 6., unit_test_utils::E_MINUS_14);

    framePoint.changeFrame(root1);

    EXPECT_NEAR(framePoint.x(), x, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), y, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), z, unit_test_utils::E_MINUS_14);
}

TEST_F(FramePointTest, testChangeFrameAndCopy)
{
    RobotDynamics::Math::SpatialTransform transform1;

    transform1.E = RobotDynamics::Math::Xrotx(M_PI_2).E;
    transform1.r.set(6., 5., 3.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1, false, 1));

    transform1.E = RobotDynamics::Math::Xroty(M_PI_2).E;
    transform1.r.set(3., -6., 4.);

    ReferenceFramePtr frameB(new ReferenceFrame("B", frameA, transform1, false, 2));

    transform1.E = RobotDynamics::Math::Xrotz(M_PI_2).E;
    transform1.r = RobotDynamics::Math::Vector3d(0., -5., 0.);
    ReferenceFramePtr frameC(new ReferenceFrame("C", frameB, transform1, false, 3));

    frameA->update();
    frameB->update();
    frameC->update();

    double x = 3.;
    double y = -1.;
    double z = 2.;

    FramePoint framePoint(root1, x, y, z);

    EXPECT_NEAR(framePoint.x(), x, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.y(), y, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint.z(), z, unit_test_utils::E_MINUS_14);

    FramePoint p1 = framePoint.changeFrameAndCopy(root1);

    EXPECT_STREQ(framePoint.getReferenceFrame()->getName().c_str(), root1->getName().c_str());
    EXPECT_NEAR(p1.x(), x, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(p1.y(), y, unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(p1.z(), z, unit_test_utils::E_MINUS_14);

    FramePoint p2 = p1;
    p2 = p1.changeFrameAndCopy(frameA);

    EXPECT_NEAR(p2.x(), -3., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(p2.y(), -1., unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(p2.z(), 6., unit_test_utils::E_MINUS_14);
}

TEST_F(FramePointTest, testDistance)
{
    FramePoint framePoint1(root1, 1, 2, 3);
    FramePoint framePoint2(root1, -1, -2, -3);

    RobotDynamics::Math::SpatialTransform transform1;

    transform1.E = RobotDynamics::Math::Xrotx(M_PI_2).E;
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1, false, 1));

    FramePoint framePoint3(frameA, 1., 2., 3.);

    EXPECT_NEAR(framePoint1.distance(framePoint2), sqrt(4. + 16. + 36.), unit_test_utils::E_MINUS_14);
    EXPECT_ANY_THROW(framePoint3.distance(framePoint2));  // frames don't match
}

TEST_F(FramePointTest, testDistance2D)
{
    FramePoint framePoint1(root1, 1, 2, 3);
    FramePoint framePoint2(root1, -1, -2, -3);

    RobotDynamics::Math::SpatialTransform transform1;

    transform1.E = RobotDynamics::Math::Xrotx(M_PI_2).E;
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1, false, 1));

    FramePoint framePoint3(frameA, 1., 2., 3.);

    EXPECT_NEAR(framePoint1.distance2D(framePoint2, 2), sqrt(4. + 16.), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint1.distance2D(framePoint2, 1), sqrt(4. + 36.), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint1.distance2D(framePoint2, 0), sqrt(16. + 36.), unit_test_utils::E_MINUS_14);

    EXPECT_ANY_THROW(framePoint1.distance2D(framePoint2, 3));  // invalid plane 3
}

TEST_F(FramePointTest, testDistancePoint3d)
{
    FramePoint framePoint1(root1, 1, 2, 3);
    Point3d p2(-1, -2, -3);
    RobotDynamics::Math::SpatialTransform transform1;

    transform1.E = RobotDynamics::Math::Xrotx(M_PI_2).E;
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1, false, 1));

    FramePoint framePoint3(frameA, 1., 2., 3.);

    EXPECT_TRUE(framePoint1.distance(p2) == sqrt(36. + 16. + 4.));
}

TEST_F(FramePointTest, testDistance2DPoint3d)
{
    FramePoint framePoint1(root1, 1, 2, 3);
    Point3d p2(-1, -2, -3);
    RobotDynamics::Math::SpatialTransform transform1;

    transform1.E = RobotDynamics::Math::Xrotx(M_PI_2).E;
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1, false, 1));

    FramePoint framePoint3(frameA, 1., 2., 3.);

    EXPECT_NEAR(framePoint1.distance2D(p2, 2), sqrt(4. + 16.), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint1.distance2D(p2, 1), sqrt(4. + 36.), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(framePoint1.distance2D(p2, 0), sqrt(16. + 36.), unit_test_utils::E_MINUS_14);
    EXPECT_ANY_THROW(framePoint1.distance2D(p2, 3));
}

TEST_F(FramePointTest, testSetIncludingFrameMismatchingFrames)
{
    FramePoint framePoint2;

    try
    {
        framePoint2.setIncludingFrame(0.1, 0.2, 0.3, nullptr);
    }
    catch (RobotDynamics::ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frame is nullptr!");
    }
}

TEST_F(FramePointTest, testDistanceMixingTypes)
{
    FramePoint framePoint1(root1, 1, 2, 3);
    FramePoint framePoint2(root1, -1, -2, -3);

    RobotDynamics::Math::SpatialTransform transform1;

    transform1.E = RobotDynamics::Math::Xrotx(M_PI_2).E;
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1.inverse(), false, 1));

    FramePoint framePoint3(frameA, 1., 2., 3.);

    EXPECT_TRUE(framePoint1.distance(framePoint2) == sqrt(36. + 16. + 4.));

    try
    {
        framePoint3.distance(framePoint2);
    }
    catch (RobotDynamics::ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frames do not match!");
    }
}

TEST_F(FramePointTest, testDistanceL1)
{
    std::shared_ptr<FramePoint> framePoint1(new RobotDynamics::Math::FramePoint(root1, 1., 2., 3.));
    std::shared_ptr<FramePoint> framePoint2(new RobotDynamics::Math::FramePoint(root1, -1., -2., -3.));

    RobotDynamics::Math::SpatialTransform transform1;

    transform1 = RobotDynamics::Math::Xrotx(M_PI_2);
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1.inverse(), false, 1));

    std::shared_ptr<FramePoint> framePoint3(new RobotDynamics::Math::FramePoint(frameA, 1., 2., 3.));

    EXPECT_TRUE(framePoint1->distanceL1(*framePoint2) == 12.);

    try
    {
        framePoint3->distanceL1(*framePoint2);
    }
    catch (RobotDynamics::ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frames do not match!");
    }
}

TEST_F(FramePointTest, testDistanceL1MixingTypes)
{
    std::shared_ptr<FramePoint> framePoint1(new RobotDynamics::Math::FramePoint(root1, 1., 2., 3.));
    std::shared_ptr<FramePoint> framePoint2(new RobotDynamics::Math::FramePoint(root1, -1., -2., -3.));

    RobotDynamics::Math::SpatialTransform transform1;

    transform1 = RobotDynamics::Math::Xrotx(M_PI_2);
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1.inverse(), false, 1));

    std::shared_ptr<FramePoint> framePoint3(new RobotDynamics::Math::FramePoint(frameA, 1., 2., 3.));

    EXPECT_TRUE(framePoint1->distanceL1(*framePoint2) == 12.);

    try
    {
        framePoint3->distanceL1(*framePoint2);
    }
    catch (RobotDynamics::ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frames do not match!");
    }
}

TEST_F(FramePointTest, testDistanceLinf)
{
    std::shared_ptr<FramePoint> framePoint1(new RobotDynamics::Math::FramePoint(root1, 1., 2., 3.));
    std::shared_ptr<FramePoint> framePoint2(new RobotDynamics::Math::FramePoint(root1, -1., -2., -3.));

    RobotDynamics::Math::SpatialTransform transform1;

    transform1 = RobotDynamics::Math::Xrotx(M_PI_2);
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1.inverse(), false, 1));

    std::shared_ptr<FramePoint> framePoint3(new RobotDynamics::Math::FramePoint(frameA, 1., 2., 3.));

    EXPECT_TRUE(framePoint1->distanceLinf(*framePoint2) == 6.);

    try
    {
        framePoint3->distanceL1(*framePoint2);
    }
    catch (RobotDynamics::ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frames do not match!");
    }
}

TEST_F(FramePointTest, testDistanceLinfMixingTypes)
{
    std::shared_ptr<FramePoint> framePoint1(new RobotDynamics::Math::FramePoint(root1, 1., 2., 3.));
    std::shared_ptr<FramePoint> framePoint2(new RobotDynamics::Math::FramePoint(root1, -1., -2., -3.));

    RobotDynamics::Math::SpatialTransform transform1;

    transform1 = RobotDynamics::Math::Xrotx(M_PI_2);
    transform1.r.set(5., 0., 0.);

    ReferenceFramePtr frameA(new ReferenceFrame("A", root1, transform1.inverse(), false, 1));

    std::shared_ptr<FramePoint> framePoint3(new RobotDynamics::Math::FramePoint(frameA, 1., 2., 3.));

    EXPECT_TRUE(framePoint1->distanceLinf(*framePoint2) == 6.);

    try
    {
        framePoint3->distanceL1(*framePoint2);
    }
    catch (RobotDynamics::ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frames do not match!");
    }
}

TEST_F(FramePointTest, testOperatorOverloads)
{
    FramePoint framePoint(root1, 1., 2., 3.);

    framePoint *= 3.;

    EXPECT_EQ(framePoint.x(), 3.);
    EXPECT_EQ(framePoint.y(), 6.);
    EXPECT_EQ(framePoint.z(), 9.);

    FramePoint p2(root1, RobotDynamics::Math::Vector3d(-1., -2., -3.));

    FrameVector v = framePoint - p2;

    EXPECT_EQ(v.x(), 4.);
    EXPECT_EQ(v.y(), 8.);
    EXPECT_EQ(v.z(), 12.);

    FramePoint p_check = p2 + v;

    EXPECT_NEAR(p_check.x(), framePoint.x(), 1.e-12);
    EXPECT_NEAR(p_check.y(), framePoint.y(), 1.e-12);
    EXPECT_NEAR(p_check.z(), framePoint.z(), 1.e-12);

    p_check = framePoint - v;

    EXPECT_NEAR(p_check.x(), p2.x(), 1.e-12);
    EXPECT_NEAR(p_check.y(), p2.y(), 1.e-12);
    EXPECT_NEAR(p_check.z(), p2.z(), 1.e-12);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS();
}
