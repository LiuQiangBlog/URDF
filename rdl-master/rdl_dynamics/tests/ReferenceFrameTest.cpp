// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "UnitTestUtils.hpp"

using namespace RobotDynamics;

class ReferenceFrameTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
        std::srand(time(NULL));

        root1 = ReferenceFrame::createARootFrame("root1");
        root2 = ReferenceFrame::createARootFrame("root2");

        frame1 = unit_test_utils::createRandomUnchangingFrame("frame1", root1, 1);
        frame2 = unit_test_utils::createRandomUnchangingFrame("frame2", frame1, 2);
        frame3 = unit_test_utils::createRandomUnchangingFrame("frame3", frame2, 3);

        frame4 = unit_test_utils::createRandomUnchangingFrame("frame4", root2, 1);
        frame5 = unit_test_utils::createRandomUnchangingFrame("frame5", frame4, 2);
        frame6 = unit_test_utils::createRandomUnchangingFrame("frame6", root2, 1);
        frame7 = unit_test_utils::createRandomUnchangingFrame("frame7", frame6, 2);
        frame8 = unit_test_utils::createRandomUnchangingFrame("frame8", frame7, 3);

        allFrames.push_back(root1);
        allFrames.push_back(frame1);
        allFrames.push_back(frame2);
        allFrames.push_back(frame3);

        allFrames.push_back(root2);
        allFrames.push_back(frame4);
        allFrames.push_back(frame5);
        allFrames.push_back(frame6);
        allFrames.push_back(frame7);
        allFrames.push_back(frame8);

        frames1.push_back(root1);
        frames1.push_back(frame1);
        frames1.push_back(frame2);
        frames1.push_back(frame3);

        frames2.push_back(root2);
        frames2.push_back(frame4);
        frames2.push_back(frame5);
        frames2.push_back(frame6);
        frames2.push_back(frame7);
        frames2.push_back(frame8);
    }
    virtual void TearDown()
    {
        allFrames.clear();
        frames1.clear();
        frames2.clear();
    }

    ReferenceFramePtr root1;
    ReferenceFramePtr root2;

    ReferenceFramePtr frame1;
    ReferenceFramePtr frame2;
    ReferenceFramePtr frame3;

    ReferenceFramePtr frame4;
    ReferenceFramePtr frame5;
    ReferenceFramePtr frame6;
    ReferenceFramePtr frame7;
    ReferenceFramePtr frame8;

    std::vector<ReferenceFramePtr> allFrames;
    std::vector<ReferenceFramePtr> frames1;
    std::vector<ReferenceFramePtr> frames2;

    unsigned int nTests = 1;

  private:
};

TEST_F(ReferenceFrameTest, constructors)
{
    ReferenceFrame testFrame(*frame1);

    EXPECT_TRUE(
        unit_test_utils::checkSpatialMatrixEpsilonClose(testFrame.getTransformToRoot().toMatrix(), frame1->getTransformToRoot().toMatrix(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(testFrame.getInverseTransformToRoot().toMatrix(), frame1->getInverseTransformToRoot().toMatrix(),
                                                                unit_test_utils::E_MINUS_10));
    EXPECT_STREQ(testFrame.getName().c_str(), frame1->getName().c_str());
    EXPECT_EQ(testFrame.getIsBodyFrame(), frame1->getIsBodyFrame());
    EXPECT_EQ(testFrame.getIsWorldFrame(), frame1->getIsWorldFrame());
}

TEST_F(ReferenceFrameTest, testRootsHaveNullParent)
{
    EXPECT_TRUE(root1->getParentFrame() == nullptr);
    EXPECT_TRUE(root2->getParentFrame() == nullptr);

    try
    {
        ReferenceFramePtr frame(new ReferenceFrame("blah", nullptr, SpatialTransform(), false, 1));
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "You are not allowed to create a frame with parentFrame=nullptr. Only a root frame and the world frame may have parentFrame=nullptr");
    }
}

TEST_F(ReferenceFrameTest, testWorldFramePointerStuff)
{
    const ReferenceFramePtr worldFrame1 = ReferenceFrame::getWorldFrame();
    const ReferenceFramePtr worldFrame2 = ReferenceFrame::getWorldFrame();

    EXPECT_EQ(worldFrame1, worldFrame2);
}

TEST_F(ReferenceFrameTest, testRootFramesArentTheSame)
{
    EXPECT_FALSE(root1 == root2);
}

TEST_F(ReferenceFrameTest, testGetRootFrame)
{
    EXPECT_TRUE(frame2->getRootFrame() == root1.get());
    EXPECT_TRUE(frame7->getRootFrame() == frame5->getRootFrame());

    frame7->verifyFramesHaveSameRoot(frame6);

    try
    {
        frame7->verifyFramesHaveSameRoot(frame1);
    }
    catch (RobotDynamics::ReferenceFrameException& e)
    {
        EXPECT_STREQ("Frames frame1 and frame7 have mismatched roots!", e.what());
    }
}

TEST_F(ReferenceFrameTest, testCheckReferenceFramesMatch)
{
    try
    {
        frame2->checkReferenceFramesMatch(nullptr);
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frame is nullptr!");
    }

    try
    {
        frame2->checkReferenceFramesMatch(frame7);
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ(e.what(), "Reference frames do not match!");
    }
}

TEST_F(ReferenceFrameTest, testGetTransformBetweenFrames)
{
    for (unsigned int i = 0; i < nTests; ++i)
    {
        unit_test_utils::updateAllFrames(allFrames);

        ReferenceFramePtr tmpFrame1 = unit_test_utils::getARandomFrame(frames1);
        ReferenceFramePtr tmpFrame2 = unit_test_utils::getARandomFrame(frames1);

        RobotDynamics::Math::SpatialTransform transform1 = tmpFrame1->getTransformToDesiredFrame(tmpFrame2);
        RobotDynamics::Math::SpatialTransform transform2 = tmpFrame2->getTransformToDesiredFrame(tmpFrame1);

        RobotDynamics::Math::SpatialTransform shouldBeIdentity = transform1 * transform2;
        RobotDynamics::Math::SpatialTransform identityTransform;

        EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(shouldBeIdentity.toMatrix(), identityTransform.toMatrix(), unit_test_utils::E_MINUS_10));
    }

    for (unsigned int i = 0; i < nTests; ++i)
    {
        unit_test_utils::updateAllFrames(allFrames);

        ReferenceFramePtr tmpFrame1 = unit_test_utils::getARandomFrame(frames2);
        ReferenceFramePtr tmpFrame2 = unit_test_utils::getARandomFrame(frames2);

        RobotDynamics::Math::SpatialTransform transform1 = tmpFrame1->getTransformToDesiredFrame(tmpFrame2);
        RobotDynamics::Math::SpatialTransform transform2 = tmpFrame2->getTransformToDesiredFrame(tmpFrame1);

        RobotDynamics::Math::SpatialTransform shouldBeIdentity = transform1 * transform2;
        RobotDynamics::Math::SpatialTransform identityTransform;

        EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(shouldBeIdentity.toMatrix(), identityTransform.toMatrix(), unit_test_utils::E_MINUS_14));
    }
}

TEST_F(ReferenceFrameTest, testGetTransformToParent)
{
    for (unsigned int i = 1; i < allFrames.size(); ++i)
    {
        ReferenceFramePtr tmpFrame2 = allFrames[i];

        ReferenceFramePtr parentFrame = tmpFrame2->getParentFrame();

        if (parentFrame != nullptr)
        {
            Eigen::Matrix4d m1, m2;
            RobotDynamics::Math::SpatialTransform t1 = tmpFrame2->getTransformToParent();
            RobotDynamics::Math::SpatialTransform t2 = tmpFrame2->getTransformToDesiredFrame(parentFrame);
            EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(t1.toMatrix(), t2.toMatrix(), unit_test_utils::E_MINUS_14));
        }
    }
}

TEST_F(ReferenceFrameTest, testGetTransformToRoot)
{
    for (unsigned int j = 0; j < nTests; ++j)
    {
        unit_test_utils::updateAllFrames(allFrames);

        for (unsigned int i = 0; i < allFrames.size(); ++i)
        {
            ReferenceFramePtr frame = allFrames[i];
            RobotDynamics::Math::SpatialTransform transformToRoot = unit_test_utils::getTransformToRootByClimbingTree(frame);

            EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(transformToRoot.toMatrix(), frame->getTransformToRoot().toMatrix(), unit_test_utils::E_MINUS_14));
        }
    }
}

TEST_F(ReferenceFrameTest, testGetTransformToSelf)
{
    for (unsigned int i = 0; i < nTests; ++i)
    {
        unit_test_utils::updateAllFrames(allFrames);

        for (unsigned int j = 0; j < allFrames.size(); ++j)
        {
            ReferenceFramePtr tmpFrame = allFrames[j];
            RobotDynamics::Math::SpatialTransform shouldBeIdentity = tmpFrame->getTransformToDesiredFrame(tmpFrame);

            RobotDynamics::Math::SpatialMatrix identityTransform = RobotDynamics::Math::SpatialMatrixIdentity;
            EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(shouldBeIdentity.toMatrix(), identityTransform, unit_test_utils::E_MINUS_14));
        }
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    return RUN_ALL_TESTS();
}
