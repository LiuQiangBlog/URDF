// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "rdl_dynamics/FrameVectorPair.hpp"
#include "UnitTestUtils.hpp"

using namespace RobotDynamics;

class FrameVectorPairTest : public ::testing::Test
{
  public:
    FrameVectorPairTest()
    {
    }

    ~FrameVectorPairTest()
    {
    }

  protected:
    virtual void SetUp()
    {
        std::srand(time(NULL));

        root = ReferenceFrame::createARootFrame("root1");
        frame1 = unit_test_utils::createRandomUnchangingFrame("frame1", root, 1);
        frame2 = unit_test_utils::createRandomUnchangingFrame("frame2", frame1, 2);
    }

    virtual void TearDown()
    {
    }

    ReferenceFramePtr root;
    ReferenceFramePtr frame1;
    ReferenceFramePtr frame2;

    int nTests = 1000;

  private:
};

TEST_F(FrameVectorPairTest, simple1)
{
    FrameVector v1(root, 1., 2., 3.);
    FrameVector v2(root, 2., 3., 4.);

    FrameVectorPair pair(v1, v2);

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    pair.changeFrame(frame2);

    ASSERT_FALSE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    v1.changeFrame(frame2);
    v2.changeFrame(frame2);

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    FrameVectorPair pair2 = pair;

    ASSERT_TRUE(pair.linear().isApprox(pair2.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(pair2.angular(), unit_test_utils::E_MINUS_12));

    FrameVectorPair pair3 = pair2.changeFrameAndCopy(frame1);

    v1.changeFrame(frame1);
    v2.changeFrame(frame1);

    ASSERT_TRUE(pair3.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair3.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair2.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair2.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(FrameVectorPairTest, simple2)
{
    FrameVector v1(root, 3., 11., 8.);
    FrameVector v2(root, 1., 33., 11.);

    FrameVectorPair pair(v1.getReferenceFrame(), v1.vec(), v2.vec());

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    pair.changeFrame(frame2);

    ASSERT_FALSE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    v1.changeFrame(frame2);
    v2.changeFrame(frame2);

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    FrameVectorPair pair2 = pair;

    ASSERT_TRUE(pair.linear().isApprox(pair2.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(pair2.angular(), unit_test_utils::E_MINUS_12));

    FrameVectorPair pair3 = pair2.changeFrameAndCopy(frame1);

    v1.changeFrame(frame1);
    v2.changeFrame(frame1);

    ASSERT_TRUE(pair3.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair3.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair2.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair2.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(FrameVectorPairTest, simple3)
{
    FrameVector v1(root, 3., 11., 8.);
    FrameVector v2(root, 1., 33., 11.);

    FrameVectorPair pair;
    pair.setIncludingFrame(v1.getReferenceFrame(), v1.vec(), v2.vec());

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    pair.changeFrame(frame2);

    ASSERT_FALSE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    v1.changeFrame(frame2);
    v2.changeFrame(frame2);

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    FrameVectorPair pair2 = pair;

    ASSERT_TRUE(pair.linear().isApprox(pair2.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(pair2.angular(), unit_test_utils::E_MINUS_12));

    FrameVectorPair pair3 = pair2.changeFrameAndCopy(frame1);

    v1.changeFrame(frame1);
    v2.changeFrame(frame1);

    ASSERT_TRUE(pair3.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair3.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair2.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair2.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));
}

TEST_F(FrameVectorPairTest, simple4)
{
    FrameVector v1(root, 3., 11., 8.);
    FrameVector v2(root, 1., 33., 11.);

    FrameVectorPair pair;
    pair.setIncludingFrame(root, v1.vec(), v2.vec());

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    pair.changeFrame(frame2);

    ASSERT_FALSE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    v1.changeFrame(frame2);
    v2.changeFrame(frame2);

    ASSERT_TRUE(pair.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    FrameVectorPair pair2 = pair;

    ASSERT_TRUE(pair.linear().isApprox(pair2.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair.angular().isApprox(pair2.angular(), unit_test_utils::E_MINUS_12));

    FrameVectorPair pair3 = pair2.changeFrameAndCopy(frame1);

    v1.changeFrame(frame1);
    v2.changeFrame(frame1);

    ASSERT_TRUE(pair3.linear().isApprox(v1, unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(pair3.angular().isApprox(v2, unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));

    ASSERT_FALSE(pair2.linear().isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_FALSE(pair2.angular().isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));

    FrameVector* lin_ptr = pair3.linearPtr();
    FrameVector* ang_ptr = pair3.angularPtr();

    ASSERT_TRUE(lin_ptr->isApprox(pair3.linear(), unit_test_utils::E_MINUS_12));
    ASSERT_TRUE(ang_ptr->isApprox(pair3.angular(), unit_test_utils::E_MINUS_12));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
