// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "Fixtures.hpp"
#include "rdl_dynamics/Kinematics.hpp"
#include "rdl_dynamics/rdl_utils.hpp"
#include "rdl_dynamics/SpatialMomentum.hpp"
#include "UnitTestUtils.hpp"

using namespace RobotDynamics::Math;

class SpatialMomentumTests : public testing::Test
{
  public:
    SpatialMomentumTests()
    {
    }

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(SpatialMomentumTests, testKineticEnergyFrameAssertion)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialTransform X_st;
    X_st.r = trans;

    SpatialMatrix X_66_matrix(SpatialMatrix::Zero(6, 6));
    X_66_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * Xtrans(trans).toMatrix();
    X_st.E = X_66_matrix.block<3, 3>(0, 0);

    ReferenceFramePtr frameA(new ReferenceFrame("frameA", ReferenceFrame::getWorldFrame(), X_st, 0, 0));

    SpatialMotion v(frameA, ReferenceFrame::getWorldFrame(), frameA, 1., 2., 3., 4., 5., 6.);
    SpatialMomentum p(ReferenceFrame::getWorldFrame(), -1., -2., -3., -4., -5., 6.);

    try
    {
        // compute kinetic energy
        p* v;
    }
    catch (ReferenceFrameException& e)
    {
        EXPECT_STREQ("Reference frames do not match!", e.what());
    }

    ForceVector f(0.1, -0.1, 0.2, -0.2, 0.3, -0.3);
    SpatialMomentum sm(frameA, f);

    SpatialMomentum sm2;
    sm2.setIncludingFrame(frameA, f);

    EXPECT_TRUE(sm.isApprox(sm2, unit_test_utils::E_MINUS_14));
}

TEST_F(FixedBase3DoFPlanar, testSetWithMomentum)
{
    SpatialMotion v(model->bodyFrames[1], ReferenceFrame::getWorldFrame(), model->bodyFrames[1], 1., 2., 3., 4., 5., 6.);
    SpatialInertia I(model->bodyFrames[1], 1., Vector3d(0.1, 1.1, 2.1), 0.1, 0.2, 0.3, 0.4, 0.5, 0.6);

    SpatialMomentum m(I, v);
    Momentum f = I * v;

    unit_test_utils::checkSpatialVectorsEpsilonClose(m, f, unit_test_utils::E_MINUS_14);

    SpatialMomentum m2;

    SpatialMomentum f_s = I * v;

    m2.setIncludingFrame(I.getReferenceFrame(), I * v);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(m, m2, unit_test_utils::E_MINUS_14));
}

TEST_F(FixedBase3DoFPlanar, testKineticEnergy)
{
    Q[0] = 0.;
    Q[1] = 0.;
    Q[2] = 0.;
    QDot[0] = 2.;
    QDot[1] = -1.;
    QDot[2] = 3.;
    QDDot[0] = 0.;
    QDDot[1] = 0.;
    QDDot[2] = 0.;

    updateKinematics(*model, Q, QDot, QDDot);

    for (size_t i = 1; i < model->mBodies.size(); i++)
    {
        ReferenceFramePtr bodyFrame = model->bodyFrames[i];
        ReferenceFramePtr parentBodyFrame = model->bodyFrames[model->lambda[i]];
        SpatialMotion m(bodyFrame, parentBodyFrame, bodyFrame, model->v[i]);

        SpatialMatrix inertia = model->Ic[i].toMatrix();
        SpatialInertia I(bodyFrame, inertia(3, 3), Vector3d(inertia(2, 4), inertia(0, 5), inertia(1, 3)), inertia(0, 0), inertia(0, 1), inertia(1, 1), inertia(0, 2),
                         inertia(1, 2), inertia(2, 2));
        EXPECT_TRUE(inertia.isApprox(I.toMatrix(), unit_test_utils::E_MINUS_14));
        SpatialMomentum momentum(I, m);
        EXPECT_EQ(momentum * m, 0.5 * model->v[i].transpose() * (model->Ic[i] * model->v[i]));
    }
}

TEST_F(SpatialMomentumTests, testChangeFrame)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialTransform X_st;
    X_st.r = trans;

    SpatialMatrix X_66_matrix(SpatialMatrix::Zero(6, 6));
    X_66_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_st.E = X_66_matrix.block<3, 3>(0, 0);

    std::shared_ptr<ReferenceFrame> frameA(new ReferenceFrame("frameA", ReferenceFrame::getWorldFrame(), X_st, 0, 0));

    Momentum v(1.1, 2.1, 3.1, 4.1, 5.1, 6.1);

    SpatialMomentum m(frameA, v);

    v.transform(X_st.inverse());

    m.changeFrame(ReferenceFrame::getWorldFrame());

    EXPECT_TRUE(m.isApprox(v, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
