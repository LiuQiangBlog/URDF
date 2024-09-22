// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "rdl_dynamics/Momentum.hpp"
#include "UnitTestUtils.hpp"

using namespace RobotDynamics::Math;

TEST(MomentumTests, testComparisonSpatialVectorCross)
{
    SpatialVector v(-1., 0.11, 0.9, 2., 0.56, -0.33);
    MotionVector m_v(v);
    RigidBodyInertia I(1.2, Vector3d(0.2, 0.3, 1.), 1., 2., 3., 4., 5., 6.);
    RigidBodyInertia I_r(1.2, Vector3d(0.2, 0.3, 1.), 1., 2., 3., 4., 5., 6.);
    Momentum m(I_r, m_v);
    MotionVector expected = m_v % m;
    SpatialVector v_sp_exp = v.crossf(m);

    SpatialVector v_out;
    v_out.setAngularPart(v.getAngularPart().toTildeForm() * m.getAngularPart() + v.getLinearPart().toTildeForm() * m.getLinearPart());  // crossf(v,I * v);
    v_out.setLinearPart(v.getAngularPart().toTildeForm() * m.getLinearPart());

    EXPECT_TRUE(v_out.isApprox(expected, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v_sp_exp.isApprox(expected, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v.crossf(m).isApprox(expected, unit_test_utils::E_MINUS_14));

    MotionVector m_2(0.1, -0.2, 0.3, -0.4, -05, -0.6);

    expected = m % m_2;

    v_sp_exp = m.crossm(m_2);

    v_out.setAngularPart(m.getAngularPart().toTildeForm() * m_2.getAngularPart());
    v_out.setLinearPart(m.getLinearPart().toTildeForm() * m_2.getAngularPart() + m.getAngularPart().toTildeForm() * m_2.getLinearPart());

    EXPECT_TRUE(v_out.isApprox(expected, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(v_sp_exp.isApprox(expected, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(m.crossm(m_2).isApprox(expected, unit_test_utils::E_MINUS_14));
}

TEST(MomentumTests, testConstructorsAndCompute)
{
    MotionVector v(1., 2., 3., 4., 5., 6.);
    RigidBodyInertia I(1., Vector3d(0.1, 1.1, 2.1), 0.1, 0.2, 0.3, 0.4, 0.5, 0.6);

    Momentum m = I * v;

    SpatialVector m_v = I.toMatrix() * v;

    EXPECT_TRUE(m.isApprox(m_v, unit_test_utils::E_MINUS_14));

    SpatialVector sv(-1.2, 2.2, 3.3, 4.1, 6.2, -0.8);

    Momentum mv(sv);

    EXPECT_TRUE(mv.isApprox(sv, unit_test_utils::E_MINUS_14));

    MotionVector mv2(sv);

    double kineticEnergy = m * mv2;

    EXPECT_EQ(kineticEnergy, m.dot(mv2) / 2.);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
