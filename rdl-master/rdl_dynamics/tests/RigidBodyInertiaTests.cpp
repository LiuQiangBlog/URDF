// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "UnitTestUtils.hpp"

using namespace RobotDynamics::Math;

class RigidBodyInertiaTests : public testing::Test
{
    RigidBodyInertiaTests()
    {
    }
};

TEST(RigidBodyInertiaTests, testSet)
{
    RigidBodyInertia I(6.1, Vector3d(12.2, 5.3, 2.7), Matrix3d(4.1, 1.5, 2.3, 5.3, 4.1, 1.5, 1.1, 4.5, 5.6));
    RigidBodyInertia I2(11.1, Vector3d(2.2, 2.3, 1.7), Matrix3d(1.1, 2.5, 3.3, 2.3, 1.1, 3.5, 2.1, 1.5, 1.6));

    I2.set(I);

    EXPECT_EQ(I2.m, I.m);

    EXPECT_EQ(I2.h[0], I.h[0]);
    EXPECT_EQ(I2.h[1], I.h[1]);
    EXPECT_EQ(I2.h[2], I.h[2]);

    EXPECT_EQ(I2.Ixx, I.Ixx);
    EXPECT_EQ(I2.Iyx, I.Iyx);
    EXPECT_EQ(I2.Izx, I.Izx);
    EXPECT_EQ(I2.Iyy, I.Iyy);
    EXPECT_EQ(I2.Izy, I.Izy);
    EXPECT_EQ(I2.Izz, I.Izz);
}

TEST(RigidBodyInertiaTests, testTimes3DofJointMotionSubspaceMatrix)
{
    RigidBodyInertia I(6.1, Vector3d(12.2, 5.3, 2.7), Matrix3d(4.1, 1.5, 2.3, 5.3, 4.1, 1.5, 1.1, 4.5, 5.6));
    Matrix63 m;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            m(i, j) = (double)i + j + i * j;
        }
    }

    Matrix63 m2 = I * m;

    EXPECT_TRUE(m2.isApprox(I.toMatrix() * m, unit_test_utils::E_MINUS_13));
}

TEST(RigidBodyInertiaTests, testSubtractSpatialMatrix)
{
    RigidBodyInertia I(6.1, Vector3d(12.2, 5.3, 2.7), Matrix3d(4.1, 1.5, 2.3, 5.3, 4.1, 1.5, 1.1, 4.5, 5.6));
    SpatialMatrix m(0.1, 0.2, 0.3, 0.3, 0.2, 0.1, 0.1, 0.2, 0.3, 0.3, 0.2, 0.1, 0.1, 0.2, 0.3, 0.3, 0.2, 0.1, 0.1, 0.2, 0.3, 0.3, 0.2, 0.1, 0.1, 0.2, 0.3, 0.3, 0.2, 0.1,
                    0.1, 0.2, 0.3, 0.3, 0.2, 0.1);

    SpatialMatrix m2 = I.subtractSpatialMatrix(m);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(m2, I.toMatrix() - m, unit_test_utils::E_MINUS_14));
}

TEST(SpatialAlgebraTests, testMultiplyOperator)
{
    RigidBodyInertia I(6.1, Vector3d(12.2, 5.3, 2.7), Matrix3d(4.1, 1.5, 2.3, 5.3, 4.1, 1.5, 1.1, 4.5, 5.6));
    SpatialVector v(-13.3355, 1.3318, -0.81, 0.114, 8.1, 3.1);

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(I * v, I.toMatrix() * v, unit_test_utils::E_MINUS_13));
}

TEST(SpatialAlgebraTests, testAdd)
{
    RigidBodyInertia I_a(1.1, Vector3d(1.2, 1.3, 1.4), Matrix3d(1.1, 0.5, 0.3, 0.5, 1.2, 0.4, 0.3, 0.4, 1.3));
    RigidBodyInertia I_b(6.1, Vector3d(12.2, 5.3, 2.7), Matrix3d(4.1, 1.5, 2.3, 5.3, 4.1, 1.5, 1.1, 4.5, 5.6));

    SpatialMatrix I_expected = I_a.toMatrix() + I_b.toMatrix();

    RigidBodyInertia I_c = I_a + I_b;

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(I_c.toMatrix(), I_expected, unit_test_utils::E_MINUS_14));
}

TEST(SpatialAlgebraTests, TestSpatialTransformApplyRigidBodyInertiaFull)
{
    RigidBodyInertia inertia(1.1, Vector3d(1.2, 1.3, 1.4), Matrix3d(1.1, 0.5, 0.3, 0.5, 1.2, 0.4, 0.3, 0.4, 1.3));
    RigidBodyInertia inertia_copy = inertia;

    SpatialTransform X(Xrotz(0.5) * Xroty(0.9) * Xrotx(0.2) * Xtrans(Vector3d(1.1, 1.2, 1.3)));

    SpatialMatrix rbi_matrix_transformed = X.toMatrixAdjoint() * inertia.toMatrix() * X.inverse().toMatrix();
    inertia.transform(X);
    inertia_copy.transform(X);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(rbi_matrix_transformed, inertia.toMatrix(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(inertia_copy.toMatrix(), inertia.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST(SpatialAlgebraTests, TestRigidBodyInertiaCreateFromMatrix)
{
    double mass = 1.1;
    Vector3d com(0., 0., 0.);
    Matrix3d inertia(1.1, 0.5, 0.3, 0.5, 1.2, 0.4, 0.3, 0.4, 1.3);
    RigidBodyInertia body_rbi(mass, com, inertia);

    SpatialMatrix spatial_inertia = body_rbi.toMatrix();

    RigidBodyInertia rbi;
    rbi.createFromMatrix(spatial_inertia);

    EXPECT_EQ(mass, rbi.m);
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(mass * com), rbi.h));
    Matrix3d rbi_I_matrix(rbi.Ixx, rbi.Iyx, rbi.Izx, rbi.Iyx, rbi.Iyy, rbi.Izy, rbi.Izx, rbi.Izy, rbi.Izz);
    EXPECT_TRUE(unit_test_utils::checkMatrix3dEq(inertia, rbi_I_matrix));
}

TEST(SpatialAlgebraTests, TestsCreateFromMassComAndInertiaAboutCom)
{
    Matrix3d Ic(1., 2., 3., 2., 3., 4., 3., 4., 5.);
    Vector3d com(-1., 3.5, -5.1);
    double mass = 13.2;

    Matrix3d comTilde = com.toTildeForm();
    Matrix3d mIdentity = Matrix3dIdentity * mass;

    RigidBodyInertia I_t(mass, com, Ic);

    SpatialMatrix m;
    m.block<3, 3>(0, 0) = Ic + comTilde * comTilde.transpose() * mass;
    m.block<3, 3>(0, 3) = com.toTildeForm() * mass;
    m.block<3, 3>(3, 0) = mass * comTilde.transpose();
    m.block<3, 3>(3, 3) = mIdentity;

    RigidBodyInertia Ia = createFromMassComInertiaC(mass, com, Ic);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(Ia.toMatrix(), m, unit_test_utils::E_MINUS_13));
    EXPECT_FALSE(unit_test_utils::checkSpatialMatrixEpsilonClose(I_t.toMatrix(), Ia.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST(SpatialAlgebraTests, TestSpatialTransformApplySpatialRigidBodyInertiaAdd)
{
    RigidBodyInertia rbi(1.1, Vector3d(1.2, 1.3, 1.4), Matrix3d(1.1, 0.5, 0.3, 0.5, 1.2, 0.4, 0.3, 0.4, 1.3));

    SpatialMatrix rbi_matrix_added = rbi.toMatrix() + rbi.toMatrix();
    RigidBodyInertia rbi_added = rbi + rbi;

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(rbi_matrix_added, rbi_added.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST(SpatialAlgebraTests, TestSpatialTransformApplySpatialRigidBodyInertiaFull)
{
    RigidBodyInertia rbi(1.1, Vector3d(1.2, 1.3, 1.4), Matrix3d(1.1, 0.5, 0.3, 0.5, 1.2, 0.4, 0.3, 0.4, 1.3));

    SpatialTransform X(Xrotz(0.5) * Xroty(0.9) * Xrotx(0.2) * Xtrans(Vector3d(1.1, 1.2, 1.3)));

    RigidBodyInertia rbi_transformed = rbi.transform_copy(X);
    SpatialMatrix rbi_matrix_transformed = X.toMatrixAdjoint() * rbi.toMatrix() * X.inverse().toMatrix();

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(rbi_matrix_transformed, rbi_transformed.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST(SpatialAlgebraTests, TestSpatialRigidBodyInertiaCreateFromMatrix)
{
    double mass = 1.1;
    Vector3d com(0., 0., 0.);
    Matrix3d inertia(1.1, 0.5, 0.3, 0.5, 1.2, 0.4, 0.3, 0.4, 1.3);
    RigidBodyInertia body_rbi(mass, com, inertia);

    SpatialMatrix spatial_inertia = body_rbi.toMatrix();

    RigidBodyInertia rbi;
    rbi.createFromMatrix(spatial_inertia);

    EXPECT_EQ(mass, rbi.m);
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(mass * com), rbi.h));
    Matrix3d rbi_I_matrix(rbi.Ixx, rbi.Iyx, rbi.Izx, rbi.Iyx, rbi.Iyy, rbi.Izy, rbi.Izx, rbi.Izy, rbi.Izz);
    EXPECT_TRUE(unit_test_utils::checkMatrix3dEq(inertia, rbi_I_matrix));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
