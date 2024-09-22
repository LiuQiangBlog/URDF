// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "rdl_dynamics/rdl_mathutils.hpp"
#include "rdl_dynamics/Body.hpp"
#include "UnitTestUtils.hpp"

using namespace std;
using namespace RobotDynamics;
using namespace RobotDynamics::Math;

class RdlBodyTests : public testing::Test
{
  public:
    RdlBodyTests(){};

    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

/* Tests whether the spatial inertia matches the one specified by its
 * parameters
 */
TEST_F(RdlBodyTests, TestComputeSpatialInertiaFromAbsoluteRadiiGyration)
{
    Body body(1.1, Vector3d(1.5, 1.2, 1.3), Vector3d(1.4, 2., 3.));

    Matrix3d inertia_C(1.4, 0., 0., 0., 2., 0., 0., 0., 3.);

    SpatialMatrix reference_inertia(4.843, -1.98, -2.145, 0, -1.43, 1.32, -1.98, 6.334, -1.716, 1.43, 0, -1.65, -2.145, -1.716, 7.059, -1.32, 1.65, 0, 0, 1.43, -1.32,
                                    1.1, 0, 0, -1.43, 0, 1.65, 0, 1.1, 0, 1.32, -1.65, 0, 0, 0, 1.1);

    RigidBodyInertia rbi = createFromMassComInertiaC(body.mMass, body.mCenterOfMass, body.mInertia);

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_inertia, rbi.toMatrix(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(inertia_C, body.mInertia, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestJoinTwoBodiesWithZeroMassThrows)
{
    Body body1(0., Vector3d(1.5, 1.2, 1.3), Vector3d(1.4, 2., 3.));
    Body body2(0., Vector3d(1.5, 1.2, 1.3), Vector3d(1.4, 2., 3.));

    try
    {
        body1.join(RobotDynamics::Math::SpatialTransform(), body2);
        FAIL();
    }
    catch (RobotDynamics::RdlException& e)
    {
        EXPECT_STREQ(e.what(), "Error: cannot join bodies as both have zero mass!");
    }
}

TEST_F(RdlBodyTests, TestBodyConstructorMassComInertia)
{
    double mass = 1.1;
    Vector3d com(1.5, 1.2, 1.3);
    Matrix3d inertia_C(8.286, -3.96, -4.29, -3.96, 10.668, -3.432, -4.29, -3.432, 11.118);

    Body body(mass, com, inertia_C);

    SpatialMatrix reference_inertia(11.729, -5.94, -6.435, 0, -1.43, 1.32, -5.94, 15.002, -5.148, 1.43, 0, -1.65, -6.435, -5.148, 15.177, -1.32, 1.65, 0, 0, 1.43, -1.32,
                                    1.1, 0, 0, -1.43, 0, 1.65, 0, 1.1, 0, 1.32, -1.65, 0, 0, 0, 1.1);

    RigidBodyInertia body_rbi = createFromMassComInertiaC(body.mMass, body.mCenterOfMass, body.mInertia);
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_inertia, body_rbi.toMatrix(), unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkMatrix3dEpsilonClose(inertia_C, body.mInertia, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyJoinNullbody)
{
    Body body(1.1, Vector3d(1.5, 1.2, 1.3), Vector3d(1.4, 2., 3.));
    Body nullbody(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));

    Body joined_body = body;
    joined_body.join(Xtrans(Vector3d(0., 0., 0.)), nullbody);

    RigidBodyInertia body_rbi(body.mMass, body.mCenterOfMass, body.mInertia);
    RigidBodyInertia joined_body_rbi(joined_body.mMass, joined_body.mCenterOfMass, joined_body.mInertia);

    ASSERT_EQ(1.1, body.mMass);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(body.mCenterOfMass, joined_body.mCenterOfMass, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(body_rbi.toMatrix(), joined_body_rbi.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyJoinTwoBodies)
{
    Body body_a(1.1, Vector3d(-1.1, 1.3, 0.), Vector3d(3.1, 3.2, 3.3));
    Body body_b(1.1, Vector3d(1.1, 1.3, 0.), Vector3d(3.1, 3.2, 3.3));

    Body body_joined(body_a);
    body_joined.join(Xtrans(Vector3d(0., 0., 0.)), body_b);

    RigidBodyInertia body_joined_rbi = createFromMassComInertiaC(body_joined.mMass, body_joined.mCenterOfMass, body_joined.mInertia);

    SpatialMatrix reference_inertia(9.918, 0, 0, 0, -0, 2.86, 0, 9.062, 0, 0, 0, -0, 0, 0, 12.98, -2.86, 0, 0, 0, 0, -2.86, 2.2, 0, 0, -0, 0, 0, 0, 2.2, 0, 2.86, -0, 0,
                                    0, 0, 2.2);

    ASSERT_EQ(2.2, body_joined.mMass);
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 1.3, 0.), body_joined.mCenterOfMass));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_inertia, body_joined_rbi.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyJoinTwoBodiesDisplaced)
{
    Body body_a(1.1, Vector3d(-1.1, 1.3, 0.), Vector3d(3.1, 3.2, 3.3));
    Body body_b(1.1, Vector3d(0., 0., 0.), Vector3d(3.1, 3.2, 3.3));

    Body body_joined(body_a);
    body_joined.join(Xtrans(Vector3d(1.1, 1.3, 0.)), body_b);

    RigidBodyInertia body_joined_rbi = createFromMassComInertiaC(body_joined.mMass, body_joined.mCenterOfMass, body_joined.mInertia);

    SpatialMatrix reference_inertia(9.918, 0, 0, 0, -0, 2.86, 0, 9.062, 0, 0, 0, -0, 0, 0, 12.98, -2.86, 0, 0, 0, 0, -2.86, 2.2, 0, 0, -0, 0, 0, 0, 2.2, 0, 2.86, -0, 0,
                                    0, 0, 2.2);

    ASSERT_EQ(2.2, body_joined.mMass);
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 1.3, 0.), body_joined.mCenterOfMass));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_inertia, body_joined_rbi.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyJoinTwoBodiesRotated)
{
    Body body_a(1.1, Vector3d(0., 0., 0.), Vector3d(3.1, 3.2, 3.3));
    Body body_b(1.1, Vector3d(0., 0., 0.), Vector3d(3.1, 3.3, 3.2));

    Body body_joined(body_a);
    body_joined.join(Xrotx(-M_PI * 0.5), body_b);

    RigidBodyInertia body_joined_rbi(body_joined.mMass, body_joined.mCenterOfMass, body_joined.mInertia);

    SpatialMatrix reference_inertia(6.2, 0., 0., 0., 0., 0., 0., 6.4, 0., 0., 0., 0., 0., 0., 6.6, 0., 0., 0., 0., 0., 0., 2.2, 0., 0., 0., 0., 0., 0., 2.2, 0., 0., 0.,
                                    0., 0., 0., 2.2);

    ASSERT_EQ(2.2, body_joined.mMass);
    EXPECT_TRUE(unit_test_utils::checkVector3dEq(Vector3d(0., 0., 0.), body_joined.mCenterOfMass));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_inertia, body_joined_rbi.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyJoinTwoBodiesRotatedAndTranslated)
{
    Body body_a(1.1, Vector3d(0., 0., 0.), Vector3d(3.1, 3.2, 3.3));
    Body body_b(1.1, Vector3d(-1., 1., 0.), Vector3d(3.2, 3.1, 3.3));

    Body body_joined(body_a);
    body_joined.join(Xrotz(M_PI * 0.5) * Xtrans(Vector3d(1., 1., 0.)), body_b);

    RigidBodyInertia body_joined_rbi(body_joined.mMass, body_joined.mCenterOfMass, body_joined.mInertia);

    SpatialMatrix reference_inertia(6.2, 0., 0., 0., 0., 0., 0., 6.4, 0., 0., 0., 0., 0., 0., 6.6, 0., 0., 0., 0., 0., 0., 2.2, 0., 0., 0., 0., 0., 0., 2.2, 0., 0., 0.,
                                    0., 0., 0., 2.2);

    ASSERT_EQ(2.2, body_joined.mMass);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), body_joined.mCenterOfMass, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_inertia, body_joined_rbi.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyConstructors)
{
    double mass = 0.1;
    Math::Vector3d v_com(0.1, 0.2, 0.3);
    Math::Matrix3d m_i = Math::Matrix3dIdentity;
    Math::Vector3d centerOfBuoyancy(-0.1, -0.2, -0.3);
    double volume(2.);
    SpatialMatrix I(1., 3., 1.1, 2.1, 3.3, 1.0, 1., 3., 1.1, 2.1, 3.3, 1.0, 1.11, 3., 3.112, 2.1, 3.3, 1.0, 1., 3., 1.1, 2.1, 3.3, 1.0, 1.908, 3., 1.221, 31.1, 3.3, 1.0,
                    1., 3., 1.1, 2.1, 331.1, 1.0);

    SpatialVector drag_ones(1., 2., 3., 4., 5., 6.);
    DragData dragData(drag_ones, 2. * drag_ones);

    Body b(mass, v_com, m_i, centerOfBuoyancy, volume, I, dragData);

    EXPECT_EQ(b.mMass, mass);
    EXPECT_EQ(b.volume, volume);
    EXPECT_TRUE(v_com.isApprox(b.mCenterOfMass));
    EXPECT_TRUE(centerOfBuoyancy.isApprox(b.mCenterOfBuoyancy));
    EXPECT_TRUE(b.mInertia.isApprox(m_i));
    EXPECT_TRUE(b.addedMassMatrix.isApprox(I));

    EXPECT_EQ(b.dragData.linearDrag[0], 1.);
    EXPECT_EQ(b.dragData.linearDrag[1], 2.);
    EXPECT_EQ(b.dragData.linearDrag[2], 3.);
    EXPECT_EQ(b.dragData.linearDrag[3], 4.);
    EXPECT_EQ(b.dragData.linearDrag[4], 5.);
    EXPECT_EQ(b.dragData.linearDrag[5], 6.);

    EXPECT_EQ(b.dragData.quadraticDrag[0], 2.);
    EXPECT_EQ(b.dragData.quadraticDrag[1], 4.);
    EXPECT_EQ(b.dragData.quadraticDrag[2], 6.);
    EXPECT_EQ(b.dragData.quadraticDrag[3], 8.);
    EXPECT_EQ(b.dragData.quadraticDrag[4], 10.);
    EXPECT_EQ(b.dragData.quadraticDrag[5], 12.);
}

TEST_F(RdlBodyTests, TestBodyJoinTwoBodiesRotatedAndTranslatedWithHydrodynamics)
{
    SpatialMatrix I_a = SpatialMatrixIdentity * 2.2;
    SpatialMatrix I_b = SpatialMatrixIdentity * 1.1;
    std::vector<double> drag_a, drag_b;
    drag_a.push_back(1.);
    drag_a.push_back(2.);
    drag_a.push_back(3.);
    drag_a.push_back(4.);
    drag_a.push_back(5.);
    drag_a.push_back(6.);
    drag_b.push_back(1.);
    drag_b.push_back(2.);
    drag_b.push_back(3.);
    drag_b.push_back(4.);
    drag_b.push_back(5.);
    drag_b.push_back(6.);

    DragData data_a(drag_a, drag_b);

    drag_a[0] = 1.;
    drag_a[1] = 2.;
    drag_a[2] = 3.;
    drag_a[3] = 4.;
    drag_a[4] = 5.;
    drag_a[5] = 6.;
    drag_b[0] = 1.;
    drag_b[1] = 2.;
    drag_b[2] = 3.;
    drag_b[3] = 4.;
    drag_b[4] = 5.;
    drag_b[5] = 6.;

    DragData data_b(drag_a, drag_b);

    Body body_a(1.1, Vector3d(0., 0., 0.), Vector3d(3.1, 3.2, 3.3), Vector3d(0., 0., 0.), 0.1, I_a, data_a);
    Body body_b(1.1, Vector3d(-1., 1., 0.), Vector3d(3.2, 3.1, 3.3), Vector3d(-1., 1., 0.), 0.1, I_b, data_b);

    Body body_joined(body_a);
    body_joined.join(Xrotz(M_PI_2) * Xtrans(Vector3d(1., 1., 0.)), body_b);

    RigidBodyInertia body_joined_rbi(body_joined.mMass, body_joined.mCenterOfMass, body_joined.mInertia);

    SpatialMatrix reference_inertia = I_a + I_b;

    ASSERT_EQ(2.2, body_joined.mMass);
    ASSERT_NEAR(0.2, body_joined.volume, unit_test_utils::E_MINUS_14);
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), body_joined.mCenterOfMass, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(Vector3d(0., 0., 0.), body_joined.mCenterOfBuoyancy, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(reference_inertia, body_joined.addedMassMatrix, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyConstructorRigidBodyInertiaMultiplyMotion)
{
    Body body(1.1, Vector3d(1.5, 1.2, 1.3), Vector3d(1.4, 2., 3.));

    RigidBodyInertia rbi = RigidBodyInertia(body.mMass, body.mCenterOfMass * body.mMass, body.mInertia);

    SpatialVector mv(1.1, 1.2, 1.3, 1.4, 1.5, 1.6);
    SpatialVector fv_matrix = rbi.toMatrix() * mv;
    SpatialVector fv_rbi = rbi * mv;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(fv_matrix, fv_rbi, unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyConstructorRigidBodyInertia)
{
    Body body(1.1, Vector3d(1.5, 1.2, 1.3), Vector3d(1.4, 2., 3.));

    RigidBodyInertia rbi = RigidBodyInertia(body.mMass, body.mCenterOfMass * body.mMass, body.mInertia);
    SpatialMatrix spatial_inertia = rbi.toMatrix();

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(spatial_inertia, rbi.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST_F(RdlBodyTests, TestBodyConstructorCopyRigidBodyInertia)
{
    Body body(1.1, Vector3d(1.5, 1.2, 1.3), Vector3d(1.4, 2., 3.));

    RigidBodyInertia rbi = RigidBodyInertia(body.mMass, body.mCenterOfMass * body.mMass, body.mInertia);

    RigidBodyInertia rbi_from_matrix;
    rbi_from_matrix.createFromMatrix(rbi.toMatrix());

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(rbi.toMatrix(), rbi_from_matrix.toMatrix(), unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
