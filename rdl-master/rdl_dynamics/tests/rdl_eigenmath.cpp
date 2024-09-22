// Copyright (c) 2023 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>
#include "UnitTestUtils.hpp"

SpatialMatrix spatial_adjoint(const SpatialMatrix& m)
{
    SpatialMatrix res(m);
    res.block<3, 3>(3, 0) = m.block<3, 3>(0, 3);
    res.block<3, 3>(0, 3) = m.block<3, 3>(3, 0);
    return res;
}

SpatialMatrix spatial_inverse(const SpatialMatrix& m)
{
    SpatialMatrix res(m);
    res.block<3, 3>(0, 0) = m.block<3, 3>(0, 0).transpose();
    res.block<3, 3>(3, 0) = m.block<3, 3>(3, 0).transpose();
    res.block<3, 3>(0, 3) = m.block<3, 3>(0, 3).transpose();
    res.block<3, 3>(3, 3) = m.block<3, 3>(3, 3).transpose();
    return res;
}

Matrix3d get_rotation(const SpatialMatrix& m)
{
    return m.block<3, 3>(0, 0);
}

Vector3d get_translation(const SpatialMatrix& m)
{
    return Vector3d(-m(4, 2), m(3, 2), -m(3, 1));
}

TEST(rdl_eigenmath, Sanitize)
{
    RobotDynamics::Math::Quaternion q1(0.1, 0.2, 0.3, 0.4);
    RobotDynamics::Math::Quaternion q2(-0.1, -0.2, -0.3, -0.4);
    q1.normalize();
    q2.normalize();
    q2.sanitize();

    EXPECT_TRUE(q1.isApprox(q2, unit_test_utils::E_MINUS_12));

    RobotDynamics::Math::Quaternion q3(0.1, 0.2, 0.3, 0.4);
    RobotDynamics::Math::Quaternion q4(0.1, 0.2, 0.3, 0.4);

    q3.normalize();
    q4.normalize();
    q4.sanitize();

    EXPECT_TRUE(q3.isApprox(q4, unit_test_utils::E_MINUS_12));
}

TEST(rdl_eigenmath, EigenConversion)
{
    RobotDynamics::Math::Quaternion q;
    Eigen::Quaterniond q_eig(1., 2., 3., 4.);

    EXPECT_EQ(q.x(), 0.);
    EXPECT_EQ(q.y(), 0.);
    EXPECT_EQ(q.z(), 0.);
    EXPECT_EQ(q.w(), 1.);

    q = q_eig;

    EXPECT_EQ(q.x(), 2.);
    EXPECT_EQ(q.y(), 3.);
    EXPECT_EQ(q.z(), 4.);
    EXPECT_EQ(q.w(), 1.);
}

TEST(rdl_eigenmath, AxisAngle)
{
    RobotDynamics::Math::Vector3d v(-0.1, -0.2, 0.3);
    v.normalize();
    RobotDynamics::Math::AxisAngle aa1(0.5, v);
    RobotDynamics::Math::Quaternion o1 = aa1;
    RobotDynamics::Math::AxisAngle aa2 = o1.toAxisAngle();

    EXPECT_NEAR(aa1.angle(), aa2.angle(), unit_test_utils::E_MINUS_14);
    EXPECT_TRUE(aa1.axis().isApprox(aa2.axis()));

    RobotDynamics::Math::SpatialTransform X = RobotDynamics::Math::Xrot(aa1.angle(), aa1.axis());
    RobotDynamics::Math::Quaternion o2 = X.E;
    EXPECT_TRUE(o2.isApprox(o1, unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, element_accessors)
{
    RobotDynamics::Math::Quaternion q1(0.2, 0.3, 0.4, 0.1);

    q1.x() = 1.1;
    q1.y() = 2.1;
    q1.z() = 3.1;
    q1.w() = 4.1;

    EXPECT_EQ(1.1, q1.x());
    EXPECT_EQ(2.1, q1.y());
    EXPECT_EQ(3.1, q1.z());
    EXPECT_EQ(4.1, q1.w());

    EXPECT_TRUE(unit_test_utils::checkVector3dEpsilonClose(q1.vec(), Vector3d(1.1, 2.1, 3.1), unit_test_utils::E_MINUS_14));
    EXPECT_EQ(q1.w(), 4.1);
}

TEST(rdl_eigenmath, multiplication)
{
    RobotDynamics::Math::Quaternion q1(0.2, 0.3, 0.4, 0.1), q2(0.2, 0.1, 0.5, 0.3);
    q1.normalize();
    q2.normalize();

    RobotDynamics::Math::Quaternion q_exp(0.5554700788944518, 0.2338821384818745, 0.3800584750330459, -0.7016464154456233);

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(q1 * q2, q_exp, unit_test_utils::E_MINUS_14));

    q1 *= q2;

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(q1, q_exp, unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, slerp)
{
    RobotDynamics::Math::Quaternion q1(0.2, 0.3, 0.4, 0.1), q2(0.2, 0.1, 0.5, 0.3);
    q1.normalize();
    q2.normalize();

    RobotDynamics::Math::Quaternion q_exp(0.3633161731320073, 0.4782736431579663, 0.7599826545340369, 0.2483587031060483);

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(q_exp, q1.slerp(0.2, q2), unit_test_utils::E_MINUS_14));

    q1 = q1 * -1.;
    q2 = q2 * -1.;

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(q_exp * -1., q1.slerp(0.2, q2), unit_test_utils::E_MINUS_14));

    q1 = q1 * -1.;
    q2 = q2 * -1.;

    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(q2, q1.slerp(1., q2), unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, conjugate)
{
    RobotDynamics::Math::Quaternion q(0.1, 0.2, -0.3, -0.4);
    EXPECT_TRUE(unit_test_utils::checkVector4dEpsilonClose(RobotDynamics::Math::Quaternion(-0.1, -0.2, 0.3, -0.4), q.conjugate(), unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, rotate)
{
    RobotDynamics::Math::Quaternion q1(0.1, 0.2, -0.3, -0.4), q2(-0.1, -0.3, -0.2, 0.);
    RobotDynamics::Math::Vector3d v(-0.1, -0.3, -0.2);
    q1.normalize();

    RobotDynamics::Math::Quaternion q_exp = q1.conjugate() * q2 * q1;
    RobotDynamics::Math::Vector3d v_out = q1.rotate(v);

    EXPECT_NEAR(v_out.x(), q_exp.x(), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(v_out.y(), q_exp.y(), unit_test_utils::E_MINUS_14);
    EXPECT_NEAR(v_out.z(), q_exp.z(), unit_test_utils::E_MINUS_14);
}

TEST(rdl_eigenmath, swingTwistDecomposition)
{
    double thx = M_PI / 4.;
    double thy = M_PI / 3.;
    double thz = -M_PI / 6.;

    RobotDynamics::Math::Quaternion qx(std::sin(thx / 2.), 0., 0., std::cos(thx / 2.)), qy(0., std::sin(thy / 2.), 0., std::cos(thy / 2.)),
        qz(0., 0., std::sin(thz / 2.), std::cos(thz / 2.));
    RobotDynamics::Math::Quaternion q = qx * qy * qz;
    RobotDynamics::Math::Quaternion q_twist, q_swing;

    q.swingTwistDecomposition(Vector3d(0., 0., 1.), q_swing, q_twist);
    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));

    q.swingTwistDecomposition(Vector3d(0., 1., 0.), q_swing, q_twist);
    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));

    q.swingTwistDecomposition(Vector3d(1., 0., 0.), q_swing, q_twist);
    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));

    q = qz * qy * qx;

    q.swingTwistDecomposition(Vector3d(0., 0., 1.), q_swing, q_twist);
    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));

    q.swingTwistDecomposition(Vector3d(0., 1., 0.), q_swing, q_twist);
    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));

    q.swingTwistDecomposition(Vector3d(1., 0., 0.), q_swing, q_twist);
    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));
}

TEST(rdl_eigenmath, swingTwistDecompositionEdgeCases)
{
    RobotDynamics::Math::Quaternion q(0., 0., 0., 1);
    RobotDynamics::Math::Quaternion q_twist, q_swing;

    q.swingTwistDecomposition(Vector3d(0., 0., 1.), q_swing, q_twist);

    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));

    q.set(0., 0., 0., -1);
    q.swingTwistDecomposition(Vector3d(0., 1., 0.), q_swing, q_twist);

    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));

    q.set(0., 0., 0., -1);
    q.swingTwistDecomposition(Vector3d(1., 0., 0.), q_swing, q_twist);

    EXPECT_TRUE(q.isApprox(q_swing * q_twist, unit_test_utils::E_MINUS_10));
}

TEST(rdl_eigenmath, fromAxisAngleToQuaternion)
{
    RobotDynamics::Math::Vector3d v(0.5, 0.11, -0.3);
    v.normalize();
    RobotDynamics::Math::Quaternion q_exp(0.2084700340099281, 0.04586340748218418, -0.1250820204059569, 0.9689124217106447);
    RobotDynamics::Math::Quaternion q(v, 0.5);

    EXPECT_TRUE(q_exp.isApprox(q, unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, fromMatrixToQuaternion)
{
    RobotDynamics::Math::Matrix3d m = (RobotDynamics::Math::Xrotx(0.1) * RobotDynamics::Math::Xroty(0.1) * RobotDynamics::Math::Xrotz(-0.1)).E;

    Eigen::Quaterniond q2(m);

    RobotDynamics::Math::Quaternion q(m);
    EXPECT_TRUE(q.isApprox(m, unit_test_utils::E_MINUS_14));

    m << 0., 1., 0., 1., 0., 0., 0., 0., -1.;

    q = Quaternion(m);
    EXPECT_TRUE(q.isApprox(m, unit_test_utils::E_MINUS_14));

    m << -1., 0., 0., 0., 1., 0., 0., 0., -1.;

    q = Quaternion(m);
    EXPECT_TRUE(q.isApprox(m, unit_test_utils::E_MINUS_14));

    m << 1., 0., 0., 0., -1., 0., 0., 0., -1.;

    q = Quaternion(m);
    EXPECT_TRUE(q.isApprox(m, unit_test_utils::E_MINUS_14));

    m << -1., 0., 0., 0., -1., 0., 0., 0., 1.;

    q = Quaternion(m);
    EXPECT_TRUE(q.isApprox(m, unit_test_utils::E_MINUS_14));

    m = (RobotDynamics::Math::Xrotx(0.1144) * RobotDynamics::Math::Xroty(-1.99331) * RobotDynamics::Math::Xrotz(-0.98011)).E;
    RobotDynamics::Math::Quaternion q_exp(-0.367164735122942, -0.7542393999032798, -0.2128590176200619, 0.5010030174893784);
    EXPECT_TRUE(q_exp.isApprox(Quaternion(m), unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, aa_conversions)
{
    RobotDynamics::Math::Vector3d v(-0.3, 0.1, 0.4);
    v.normalize();

    double angle = 0.5;

    RobotDynamics::Math::AxisAngle a(angle, v);

    RobotDynamics::Math::Quaternion q(a);

    a = q.toAxisAngle();

    EXPECT_TRUE(a.axis().isApprox(v, 1e-12));

    q = Quaternion(0.1, 0.2, 0.3, 0.4);

    a = q.toAxisAngle();

    RobotDynamics::Math::Quaternion q2 = q;
    q.normalize();

    RobotDynamics::Math::AxisAngle a2;

    a2 = q2.toAxisAngle();

    EXPECT_TRUE(a.axis().isApprox(a2.axis(), 1e-12));
    EXPECT_NEAR(a.angle(), a2.angle(), 1e-12);

    v = RobotDynamics::Math::Vector3d(0.1, 0.2, 0.3);
    v.normalize();
    q = RobotDynamics::Math::Quaternion(v.x(), v.y(), v.z(), 0.0);
    q.normalize();

    a = q.toAxisAngle();

    EXPECT_TRUE(a.axis().isApprox(q.vec(), 1e-12));
    EXPECT_NEAR(a.angle(), M_PI, 1e-12);

    q = RobotDynamics::Math::Quaternion(-0.000194225, 0.000545134, 0.000810327, 1.);

    a = q.toAxisAngle();

    EXPECT_NEAR(a.axis().x(), -0.195053, 1.0e-6);
    EXPECT_NEAR(a.axis().y(), 0.547459, 1.0e-6);
    EXPECT_NEAR(a.axis().z(), 0.813783, 1.0e-6);
}

/// \brief Checks the multiplication of a SpatialMatrix with a SpatialVector
TEST(rdl_eigenmath, TestSpatialMatrixTimesSpatialVector)
{
    SpatialMatrix s_matrix(1., 0., 0., 0., 0., 7., 0., 2., 0., 0., 8., 0., 0., 0., 3., 9., 0., 0., 0., 0., 6., 4., 0., 0., 0., 5., 0., 0., 5., 0., 4., 0., 0., 0., 0.,
                           6.);
    SpatialVector s_vector(1., 2., 3., 4., 5., 6.);

    SpatialVector result;
    result = s_matrix * s_vector;

    SpatialVector test_result(43., 44., 45., 34., 35., 40.);
    EXPECT_EQ(test_result, result);
}

TEST(rdl_eigenmath, TestSpatialTransformInverse)
{
    Matrix3d m(cos(0.4), sin(0.4), 0, -sin(0.4), cos(0.4), 0, 0, 0, 1);
    Vector3d v(1, 2, 3);
    SpatialTransform t1(m, v);
    SpatialTransform t2(m, v);

    SpatialTransform t3 = t1.inverse();
    t2.invert();

    EXPECT_EQ(t3.toMatrix(), t2.toMatrix());
}

/// \brief Checks the multiplication of a scalar with a SpatialVector
TEST(rdl_eigenmath, TestScalarTimesSpatialVector)
{
    SpatialVector s_vector(1., 2., 3., 4., 5., 6.);

    SpatialVector result;
    result = 3. * s_vector;

    SpatialVector test_result(3., 6., 9., 12., 15., 18.);

    EXPECT_EQ(test_result, result);
}

/// \brief Checks the multiplication of a scalar with a SpatialMatrix
TEST(rdl_eigenmath, TestScalarTimesSpatialMatrix)
{
    SpatialMatrix s_matrix(1., 0., 0., 0., 0., 7., 0., 2., 0., 0., 8., 0., 0., 0., 3., 9., 0., 0., 0., 0., 6., 4., 0., 0., 0., 5., 0., 0., 5., 0., 4., 0., 0., 0., 0.,
                           6.);

    SpatialMatrix result;
    result = 3. * s_matrix;

    SpatialMatrix test_result(3., 0., 0., 0., 0., 21., 0., 6., 0., 0., 24., 0., 0., 0., 9., 27., 0., 0., 0., 0., 18., 12., 0., 0., 0., 15., 0., 0., 15., 0., 12., 0., 0.,
                              0., 0., 18.);

    EXPECT_EQ(test_result, result);
}

/// \brief Checks the multiplication of a scalar with a SpatialMatrix
TEST(rdl_eigenmath, TestSpatialMatrixTimesSpatialMatrix)
{
    SpatialMatrix s_matrix(1., 0., 0., 0., 0., 7., 0., 2., 0., 0., 8., 0., 0., 0., 3., 9., 0., 0., 0., 0., 6., 4., 0., 0., 0., 5., 0., 0., 5., 0., 4., 0., 0., 0., 0.,
                           6.);

    SpatialMatrix result;
    result = s_matrix * s_matrix;

    SpatialMatrix test_result(29., 0., 0., 0., 0., 49., 0., 44., 0., 0., 56., 0., 0., 0., 63., 63., 0., 0., 0., 0., 42., 70., 0., 0., 0., 35., 0., 0., 65., 0., 28., 0.,
                              0., 0., 0., 64.);

    EXPECT_EQ(test_result, result);
}

/// \brief Checks the adjoint method
//
// This method computes a spatial force transformation from a spatial
// motion transformation and vice versa
TEST(rdl_eigenmath, TestSpatialMatrixTransformAdjoint)
{
    SpatialMatrix s_matrix(1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16., 17., 18., 19., 20., 21., 22., 23., 24., 25., 26., 27., 28., 29., 30.,
                           31., 32., 33., 34., 35., 36.);

    SpatialMatrix result = spatial_adjoint(s_matrix);

    SpatialMatrix test_result_matrix(1., 2., 3., 19., 20., 21., 7., 8., 9., 25., 26., 27., 13., 14., 15., 31., 32., 33., 4., 5., 6., 22., 23., 24., 10., 11., 12., 28.,
                                     29., 30., 16., 17., 18., 34., 35., 36.);

    EXPECT_EQ(test_result_matrix, result);
}

TEST(rdl_eigenmath, TestSpatialMatrixInverse)
{
    SpatialMatrix s_matrix(0, 1, 2, 0, 1, 2, 3, 4, 5, 3, 4, 5, 6, 7, 8, 6, 7, 8, 0, 1, 2, 0, 1, 2, 3, 4, 5, 3, 4, 5, 6, 7, 8, 6, 7, 8);

    SpatialMatrix test_inv(0, 3, 6, 0, 3, 6, 1, 4, 7, 1, 4, 7, 2, 5, 8, 2, 5, 8, 0, 3, 6, 0, 3, 6, 1, 4, 7, 1, 4, 7, 2, 5, 8, 2, 5, 8);

    EXPECT_EQ(test_inv, spatial_inverse(s_matrix));
}

TEST(rdl_eigenmath, TestSpatialMatrixGetRotation)
{
    SpatialMatrix spatial_transform(1., 2., 3., 0., 0., 0., 4., 5., 6., 0., 0., 0., 7., 8., 9., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                                    0., 0., 0.);

    //	Matrix3d rotation = spatial_transform.block<3,3>(0,0);
    Matrix3d rotation = get_rotation(spatial_transform);
    Matrix3d test_result(1., 2., 3., 4., 5., 6., 7., 8., 9.);

    EXPECT_EQ(test_result, rotation);
}

TEST(rdl_eigenmath, TestSpatialMatrixGetTranslation)
{
    SpatialMatrix spatial_transform(0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., -3., 2., 0., 0., 0., 0., 0., -1., 0., 0., 0., 0., 0., 0.,
                                    0., 0., 0.);

    Vector3d translation = get_translation(spatial_transform);
    Vector3d test_result(1., 2., 3.);

    EXPECT_EQ(test_result, translation);
}

TEST(rdl_eigenmath, TestSpatialVectorCross)
{
    SpatialVector s_vec(1., 2., 3., 4., 5., 6.);

    SpatialMatrix test_cross(0., -3., 2., 0., 0., 0., 3., 0., -1., 0., 0., 0., -2., 1., 0., 0., 0., 0., 0., -6., 5., 0., -3., 2., 6., 0., -4., 3., 0., -1., -5., 4., 0.,
                             -2., 1., 0.);

    SpatialMatrix s_vec_cross(s_vec.crossm());
    EXPECT_EQ(test_cross, s_vec_cross);

    SpatialMatrix s_vec_crossf(s_vec.crossf());
    SpatialMatrix test_crossf = -1. * s_vec.crossm().transpose();

    EXPECT_EQ(test_crossf, s_vec_crossf);
}

TEST(rdl_eigenmath, TestSpatialVectorCrossmCrossf)
{
    SpatialVector s_vec(1., 2., 3., 4., 5., 6.);
    SpatialVector t_vec(9., 8., 7., 6., 5., 4.);

    // by explicitly building the matrices (crossm/f with only one vector)
    SpatialVector crossm_s_x_t = s_vec.crossm() * t_vec;
    SpatialVector crossf_s_x_t = s_vec.crossf() * t_vec;

    // by using direct computation that avoids building of the matrix
    SpatialVector crossm_s_t = s_vec.crossm(t_vec);
    SpatialVector crossf_s_t = s_vec.crossf(t_vec);

    EXPECT_EQ(crossm_s_x_t, crossm_s_t);
    EXPECT_EQ(crossf_s_x_t, crossf_s_t);
}

TEST(rdl_eigenmath, TestSpatialTransformApply)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialTransform X_st;
    X_st.r = trans;

    SpatialMatrix X_66_matrix(SpatialMatrix::Zero(6, 6));
    X_66_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_st.E = X_66_matrix.block<3, 3>(0, 0);

    SpatialVector v(1.1, 2.1, 3.1, 4.1, 5.1, 6.1);
    SpatialVector v_66_res = X_66_matrix * v;
    SpatialVector v_st_res = X_st.apply(v);

    EXPECT_TRUE(v_66_res.isApprox(v_st_res, unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, TestSpatialTransformApplyTranspose)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialTransform X_st;
    X_st.r = trans;

    SpatialMatrix X_66_matrix(SpatialMatrix::Zero(6, 6));
    X_66_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_st.E = X_66_matrix.block<3, 3>(0, 0);

    SpatialVector v(1.1, 2.1, 3.1, 4.1, 5.1, 6.1);
    SpatialVector v_66_res = X_66_matrix.transpose() * v;
    SpatialVector v_st_res = X_st.applyTranspose(v);

    EXPECT_TRUE(v_66_res.isApprox(v_st_res, unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, TestSpatialTransformApplyAdjoint)
{
    SpatialTransform X(Xrotz(0.5) * Xroty(0.9) * Xrotx(0.2) * SpatialTransform(Vector3d(1.1, 1.2, 1.3)));

    SpatialMatrix X_adjoint = X.toMatrixAdjoint();

    SpatialVector f(1.1, 2.1, 4.1, 9.2, 3.3, 0.8);
    SpatialVector f_apply = X.applyAdjoint(f);
    SpatialVector f_matrix = X_adjoint * f;

    EXPECT_TRUE(unit_test_utils::checkSpatialVectorsEpsilonClose(f_matrix, f_apply, unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, TestSpatialTransformToMatrix)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialMatrix X_matrix(SpatialMatrix::Zero(6, 6));
    X_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();

    SpatialTransform X_st;
    X_st.E = X_matrix.block<3, 3>(0, 0);
    X_st.r = trans;

    EXPECT_TRUE(X_matrix.isApprox(X_st.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, TestSpatialTransformToMatrixAdjoint)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialMatrix X_matrix(SpatialMatrix::Zero(6, 6));
    X_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();

    SpatialTransform X_st;
    X_st.E = X_matrix.block<3, 3>(0, 0);
    X_st.r = trans;

    EXPECT_TRUE(spatial_adjoint(X_matrix).isApprox(X_st.toMatrixAdjoint(), unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, TestSpatialTransformToMatrixTranspose)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialMatrix X_matrix(SpatialMatrix::Zero(6, 6));
    X_matrix = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();

    SpatialTransform X_st;
    X_st.E = X_matrix.block<3, 3>(0, 0);
    X_st.r = trans;

    // we have to copy the matrix as it is only transposed via a flag and
    // thus data() does not return the proper data.
    SpatialMatrix X_matrix_transposed = X_matrix.transpose();

    EXPECT_TRUE(X_matrix_transposed.isApprox(X_st.toMatrixTranspose(), unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, TestSpatialTransformMultiply)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialMatrix X_matrix_1(SpatialMatrix::Zero(6, 6));
    SpatialMatrix X_matrix_2(SpatialMatrix::Zero(6, 6));

    X_matrix_1 = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_matrix_2 = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();

    SpatialTransform X_st_1;
    SpatialTransform X_st_2;

    X_st_1.E = X_matrix_1.block<3, 3>(0, 0);
    X_st_1.r = trans;
    X_st_2.E = X_matrix_2.block<3, 3>(0, 0);
    X_st_2.r = trans;

    SpatialTransform X_st_res = X_st_1 * X_st_2;
    SpatialMatrix X_matrix_res = X_matrix_1 * X_matrix_2;

    EXPECT_TRUE(X_matrix_res.isApprox(X_st_res.toMatrix(), unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, TestSpatialTransformMultiplyEqual)
{
    Vector3d rot(1.1, 1.2, 1.3);
    Vector3d trans(1.1, 1.2, 1.3);

    SpatialMatrix X_matrix_1(SpatialMatrix::Zero(6, 6));
    SpatialMatrix X_matrix_2(SpatialMatrix::Zero(6, 6));

    X_matrix_1 = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();
    X_matrix_2 = Xrotz(rot[2]).toMatrix() * Xroty(rot[1]).toMatrix() * Xrotx(rot[0]).toMatrix() * SpatialTransform(trans).toMatrix();

    SpatialTransform X_st_1;
    SpatialTransform X_st_2;

    X_st_1.E = X_matrix_1.block<3, 3>(0, 0);
    X_st_1.r = trans;
    X_st_2.E = X_matrix_2.block<3, 3>(0, 0);
    X_st_2.r = trans;

    SpatialTransform X_st_res = X_st_1;
    X_st_res *= X_st_2;
    SpatialMatrix X_matrix_res = X_matrix_1 * X_matrix_2;

    EXPECT_TRUE(X_st_res.toMatrix().isApprox(X_matrix_res, unit_test_utils::E_MINUS_14));
}

TEST(rdl_eigenmath, euler_to_quaternion)
{
    double th = 0.3;
    RobotDynamics::Math::Quaternion qx(std::sin(th / 2.), 0., 0., std::cos(th / 2.)), qy(0., std::sin(th / 2.), 0., std::cos(th / 2.)),
        qz(0., 0., std::sin(th / 2.), std::cos(th / 2.));

    EXPECT_TRUE(intrinsicXYZAnglesToQuaternion(0.3, 0.3, 0.3).isApprox(qx * qy * qz, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(RobotDynamics::Math::intrinsicZYXAnglesToQuaternion(0.3, 0.3, 0.3).isApprox(qz * qy * qx, unit_test_utils::E_MINUS_14));
    EXPECT_TRUE(RobotDynamics::Math::intrinsicYXZAnglesToQuaternion(0.3, 0.3, 0.3).isApprox(qy * qx * qz, unit_test_utils::E_MINUS_14));

    RobotDynamics::Math::Vector3d euler(-0.3, 0.112, -1.316);

    Vector3d euler_from_matrix = RobotDynamics::Math::XeulerZYX(euler).E.toIntrinsicZYXAngles();
    Vector3d euler_from_quaternion = Quaternion(RobotDynamics::Math::XeulerZYX(euler).E).toIntrinsicZYXAngles();

    EXPECT_TRUE(euler.isApprox(euler_from_quaternion, 1.e-10));
    EXPECT_TRUE(euler.isApprox(euler_from_matrix, 1.e-10));

    // should be at a singularity for yaw-pitch-roll angles
    euler = RobotDynamics::Math::Vector3d(-0.3, M_PI_2, -1.316);

    euler_from_matrix = RobotDynamics::Math::XeulerZYX(euler).E.toIntrinsicZYXAngles(euler[0]);
    euler_from_quaternion = Quaternion(RobotDynamics::Math::XeulerZYX(euler).E).toIntrinsicZYXAngles(euler[0]);

    EXPECT_TRUE(euler.isApprox(euler_from_quaternion, 1.e-10));
    EXPECT_TRUE(euler.isApprox(euler_from_matrix, 1.e-10));

    // should be at a singularity for yaw-pitch-roll angles
    euler = RobotDynamics::Math::Vector3d(-0.5, -M_PI_2, -1.16);

    euler_from_matrix = RobotDynamics::Math::XeulerZYX(euler).E.toIntrinsicZYXAngles(euler[0]);
    euler_from_quaternion = Quaternion(RobotDynamics::Math::XeulerZYX(euler).E).toIntrinsicZYXAngles(euler[0]);

    EXPECT_TRUE(euler.isApprox(euler_from_quaternion, 1.e-10));
    EXPECT_TRUE(euler.isApprox(euler_from_matrix, 1.e-10));
}

TEST(rdl_eigenmath, TestXrotAxis)
{
    SpatialTransform X_rotX = Xrotx(M_PI * 0.15);
    SpatialTransform X_rotX_axis = Xrot(M_PI * 0.15, Vector3d(1., 0., 0.));
    EXPECT_TRUE(X_rotX.toMatrix().isApprox(X_rotX_axis.toMatrix(), unit_test_utils::E_MINUS_14));

    // all the other axes
    SpatialTransform X_rotX_90 = Xrotx(M_PI * 0.5);
    SpatialTransform X_rotX_90_axis = Xrot(M_PI * 0.5, Vector3d(1., 0., 0.));
    EXPECT_TRUE(X_rotX_90.toMatrix().isApprox(X_rotX_90_axis.toMatrix(), unit_test_utils::E_MINUS_14));

    SpatialTransform X_rotY_90 = Xroty(M_PI * 0.5);
    SpatialTransform X_rotY_90_axis = Xrot(M_PI * 0.5, Vector3d(0., 1., 0.));
    EXPECT_TRUE(X_rotY_90.toMatrix().isApprox(X_rotY_90_axis.toMatrix(), unit_test_utils::E_MINUS_14));

    SpatialTransform X_rotZ_90 = Xrotz(M_PI * 0.5);
    SpatialTransform X_rotZ_90_axis = Xrot(M_PI * 0.5, Vector3d(0., 0., 1.));
    EXPECT_TRUE(X_rotZ_90.toMatrix().isApprox(X_rotZ_90_axis.toMatrix(), unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
