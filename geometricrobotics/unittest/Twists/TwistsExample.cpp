/*
 * Copyright (c) 2022, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "GeometricRobotics/Computations.h"
#include "GeometricRobotics/Utils.h"
#include "gtest/gtest.h"
#include <limits>

namespace GeoRobotics
{

class TwistsExample : public testing::Test
{

protected:
  void SetUp() override
  {

    // Set up the params

    grVec3d(axes, size);
    grVec3d(points, size, -2.0, 2.0, false);

    /*
    std::cout << "The points are: " << std::endl;
    for(auto & point : points)
    {

      std::cout << point.transpose() << std::endl;
    }
    std::cout << "The axes are: " << std::endl;
    for(auto & axis : axes)
    {

      std::cout << axis.transpose() << std::endl;
    }
    */

    // Revolute joints
    for(size_t idx = 0; idx < size / 2; idx++)
    {
      tcs.emplace_back(points[idx], axes[idx]);
    }
    // Prismatic joints
    for(size_t idx = size / 2; idx < size; idx++)
    {
      tcs.emplace_back(axes[idx]);
    }
  }

  void TearDown() override {}

  size_t size = 100;
  std::vector<Vector3d> points;
  std::vector<Vector3d> axes;
  std::vector<TwistCoordinate> tcs;

  double abs_error = 1e-5;
};

TEST_F(TwistsExample, caseOne)
{

  // Test the properties of the 6D cross product.

  for(size_t idx = 0; idx < size - 1; idx++)
  {
    // -------- Property 0: skew(v)w  == v cross w

    auto & v = axes[idx];
    auto & w = axes[idx + 1];

    ASSERT_NEAR((skew(v) * w - v.cross(w)).norm(), 0.0, abs_error);
    v = points[idx];
    w = points[idx + 1];

    ASSERT_NEAR((skew(v) * w - v.cross(w)).norm(), 0.0, abs_error);
    // -------- Property 0.1: skew(v cross w)  == skew(v)skew(w) - skew(w)skew(v)

    ASSERT_NEAR((skew(v.cross(w)) - (skew(v) * skew(w) - skew(w) * skew(v))).norm(), 0.0, abs_error);

    // -------- Property 0.2: v cross w  not equal skew(v)w - skew(w)v

    // ASSERT_NEAR((v.cross(w) - (skew(v) * w - skew(w) * v)).norm(), 0.0, abs_error);
    // ASSERT_NEAR((v.cross(w) - skew(v) * w).norm(), 0.0, abs_error);
  }
  for(size_t idx = 0; idx < size - 2; idx++)
  {
    // -------- Property 1: xi cross xi == 0
    auto & tc1 = tcs[idx];
    auto & tc2 = tcs[idx + 1];
    auto & tc3 = tcs[idx + 2];
    ASSERT_EQ((tc1 * tc1).norm(), 0.0);

    ASSERT_EQ(cross(tc1.vector(), tc1.vector()).norm(), 0.0);

    // -------- Property 2: xi_1 cross xi_2 == -   xi_2 cross xi_1
    auto p2 = tc1 * tc2 + tc2 * tc1;
    ASSERT_NEAR(p2.norm(), 0.0, abs_error);

    Vector6d v2 = cross(tc1.vector(), tc2.vector()) + cross(tc2.vector(), tc1.vector());
    ASSERT_NEAR(v2.norm(), 0.0, abs_error);

    // -------- Property 3:
    // xi_1 cross ( xi_2 cross xi_3) + xi_2 cross (xi_3 cross xi_1) + xi_3 cross (xi_1 cross xi_2) ==  0.0
    auto p3 = tc1 * (tc2 * tc3) + tc2 * (tc3 * tc1) + tc3 * (tc1 * tc2);

    ASSERT_NEAR(p3.norm(), 0.0, abs_error);

    Vector6d v3 = cross(tc1.vector(), cross(tc2.vector(), tc3.vector()))
                  + cross(tc2.vector(), cross(tc3.vector(), tc1.vector()))
                  + cross(tc3.vector(), cross(tc1.vector(), tc2.vector()));

    ASSERT_NEAR(v3.norm(), 0.0, abs_error);

    // -------- Test the 6x6 cross matrix
    // cross matrix
    auto p4 = tc1 * tc2;

    auto p5 = tc1.crossMatrix() * tc2.vector();

    ASSERT_NEAR((p4 - p5).norm(), 0.0, abs_error);

    Matrix6d result =
        p4.crossMatrix() - (tc1.crossMatrix() * tc2.crossMatrix() - tc2.crossMatrix() * tc1.crossMatrix());
    ASSERT_NEAR(result.norm(), 0.0, abs_error);

    // ASSERT_NEAR((p4.linear - p5.head<3>()).norm(), 0.0, abs_error);

    // ASSERT_NEAR((p4.angular - p5.tail<3>()).norm(), 0.0, abs_error);
  }
} // End of caseOne

TEST_F(TwistsExample, caseTwo)
{
  // Try the adjoint transforms
  // Test the properties of the 6D cross product.

  for(size_t idx = 0; idx < size - 1; idx++)
  {
    // auto & t1 = tcs[idx];
    // auto & t2 = tcs[idx + 1];
    TwistCoordinate tc1, tc2;
    TwistCoordinate & t = tcs[idx];

    Iso3d transform;

    transform.linear() = grRotation();

    std::default_random_engine re;
    transform.translation() = grVec3d(re);

    // 0. AdgInv == AdgInvTwo
    metaAdgInv(transform, t, tc1);
    metaAdgInvTwo(transform, t, tc2);

    const Matrix6d & adgInv = metaAdgInvMatrix(transform);
    const Matrix6d & adgInvTwo = metaAdgInvTwoMatrix(transform);

    ASSERT_EQ(tc1 == tc2, true);

    ASSERT_NEAR((adgInv * t.vector() - adgInvTwo * t.vector()).norm(), 0.0, abs_error);
    ASSERT_NEAR((adgInv * t.vector() - tc1.vector()).norm(), 0.0, abs_error);
    ASSERT_NEAR((adgInv * t.vector() - tc2.vector()).norm(), 0.0, abs_error);

    // 1. AdgInv * Adg == identity

    TwistCoordinate tc3, tc4;

    metaAdg(transform, tc1, tc3);
    metaAdg(transform, tc2, tc4);

    ASSERT_EQ(tc3 == tc4, true);
    ASSERT_EQ(tc3 == t, true);
    ASSERT_EQ(tc4 == t, true);

    // 2. Ad^-1g == Adg^-1

    Iso3d tInv = transform.inverse(Eigen::Isometry);
    Vector3d t_test = tInv.translation() + transform.linear().transpose() * transform.translation();
    ASSERT_NEAR(t_test.norm(), 0.0, abs_error);

    ASSERT_NEAR((tInv.linear() * transform.linear() - Matrix3d::Identity()).norm(), 0.0, abs_error);

    TwistCoordinate tc5;
    metaAdg(tInv, t, tc5);

    ASSERT_EQ(tc1 == tc5, true);
    ASSERT_EQ(tc2 == tc5, true);
  }

} // End of caseTwo

TEST_F(TwistsExample, caseThree)
{
  for(size_t i = 0; i < size; i++)
  {
    // 1. Check the exponential maps:  exp^so(3) --> SO(3)
    Eigen::AngleAxisd m = grAngleAxis();

    double angle = m.angle();
    Vector3d axis = m.axis();

    Matrix3d R_1 = ::GeoRobotics::expMap(axis, angle);
    ASSERT_NEAR((R_1 - m.matrix()).norm(), 0.0, abs_error);

    // 2. Check the exponential maps:  exp^se(3) --> SE(3)
    // Following proposition 2.9 of Murray et al. 1994.

    // 2.1 When the rotation is identity
    Iso3d tf_id;
    tf_id.linear().setIdentity();
    tf_id.translation() = points[i];

    double prismatic_theta = points[i].norm();
    TwistCoordinate tc1(points[i] / prismatic_theta);

    Iso3d tctf = expMap(tc1, prismatic_theta);

    ASSERT_NEAR((tctf.linear() - tf_id.linear()).norm(), 0.0, abs_error);
    ASSERT_NEAR((tctf.translation() - tf_id.translation()).norm(), 0.0, abs_error);

    // 2.2 When the rotation is arbitrary

    Iso3d tf;

    tf.linear() = m.matrix();
    tf.translation() = points[i];
    Matrix3d A = (Matrix3d::Identity() - expMap(axis, angle)) * skew(axis) + axis * axis.transpose() * angle;
    TwistCoordinate tc3;
    tc3.angular = axis;
    tc3.linear = A.inverse() * tf.translation();

    Iso3d tctf_two = expMap(tc3, angle);
    // TwistCoordinate tc1;
    ASSERT_NEAR((tctf_two.linear() - tf.linear()).norm(), 0.0, abs_error);

    ASSERT_NEAR((tc3.linear - A.inverse() * tf.translation()).norm(), 0.0, abs_error);
    ASSERT_NEAR((tctf_two.translation() - tf.translation()).norm(), 0.0, abs_error);
  }

} // End of caseThree

TEST_F(TwistsExample, caseFour)
{
  for(size_t i = 0; i < size; i++)
  {
    Iso3d tf;
    // Always initialize. The fourth row is not zero by default.
    tf.setIdentity();

    std::default_random_engine re;
    tf.linear() = grRotation();
    tf.translation() = grVec3d(re);

    const TwistCoordinate & xi = tcs[i];

    // property 1: hat(adg xi) == g * hat(xi) * g^{-1}
    TwistCoordinate xi_prime;
    metaAdg(tf, xi, xi_prime);
    const Matrix6d & adg = metaAdgMatrix(tf);

    const Matrix4d & result = hat(xi_prime) - tf.matrix() * hat(xi) * tf.inverse(Eigen::Isometry).matrix();
    ASSERT_NEAR(result.norm(), 0.0, abs_error);

    ASSERT_NEAR((adg * xi.vector() - xi_prime.vector()).norm(), 0.0, abs_error);

    // Lemma 2.13
    // property 2: exp(xi_prime, theta) == g * exp(xi, theta) * g^-1

    double theta = grNumber();
    const Iso3d & tf_xi_prime = expMap(xi_prime, theta);
    const Iso3d & tf_xi = expMap(xi, theta);
    const Iso3d & tf_test = tf * tf_xi * tf.inverse(Eigen::Isometry);

    ASSERT_NEAR((tf_xi_prime.linear() - tf_test.linear()).norm(), 0.0, abs_error);
    ASSERT_NEAR((tf_xi_prime.translation() - tf_test.translation()).norm(), 0.0, abs_error);

    // Exercise 2.8.8.b g * exp(A) * g^-1 = exp(g * A * g^-1)
  }

} // End of caseFour

} // namespace GeoRobotics
