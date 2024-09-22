// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <gtest/gtest.h>

#include "UnitTestUtils.hpp"

using namespace std;
using namespace RobotDynamics::Math;

struct MathFixture : public testing::Test
{
};

TEST(MathFixture, GaussElimPivot)
{
    MatrixNd A;
    A.resize(3, 3);
    VectorNd b(3);
    VectorNd x(3);

    A(0, 0) = 0;
    A(0, 1) = 2;
    A(0, 2) = 1;
    A(1, 0) = 1;
    A(1, 1) = 1;
    A(1, 2) = 5;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    b[0] = 1;
    b[1] = 2;
    b[2] = 3;

    VectorNd test_result(3);

    test_result[0] = -12;
    test_result[1] = -1;
    test_result[2] = 3;

    linSolveGaussElimPivot(A, b, x);

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(test_result, x, unit_test_utils::E_MINUS_14));

    A(0, 0) = 0;
    A(0, 1) = -2;
    A(0, 2) = 1;
    A(1, 0) = 1;
    A(1, 1) = 1;
    A(1, 2) = 5;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    linSolveGaussElimPivot(A, b, x);
    test_result[0] = -14;
    test_result[1] = 1;
    test_result[2] = 3;

    EXPECT_TRUE(unit_test_utils::checkVectorNdEpsilonClose(test_result, x, unit_test_utils::E_MINUS_14));
}

TEST(MathFixture, Dynamic_1D_initialize_value)
{
    VectorNd myvector_10 = VectorNd::Constant((size_t)10, 12.);

    double* test_values = new double[10];
    for (unsigned int i = 0; i < 10; i++)
    {
        test_values[i] = 12.;
    }

    for (int i = 0; i < myvector_10.rows(); i++)
    {
        EXPECT_EQ(myvector_10(i), test_values[i]);
    }

    delete[] test_values;
}

TEST(MathFixture, Dynamic_2D_initialize_value)
{
    MatrixNd mymatrix_10x10 = MatrixNd::Constant(10, 10, 12.);

    double* test_values = new double[10 * 10];
    for (unsigned int i = 0; i < 10; i++)
    {
        for (unsigned int j = 0; j < 10; j++)
        {
            test_values[i * 10 + j] = 12.;
        }
    }

    for (int i = 0; i < mymatrix_10x10.rows(); i++)
    {
        for (int j = 0; j < mymatrix_10x10.cols(); j++)
        {
            EXPECT_EQ(test_values[i * 10 + j], mymatrix_10x10(i, j));
        }
    }

    //    EXPECT_TRUE(unit_test_utils::checkArraysEq(test_values, mymatrix_10x10.data(), 10 * 10));
    delete[] test_values;
}

TEST(MathFixture, SpatialMatrix_Multiplication)
{
    SpatialMatrix X_1(1., 2., 3., 4., 5., 6., 11., 12., 13., 14., 15., 16., 21., 22., 23., 24., 25., 26., 31., 32., 33., 34., 35., 36., 41., 42., 43., 44., 45., 46., 51.,
                      52., 53., 54., 55., 56.);

    SpatialMatrix X_2(X_1);

    X_2 *= 2;

    SpatialMatrix correct_result(1442, 1484, 1526, 1568, 1610, 1652, 4562, 4724, 4886, 5048, 5210, 5372, 7682, 7964, 8246, 8528, 8810, 9092, 10802, 11204, 11606, 12008,
                                 12410, 12812, 13922, 14444, 14966, 15488, 16010, 16532, 17042, 17684, 18326, 18968, 19610, 20252);

    SpatialMatrix test_result = X_1 * X_2;

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(correct_result, test_result, unit_test_utils::E_MINUS_14));

    // check the *= operator:
    test_result = X_1;
    test_result *= X_2;

    EXPECT_TRUE(unit_test_utils::checkSpatialMatrixEpsilonClose(correct_result, test_result, unit_test_utils::E_MINUS_14));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
