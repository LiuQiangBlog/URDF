// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include "rdl_dynamics/RigidBodyInertia.hpp"

namespace RobotDynamics
{
namespace Math
{
void RigidBodyInertia::operator+=(const RigidBodyInertia& rbi)
{
    this->m += rbi.m;
    this->h += rbi.h;
    this->Ixx += rbi.Ixx;
    this->Iyx += rbi.Iyx;
    this->Iyy += rbi.Iyy;
    this->Izx += rbi.Izx;
    this->Izy += rbi.Izy;
    this->Izz += rbi.Izz;
}

void RigidBodyInertia::transform(const SpatialTransform& X)
{
    double hx = h[0];
    double hy = h[1];
    double hz = h[2];

    double rx = X.r[0];
    double ry = X.r[1];
    double rz = X.r[2];

    double E00 = X.E(0, 0);
    double E01 = X.E(0, 1);
    double E02 = X.E(0, 2);
    double E10 = X.E(1, 0);
    double E11 = X.E(1, 1);
    double E12 = X.E(1, 2);
    double E20 = X.E(2, 0);
    double E21 = X.E(2, 1);
    double E22 = X.E(2, 2);

    double alpha = 2. * hx * rx;
    double beta = 2. * hy * ry;
    double gamma = 2. * hz * rz;
    double epsilon = m * rx * rx;
    double phi = m * ry * ry;
    double psi = m * rz * rz;
    double pi = m * rx * ry;
    double a = m * rx * rz;
    double b = m * ry * rz;
    double c = hx * ry;
    double d = hx * rz;
    double e = hy * rz;
    double f = hy * rx;
    double g = hz * rx;
    double j = hz * ry;

    double b00 = Ixx - gamma - beta + psi + phi;
    double b01 = Iyx + f + c - pi;
    double b02 = Izx + g + d - a;
    double b11 = Iyy - gamma - alpha + psi + epsilon;
    double b12 = Izy + j + e - b;
    double b22 = Izz - beta - alpha + phi + epsilon;

    double w00 = E00 * b00 + E01 * b01 + E02 * b02;
    double w01 = E00 * b01 + E01 * b11 + E02 * b12;
    double w02 = E00 * b02 + E01 * b12 + E02 * b22;

    double w10 = E10 * b00 + E11 * b01 + E12 * b02;
    double w11 = E10 * b01 + E11 * b11 + E12 * b12;
    double w12 = E10 * b02 + E11 * b12 + E12 * b22;

    double w20 = E20 * b00 + E21 * b01 + E22 * b02;
    double w21 = E20 * b01 + E21 * b11 + E22 * b12;
    double w22 = E20 * b02 + E21 * b12 + E22 * b22;

    h = X.E * (h - m * X.r);
    Ixx = w00 * E00 + w01 * E01 + w02 * E02;
    Iyx = w00 * E10 + w01 * E11 + w02 * E12;
    Iyy = w10 * E10 + w11 * E11 + w12 * E12;
    Izx = w00 * E20 + w01 * E21 + w02 * E22;
    Izy = w10 * E20 + w11 * E21 + w12 * E22;
    Izz = w20 * E20 + w21 * E21 + w22 * E22;
}

void RigidBodyInertia::createFromMatrix(const SpatialMatrix& Ic)
{
    m = Ic(3, 3);
    h.set(-Ic(1, 5), Ic(0, 5), -Ic(0, 4));
    Ixx = Ic(0, 0);
    Iyx = Ic(1, 0);
    Iyy = Ic(1, 1);
    Izx = Ic(2, 0);
    Izy = Ic(2, 1);
    Izz = Ic(2, 2);
}

SpatialMatrix RigidBodyInertia::toMatrix() const
{
    SpatialMatrix result;
    result(0, 0) = Ixx;
    result(0, 1) = Iyx;
    result(0, 2) = Izx;
    result(1, 0) = Iyx;
    result(1, 1) = Iyy;
    result(1, 2) = Izy;
    result(2, 0) = Izx;
    result(2, 1) = Izy;
    result(2, 2) = Izz;

    result.block<3, 3>(0, 3) = h.toTildeForm();
    result.block<3, 3>(3, 0) = -h.toTildeForm();
    result.block<3, 3>(3, 3) = Matrix3d::Identity(3, 3) * m;

    return result;
}

void RigidBodyInertia::setSpatialMatrix(SpatialMatrix& mat) const
{
    mat(0, 0) = Ixx;
    mat(0, 1) = Iyx;
    mat(0, 2) = Izx;
    mat(1, 0) = Iyx;
    mat(1, 1) = Iyy;
    mat(1, 2) = Izy;
    mat(2, 0) = Izx;
    mat(2, 1) = Izy;
    mat(2, 2) = Izz;

    mat(3, 0) = 0.;
    mat(3, 1) = h[2];
    mat(3, 2) = -h[1];
    mat(4, 0) = -h[2];
    mat(4, 1) = 0.;
    mat(4, 2) = h[0];
    mat(5, 0) = h[1];
    mat(5, 1) = -h[0];
    mat(5, 2) = 0.;

    mat(0, 3) = 0.;
    mat(0, 4) = -h[2];
    mat(0, 5) = h[1];
    mat(1, 3) = h[2];
    mat(1, 4) = 0.;
    mat(1, 5) = -h[0];
    mat(2, 3) = -h[1];
    mat(2, 4) = h[0];
    mat(2, 5) = 0.;

    mat(3, 3) = m;
    mat(3, 4) = 0.;
    mat(3, 5) = 0.;
    mat(4, 3) = 0.;
    mat(4, 4) = m;
    mat(4, 5) = 0.;
    mat(5, 3) = 0.;
    mat(5, 4) = 0.;
    mat(5, 5) = m;
}
}  // namespace Math
}  // namespace RobotDynamics
