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
#pragma once

#include <Eigen/Dense>
#include <RoboticsUtils/utils.h>
#include <algorithm>
#include <iostream>
#include <map>
#include <random>

namespace GeoRobotics
{

// class Model;
static double const GRAVITY = 9.80665;

static double const m_pi = static_cast<double>(M_PI);

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector4d Vector4d;

typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Isometry3d Iso3d;
typedef Eigen::AngleAxisd AngleAxisd;

// This is a locale-safe version of string-to-double, which is suprisingly
// difficult to do correctly.  This function ensures that the C locale is used
// for parsing, as that matches up with what the XSD for double specifies.
// On success, the double is returned; on failure, a std::runtime_error is
// thrown.
static inline double strToDouble(const char * in)
{
  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << in;

  double out;
  ss >> out;

  if(ss.fail() || !ss.eof())
  {
    throw std::runtime_error("Failed converting string to double");
  }

  return out;
}

// Replacement for boost::split( ... , ... , boost::is_any_of(" "))
inline void split_string(std::vector<std::string> & result, const std::string & input, const std::string & isAnyOf)
{
  std::string::size_type start = 0;
  std::string::size_type end = input.find_first_of(isAnyOf, start);
  while(end != std::string::npos)
  {
    result.push_back(input.substr(start, end - start));
    start = end + 1;
    end = input.find_first_of(isAnyOf, start);
  }
  if(start < input.length())
  {
    result.push_back(input.substr(start));
  }
}

/*! \brief Parse the 3D vector from a string.
 */
Vector3d parseVector3d(const std::string & vector_str);

void printMatrix(const std::string & name, const MatrixXd & matrix);
template<typename input>
std::string toString(const input & mat);
} // namespace GeoRobotics
