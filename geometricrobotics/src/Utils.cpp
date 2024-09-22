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
#include "GeometricRobotics/Utils.h"

namespace GeoRobotics
{

template<typename input>
std::string toString(const input & vec)
{
  std::stringstream ss;
  ss << vec;
  return ss.str();
}

template std::string toString<Eigen::MatrixXd>(const Eigen::MatrixXd & vec);

/*
template
std::string toString<Eigen::VectorXd>(const Eigen::VectorXd & vec);
*/

template std::string toString<Eigen::Vector3d>(const Eigen::Vector3d & vec);

Vector3d parseVector3d(const std::string & vector_str)
{
  Vector3d vec;
  std::vector<std::string> pieces;
  // std::vector<double> xyz;
  split_string(pieces, vector_str, " ");
  for(unsigned int i = 0; i < pieces.size(); ++i)
  {
    if(pieces[i] != "")
    {
      try
      {
        vec(i) = strToDouble(pieces[i].c_str());
      }
      catch(std::runtime_error &)
      {
        RoboticsUtils::throw_runtime_error("Unable to parse component [" + pieces[i]
                                               + "] to a double (while parsing a vector value)",
                                           __FILE__, __LINE__);
      }
    }
  }

  if(vec.size() != 3)
    RoboticsUtils::throw_runtime_error("Parser found " + std::to_string(vec.size())
                                           + " elements but 3 expected while parsing vector [" + vector_str + "]",
                                       __FILE__, __LINE__);

  return vec;
}

void printMatrix(const std::string & name, const MatrixXd & matrix)
{
  RoboticsUtils::quickInfo("The " + name + " is: ");
  std::cout << matrix << std::endl;
  std::cout << "-------" << std::endl;
}

} // namespace GeoRobotics
