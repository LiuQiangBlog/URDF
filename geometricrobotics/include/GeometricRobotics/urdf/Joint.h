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

#include "GeometricRobotics/Utils.h"
#include "GeometricRobotics/urdf/Quaternion.h"

namespace GeoRobotics
{

enum struct JointType
{
  PRISMATIC,
  REVOLUTE,
  CONTINUOUS,
  FLOATING,
  HELICAL,
  PLANAR,
  FIXED,
  UNKNOWN

};

inline std::string ToString(const JointType & v)
{
  switch(v)
  {
    case JointType::PRISMATIC:
      return "PRISMATIC";
    case JointType::REVOLUTE:
      return "REVOLUTE";
    case JointType::CONTINUOUS:
      return "CONTINUOUS";
    case JointType::FLOATING:
      return "FLOATING";
    case JointType::HELICAL:
      return "HELICAL";
    case JointType::PLANAR:
      return "PLANAR";
    case JointType::FIXED:
      return "FIXED";
    case JointType::UNKNOWN:
      return "UNKNOWN";
    default:
      return "[Unknown JointType]";
  }
}

/*
std::string operator+(std::string out, const JointType value)
{
  static std::map<JointType, std::string> strings;
  if(strings.size() == 0)
  {
#define INSERT_ELEMENT(p) strings[p] = #p
    INSERT_ELEMENT(JointType::PRISMATIC);
    INSERT_ELEMENT(JointType::REVOLUTE);
    INSERT_ELEMENT(JointType::CONTINUOUS);
    INSERT_ELEMENT(JointType::FLOATING);
    INSERT_ELEMENT(JointType::HELICAL);
    INSERT_ELEMENT(JointType::PLANAR);
    INSERT_ELEMENT(JointType::FIXED);
    INSERT_ELEMENT(JointType::UNKNOWN);
#undef INSERT_ELEMENT
  }
  return out = strings[value];
}

std::ostream & operator<<(std::ostream & out, const JointType value)
{
  static std::map<JointType, std::string> strings;
  if(strings.size() == 0)
  {
#define INSERT_ELEMENT(p) strings[p] = #p
    INSERT_ELEMENT(JointType::PRISMATIC);
    INSERT_ELEMENT(JointType::REVOLUTE);
    INSERT_ELEMENT(JointType::CONTINUOUS);
    INSERT_ELEMENT(JointType::FLOATING);
    INSERT_ELEMENT(JointType::HELICAL);
    INSERT_ELEMENT(JointType::PLANAR);
    INSERT_ELEMENT(JointType::FIXED);
    INSERT_ELEMENT(JointType::UNKNOWN);
#undef INSERT_ELEMENT
  }

  return out << strings[value];
}
*/

class Link;

class JointDynamics
{
public:
  JointDynamics()
  {
    reset();
  };
  double damping;
  double friction;

  void reset()
  {
    damping = 0.0;
    friction = 0.0;
  };
};

class JointLimits
{
public:
  JointLimits()
  {
    reset();
  };
  double lower;
  double upper;
  double effort;
  double velocity;

  void resetFloatingBase()
  {
    lower = -std::numeric_limits<double>::infinity();
    upper = std::numeric_limits<double>::infinity();
    effort = std::numeric_limits<double>::infinity();
    velocity = std::numeric_limits<double>::infinity();
  };

  void reset()
  {
    lower = 0.0;
    upper = 0.0;
    effort = 0.0;
    velocity = 0.0;
  };
};

class Joint
{
public:
  Joint()
  {
    // RoboticsUtils::quickInfo("Empty joint created.");
  }
  ~Joint() {}

  ///< parent link name
  std::string parentLinkName;

  ///< parent link name
  std::string childLinkName;
  std::string name;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            HELICAL     translation/rotation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Vector3d axis;

  void resetFloatingBase()
  {
    limits->resetFloatingBase();
  }

  void reset()
  {
    // axis.clear();
    // child_link_name.clear();
    // parent_link_name.clear();
    // parent_to_joint_origin_transform.clear();
    // dynamics.reset();
    // limits.reset();
    // safety.reset();
    // calibration.reset();
    // mimic.reset();
    transform.reset();
    type = JointType::UNKNOWN;
  };

  ///< The link pose with respect to the parent link frame origin.
  MetaPose transform;
  // By default, we choose revolute joints.
  JointType type = JointType::REVOLUTE;
  /// Joint Dynamics
  std::shared_ptr<JointDynamics> dynamics;

  /// Joint Limits
  std::shared_ptr<JointLimits> limits;

protected:
};

} // namespace GeoRobotics
