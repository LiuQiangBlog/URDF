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
#include "GeometricRobotics/urdf/Geometry.h"
#include "GeometricRobotics/urdf/Joint.h"
#include <algorithm>
#include <vector>

namespace GeoRobotics
{

/*
class Inertia
{
public:
Inertia(const double & mass = 1.0)
{
  setMass(mass);
}

void reset(const MetaInertia & mi)
{
  setMass(mi.mass);
  setRotationalInertia(mi.ixx, mi.ixy, mi.ixz, mi.iyy, mi.iyz, mi.izz);
}

Inertia(const MetaInertia & mi)
{
  setMass(mi.mass);
  setRotationalInertia(mi.ixx, mi.ixy, mi.ixz, mi.iyy, mi.iyz, mi.izz);
}

~Inertia() {}

void setMass(const double & mass)
{
  mass_ = mass;
  massMatrix_ = Matrix3d::Identity();
  massMatrix_ *= mass_;
  // bfGInertia_.block<3,3>(0,0) *= mass_;
}

void setRotationalInertia(const double & ixx,
                          const double & ixy,
                          const double & ixz,
                          const double & iyy,
                          const double & iyz,
                          const double & izz)
{
  rInertia_(0, 0) = ixx;
  rInertia_(0, 1) = ixy;
  rInertia_(0, 2) = ixz;
  rInertia_(1, 0) = ixy;
  rInertia_(2, 0) = ixz;

  rInertia_(1, 1) = iyy;
  rInertia_(1, 2) = iyz;
  rInertia_(2, 1) = iyz;

  rInertia_(2, 2) = izz;
}

Matrix6d gInertia() const
{
  Matrix6d m;

  m << mMatrix(), ur, ll, rInertia();

  return m;
}

const Matrix3d & rInertia() const
{
  return rInertia_;
}
const Matrix3d & mMatrix() const
{
  return massMatrix_;
}

protected:
double mass_ = 1.0;
// double ixx = 0.0,ixy = 0.0,ixz = 0.0,iyy = 0.0,iyz = 0.0,izz = 0.0;

// Generalized inertia in its own body frame locates at the COM.
// Eigen::MatrixXd bfGInertia_ = Eigen::MatrixXd::Identity(6,6);

Matrix3d rInertia_ = Matrix3d::Identity();

Matrix3d massMatrix_ = Matrix3d::Identity();
Matrix3d ur = Matrix3d::Zero();
Matrix3d ll = Matrix3d::Zero();

}; // End of Inertia
*/
class Link
{
public:
  Link()
  {
    // RoboticsUtils::quickInfo("Empty link created.");
  }
  Link(const std::string & linkName) : name(linkName)
  {
    // std::cout << RoboticsUtils::info << "Link" << name << " has been created." << RoboticsUtils::reset << std::endl;
  }
  ~Link() {}

  const std::shared_ptr<Joint> parentJoint()
  {
    return parent_;
  }
  void setParentJoint(std::shared_ptr<Joint> parent)
  {
    parent_ = parent;
  }

  void removeChildJoint(std::shared_ptr<Joint> joint);
  void addChildJoint(std::shared_ptr<Joint> joint)
  {
    childrenJoints_.push_back(joint);
  }
  const std::vector<std::shared_ptr<Joint>> & childrenJoints()
  {
    return childrenJoints_;
  }

  std::shared_ptr<Link> parentLink() const
  {
    return parentLink_.lock();
  };

  void setParentLink(const std::shared_ptr<Link> & parent)
  {
    parentLink_ = parent;
  }

  void addChildLink(std::shared_ptr<Link> link)
  {
    childrenLinks_.push_back(link);
  }
  const std::vector<std::shared_ptr<Link>> & childrenLinks()
  {
    return childrenLinks_;
  }

  /// visual element
  std::shared_ptr<Visual> visual;

  /// collision element
  std::shared_ptr<Collision> collision;

  std::shared_ptr<MetaInertia> metaInertia;

  /// if more than one collision element is specified, all collision elements are placed in this array (the collision
  /// member points to the first element of the array)
  std::vector<std::shared_ptr<Collision>> collisionArray;

  /// if more than one visual element is specified, all visual elements are placed in this array (the visual member
  /// points to the first element of the array)
  std::vector<std::shared_ptr<Visual>> visualArray;

  std::string name;

  bool emptyLink() const
  {
    return name == "EmptyLink";
  }
  // MetaInertia metaInertia;
  void reset()
  {
    name.clear();
    name = "EmptyLink";
    metaInertia = std::make_shared<MetaInertia>();
    visual = std::make_shared<Visual>();
    collision = std::make_shared<Collision>();
    collisionArray.clear();
    visualArray.clear();

    parent_.reset();
    parentLink_.reset();
    childrenJoints_.clear();
    childrenLinks_.clear();
  };

  /*
  Inertia & inertia()
  {
    return inertia_;
  }
  */

protected:
  ///< At most one parent joint
  std::shared_ptr<Joint> parent_;

  std::weak_ptr<Link> parentLink_;

  std::vector<std::shared_ptr<Joint>> childrenJoints_;

  ///< Links without joints
  std::vector<std::shared_ptr<Link>> childrenLinks_;

  // Inertia inertia_;

}; // End of Link

} // namespace GeoRobotics
