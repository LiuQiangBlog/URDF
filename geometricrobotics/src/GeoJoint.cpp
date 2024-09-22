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
#include "GeometricRobotics/Kinematics/GeoJoint.h"

namespace GeoRobotics
{

bool GeoJoint::checkLimits_q_(const double & q) const
{
  if((q >= ujPtr_->limits->lower) && (q <= ujPtr_->limits->upper))
  {
    return true;
  }
  RoboticsUtils::throw_runtime_error(ujPtr_->name + " Joint position: " + std::to_string(q)
                                         + " violated the limits: " + std::to_string(ujPtr_->limits->lower) + ", "
                                         + std::to_string(ujPtr_->limits->upper) + ". ",
                                     __FILE__, __LINE__);
  return false;
}
bool GeoJoint::checkLimits_dq_(const double & dq) const
{
  if((fabs(dq) <= ujPtr_->limits->velocity))
  {
    return true;
  }
  RoboticsUtils::throw_runtime_error(ujPtr_->name + " Joint velocity: " + std::to_string(dq)
                                         + " violated the limits:  " + std::to_string(ujPtr_->limits->velocity) + ".",
                                     __FILE__, __LINE__);
  return false;
}

bool GeoJoint::checkLimits_tau_(const double & tau) const
{
  if((fabs(tau) <= ujPtr_->limits->effort))
  {
    return true;
  }
  RoboticsUtils::throw_runtime_error(ujPtr_->name + " Joint torque: " + std::to_string(tau)
                                         + " violated the limits:  " + std::to_string(ujPtr_->limits->effort) + ".",
                                     __FILE__, __LINE__);
  return false;
}

void GeoJoint::velIntegration(const double & dt)
{
  q_ = q_ + dq_ * dt;
}

void GeoJoint::setRandom(const size_t & status, std::default_random_engine & re, double safety_factor)
{
  auto randomVec = [](std::default_random_engine & re, const bool & normalize = false) -> Vector3d
  {
    Vector3d vec;
    vec.setZero();
    vec.x() = grNumber(re, -1.0, 1.0);
    vec.y() = grNumber(re, -1.0, 1.0);
    vec.z() = grNumber(re, -1.0, 1.0);

    if(normalize)
    {
      vec.normalize();
    }

    return vec;
  };
  auto randomQuaternion = [randomVec](std::default_random_engine & re) -> Eigen::Quaterniond
  {
    const Vector3d & vec = randomVec(re, true);
    ;
    return Eigen::Quaterniond(grNumber(re, 0.0, 1.0), vec.x(), vec.y(), vec.z());
  };
  if(jointType() != JointType::FLOATING)
  {
    switch(status)
    {
      case 0:
      {
        q_ = grNumber(re, safety_factor * ujPtr_->limits->lower, safety_factor * ujPtr_->limits->upper);
        break;
      }
      case 1:
      {
        q_ = grNumber(re, safety_factor * ujPtr_->limits->lower, safety_factor * ujPtr_->limits->upper);
        dq_ = grNumber(re, -ujPtr_->limits->velocity, ujPtr_->limits->velocity);
        break;
      }
      case 2:
      {
        q_ = grNumber(re, safety_factor * ujPtr_->limits->lower, safety_factor * ujPtr_->limits->upper);
        dq_ = grNumber(re, -ujPtr_->limits->velocity, ujPtr_->limits->velocity);
        ddq_ = grNumber(re, ujPtr_->limits->lower, ujPtr_->limits->upper);
        break;
      }

      default:
        RoboticsUtils::throw_runtime_error("Unknown status caught: " + std::to_string(status), __FILE__, __LINE__);
    }

    updateTransform_(q_);
  }
  else
  {
    // For floating-base joint
    switch(status)
    {
      case 0:
      {
        // Linear
        transform_.translation() = randomVec(re, false);
        // Rotation
        transform_.linear() = randomQuaternion(re).toRotationMatrix();

        break;
      }
      case 1:
      {
        // Linear
        transform_.translation() = randomVec(re, false);
        // Rotation
        transform_.linear() = randomQuaternion(re).toRotationMatrix();

        jointVel_.head(3) = randomVec(re, true);
        jointVel_.tail(3) = randomVec(re, true);

        break;
      }
      case 2:
      {
        // Linear
        transform_.translation() = randomVec(re, false);
        // Rotation
        transform_.linear() = randomQuaternion(re).toRotationMatrix();

        jointVel_.head(3) = randomVec(re, true);
        jointVel_.tail(3) = randomVec(re, true);

        // Acceleration is not set
        break;
      }

      default:
        RoboticsUtils::throw_runtime_error("Unknown status caught: " + std::to_string(status), __FILE__, __LINE__);
    }
    incrementalTransform_ = transform_;
  }
}

void GeoJoint::update(const double & q)
{
  if(dof() == 1)
  {
    // Update the joint values
    if(checkLimits_q_(q))
    {
      q_ = q;
    }

    // Update the transform
    updateTransform_(q_);
  }
  else
  {
    RoboticsUtils::throw_runtime_error("Updatating dof " + std::to_string(dof()) + " joint " + name() + " with position"
                                           + std::to_string(q),
                                       __FILE__, __LINE__);
  }
}

void GeoJoint::update(const double & w, const double & x, const double & y, const double & z, const Vector3d & t)
{
  if(jointType() != JointType::FLOATING)
  {
    RoboticsUtils::throw_runtime_error("This is not a floating-base joint", __FILE__, __LINE__);
  }

  Eigen::Quaterniond q = Eigen::Quaterniond(w, x, y, z);

  transform_.translation() = t;
  transform_.linear() = q.toRotationMatrix();
}

void GeoJoint::update(const double & x, const double & y, const double & z, const Vector3d & t)
{
  if(jointType() != JointType::FLOATING)
  {
    RoboticsUtils::throw_runtime_error("This is not a floating-base joint", __FILE__, __LINE__);
  }

  Vector3d r;
  r.x() = x;
  r.y() = y;
  r.z() = z;

  double n = r.norm();

  Eigen::AngleAxisd q = Eigen::AngleAxisd(n, r / n);

  transform_.translation() = t;
  transform_.linear() = q.toRotationMatrix();
}
void GeoJoint::updateDq(const Vector3d & v, const Vector3d & w)
{

  if(jointType() != JointType::FLOATING)
  {
    RoboticsUtils::throw_runtime_error("This is not a floating-base joint", __FILE__, __LINE__);
  }

  jointVel_.head(3) = v;
  jointVel_.tail(3) = w;
}

void GeoJoint::updateDq(const double & dq)
{
  if(dof() == 1)
  {
    if(checkLimits_dq_(dq))
    {
      dq_ = dq;
    }
  }
  else
  {
    RoboticsUtils::throw_runtime_error("Updatating dof " + std::to_string(dof()) + " joint " + name()
                                           + " with velocity " + std::to_string(dq),
                                       __FILE__, __LINE__);
  }
}

void GeoJoint::updateDDq(const double & ddq)
{
  // update(q, dq);
  if(dof() == 1)
  {

    ddq_ = ddq;
  }
  else
  {
    RoboticsUtils::throw_runtime_error("Updatating dof " + std::to_string(dof()) + " joint " + name()
                                           + " with acceleration " + std::to_string(ddq),
                                       __FILE__, __LINE__);
  }
}
void GeoJoint::updateTau(const double & tau)
{
  if(dof() == 1)
  {

    tau_ = tau;
  }
  else
  {
    RoboticsUtils::throw_runtime_error("Updatating dof " + std::to_string(dof()) + " joint " + name() + " with torque"
                                           + std::to_string(tau),
                                       __FILE__, __LINE__);
  }
}

void GeoJoint::updateTransform_(const double & q)
{
  if(jointType() == JointType::FIXED)
  {
    RoboticsUtils::throw_runtime_error("Can not update transform for JointType: FIXED", __FILE__, __LINE__);
  }
  // Eq.~2.45 of Murray et al. 1994
  incrementalTransform_ = expMap(tc_, q);

  // relativeTransform_ = incrementalTransform_ * initialRelativeTransform(); // This is not the correct way for
  // computing relative transform. The tc_ is not aligned to the local frame.

  // Update the body Jacobian twist coordinate, which only depends on joint position and the initial inertial frame
  // transform.
  const Iso3d & btf = incrementalTransform() * initialTransform();
  const Vector6d & Si = metaAdgInv(btf, twistCoordinate()).vector();
  // RoboticsUtils::quickHL("Joint: ", name(), ", position: ", q_);
  // RoboticsUtils::quickPrint("btf:", btf.matrix());
  // RoboticsUtils::quickPrint("twist Coordinate:", twistCoordinate().vector());
  // RoboticsUtils::quickPrint("Si:", Si);
  setBodyTC(Si);
}

void GeoJoint::computeLocalJacobian_()
{
  sXi_ = tc_.vector();
  // Apply the spatial frame to body frame transform:
  // bXi_ = btc_.vector();
}

void GeoJoint::iniTransform_(const Iso3d & offset)
{
  // 1. Set the initial relative transform:
  initialRelativeTransform_ = ujPtr_->transform.toMatrix();

  // 2. Update the relative transform
  updateTransform_(0.0);
}

TwistCoordinate GeoJoint::computeTwistCoordinate(const Iso3d & tf) const
{

  TwistCoordinate output;

  if(jointType() == JointType::FIXED)
  {
    return output;
  }
  // (1) Convert the point and the axis into the reference frame:

  // Vector3d point = tf * ujPtr_->transform.position;
  // Use the origin;
  Vector3d point = tf * Vector3d::Zero();
  Vector3d axis = tf.rotation() * ujPtr_->axis;

  // (2) compute the twist coordinate:

  switch(jointType())
  {
    case JointType::REVOLUTE:
    {
      // Use the origin as the point: q
      output.revolute_reset(point, axis);
      break;
    }
    case JointType::PRISMATIC:
    {
      output.prismatic_reset(axis);
      break;
    }
    default:
    {
      // clang-format off
      RoboticsUtils::throw_runtime_error("The joint type: " + ToString(ujPtr_->type) + " is not defined.", __FILE__, __LINE__);
      // clang-format on
    }
  }

  return output;
}

// void GeoJoint::iniTC_()
//{
//  tc_ = computeTwistCoordinate(initialRelativeTransform());
//  /*
//  switch(ujPtr_->type)
//  {
//    case JointType::REVOLUTE:
//    {
//      // Use the origin as the point: q
//      tc_.revolute_reset(Vector3d::Zero(), ujPtr_->axis);
//      // tc_.revolute_reset(ujPtr_->transform.position, ujPtr_->axis);
//      break;
//    }
//    case JointType::PRISMATIC:
//    {
//      tc_.prismatic_reset(ujPtr_->axis);
//      break;
//    }
//    default:
//    {
//      RoboticsUtils::throw_runtime_error("The joint type: " + ToString(ujPtr_->type) + " is not defined.", __FILE__,
//                                         __LINE__);
//    }
//  }
//  */
//
//  // Compute the body frame twist using the adjoint transform:
//  // This computation is actually wrong, see P112 of Murray et al. 1994.
//  // metaAdgInv(initialRelativeTransform(), tc_, btc_);
//}
void GeoJoint::constructBasis_(const Vector3d & axis,
                               const bool & revolute,
                               Vector6d & S,
                               Eigen::Matrix<double, 6, 5> & T)
{

  auto setcorner = [](const bool & revolute, const int & idx, Eigen::Matrix<double, 6, 5> & T, Vector6d & S)
  {
    S.setZero();
    T.setZero();

    int offset = 0;

    if(revolute)
    {
      T.block<3, 3>(0, 0).setIdentity();
      offset += 3;
    }
    else
    {
      T.block<3, 3>(3, 2).setIdentity();
    }

    S(offset + idx) = 1;

    // T.block<3,2>(offset, offset)
    switch(idx)
    {
      case 0:
      {
        // T.block<3,2>(offset, offset) << 0, 0, 1, 0, 0, 1;
        T.block<3, 1>(offset, offset) = Vector3d::UnitY();
        T.block<3, 1>(offset, offset + 1) = Vector3d::UnitZ();
        break;
      }
      case 1:
      {
        T.block<3, 1>(offset, offset) = Vector3d::UnitX();
        T.block<3, 1>(offset, offset + 1) = Vector3d::UnitZ();
        // T.block<3,2>(offset, offset) << 1, 0, 0, 0, 0, 1;
        break;
      }
      case 2:
      {
        T.block<3, 1>(offset, offset) = Vector3d::UnitX();
        T.block<3, 1>(offset, offset + 1) = Vector3d::UnitY();
        // T.block<3,2>(offset, offset) << 0, 0, 1, 0, 0, 1;
        break;
      }
      default:
        RoboticsUtils::throw_runtime_error("idx: " + std::to_string(idx) + " is not knwon! ", __FILE__, __LINE__);
    }
  };

  auto countNonzeroElement = [](Vector3d const & input) -> int
  {
    // input(0)  =1.0;
    int count = 0;
    int output = 0;

    for(int idx = 0; idx < input.size(); idx++)
    {
      if(input(idx) != 0)
      {
        count++;
        output = idx;
      }
    }

    if(count != 1)
    {
      RoboticsUtils::throw_runtime_error("Axis: " + toString(input) + " does not have only one non-zero element! ",
                                         __FILE__, __LINE__);
    }

    return output;
  };

  int idx = countNonzeroElement(axis);

  setcorner(revolute, idx, T, S);

  // Vector3d s;
  // Eigen::Matrix<double, 3, 2> corner;

  /*
  switch(idx)
  {
    case 0: // x
      s = Vector3d::UnitX();
      break;
    case 1: // y
      s = Vector3d::UnitY();
      break;
    case 2: // z
      s = Vector3d::UnitZ();
      break;
    default:
      RoboticsUtils::throw_runtime_error("Axis: " + ToString(axis) +" has more than one non-zero element! ", __FILE__,
  __LINE__); } // End of switch
  */
}

void GeoJoint::computeBasisVectors_()
{
  // For now, we only deal with axes that have one non-zero element.
  switch(ujPtr_->type)
  {
    case JointType::REVOLUTE:
    {
      dof_ = 1;
      constructBasis_(ujPtr_->axis, true, S_, T_);
      break;
    }
    case JointType::PRISMATIC:
    {
      dof_ = 1;
      constructBasis_(ujPtr_->axis, false, S_, T_);
      break;
    }
    case JointType::FLOATING:
    {
      dof_ = 6;
      break;
    }
    default:
    {
      // clang-format off
      RoboticsUtils::throw_runtime_error("The joint type: " + ToString(ujPtr_->type) + " is not defined for computing basis vectors.", __FILE__, __LINE__);
      // clang-format on
    }
  }

  Ta_ = childLink_->localInertiaTensor() * sBasis();
}

void GeoJoint::print() const
{
  RoboticsUtils::quickInfo("The joint " + name() + " has parent link:  " + parentLink_->name()
                           + " and child link: " + childLink_->name());
}
} // namespace GeoRobotics
