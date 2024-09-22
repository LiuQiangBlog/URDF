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

#include "GeometricRobotics/Computations.h"
#include "GeometricRobotics/Kinematics/GeoLink.h"
#include "GeometricRobotics/Utils.h"
#include "GeometricRobotics/urdf/Joint.h"
#include <RoboticsUtils/utils.h>
#include <unordered_map>

namespace GeoRobotics
{

class JointStateObserver : public std::enable_shared_from_this<JointStateObserver>
{
public:
  JointStateObserver(std::string name) : name_(name)
  {
    // RoboticsUtils::quickInfo("Created JointStateObserver: ", name_);
  }

  std::string name() const
  {
    return name_;
  }

  virtual void updateObserver() = 0;

protected:
  std::string name_ = "observer";
};

class GeoJoint : public std::enable_shared_from_this<GeoJoint>
{
public:
  std::shared_ptr<GeoJoint> getPtr()
  {
    return shared_from_this();
  }

  ~GeoJoint() {}

  /*! \brief  Update the relative transform, Jacobian according to the latest joint configuration.
   */
  void update(const double & q);
  void updateDq(const double & dq);
  void updateDDq(const double & ddq);
  void updateTau(const double & tau);

  /*! \brief  Update the floating-base link velocity, transform
   *  \param The first four specify the quaternion
   *  \param t is the translation \f$ p_{O,B}\f$
   */
  void update(const double & w, const double & x, const double & y, const double & z, const Vector3d & t);

  /*! \brief  Update the floating-base link velocity, transform
   *  \param The first four specify the quaternion
   *  \param t is the translation \f$ p_{O,B}\f$
   */
  void update(const double & x, const double & y, const double & z, const Vector3d & t);

  /*! \brief Update the floating-base velocity \f$ V^b_{OB} \f$ w.r.t. the inertial frame \f$ \mathcal{F}_{O}\f$.
   *  \param v the linear velocity  \f$ v^b_{OB} = R^\top_{OB} \dot{p}_{OB}\f$.
   *  \param w the angular velocity  \f$ w^b_{OB} = (R^\top_{OB} \dot{R}^\top_{OB})^\vee \f$.
   *
   */
  void updateDq(const Vector3d & v, const Vector3d & w);

  /*! \brief If the joint limits are fulfilled.
   */
  bool checkLimits_q_(const double & q) const;
  bool checkLimits_dq_(const double & dq) const;
  bool checkLimits_tau_(const double & tau) const;
  // bool checkLimits_ddq_(const double & ddq) const;

  /*! \brief returns the joint position
   */
  const double & q() const
  {
    return q_;
  }

  /*! \brief returns the joint velocity
   */
  const double & dq() const
  {
    return dq_;
  }

  /*! \brief returns the joint acceleration
   */
  const double & ddq() const
  {
    return ddq_;
  }

  /*! \brief returns the joint torque
   */
  const double & tau() const
  {
    return tau_;
  }
  /*! \brief set a feasible random joint configurations
   *  \param status 0: position, 1: position + velocity, 2: position, velocity and acc.
   *  \param re seed of the random number generator.
   *  \param safety_factor.
   */
  void setRandom(const size_t & status, std::default_random_engine & re, double safety_factor = 1.0);

  /*! \brief Integrate the joint position one step forward concerning the current joint velocities.
   */
  void velIntegration(const double & dt);

  /*! \brief returns the joint information from the URDF
   */
  const std::shared_ptr<Joint> & getURDFJoint() const
  {
    return ujPtr_;
  }

  /*! \brief Set the transform w.r.t. inertial frame: \f$ g_{oi} \f$
   */
  void setTransform(const Iso3d & tf)
  {
    transform_ = tf;
    notifyObservers();
  }

  /*! \brief Get the transform w.r.t. inertial frame: \f$ g_{oi} \f$
   */
  const Iso3d & transform() const
  {
    return transform_;
  }

  /*! \brief Get the transform w.r.t. inertial frame: \f$ g_{oi} \f$
   */
  Vector3d tfAngleAxis() const
  {
    Eigen::AngleAxisd aa(transform().linear());
    double angle = aa.angle();
    return aa.axis() * angle;
  }
  /*! \brief Get the current relative transform (w.r.t. current joint position), i.e., g(parent, child),  transform to
   * the the child frame w.r.t. the parent frame.
   */
  // const Iso3d & relativeTransform() const
  // {
  //   return relativeTransform_; // This is not the correct computation
  // }

  /*! \brief Get the incremental transform due to the current joint position
   */
  const Iso3d & incrementalTransform() const
  {
    return incrementalTransform_;
  }

  /*! \brief Get the initial relative transform between the child frame and the parent frame
   */
  const Iso3d & initialRelativeTransform() const
  {
    return initialRelativeTransform_;
  }

  /*! \brief Get the initial transform between the robot base_link (root link) frame and the joint frame \f$ g_{Oi}(0)
   * \f$
   */
  const Iso3d & initialTransform() const
  {
    return initialInertialTransform_;
  }

  /*! \brief set the initial transform between the inertial frame and the joint frame
   * \param tf
   */
  void setInitialTransform(const Iso3d & tf)
  {
    initialInertialTransform_ = tf;
  }
  /*! \brief Get the relative Jacobian: J(parent, child) in body-coordinates, i.e., in the child frame.
   */
  // const Vector6d & relativeBodyJacobian() const;

  /*! \brief Get the relative Jacobian: J(parent, child) in spatial-coordinates, i.e., in the parent frame.
   */
  // const Vector6d & relativeSpatialJacobian() const;

  /*! \brief Get the twist coordinate in the local frame
   */
  const TwistCoordinate & twistCoordinate() const
  {
    return tc_;
  }

  /*! \brief Given the reference frame transform (in the inertial frame), e.g., \f$ g_{O, j} \f$, it computes the twist
   * coordinate in the inertial frame.
   */
  TwistCoordinate computeTwistCoordinate(const Iso3d & tf) const;

  const std::string & name() const
  {
    return getURDFJoint()->name;
  }

  /*! \brief the orthonormal basis \f$ T_a \f$ of the actuation wrench subspace in the local joint frame.
   */
  const Vector6d & awBasis() const
  {
    return Ta_;
  }

  /*! \brief the orthonormal basis \f$ T \f$ of the constraint wrench subspace in the local joint frame.
   */
  const Eigen::Matrix<double, 6, 5> & cwBasis() const
  {
    return T_;
  }

  /*! \brief the orthonormal basis \f$ S \f$ of the motion subspace in the local joint frame.
   */
  Vector6d const & sBasis() const
  {
    return S_;
  }
  void printBasis() const
  {
    RoboticsUtils::quickInfo("The basis vectors of " + ToString(getURDFJoint()->type) + " joint " + name()
                             + " with axis " + toString(getURDFJoint()->axis) + " are:");
    printMatrix("Constraint wrench subspace basis T: ", T_);
    printMatrix("Motion subspace basis S: ", S_);
    printMatrix("Inertia in local frame M: ", childLink_->localInertiaTensor());

    printMatrix("Actuation wrench subspace basis Ta: ", Ta_);
  }

  bool isFloatingBase() const
  {
    return (jointType() == JointType::FLOATING);
  }
  std::shared_ptr<GeoLink> const & parentLink()
  {
    if(isFloatingBase())
    {
      RoboticsUtils::throw_runtime_error("Floating-base joint does not have a parent link", __FILE__, __LINE__);
    }
    return parentLink_;
  }

  std::shared_ptr<GeoLink> const & childLink()
  {
    return childLink_;
  }

  void setJointVel(Vector6d const & jv)
  {
    jointVel_ = jv;
  }

  /*! \brief returns the joint Velocity in body coordinate: \f$ V^b_{(i-1), i} \f$
   * If it is a floating-base joint, the returned velocity is:   \f$ V^b_{O, B} \f$
   */
  const Vector6d & jointVelocity() const
  {
    return jointVel_;
  }

  /*! \brief Set last column of the corresponding body velocity Jacobian.
   */
  void setBodyTC(const Vector6d & bTC)
  {
    xi_dagger_ = bTC;
  }

  /*!
   * \return last column of the corresponding body velocity Jacobian.
   */
  const Vector6d & bodyTC() const
  {
    return xi_dagger_;
  }

  void setTwistCoordinate(const TwistCoordinate & tc)
  {
    tc_ = tc;
  }

  /*!
   * \brief degrees of freedom of the joint
   */
  int dof()
  {
    return dof_;
  }
  void print() const;

  std::shared_ptr<Joint> createRootLinkParentJoint(const bool & floatingBase,
                                                   const std::shared_ptr<GeoLink> childLink,
                                                   const Iso3d & initialTf = Iso3d::Identity())
  {
    std::shared_ptr<Joint> fbJ;
    fbJ = std::make_shared<Joint>();

    fbJ->limits = std::make_shared<JointLimits>();
    fbJ->dynamics = std::make_shared<JointDynamics>();

    if(floatingBase)
    {
      fbJ->type = JointType::FLOATING;
      fbJ->name = "Floating-base joint";
      fbJ->resetFloatingBase();
      dof_ = 6;
    }
    else
    {
      fbJ->type = JointType::FIXED;
      fbJ->name = "Fixed-base joint";
      dof_ = 0;
    }

    fbJ->childLinkName = childLink->name();
    fbJ->transform.position << initialTf.translation().x(), initialTf.translation().y(), initialTf.translation().z();
    auto q = Eigen::Quaterniond(initialTf.rotation());
    fbJ->transform.rotation.setFromQuaternion(q.x(), q.y(), q.z(), q.w());
    return fbJ;
  }
  /*! \brief  The ordered-dependent articulated joints with DOF = 1; #1 joint index, #2 index in the inertia matrix
   * Namely, the articulated joints between the root link and this joint.
   */
  std::vector<std::pair<int, int>> const & articulatedJoints() const
  {
    return articulatedJoints_;
  }

  /*! \brief  The joint index (including the fixed-joint).
   * */
  int jointIndex() const
  {
    return index_.first;
  }
  /*! \brief  The InertiaMatrix's joint index, i.e., corresponding column in the inertia matri.
   * \return -1 if the joint is not an articulated joint.
   * */
  int inertiaMatrixJointIndex() const
  {
    return index_.second;
  }
  void setIndices(std::pair<int, int> const & indices)
  {
    index_ = indices;
  }

  JointType const & jointType() const
  {
    return ujPtr_->type;
  }

  static std::shared_ptr<GeoJoint> createGeoJoint(std::shared_ptr<Joint> const ujPtr,
                                                  std::shared_ptr<GeoLink> const childLink,
                                                  std::shared_ptr<GeoLink> const parentLink,
                                                  std::vector<std::pair<int, int>> const & articulatedJoints,
                                                  Iso3d const & initialInertialTransform = Iso3d::Identity(),
                                                  Iso3d const & offset = Iso3d::Identity())
  {
    // Not using std::make_shared<GeoJoint> because the c'tor is private.
    return std::shared_ptr<GeoJoint>(
        new GeoJoint(ujPtr, childLink, parentLink, articulatedJoints, initialInertialTransform, offset));
  }

  static std::shared_ptr<GeoJoint> createGeoJoint(std::shared_ptr<GeoLink> const childLink,
                                                  bool const & floatingBase,
                                                  Iso3d const & initialInertialTransform = Iso3d::Identity())
  {
    // Not using std::make_shared<GeoJoint> because the c'tor is private.
    return std::shared_ptr<GeoJoint>(new GeoJoint(childLink, floatingBase, initialInertialTransform));
  }
  bool hasObserver(std::string const & name) const
  {
    return jsObservers_.find(name) != jsObservers_.end();
  }

  std::shared_ptr<JointStateObserver> findObserver(std::string const & name) const
  {
    auto it = jsObservers_.find(name);

    if(it != jsObservers_.end())
    {
      return it->second;
    }

    RoboticsUtils::throw_runtime_error("The JointStateObserver: " + name + " does not exist.", __FILE__, __LINE__);
  }
  void printObservers() const
  {
    if(jsObservers_.size() > 0)
    {
      RoboticsUtils::quickHL("Joint: ", name(), " has the following JointStateObserver:");
      for(auto const & o : jsObservers_)
      {
        RoboticsUtils::quickInfo("JointStateObserver: ", o.second->name());
      }
    }
    else
    {
      RoboticsUtils::quickError("Joint: ", name(), " does not have attached any JointStateObserver.");
    }
  }

  /*! \brief Attach an observer of the joint state
   */
  void attachObserver(std::shared_ptr<JointStateObserver> ob)
  {
    jsObservers_[ob->name()] = ob;
  }

  void deleteObserver(std::string const & key)
  {
    auto it = jsObservers_.find(key);

    if(it != jsObservers_.end())
    {
      jsObservers_.erase(it);
    }
  }
  void notifyObservers()
  {
    for(auto & o : jsObservers_)
    {
      o.second->updateObserver();
    }
  }

private:
  /*! \brief This is the generic constructor
   */
  explicit GeoJoint(std::shared_ptr<Joint> const ujPtr,
                    std::shared_ptr<GeoLink> const childLink,
                    std::shared_ptr<GeoLink> const parentLink,
                    std::vector<std::pair<int, int>> const & articulatedJoints,
                    Iso3d const & initialInertialTransform = Iso3d::Identity(),
                    Iso3d const & offset = Iso3d::Identity())
  : ujPtr_(ujPtr), childLink_(childLink), parentLink_(parentLink), articulatedJoints_(articulatedJoints),
    initialInertialTransform_(initialInertialTransform)
  {

    if(ujPtr_ == nullptr)
    {
      RoboticsUtils::throw_runtime_error("The joint pointer is null", __FILE__, __LINE__);
    }

    if(parentLink_ == nullptr)
    {
      RoboticsUtils::throw_runtime_error("The parent link pointer is null", __FILE__, __LINE__);
    }

    // RoboticsUtils::quickInfo("Creating GeoJoint: ", name(), " with joint type: ", GeoRobotics::ToString(jointType()),
    //                          " child link: ", childLink->name(), " and parent link: ", parentLink->name());

    if(childLink_ == nullptr)
    {
      RoboticsUtils::throw_runtime_error("The child link pointer is null", __FILE__, __LINE__);
    }

    if(jointType() == JointType::FIXED)
    {
      initialRelativeTransform_ = ujPtr_->transform.toMatrix();
      dof_ = 0;
      return;
    }
    else
    {
      // 0. Set the initial relative transform:
      iniTransform_(offset);

      // 1. Set the twist coordinate.
      // iniTC_();
      if(dof() == 1) // If this is an articulated joint
      {
        setTwistCoordinate(computeTwistCoordinate(initialTransform()));
      }
      // 2. Employing the twist relative to the parent frame
      // computeLocalJacobian_();

      // 3. Compute the basis of the motion and wrench subspaces.
      computeBasisVectors_();
    }
  }
  /*! \brief This is a special constructor reserved for creating the floating-base joint of a root link.
   */
  explicit GeoJoint(const std::shared_ptr<GeoLink> childLink,
                    const bool & floatingBase,
                    const Iso3d & initialInertialTransform = Iso3d::Identity())
  : ujPtr_(createRootLinkParentJoint(floatingBase, childLink, initialInertialTransform)), childLink_(childLink)
  {
    RoboticsUtils::quickHL("Creating Floating-base geoJoint: ", name(),
                           " with joint type: ", GeoRobotics::ToString(jointType()),
                           " child link: ", childLink->name());

    if(childLink_ == nullptr)
    {
      RoboticsUtils::throw_runtime_error("The child link pointer is null", __FILE__, __LINE__);
    }

    // Creating the parent link: origin

  } // End of the special constructor

protected:
  // Twist coordinate in the inertial frame
  TwistCoordinate tc_{};

  // Twist coordinate in the link frame, i.e., xi_(i-1, i)
  // TwistCoordinate btc_;

  Vector6d sXi_ = Vector6d::Zero();
  // Vector6d bXi_ = Vector6d::Zero();

  Vector6d xi_dagger_ = Vector6d::Zero();

  /*! initialize the tc_ according to the joint information. Note that we define the TC w.r.t. the parent coordinate
   * frame
   */
  // void iniTC_();

  double q_ = 0.0;
  double dq_ = 0.0;
  double ddq_ = 0.0;
  double tau_ = 0.0;

  int dof_ = 1;

  const std::shared_ptr<Joint> ujPtr_;

  void iniTransform_(const Iso3d & offset);
  Iso3d transform_ = Iso3d::Identity();
  Iso3d initialRelativeTransform_ = Iso3d::Identity();
  Iso3d relativeTransform_ = Iso3d::Identity();
  Iso3d incrementalTransform_ = Iso3d::Identity();

  Vector6d jointVel_ = Vector6d::Zero();

  /*! \brief Update the relative transform w.r.t. q
   */
  void updateTransform_(const double & q);

  /*! \brief Update the local Jacobian w.r.t. q
   */
  void computeLocalJacobian_();

  void computeBasisVectors_();
  // The actuation wrench basis
  Vector6d Ta_ = Vector6d::Zero();

  // The motion subspace basis:
  Vector6d S_ = Vector6d::Zero();

  // The constraint wrench basis: S^perp:
  Eigen::Matrix<double, 6, 5> T_ = Eigen::Matrix<double, 6, 5>::Zero();

  const std::shared_ptr<GeoLink> childLink_;
  const std::shared_ptr<GeoLink> parentLink_;

  std::unordered_map<std::string, std::shared_ptr<JointStateObserver>> jsObservers_;

  std::vector<std::pair<int, int>> articulatedJoints_;
  std::pair<int, int> index_;

  Iso3d initialInertialTransform_ = Iso3d::Identity();

  void constructBasis_(const Vector3d & axis, const bool & revolute, Vector6d & S, Eigen::Matrix<double, 6, 5> & T);
};

} // namespace GeoRobotics
