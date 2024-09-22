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
#include "GeometricRobotics/urdf/Link.h"

namespace GeoRobotics
{
class GeoLink : public std::enable_shared_from_this<GeoLink>
{
public:
  std::shared_ptr<GeoLink> getPtr()
  {
    return shared_from_this();
  }
  ~GeoLink() {}

  /*!  \brief link information from the URDF model
   */
  const std::shared_ptr<Link> & urdfLink() const
  {
    return mLink_;
  }

  /*!  \brief compute the inertia matrix w.r.t. a reference frame
   *  \param transform: the link i's pose w.r.t. to the reference frame o, i.e., \f$ g_{oi} \f$.
   */
  Matrix6d spatialInertia(const Iso3d & transform);

  /*!  \brief get the diagonal generalized inertia matrix sits at the COM.
   */
  const Matrix6d & comInertiaTensor() const
  {
    return comInertiaTensor_;
  }

  /*!  \brief get the generalized inertia matrix w.r.t. to the link local frame
   */
  const Matrix6d & localInertiaTensor() const
  {
    return localInertiaTensor_;
  }

  /*!  \brief COM location in the body frame.
   */
  double mass() const
  {
    return mLink_->metaInertia->mass;
  }
  /*!  \brief COM location in the body frame.
   */
  const Vector3d & localCOM() const
  {
    return mLink_->metaInertia->origin.position;
  }

  /*!  \brief COM pose in the local frame
   */
  const Iso3d & localCOMTransform() const
  {
    return localCOMtransform_;
  }

  void printInertiaTensors(const int & choice) const;
  const std::string & parentJointName() const
  {
    return mLink_->parentJoint()->name;
  }

  const std::string & name() const
  {
    return mLink_->name;
  }

  /*! \brief Set the resultant wrench in the local frame.
   */
  void setWrench(const Vector6d & w)
  {
    wrench_ = w;
  }

  /*! \brief Compute the resultant wrench in the local frame given the link acc and vel in its body coordinates, see
   * RNEA algorithm in Table~5.1 of Featherstone's book. Notice: we negelect the external force term.
   */
  void calcWrench();

  /*! \brief Read wrench in the local frame.
   */
  const Vector6d & wrench() const
  {
    return wrench_;
  }

  void setBodyVel(const Vector6d & bv)
  {
    vel_ = bv;
  }

  /*! \brief read the body velocity: \f$ V^b_{Oi} \f$
   */
  const Vector6d & bodyVelocity() const
  {
    return vel_;
  }

  void setBodyAcc(const Vector6d & ba)
  {
    acc_ = ba;
  }
  /*! \brief read the body acceleration: \f$ a^b_{Oi} \f$
   */
  const Vector6d & bodyAcceleration() const
  {
    return acc_;
  }

  static std::shared_ptr<GeoLink> createGeoLink(const std::shared_ptr<Link> link)
  {
    // Not using std::make_shared<GeoLink> because the c'tor is private.
    return std::shared_ptr<GeoLink>(new GeoLink(link));
  }

private:
  explicit GeoLink(const std::shared_ptr<Link> link) : mLink_(link)
  {
    // RoboticsUtils::quickInfo("Creating GeoLink: " + urdfLink()->name);
    if(urdfLink()->metaInertia == nullptr)
    {
      RoboticsUtils::quickError("Link " + urdfLink()->name + " has empty inertia entries. We applied all zeros.");

      MetaInertia emptyInertia;
      initializeInertiaTensors_(emptyInertia);
    }
    else
    {
      initializeInertiaTensors_(*mLink_->metaInertia);
    }
    // RoboticsUtils::quickInfo("Created GeoLink: " + urdfLink()->name);
  }

protected:
  const std::shared_ptr<Link> mLink_;

  void initializeInertiaTensors_(const MetaInertia & metaInertia);

  Matrix6d comInertiaTensor_ = Matrix6d::Identity();
  Matrix6d localInertiaTensor_ = Matrix6d::Identity();
  Iso3d localCOMtransform_ = Iso3d::Identity();

  Vector6d wrench_ = Vector6d::Zero();
  Vector6d vel_ = Vector6d::Zero();
  Vector6d acc_ = Vector6d::Zero();

}; // end of class

} // namespace GeoRobotics
