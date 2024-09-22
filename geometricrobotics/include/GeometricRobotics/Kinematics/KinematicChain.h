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
#include "GeometricRobotics/Kinematics/GeoJoint.h"
#include "GeometricRobotics/Kinematics/GeoLink.h"
#include "GeometricRobotics/Utils.h"
#include "GeometricRobotics/urdf/Model.h"
#include <list>
#include <unordered_map>

namespace GeoRobotics
{

class KinematicChain;

class EndEffector
/*! \brief It is defined w.r.t. a parent joint according to a fixed transform.
 */
{
public:
  /*!
   * \param name End-effector name.
   * \param robot The host robot.
   * \param jointName The parent joint.
   * \param tf The relative transform w.r.t. the parent joint.
   */
  EndEffector(std::string const & name,
              std::string const & parentJointName,
              Iso3d const & tf,
              std::shared_ptr<KinematicChain> const robot);

  ~EndEffector() {}

  std::shared_ptr<GeoJoint> const & parentJoint() const;
  std::shared_ptr<GeoLink> const & parentLink() const;

  /*! \brief The transform w.r.t. the parent joint frame.
   */
  Iso3d const & relativeTransform() const
  {
    return relativeTf_;
  }

  /*! \brief The transform w.r.t. the inertial frame.
   */
  Iso3d const & transform() const
  {
    return tf_;
  }

  void update();

  MatrixXd const & spatialJacobian() const
  {
    return jac_;
  }
  MatrixXd const & bodyJacobian() const
  {
    return bodyJac_;
  }

  std::string const & name() const
  {
    return name_;
  }
  std::shared_ptr<KinematicChain> const & robot() const
  {
    return robot_;
  }
  long const & dof() const
  {
    return dof_;
  }

  Vector3d const & angularVelocity(bool bodyFrame = true) const
  {
    if(bodyFrame)
    {
      // Returns (R^T_ab * Rd_ab) V
      return aVelBody_;
    }
    else
    {
      // Returns (Rd_ab * R^T_ab) V
      return aVel_;
    }
  }
  Vector3d const & pointVelocity(bool bodyFrame = true) const
  {
    if(bodyFrame)
    {
      // Returns R^T_ab * pd_ab
      return pVelBody_;
    }
    else
    {
      // Returns pd_ab
      return pVel_;
    }
  }

protected:
  std::string name_{"emptyEndeffector"};

  std::string jointName_{"emptyParentJoint"};

  Iso3d relativeTf_{Iso3d::Identity()};
  Iso3d tf_{Iso3d::Identity()};

  std::shared_ptr<KinematicChain> robot_;

  MatrixXd jac_;
  MatrixXd bodyJac_;

  Vector3d pVel_{Vector3d::Zero()};
  Vector3d pVelBody_{Vector3d::Zero()};

  Vector3d aVel_{Vector3d::Zero()};
  Vector3d aVelBody_{Vector3d::Zero()};

  long dof_{0};

  Matrix6d twistTransform_{Matrix6d::Identity()};
};

class KinematicChain : public std::enable_shared_from_this<KinematicChain>
/*! \brief keeps track of the joints and links between a pair of root and leaf links
 *
 */
{
public:
  KinematicChain(std::string const & name,
                 std::string const & root,
                 std::string const & leaf,
                 std::vector<std::shared_ptr<GeoJoint>> const & joints,
                 std::vector<std::shared_ptr<GeoLink>> const & links)
  : chainName_(name), rootLinkName_(root), leafLinkName_(leaf), joints_(joints), links_(links)
  {
    initializeChain_();
  }
  ~KinematicChain() {}

  std::shared_ptr<KinematicChain> getPtr()
  {
    return shared_from_this();
  }

  /*! \brief The transform from Root Frame to Leaf frame w.r.t. the current joint positions.
   * It supports: (1) Inertial frame, (2) joint frames, (3) end-effector frames, and (4) com frame.
   */
  Iso3d const & transform(std::string const & name = "null") const;

  /*! \brief Computes the twist transform from the current frame to the target frame
   * \param currentFrame (a)
   * \param targetFrame (b)
   * \return The adjoint transform: \f$Adg^{-1}_{ab} == Adg_{ba} \f$
   */
  Matrix6d velTransform(std::string const & currenFrame, std::string const & targetFrame) const;

  /*! \brief Computes the derivative of the twist transform from the current frame to the target frame
   * \param currentFrame (a)
   * \param targetFrame (b)
   * \return The derivative of the adjoint transform: \f$ \frac{\partial{Adg^{-1}_{ab}}}{dt} =  Adg^{-1}_{ab}
   * \hat{V}^b_{ba}\f$
   */
  Matrix6d velTransformDerivative(std::string const & currenFrame, std::string const & targetFrame) const;

  /*! \brief Computes the 6-dimensional body velocity \f$ V^{b}_{ab} \f$
   * \f$ V^{b}_{ab}  = V^{b}_{Ob} - Adg^{-1}_{a,b}V^{b}_{Oa}\f$
   * \param referenceFrame (a)
   * \param bodyFrame (b)
   * \return \f$ V^{b}_{ab}  = \begin{bmatrix} v^{b}_{ab} \\ w^{b}_{ab} \end{bmatrix}\f$
   */
  Vector6d computeBodyVel(std::string const & referenceFrame, std::string const & bodyFrame) const;

  /*! \brief Computes the 6-dimensional body velocity \f$ V^{b}_{Ob} \f$
   * \param bodyFrame
   * \return \f$ V^{b}_{Ob} = \begin{bmatrix} v^{b}_{Ob} \\ w^{b}_{Ob} \end{bmatrix}\f$
   */
  Vector6d inertialFrameBodyVel(std::string const & bodyFrame) const;

  /*! \brief Computes the inertia transformed from the current frame to the target frame
   * \param Inertia: inertia in the current frame
   * \param currentFrame (a)
   * \param targetFrame (b)
   * \return The adjoint transform: twistTransform(b,a)^T * inertia * twistTransform(b,a)
   */
  Matrix6d inertiaTransform(Matrix6d const & inertia,
                            std::string const & currenFrame,
                            std::string const & targetFrame) const;

  /*! \brief Computes the derivative of the inertia transformed from the current frame to the target frame
   * \param Inertia: inertia in the current frame
   * \param currentFrame (a)
   * \param targetFrame (b)
   * \return The adjoint transform: twistTransformDerivative(b,a)^T * inertia * twistTransformDerivative(b,a)
   */
  Matrix6d inertiaTransformDerivative(Matrix6d const & inertia,
                                      std::string const & currenFrame,
                                      std::string const & targetFrame) const;

  /*! \brief Computes the wrench transform from the current frame to the target frame
   * \param currentFrame  (a)
   * \param targetFrame (b)
   * \return The adjoint transform: \f$(Ad^{-1} g_{ba})^T == (Adg_{ab})^T \f$
   */
  Matrix6d wrenchTransform(std::string const & currenFrame, std::string const & targetFrame) const;

  /*! \brief Computes the derivative of the wrench transform from the current frame to the target frame
   * \param currentFrame  (a)
   * \param targetFrame (b)
   * \return The adjoint transform: \f$\frac{\partial (Adg_{ab})^T}{dt} == (\hat{V}^b_{ab})^T Adg_{ab})^T\f$
   */
  Matrix6d wrenchTransformDerivative(std::string const & currenFrame, std::string const & targetFrame) const;

  /*! \brief The link momentum w.r.t. root frame and represented in the link body frame.
   * \param link name
   */
  Vector6d const & linkMomentum(std::string const & linkName) const
  {
    return linkMomentums_[static_cast<size_t>(linkNameToIndex(linkName))];
  }
  /*! \brief The link momentum w.r.t. root frame and represented in the link body frame.
   * \param link index
   */
  Vector6d const & linkMomentum(size_t const & idx) const
  {
    if(idx >= links_.size())
    {
      RoboticsUtils::throw_runtime_error("idx: " + std::to_string(idx) + "is out of the range.", __FILE__, __LINE__);
    }
    return linkMomentums_[idx];
  }

  /*! \brief The transform from Root Frame to Leaf frame w.r.t. the current joint positions.
   * \param index of the joint.
   */
  Iso3d const & transform(size_t const & idx) const
  {
    if(idx >= joints_.size())
    {
      RoboticsUtils::throw_runtime_error("idx: " + std::to_string(idx) + "is out of the range.", __FILE__, __LINE__);
    }

    return tfs_[idx];
  }

  /*! \brief The transform from Root Frame to Leaf frame when All the joint values at zero.
   */
  Iso3d const & initialTransform() const
  {
    return initialTfs_.back();
  }

  /*! \brief compute the spatial Jacobian, see eq.3.54 of Murray et al. 1994
   * \return Jacobian in the root frame of the kinematic chain.
   */
  Eigen::MatrixXd computeSpatialJacobian() const;

  Eigen::MatrixXd computeSpatialJacobianDerivative() const;
  Eigen::MatrixXd computeSpatialJacobianDerivative(std::string const & jointName) const;

  /*! \brief compute the spatial Jacobian of a joint, see eq.3.54 of Murray et al. 1994
   * \param fullSize If true, the Jacobian has the same number of columns of the robot DOF. Otherwise, the Jacobian only
   * has columns corresponding to the dependent joints. \return Jacobian in the root frame of the kinematic chain.
   */
  Eigen::MatrixXd computeSpatialJacobian(std::string const & jointName, bool const & fullSize = false) const;

  /*! \brief get the ready-made Jacobian
   */
  Eigen::MatrixXd const & spatialJacobian() const
  {
    return spatialJacobian_;
  }

  // Eigen::MatrixXd spatialJacobian(const std::string & jointName) const
  //{
  //  return spatialJacobian_.leftCols(jointNameToIndex(jointName) + 1);
  //}

  Eigen::MatrixXd const & spatialJacobian(std::string const & jointName, bool const & fullSize = false) const
  {
    if(fullSize)
    {
      return spatialJacsFull_[static_cast<size_t>(jointNameToIndex(jointName))];
    }
    else
    {
      return spatialJacs_[static_cast<size_t>(jointNameToIndex(jointName))];
    }
  }

  MatrixXd const & bodyJacobian() const
  {
    return bodyJacobian_;
  }
  MatrixXd const & bodyJacobian(std::string const & jointName, bool const & fullSize = false) const
  {
    if(fullSize)
    {
      return bodyJacsFull_[static_cast<size_t>(jointNameToIndex(jointName))];
    }
    else
    {
      return bodyJacs_[static_cast<size_t>(jointNameToIndex(jointName))];
    }
  }
  MatrixXd const & rVelJacobian(std::string const & jointName) const
  {
    return rVelJacs_[static_cast<size_t>(jointNameToIndex(jointName))];
  }
  // {
  //   const Iso3d & j_tf = transform(jointName);
  //   return metaAdgInvMatrix(j_tf) * spatialJacobian(jointName);
  // }

  /*! \brief compute the body Jacobian, see eq.3.55 of Murray et al. 1994
   * \return Jacobian in the leaf frame of the kinematic chain.
   */
  MatrixXd computeBodyJacobian() const;

  Eigen::MatrixXd computeBodyJacobianDerivative() const;
  Eigen::MatrixXd computeBodyJacobianDerivative(std::string const & jointName) const;
  Eigen::MatrixXd computeBodyJacobianDerivativeRF(std::string const & jointName) const;

  /*! \brief compute the spatial Jacobian derivative w.r.t. a specific joint.
   */
  Eigen::MatrixXd computeSpatialJacobianDerivative(std::string const & jointName, std::string const & deltaJoint) const
  {
    return computeSpatialJacobianDerivative(jointNameToIndex(jointName), jointNameToIndex(deltaJoint));
  }
  Eigen::MatrixXd computeSpatialJacobianDerivative(int const & idx, int const & deltaIdx) const;

  /*! \brief compute the body Jacobian derivative w.r.t. a specific joint.
   */
  Eigen::MatrixXd computeBodyJacobianDerivative(std::string const & jointName, std::string const & deltaJoint) const
  {
    return computeBodyJacobianDerivative(jointNameToIndex(jointName), jointNameToIndex(deltaJoint));
  }
  Eigen::MatrixXd computeBodyJacobianDerivative(int const & idx, int const & deltaIdx) const;

  /*! \brief compute the inertia tensor derivative w.r.t. a specific joint.
   */
  Eigen::MatrixXd computeMd(std::string const & jointName) const
  {
    return computeMd(jointNameToIndex(jointName));
  }
  Eigen::MatrixXd computeMd(int const & idx) const;

  MatrixXd computeBodyJacobian(std::string const & jointName, bool const & fullSize = false) const;
  /*! \brief Returns the degrees of freedom (number of articulated joints).
   */
  int dof() const
  {
    return static_cast<int>(joints_.size());
  }

  /*! \brief the orthonormal basis \f$ T_a \f$ of the actuation wrench subspace
   */
  Vector6d awBasis(std::string const & jointName) const;

  /*! \brief the orthonormal basis \f$ T \f$ of the constraint wrench subspace
   * \param jointName
   * \param spatialFrame If false, returns the bodyframe correspondance.
   */
  Eigen::Matrix<double, 6, 5> cwBasis(std::string const & jointName) const;

  /*! \brief the orthonormal basis \f$ S \f$ of the motion subspace in the root frame.
   */
  Vector6d mBasis(std::string const & jointName) const;
  void printKinematicChain() const;
  void printSpatialTwists() const;

  /*! choice 0: local COM frame, 1: local link frame;
   */
  void printInertiaTensors(int const & choice) const;

  /*! \brief Returns the Coriolis and centrifugal forces.
   */
  MatrixXd const & C() const
  {
    return CC_;
  }

  /*! \brief Returns the potential energy
   */
  double potentialEnergy() const
  {
    return Vp_;
  }

  /*! \brief Returns the kinetic energy
   */
  double kineticEnergy() const
  {
    return Ke_;
  }
  /*! \brief Returns the potential energy derivative: \f$ N = \frac{\partial V}{\partial\theta_i} \f$
   */
  VectorXd const & N() const
  {
    return NN_;
  }

  /*! \brief Returns partial derivatives: \f$ \frac{\partial M}{\partial \theta_i }\f$
   */
  std::vector<MatrixXd> const & Mdv() const
  {
    return Mdv_;
  }

  /*! \brief Derivative of the \f$ n \times n \f$ inertia matrix M: i.e., \f$ \sum^n_{i=1} ( \dot{J}_b^T M J_b + J_b^T M
   * \dot{J}_b) \f$, where we applied body coordinates computation. Note: the result is the same by either body or
   * spatial coordinates computation. Whereas, computation by body coordinates is easier.
   */
  MatrixXd const & Md() const
  {
    return Md_;
  }

  /*! \brief The \f$ n \times n \f$ inertia matrix M, i.e., \f$ \sum^n_i J_i^T M_i  J_i \f$
   */
  MatrixXd const & M() const
  {
    return M_;
  }

  /*! \brief Compute the nxn inertia matrix M in spatial coordinates, i.e., \f$ \sum_{i=1}^n J_i^T M_i J_i \f$
   */
  MatrixXd computeMs() const;

  void setJointPositions(std::vector<double> const & q);
  void setJointPositions(VectorXd const & q);

  void setJointVelocities(std::vector<double> const & dq);
  void setJointVelocities(VectorXd const & dq);

  /*! \brief read the joint position of a specific joint.
   */
  double readJointPosition(int const & i) const
  {
    return joint(i)->q();
  }

  void readJointPositions(std::vector<double> & q) const;
  void readJointVelocities(std::vector<double> & dq) const;

  /*! \brief read the joint velocity of a specific joint.
   */
  double readJointVelocity(int i) const
  {
    return joint(i)->dq();
  }

  void readJointAccelerations(std::vector<double> & ddq) const;

  void readJointPositions(VectorXd & q) const;
  VectorXd const & jointPositions() const
  {
    return q_;
  }
  void readJointVelocities(VectorXd & dq) const;
  VectorXd const & jointVelocities() const
  {
    return dq_;
  }
  VectorXd const & jointAccelerations() const
  {
    return ddq_;
  }
  void readJointAccelerations(VectorXd & ddq) const;

  void readJointTorques(std::vector<double> & tau) const;
  void readJointTorques(VectorXd & tau) const;

  /*! \brief set a feasible random joint configurations
   *  0: position, 1: position + velocity, 2: position, velocity and acc.
   */
  void setRandom(size_t status, std::default_random_engine & re);

  /*! \brief Integrate the joint position one step forward concerning the current joint velocities.
   */
  void velIntegration(double dt);

  /*! \brief compute the incremental transform due to a specitic joint. Measured in the reference frame coordinates.
   *  The relative transform due to a joint writes: incrementalTransform * initialTransform.
   */
  Iso3d incrementalTransform(std::string const & jointName) const;

  Iso3d initialTransform(std::string const & jointName) const
  {
    return initialTfs_[static_cast<size_t>(jointNameToIndex(jointName))];
  }

  /*! \brief returns the twist coordinates in the inertial frame
   */
  std::vector<TwistCoordinate> const & twistCoordinates() const
  {
    return tcs_;
  }

  std::vector<std::shared_ptr<GeoJoint>> const & joints() const
  {
    return joints_;
  }

  std::vector<std::shared_ptr<GeoLink>> const & links() const
  {
    return links_;
  }

  /*! \brief Randomly generates a name among the following categories: (1) Inertial frame, (2) joint frames, (3)
   * end-effector frames, and (4) com frame.
   */
  std::string randomFrameName() const;

  std::shared_ptr<GeoJoint> const & joint(std::string const & name) const
  {
    return joints_[static_cast<size_t>(jointNameToIndex(name))];
  }
  /*! \brief get the joint by index.
   */
  std::shared_ptr<GeoJoint> const & joint(int idx) const
  {
    return joints_[static_cast<size_t>(idx)];
  }
  std::shared_ptr<GeoLink> const & link(std::string const & name) const
  {
    return links_[static_cast<size_t>(linkNameToIndex(name))];
  }

  /*! \brief get the link by index.
   */
  std::shared_ptr<GeoLink> const & link(int idx) const
  {
    return links_[static_cast<size_t>(idx)];
  }
  /*
  const TwistCoordinate & bodyTwistCoordinate(const std::string & name) const
  {
    return btcs_[static_cast<size_t>(jointNameToIndex(name))];
  }
  */
  TwistCoordinate const & twistCoordinate(std::string const & name) const
  {
    return tcs_[static_cast<size_t>(jointNameToIndex(name))];
  }

  Vector3d const & com() const
  {
    return com_;
  }

  /*! \brief the com velocity: \f$ J_{com}\dot{q}\f$
   *  As the rotation to the com is identity, the com velocity is the same in body or spatial coordinates.
   */
  Vector3d const & comVelocity() const
  {
    return comVelocity_;
  }

  /*! \brief The link COM velocity: \f$ \dot{p} \f$
   */
  Vector3d const & linkComVelocity(std::string const & linkName) const
  {
    return linkComVels_[static_cast<size_t>(linkNameToIndex(linkName))];
  }

  /*!  \brief COM of a link in the spatial frame.
   *
   */
  Vector3d computeLinkCom(std::string const & linkName) const
  {
    auto l = link(linkName);
    return transform(l->parentJointName()) * l->localCOM();
  }

  double mass() const
  {
    return mass_;
  }

  /*! \brief Computes the momentum of a link
   *  \param bodyFrame, If true, returns the parent joint frame momentum, otherwise, returns the spatial frame (chain
   * root frame) momentum.
   */
  MatrixXd computeLinkMomentumJacobian(std::string const & linkName, bool bodyFrame = true) const;

  /*! Compute the centroidal Momentum matrix
   */
  MatrixXd computeCMM() const;

  /*! Compute the time derivative of the centroidal Momentum matrix
   */
  MatrixXd computeCMMDerivative() const;

  /*! Simplified the CMM computation through inertia transform
   */
  MatrixXd computeCMMTwo() const;

  MatrixXd const & cmm() const
  {
    return cmm_;
  }

  /*! Compute the relative velocity Jacobians
   * \param ci The centroidal inertia.
   * \param cmm The cmm matrix.
   */
  MatrixXd computeRelativeJacs(MatrixXd const & ci, MatrixXd const & cmm) const;

  /*! \brief Computes the average vel: \f$ v= I^{-1} h \f$
   *  All the quantities are expressed in the centroidal frame.
   */
  Vector6d computeAverageVelocity(Matrix6d const & centroidalInertia, MatrixXd const & cmm) const
  {
    return centroidalInertia.colPivHouseholderQr().solve(cmm * jointVelocities());
  }

  Vector6d const & averageVelocity() const
  {
    return aVel_;
  }

  /*! \brief The average velocity Jacobian
   */
  MatrixXd const & avJacobian() const
  {
    return aVelJac_;
  }

  /*! returns the transform from the inertial frame to the centroidal frame: g_{o, cmm}
   */
  Iso3d const & centroidalTransform() const
  {
    return cmmtf_;
  }

  Matrix6d computeCentroidalInertia() const;

  /*! \brief Compute the centroidal inertia according to Orin's paper.
   */
  Matrix6d computeCentroidalInertiaTwo() const;
  Matrix6d const & centroidalInertia() const
  {
    return ci_;
  }
  MatrixXd const & relativeVelocityInertia() const
  {
    return rCi_;
  }

  /*! returns the com Jacobian: comd = jac * dq;
   * Note that this is not a body velocity Jacoiban!
   */
  MatrixXd const & comBodyJac() const
  {
    return comJac_;
  }

  /*! \brief returns the Body Jacobian of a link COM
   */
  MatrixXd const & linkComBodyJac(std::string const & linkName) const
  {
    return linkComBodyJacs_[static_cast<size_t>(linkNameToIndex(linkName))];
  }

  /*! \brief returns \f$ \dot{p} \f$ in the spatial frame. \f$ \dot{p}\f$ w.r.t. to the joint frame is zero.
   * \param \f$ t \f$ denotes the translation w.r.t. the joint frame, i.e., \f$ g_{jp} \f$.
   * NOT tested.
   */
  Vector3d computePointVel(std::string const & jointName, Vector3d const & t) const;

  int jointNameToIndex(std::string const & jointName) const;
  std::string const & jointIndexToName(size_t const & idx) const;

  int linkNameToIndex(std::string const & linkName) const;
  std::string const & linkIndexToName(size_t const & idx) const;

  std::string const & name() const
  {
    return chainName_;
  }

  void createEndEffector(std::string const & name, std::string const & parentJointName, Iso3d const & tf);

  std::unordered_map<std::string, std::shared_ptr<EndEffector>> endEffectors() const
  {
    return endEffectors_;
  }

  std::string const & lastJoint() const
  {
    return joints_.back()->name();
  }

  std::shared_ptr<EndEffector> const endEffector(std::string const & name) const
  {
    // return metaGetEndEffector_(name).lock();
    return metaGetEndEffector_(name);
  }
  void removeEndEffector(std::string const & name);

  /*!  \brief Update the COM, COM Velocities, COM Jacobians, link COM Jacobians.
   */
  void updateCOM();

  /*!  \brief Check if a frame exists, and what is its type.
   * \return -1, if the frame does not exist; 1, if it is a joint name; 2, if it is an end-effector; 3, if it is the
   * com.
   */
  int frameType(const std::string & name) const;

  /*! \brief Forward Dynamics: computes the inertia matrix given the current joint configuration.
   * The implementation follows the CRB algorithm in Table 6.2 of Featherstone's book.
   *  \param M, the inertia matrix.
   */
  void MCalcCRB(MatrixXd & M);

  /*! \brief Inverse Dynamics: Computes the joint torque given: \f$ q, \dot{q}, \ddot{q} \f$.
   *  The implementation follows the RENA algorithm in Table 5.1 of Featherstone's book.
   *  It also computes and sets the link velocity, acceleration, and body force.
   *  \param q, the joint positions.
   *  \param dq, the joint velocities.
   *  \param ddq, the joint accelerations.
   */
  void IdCalc(VectorXd const & q, VectorXd const & dq, VectorXd const & ddq);

  /*! \brief Computes the vector \f$ C \dot{q} \f$ with the RNEA algorithm. We assume the following are given: \f$ q,
   * \dot{q}, \ddot{q} \f$.
   *
   * \param cc The Coriolis and centrifugal forces.
   */
  void CoriolisAndCentrifugalCalc(VectorXd & cc);

  std::string const & rootLinkName() const
  {
    return rootLinkName_;
  }
  std::string const & leafLinkName() const
  {
    return leafLinkName_;
  }

protected:
  std::string chainName_{"emptyChain"};
  ///< This is the default reference frame:
  std::string rootLinkName_{"emptyRoot"};
  std::string leafLinkName_{"emptyLeaf"};

  // Joint 1 sits between the "baseLink_" and "link 1"
  // std::shared_ptr<GeoLink> baselink_;

  ///< The joins between the root and leaf links.
  std::vector<std::shared_ptr<GeoJoint>> joints_;
  std::unordered_map<std::string, int> jointIndices_;

  /*!  \brief add the frame name to the hash table.
   * \param frameType: 1, a joint name; 2, end-effector; 3, com
   */
  void addFrame_(std::string const & name, int const & frameType);

  /*!  \brief remove the frame from the hash table.
   */
  void removeFrame_(std::string const & name);

  std::unordered_map<std::string, int> frames_;

  MatrixXd spatialJacobian_;
  MatrixXd spatialJacobianDot_;
  MatrixXd bodyJacobian_;
  MatrixXd bodyJacobianDot_;

  // MatrixXd Mb_;
  MatrixXd Md_;
  MatrixXd M_;

  MatrixXd CC_;
  VectorXd NN_;
  double Vp_{0.0};
  double Ke_{0.0};
  void updateInertia_CoriolisAndCentrifugalForces_();

  void updateCentroidalDynamics_();

  std::vector<MatrixXd> Mdv_;

  std::vector<std::shared_ptr<GeoLink>> links_;

  std::vector<MatrixXd> linkComBodyJacs_;
  std::vector<MatrixXd> bodyJacs_;
  std::vector<MatrixXd> bodyJacsFull_;
  std::vector<MatrixXd> spatialJacsFull_;
  std::vector<MatrixXd> spatialJacs_;

  std::vector<MatrixXd> rVelJacs_; ///< The relative velocity Jacobians.

  Matrix6d ci_{Matrix6d::Identity()}; ///< The centroidal inertia.
  MatrixXd cmm_; ///< The centroidal inertia.

  MatrixXd aVelJac_; ///< The average velocity Jacobian: avg_vel = aVelJac_ * dq;
  Vector6d aVel_{Vector6d::Zero()}; ///< The average velocity: av = ci^-1 * (cmm * dq);
  MatrixXd rCi_; ///< The generalized inertia due to the relative velocity: I' = J'^T * I * J';

  std::unordered_map<std::string, int> linkIndices_;

  ///< The transform from the root link frame to each joint frames:
  std::vector<Iso3d> tfs_;

  ///< Momentums in the body frame
  std::vector<Vector6d> linkMomentums_;

  VectorXd q_, dq_, ddq_, tau_;

  ///< Inertial frame twist coordinates:
  std::vector<TwistCoordinate> tcs_;

  ///< Body frame twist coordinates, e.g., the ith twist is in F_(i-1, i)
  // std::vector<TwistCoordinate> btcs_;

  ///< The vector of initial transforms:
  // std::vector<Iso3d> iniTfs_;

  /*! \brief Updates everything.
   */
  void updateChain_();

  /*! \brief Only update the joint velocity dependent components, e.g., the derivative of the inertia matrix.
   *  When only the joint velocities are changed, it is a light weight alternative to 'updateChain_()'
   */
  // void updateJointVelDependentComponents_();
  void updateTransform_();
  void updateInertiaTensor_();
  void initializeChain_();

  std::vector<Iso3d> initialTfs_;
  Iso3d transform_{Iso3d::Identity()};

  Iso3d inertial_transform_{Iso3d::Identity()}; ///< Inertial frame transform

  /*! \brief Computes the spatial Jacobian of a joint frame given the joint idx
   * The reference frame is the first joint frame of the chain
   */
  void computeSpatialJacobian_(size_t const & idx, Eigen::MatrixXd & output, bool const & fullSize = false) const;

  /*! \brief Computes the spatial Jacobian derivative of a joint frame given the joint idx
   * The reference frame is the first joint frame of the chain
   * \param jacobian, the ready-made spatial Jacobian, whose column is xi_prime
   * \param idx, of the interested joint
   * \param dq, joint velocities. Note, J_dot == 0, if the joint velocities are zero.
   * \param output, the computed jacobian derivative, which collectes xi_prime_dot
   */
  void computeSpatialJacobianDot_(MatrixXd const & jacobian,
                                  size_t const & idx,
                                  VectorXd const & dq,
                                  Eigen::MatrixXd & output) const;

  /*! \brief Computes the spatial Jacobian derivative of a joint frame given the joint idx
   * The reference frame is the first joint frame of the chain
   * \param jacobian, the ready-made body Jacobian, whose column is xi_prime
   * \param idx, of the interested joint
   * \param dq, joint velocities. Note, J_dot == 0, if the joint velocities are zero.
   * \param output, the computed jacobian derivative, which collectes xi_prime_dot
   */
  void computeBodyJacobianDot_(MatrixXd const & jacobian,
                               size_t const & idx,
                               VectorXd const & dq,
                               Eigen::MatrixXd & output) const;

  /*! \brief A meta step for computing the spatial Jacobian derivative: derivative of a transformed-twist-coordinate
   * employing the 6D cross product.
   */
  void tc_prime_dot_(MatrixXd const & jacobian, size_t const & idx, VectorXd const & dq, Vector6d & output) const;

  /*! \brief A meta step for computing the body Jacobian derivative.
   * Derivative of a tc_dagger employing the 6D cross product.
   */
  void tc_dagger_dot_(MatrixXd const & jacobian,
                      size_t const & idx,
                      int const & localDof,
                      VectorXd const & dq,
                      Vector6d & output) const;

  /*! \brief Computes the body Jacobian of a joint frame given the joint idx
   * w.r.t. the first joint frame of the chain.
   */
  void computeBodyJacobian_(size_t const & idx, Eigen::MatrixXd & output, bool const & fullSize = false) const;

  void updateJacobians_();

  void initializeVars_();
  Vector3d com_{Vector3d::Zero()};
  Vector3d comVelocity_{Vector3d::Zero()};
  std::vector<Vector3d> linkComVels_;
  MatrixXd comJac_;
  Iso3d cmmtf_{Iso3d::Identity()};

  double mass_{0.0};

  MatrixXd computeLinkComBodyJac_(std::string const & linkName) const;

  std::shared_ptr<EndEffector> metaGetEndEffector_(std::string const & name) const;
  std::unordered_map<std::string, std::shared_ptr<EndEffector>> endEffectors_;

  void setJacFullSize_(MatrixXd & jac, int const & jDof, bool const & fullSize) const
  {
    if(fullSize)
    {
      jac.resize(6, dof());
    }
    else
    {
      jac.resize(6, jDof);
    }
  }
};
} // namespace GeoRobotics
