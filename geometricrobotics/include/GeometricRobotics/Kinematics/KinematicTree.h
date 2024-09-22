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

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#include "GeometricRobotics/Computations.h"
#include "GeometricRobotics/Kinematics/GeoFrame.h"
#include "GeometricRobotics/Kinematics/GeoJoint.h"
#include "GeometricRobotics/Kinematics/GeoLink.h"
#include "GeometricRobotics/Kinematics/KinematicChain.h"
#include "GeometricRobotics/Utils.h"
#include "GeometricRobotics/urdf/Model.h"
#include <RoboticsUtils/utils.h>
#include <fstream>
#include <list>
#include <unordered_map>
//#include <unordered_set>
#include <map>

namespace GeoRobotics
{

class KinematicTree;

class Surface : public std::enable_shared_from_this<Surface>

/*! \brief A surface is defined  w.r.t. a fixed transform from a parent joint.
 * Employing the joints from the floating-base frame to a surface, Surface class can compute the equivalent CRB inertial
 * properties at a contact application point.
 */
{
public:
  std::shared_ptr<Surface> getPtr()
  {
    return shared_from_this();
  }

  ~Surface() {}

  std::shared_ptr<GeoJoint> const parentJoint() const;
  std::shared_ptr<GeoLink> const parentLink() const;

  /*! \brief The transform w.r.t. the parent joint frame.
   */
  Iso3d const & relativeTransform() const
  {
    return relativeTf_;
  }

  std::string const & parentJointName() const
  {
    return parentJointName_;
  }

  VectorXd const & jointVelocities() const
  {
    return dq_;
  }

  /*! \brief The transform w.r.t. the inertial frame.
   */
  Iso3d const & transform() const
  {
    return tf_;
  }

  void update();

  MatrixXd const & bodyJacobian() const
  {
    return bodyJac_;
  }

  std::string const & name() const
  {
    return name_;
  }
  std::shared_ptr<KinematicTree const> const robot() const
  {
    return robot_;
  }

  /*! \brief Degrees of freedom (including the FB joints if there are).
   */
  int const & dof() const
  {
    return dof_;
  }
  /*! \brief Actuated joints' Degrees of freedom
   */
  int const & aDof() const
  {
    return aDof_;
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
  /*! \brief Center of mass of the kinematic chain in the inertial frame \f$ \mathcal{F}_O \f$
   */
  Vector3d const & com() const
  {
    return com_;
  }
  /*! \brief Transform (of the kinematic chain) from the inertial frame to the centroidal frame: \f$ g_{o, cmm} \f$
   */
  Iso3d const & centroidalTransform() const
  {
    return cmmtf_;
  }

  /*! \brief Indices of the dependent joints (including the floating-base joint if the robot is a floating-base robot).
   */
  std::deque<int> const & joints() const
  {
    return joints_;
  }

  /*! \brief weight of the kinemataic chain.
   */
  double mass() const
  {
    return weight_;
  }

  /*! \brief Centroidal inertia
   */
  Matrix6d const & centroidalInertia() const
  {
    return ci_;
  }
  /*! \brief Average velocity \f$ V = I_{CRB}^{-1}h\f$
   */
  Vector6d const & averageVelocity() const
  {
    return av_;
  }
  /*! \brief Centroidal momentum in the inertial frame \f$ h_{O,c}\f$
   */
  Vector6d const & cm() const
  {
    return cm_;
  }
  /*! \brief The centroidal momentum matrix
   */
  MatrixXd const & cmm() const
  {
    return cmm_;
  }

  /*! \brief The robot's contact surface
   * \param name The surface name
   * \param parentJointName The parent joint's name
   * \param tf The relative transform from the parent joint to the surface
   * \param robot The host kinematic tree
   */
  static std::shared_ptr<Surface> createSurface(std::string const & name,
                                                std::string const & parentJointName,
                                                Iso3d const & tf,
                                                std::shared_ptr<KinematicTree const> const robot)
  {
    // Not using std::make_shared<GeoJoint> because the c'tor is private.
    return std::shared_ptr<Surface>(new Surface(name, parentJointName, tf, robot));
  }

private:
  /*!
   * \param name Surface name.
   * \param robot The host robot.
   * \param jointName The parent joint.
   * \param tf The relative transform w.r.t. the parent joint.
   */
  Surface(std::string const & name,
          std::string const & parentJointName,
          Iso3d const & tf,
          std::shared_ptr<KinematicTree const> const robot);

protected:
  /*! \brief Update the centroidal frame, and the CRB inertia, and the average velocity.
   */
  void updateCentroidalFrame_();
  std::string name_{"emptySurface"};
  double weight_{0.0};
  Vector3d com_ = Vector3d::Zero();
  Iso3d cmmtf_ = Iso3d::Identity();

  MatrixXd cmm_;
  Matrix6d ci_ = Matrix6d::Zero();
  Vector6d av_ = Vector6d::Zero();
  Vector6d cm_ = Vector6d::Zero();
  VectorXd dq_;

  std::deque<int> joints_; ///< The dependent joints

  std::string parentJointName_{"empty"};

  Iso3d relativeTf_ = Iso3d::Identity();
  Iso3d tf_ = Iso3d::Identity();

  std::shared_ptr<KinematicTree const> robot_;

  MatrixXd bodyJac_;

  Vector3d pVel_ = Vector3d::Zero();
  Vector3d pVelBody_ = Vector3d::Zero();

  Vector3d aVel_ = Vector3d::Zero();
  Vector3d aVelBody_ = Vector3d::Zero();

  int dof_ = 0, aDof_ = 0;

  Matrix6d twistTransform_ = Matrix6d::Zero();
};

///< A step means go to a link via a joint. The first element corresponds to the child link, and the second to the joint.
typedef std::pair<std::string, std::string> kStep;

typedef std::list<kStep> kPath;

// clang-format-off
typedef boost::adjacency_list<boost::vecS,
                              boost::vecS,
                              boost::directedS,
                              boost::property<boost::vertex_color_t, boost::default_color_type>,
                              boost::property<boost::edge_weight_t, int>,
                              boost::property<boost::edge_name_t, std::string>>
    BoostGraph;
// clang-format-on

inline bool operator==(kPath const & lhs, kPath const & rhs)
{

  if(lhs.size() != rhs.size())
  {
    return false;
  }

  for(auto il = lhs.begin(), ir = rhs.begin(); il != lhs.end(); il++, ir++)
  {
    if(*il != *ir)
    {
      return false;
    }
  }
  return true;
}

struct SpanningTree
/*! \brief a spanning tree rooted from the floating-base link.
 * The spanning tree rooted from the base link, e.g., the floating-base link. Depending on the connectivity, there might
 * exists several Subgraph from the base link.
 */
{
  std::string const & rootNode() const
  {
    return rootNode_;
  }
  std::unordered_map<std::string, int>
      nodes; ///< Ordered map of the link names. The integer is the number (index) of the link
  std::unordered_map<std::string, int>
      edges; ///< Ordered map of the joint names. The integer is the number (index) of the link
  std::unordered_map<std::string, kStep>
      steps; ///< edge.first: joint name, edge.second.first: child-link name, edge.second.second: joint name.
  void createGeoLink(std::shared_ptr<Link> const newLink,
                     int const & idx,
                     std::vector<std::shared_ptr<GeoLink>> & links);
  void createGeoJoint(std::vector<std::shared_ptr<GeoLink>> const & links,
                      int const & idx,
                      std::shared_ptr<Joint> const urdfJoint,
                      std::vector<std::shared_ptr<GeoJoint>> & joints,
                      std::vector<std::pair<int, int>> & articulatedJoints,
                      std::vector<std::pair<int, int>> & temp_articulatedJoints,
                      Iso3d & iniTF);
  void insertNode(std::string const & newNode, int & idx)
  {
    // Insert the nodes
    nodes.insert(std::make_pair(newNode, idx));
    // increment the idx
    idx++;
  }
  void insertEdge(kStep const & newEdge,
                  int const & idx,
                  std::shared_ptr<Link> const newLink,
                  std::shared_ptr<Joint> const urdfJoint,
                  std::vector<std::shared_ptr<GeoJoint>> & joints,
                  std::vector<std::pair<int, int>> & articulatedJoints,
                  std::vector<std::pair<int, int>> & temp_articulatedJoints,
                  std::vector<std::shared_ptr<GeoLink>> & links,
                  Iso3d & iniTF)
  {
    // Insert the edge
    steps.insert(std::make_pair(newEdge.second, newEdge));
    // Insert the joint index
    edges.insert(std::make_pair(newEdge.second, idx));

    // Create the child GeoLink
    createGeoLink(newLink, idx, links);

    // Increment the initial transform
    auto currentTF = urdfJoint->transform.toMatrix();
    iniTF = iniTF * currentTF;

    // Create the joint
    createGeoJoint(links, idx, urdfJoint, joints, articulatedJoints, temp_articulatedJoints, iniTF);
  }

  bool hasNode(std::string const & node)
  {
    if(nodes.find(node) == nodes.end())
      return false;
    else
      return true;
  }

  void reset(std::string const & newRootNode)
  {
    nodes.clear();
    edges.clear();
    steps.clear();
    rootNode_ = newRootNode;
    int idx = 0;
    insertNode(rootNode(), idx);
  }

  void print() const
  {
    RoboticsUtils::quickHL("The spanning tree rooted at link: ", rootNode(), " has: ", nodes.size(), " links, and ",
                           steps.size(), " joints.");
  }

private:
  std::string rootNode_{"Null"};
};

class Graph
/*! \brief Un-directed graph implementation based on adjacency lists.
 */
{
public:
  explicit Graph(std::shared_ptr<Model> const & model,
                 std::string const & baseLinkName = "world",
                 bool const & floatingBase = false);

  explicit Graph() {}
  ~Graph() {}

  void addEdge(Link const & one, Link const & two, Joint const & joint);
  /*! \brief Add
   *
   */
  void addEdge(std::string const & nodeOne, std::string const & nodeTwo, std::string const & edgeName);
  /*! \brief Each joint corresponds to an (undirected) edge
   */
  void addEdge(std::shared_ptr<Joint> const & joint);

  /*! \brief Each link corresponds to a node
   */
  /*
  void addNode(const Link & one)
  {
    addNode(one.name);
  }
  void addNode(const std::string & name);
  */

  /*! \brief Mark the node as visited
   */
  void markVisited(Link const & link);

  /*! \brief Check if a node is visited.
   */
  bool visited(Link const & link);

  /*! \brief find the path between the start link and the goal link.
   */
  bool findPath(std::string const & start, std::string const & goal, kPath & path);

  void printPath(kPath const & path) const;
  void printGraph() const;

  /*! \brief
   * \return The spanning three rooted at the base and constructed by DFS search.
   */
  SpanningTree const & spanningTree() const
  {
    return sTree_;
  }

  /*! \brief Constructs the spanning tree according to the current joints and links.
   * \param rootNode The fixed-base link name or the floating-base link name.
   * \param model The model parsed from the URDF file.
   */
  void makeTree(std::string const & rootNode, std::shared_ptr<Model> const & model, bool const & floatingBase = false);

  /*! \brief Return the set of ordered-dependent joints.
   *  The index increases from the root link to the interested link
   * \parem linkName
   */
  std::vector<int> const & odLinks(std::string const & linkName)
  {
    auto links = orderedDependentLinks_.find(linkName);

    if(links == orderedDependentLinks_.end())
    {
      RoboticsUtils::THROW_RUNTIME_ERROR("The link: " + linkName + " is not found by ");
    }

    return links->second;
  }
  std::vector<std::shared_ptr<GeoJoint>> & joints()
  {
    return joints_;
  }
  /*! \brief Degrees of freedom.
   * We only count the joints whose DOF is one.
   */
  int dof() const
  {
    return dof_;
  }

  /*! \brief  The articulated joints with DOF = 1; #1 joint index, #2 index in the inertia matrix
   */
  std::vector<std::pair<int, int>> const & articulatedJoints() const
  {
    return articulatedJoints_;
  }

  std::vector<std::shared_ptr<GeoLink>> & links()
  {
    return links_;
  }

  std::unordered_map<std::string, int> const & linkIndices() const
  {
    return spanningTree().nodes;
  }
  std::unordered_map<std::string, int> const & jointIndices() const
  {
    return spanningTree().edges;
  }

  std::unordered_map<std::string, kPath> const & adjacencyList() const
  {
    return adj_;
  }

protected:
  /*! \brief We do not share the ownership internally.
   */
  // const std::shared_ptr<Model> & getModel_()
  //{
  //  return modelPtr_;
  //}
  // std::shared_ptr<Model> modelPtr_;
  std::unordered_map<std::string, bool> visited_;
  // std::map<std::string, std::list<std::string>> adj_;
  std::unordered_map<std::string, std::vector<int>>
      orderedDependentLinks_; ///< The ordered dependent (parent/predecessor) links of a link
  std::unordered_map<std::string, kPath> adj_;

  bool hasNode_(std::string const & name) const;

  // The DFS for searching the path from a start link to a goal link.
  bool DFS_(kStep const & thisStep, std::string const & goal, kPath & path);

  SpanningTree sTree_;
  // The DFS for constructing the spanning tree.
  void DFSVisit_(std::shared_ptr<Model> const & model,
                 std::string const & currentNode,
                 std::vector<int> & odLinks,
                 Iso3d & iniTF,
                 int & idx,
                 SpanningTree & st,
                 std::vector<std::pair<int, int>> & articulatedJoints);

  std::vector<std::shared_ptr<GeoLink>> links_;
  std::vector<std::shared_ptr<GeoJoint>> joints_;

  std::vector<std::pair<int, int>>
      articulatedJoints_; ///<  The articulated joints with DOF = 1; #1 joint index, #2 index in the inertia matrix

  // std::map<std::string, int> linkIndices_;
  // std::map<std::string, int> jointIndices_;
  void initializeContainers_(std::string const & rootNode,
                             std::shared_ptr<Model> const & model,
                             bool const & floatingBase);
  int dof_{0};
};

class KinematicTreeObserver : public std::enable_shared_from_this<KinematicTreeObserver>
{
public:
  KinematicTreeObserver(std::string name) : name_(name)
  {
    RoboticsUtils::quickInfo("Created KinematicTreeObserver: ", name_);
  }

  std::string name() const
  {
    return name_;
  }

  virtual void updateKTreeObserver() = 0;

protected:
  std::string name_ = "observer";
};

class FixedFrame : public virtual GeoFrame
{
public:
  FixedFrame(std::string const & frameName, std::shared_ptr<KinematicTree> const robot) : GeoFrame(frameName, robot)
  {
    inertialTransform_.linear() = Matrix3d::Identity();
    inertialTransform_.translation() = Vector3d::Zero();

    RoboticsUtils::quickInfo("Created FixedFrame: " + frameName);
  }
  ~FixedFrame() {}

  /*! \brief Computes the transform w.r.t. the inertial frame \f$ g_{O, c} \f$
   * Inherited from GeoFrame
   */
  Iso3d const & transform() override
  {
    return inertialTransform_;
  }
  /*! \brief Computes the body velocity Jacobian
   * Inherited from GeoFrame
   */
  MatrixXd computeBodyJacobian(bool const & fullJacobian = true) const override;

  void setTransform(Iso3d const & newTf)
  {
    inertialTransform_ = newTf;
  }

  /*! \brief body velocity w.r.t. the inertial frame is zero.
   * Inherited from GeoFrame
   */
  Vector6d const inertialFrameBodyVel() override
  {
    return Vector6d::Zero();
  }

protected:
  Iso3d inertialTransform_{Iso3d::Identity()};
};

class ComFrame : public virtual KinematicTreeObserver, public virtual GeoFrame
{
public:
  ComFrame(std::string const & frameName, std::shared_ptr<KinematicTree> const robot);
  ~ComFrame() {}

  /*! \brief Computes the transform w.r.t. the inertial frame \f$ g_{O, c} \f$
   * Inherited from GeoFrame
   */
  Iso3d const & transform() override
  {
    return inertialTransform_;
  }
  /*! \brief Computes the body velocity Jacobian
   * Inherited from GeoFrame
   */
  MatrixXd computeBodyJacobian(bool const & fullJacobian = true) const override;

  /*! \brief Computes the body velocity Jacobian
   * Inherited from KinematicTreeObserver
   */
  void updateKTreeObserver() override;

  /*! \brief body velocity w.r.t. the inertial frame.
   * Inherited from GeoFrame
   */
  Vector6d const inertialFrameBodyVel() override;

protected:
  Iso3d inertialTransform_ = Iso3d::Identity();
};

class JointDependentFrame : public virtual JointStateObserver, public virtual GeoFrame
{
public:
  /*!
   * \param name Frame name.
   * \param parentFrameName the parent GeoJoint name.
   * \param tf The relative transform w.r.t. the parent joint.
   * \param robot The host robot.
   * Each joint, and link from the URDF file has a JointFrame
   */
  JointDependentFrame(std::string const & name,
                      std::string const & parentGeoJoint,
                      Iso3d const & tf,
                      std::shared_ptr<KinematicTree> const robot);

  ~JointDependentFrame() {}

  /*! \brief Get the offset transform from its parent articulated joint frame.
   */
  Iso3d const & offset() const
  {
    return offset_;
  }

  /*! \brief Compute the transform from the base frame to the current frame \f$ g_{o,i} \f$
   */
  Iso3d computeTransform() const;

  /*! \brief Compute the body Jacobian \f$ J^b_{o,i} \f$.
   * \param fullJacobian If yes, the Jacobian include columns correspond to all the joints. Otherwise, it only limits
   * to the dependent joints.
   */
  MatrixXd computeBodyJacobian(bool const & fullJacobian = true) const override;

  // std::vector<int> const & dependentJoints() const
  // {
  //   return dependentJoints_;
  // }

  /*! \brief returns the inertial frame transform
   */
  Iso3d const & transform() override
  {
    return inertialTransform_;
  }

  void updateObserver() override;

  std::string parentJointName() const
  {
    return parentJointName_;
  }

  /*! \brief body velocity w.r.t. the inertial frame.
   * Inherited from GeoFrame
   */
  Vector6d const inertialFrameBodyVel() override;

  // VectorXd const & jointVelocities() const
  // {
  //   return dq_;
  // }

  int dof() const
  {
    return dof_;
  }

  Vector3d computePointVelocity(Vector3d const & p) const;

  MatrixXd computePointJacobian(Vector3d const & p, bool const fullSize = false) const;

protected:
  // std::string name_{"emptyFrame"};

  // VectorXd dq_;

  int dof_ = 0;

  std::string parentJointName_{"emptyParent"};

  Iso3d offset_ = Iso3d::Identity();

  Iso3d inertialTransform_ = Iso3d::Identity();

  Matrix6d twistTransform_ = Matrix6d::Zero();

  // std::vector<int> dependentJoints_;

  // Compute the Jacobian:

}; // End of class JointDependentFrame

class KinematicTree : public std::enable_shared_from_this<KinematicTree>
/*! \brief We build it from an URDF model, and we need to select a root link (which could be a floating base)
 *
 */
{
public:
  KinematicTree(std::shared_ptr<Model> model,
                std::string const & rootLinkName = "world",
                std::string const & treeName = "robot",
                bool const & floatingBase = false);

  ~KinematicTree() {}

  /*! \brief Create the GeoFrames for all the joints and links
   */
  void initializeFrames();

  bool const & isFloatingBase() const
  {
    return floatingBase_;
  }
  /*!  \brief Joint space inertia matrix: \f$ M(q) \f$
   */
  MatrixXd const & JSIM() const
  {
    return JSIM_;
  }

  std::shared_ptr<KinematicTree> getPtr()
  {
    return shared_from_this();
  }

  /*! \brief returns the model from URDF parsing.
   * Pass by reference without sharing the ownership
   */
  std::shared_ptr<Model> const & model() const
  {
    return modelPtr_;
  }

  /*! \brief Degrees of freedom.
   * We only count the joints whose DOF is one.
   */
  int dof() const
  {
    return graph()->dof();
  }

  /*! \brief Get the kinematic graph (tree)
   */
  std::shared_ptr<Graph> const & graph() const
  {
    return kGraph_;
  }

  /*! \brief Computes the 6-dimensional body velocity \f$ V^{b}_{Ob} \f$
   * \param bodyFrame
   * \return \f$ V^{b}_{Ob} = \begin{bmatrix} v^{b}_{Ob} \\ w^{b}_{Ob} \end{bmatrix}\f$
   */
  Vector6d inertialFrameBodyVel(std::string const & bodyFrame) const;

  /*! \brief Computes the 6-dimensional body velocity \f$ V^{b}_{ab} \f$
   * \f$ V^{b}_{ab}  = V^{b}_{Ob} - Adg^{-1}_{a,b}V^{b}_{Oa}\f$
   * \param referenceFrame (a)
   * \param bodyFrame (b)
   * \return \f$ V^{b}_{ab}  = \begin{bmatrix} v^{b}_{ab} \\ w^{b}_{ab} \end{bmatrix}\f$
   */
  Vector6d computeBodyVel(std::string const & referenceFrame, std::string const & bodyFrame) const;

  /*! \brief Get the transform from the reference frame to the target frame, i.e., g_{ref, target}
   */
  Iso3d transform(std::string const & refrenceFrameName,
                  std::string const & targetFrameName,
                  bool const & initialTransform = false);

  /*! \brief Get the transform from the reference frame to the target frame, i.e., g_{ref, target}
   */
  Iso3d transformTwo(std::string const & refrenceFrameName, std::string const & targetFrameName);

  /*! \brief Add an edge on the tree (or graph)
   */
  // bool addJoint();

  /*! \brief Add an node on the tree (or graph)
   */
  // bool addLink();

  /*! \brief Randomly generates a frame's name
   */
  std::string randomFrameName() const;
  std::string randomJointName() const;
  std::string randomLinkName() const;

  std::shared_ptr<KinematicChain> const createKinematicChain(std::string const & chainName,
                                                             std::string const & root,
                                                             std::string const & leaf);

  /*! \brief  The articulated joints with DOF = 1; #1 joint index, #2 index in the inertia matrix
   */
  std::vector<std::pair<int, int>> const & articulatedJoints() const
  {
    return graph()->articulatedJoints();
  }

  std::vector<std::shared_ptr<GeoJoint>> const & joints() const
  {
    return graph()->joints();
  }

  std::vector<std::shared_ptr<GeoLink>> const & links() const
  {
    return graph()->links();
  }

  std::shared_ptr<GeoLink> const link(int const & idx) const
  {
    return links()[static_cast<size_t>(idx)];
  }
  std::shared_ptr<GeoLink> const link(std::string const & name) const
  {
    return links()[static_cast<size_t>(linkNameToIndex(name))];
  }

  std::shared_ptr<GeoJoint> const joint(std::string const & name) const
  {
    return joints()[static_cast<size_t>(jointNameToIndex(name))];
  }

  /*! \brief get the joint by index.
   */
  std::shared_ptr<GeoJoint> const joint(int const & idx) const
  {
    return joints()[static_cast<size_t>(idx)];
  }

  /*! \brief Number of all the joints including Fixed, Floating-base, prismatic, and etc.
   */
  size_t numberOfJoints() const
  {
    return joints().size();
  }

  /*! \brief Number of all the links
   */
  size_t numberOfLinks() const
  {
    return links().size();
  }

  bool hasArticulatedJoint(std::string const & jointName) const
  {
    if(jointIndices_().find(jointName) != jointIndices_().end())
    {
      return true;
    }
    else
      return false;
  }
  int jointNameToIndex(std::string const & jointName) const;
  /*! \brief find the inertiam matrix joint index.
   */
  int jointNameToIMIndex(std::string const & jointName) const;
  int linkNameToIndex(std::string const & linkName) const;
  std::string jointIndexToName(size_t const & idx) const;

  std::string linkIndexToName(size_t const & idx) const;

  /*! @return the transform from the inertial frame to the root link frame.
   */
  // void setTransform(const Iso3d & tf)
  //{
  //  baseTransform_ = tf;
  //}

  /*! @return the transform of the first joint, i.e., the transform from the inertial frame to the root link frame.
   */
  Iso3d const & transform() const
  {
    return joints()[0]->transform();
  }

  std::string const & name() const
  {
    return name_;
  }

  /*
  const Iso3d & initialTransform(const std::string & jointName) const
  {
    return initialTfs_[static_cast<size_t>(jointNameToIndex(jointName))];
  }
  */

  void printTree() const;

  /*! /brief Visualize the kinematic tree with boost::graph::graphviz
   * \return execution status of the system command std::system(command.c_str());
   */
  int visualize() const;

  /*! \brief Number the links according to the spanning tree of the kinematic graph according to Sec.~4.1.2 of
   * Featherstone.
   */
  // void numberTheLinksAndJoints_();

  /*! \brief If the joint is not yet indexed, index it incrementally.
   */
  // int initializeJointIdx_(const std::string & jointName, int & idx);

  /*! \brief the shared pointer to the parent link
   */
  std::shared_ptr<GeoLink> const parentLink(std::string const & linkName) const
  {
    if(linkName == rootLinkName())
    {
      return link(linkName);
    }
    return joint(link(linkName)->parentJointName())->parentLink();
  }

  /*! \brief Find the set of ordered joints.
   * \param set The vector of depedent joints. The index increases from the root link to the interested link
   * \param linkName
   */
  void odJoints(std::deque<int> & set, std::string const & linkName) const;

  /*! \brief Find the set of ordered articuted joints.
   * \param set The vector of depedent joints. The index increases from the root link to the interested link
   * \param linkName
   */
  void odArticulatedJoints(std::deque<int> & set, std::string const & linkName) const;
  /*! \brief Print the dependent articulated joints from  root link to the interested link.
   */
  void printOdArticulatedJoints(std::string const & linkName) const;

  const std::string rootLinkName() const
  {
    return graph()->spanningTree().rootNode();
  }

  /*! \brief set a feasible random joint configurations only for the articulated joints.
   *  0: position, 1: position + velocity, 2: position, velocity and acc.
   */
  void setRandom(size_t status, std::default_random_engine & re, double safety_factor = 1.0);

  // void update();

  /*! \brief Read the joint torque of: the articulated joints.
   *  \param tau
   */
  void readJointTorques(VectorXd & tau) const;

  /*! \brief Read the joint positions of: (1) the six floating-base joint, and (2) the articulated joints.
   *  \param q
   */
  void readJointPositions(VectorXd & q) const;

  /*! \brief Read the joint velocities of: (1) the six floating-base joint, and (2) the articulated joints.
   *  \param dq
   */
  void readJointVelocities(VectorXd & dq) const;

  /*! \brief Read the dependent joint velocities of a specific joint: (1) the six floating-base joint, and (2) the
   * dependent-articulated joints, including the joint itself. \param dq \param jointName
   */
  void readJointVelocities(VectorXd & dq, std::string const & jointName) const;

  /*! \brief Read the joint accelerations of: (1) the six floating-base joint, and (2) the articulated joints.
   *  \param ddq
   */
  void readJointAccelerations(VectorXd & ddq) const;

  /*! \brief Set the joint velocities of: (1) the six floating-base joint, and (2) the articulated joints.
   *  \param dq
   */
  void setJointVelocities(VectorXd const & dq);

  /*! \brief Set the joint accelerations of: (1) the six floating-base joint, and (2) the articulated joints.
   *  \param dq
   */
  void setJointAccelerations(VectorXd const & dq);

  /*! \brief Set the joint positions of: (1) the six floating-base joint, and (2) the articulated joints.
   *  \param q
   */
  void setJointPositions(VectorXd const & q);

  /*! \brief Forward Dynamics: computes the inertia matrix given the current joint configuration.
   * The implementation follows the CRB algorithm in Table 6.2 of Featherstone's book.
   *  \param M, the inertia matrix.
   */
  void MCalcCRB(MatrixXd & M);

  /*! \brief Inverse Dynamics: Computes the joint torque given: q, qd, and qdd.
   *  The implementation follows the RENA algorithm in Table 5.1 of Featherstone's book.
   *  It also computes and sets the link velocity, acceleration, and body force.
   *  \param q, the joint positions.
   *  \param dq, the joint positions.
   *  \param ddq, the joint positions.
   */
  void IdCalc(VectorXd const & q, VectorXd const & dq, VectorXd const & ddq);

  VectorXd const & jointPositions() const
  {
    return q_;
  }
  VectorXd const & jointVelocities() const
  {
    return dq_;
  }
  VectorXd const & jointAccelerations() const
  {
    return ddq_;
  }

  /*! \brief Computes the twist transform from the current frame to the target frame
   * It performs the computation: \f$ g_{12} = g^{-1}_{o2}g_{o2} \f$, where \f$ g_{o1}\f$ and \f$ g_{o2}\f$ are known.
   *
   * \param currentFrame (1)
   * \param targetFrame (2)
   * \return The adjoint transform: \f$Ad^{-1} g_{12} == Adg_{21} \f$
   */
  Matrix6d velTransform(std::string const & currenFrame, std::string const & targetFrame) const;

  /*! \brief Computes the derivative of the twist transform from the current frame to the target frame
   * \param currentFrame (a)
   * \param targetFrame (b)
   * \return The derivative of the adjoint transform: \f$ \frac{\partial{Adg^{-1}_{ab}}}{dt} =  Adg^{-1}_{ab}
   * \hat{V}^b_{ba}\f$
   */
  Matrix6d velTransformDerivative(std::string const & currenFrame, std::string const & targetFrame) const;

  /*! \brief Computes the inertia transformed from the current frame to the target frame
   * \param Inertia: inertia in the current frame
   * \param currentFrame (1)
   * \param targetFrame (2)
   * \return The adjoint transform: twistTransform(2,1)^T * inertia * twistTransform(2,1)
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
   * \param currentFrame  (1)
   * \param targetFrame (2)
   * \return The adjoint transform: \f$(Ad^{-1} g_{21})^T == (Adg_{12})^T \f$
   */
  Matrix6d wrenchTransform(std::string const & currenFrame, std::string const & targetFrame) const;

  /*! \brief Computes the derivative of the wrench transform from the current frame to the target frame
   * \param currentFrame  (a)
   * \param targetFrame (b)
   * \return The adjoint transform: \f$\frac{\partial (Adg_{ab})^T}{dt} == (\hat{V}^b_{ab})^T Adg_{ab})^T\f$
   */
  Matrix6d wrenchTransformDerivative(std::string const & currenFrame, std::string const & targetFrame) const;

  Vector3d const & com() const
  {
    return com_;
  }
  /*! \brief Transform from the inertial frame to the centroidal frame: \f$ g_{o, cmm} \f$
   */
  Iso3d const & centroidalTransform() const
  {
    return cmmtf_;
  }

  double mass() const
  {
    return mass_;
  }
  Matrix6d computeCentroidalInertia() const;

  bool hasFrame(std::string const & name) const
  {
    return frames_.find(name) != frames_.end();
  }

  std::shared_ptr<GeoFrame> frame(std::string const & name) const
  // std::shared_ptr<JointDependentFrame> frame(std::string const & name) const
  {
    auto it = frames_.find(name);

    if(it == frames_.end())
    {
      RoboticsUtils::THROW_RUNTIME_ERROR("The frame: " + name + " does not exist.");
    }

    return it->second;
  }

  /*! \brief Compute the body Jacobian \f$ J^b_{o,i} \f$ of a specific articulated joint.
   * \param jointName Name of the articulated joint.
   * \param fullJacobian If yes, the Jacobian include columns correspond to all the joints. Otherwise, it only limites
   * to the dependent joints.
   */
  MatrixXd computeBodyJacobian(std::string const & jointName, bool const & fullJacobian = true) const;

  /*! \brief Compute the body-Jacobian's derivative \f$ \dot{J}^b_{o,i} \f$.
   * \param jointName Name of the articulated joint.
   * \param fullJacobian If yes, the Jacobian's derivative include columns correspond to all the joints. Otherwise, it
   * only limits to the dependent joints.
   */
  MatrixXd computeBodyJacobianDerivative(std::string const & jointName, bool const & fullJacobian = true) const;

  /*! \brief Computes the vector \f$ C \dot{q} \f$ with the RNEA algorithm. We assume the following are given: \f$ q,
   * \dot{q}, \ddot{q} \f$.
   * \param cc the Coriolis, centrifugal forces and potential terms. According to Page.~183 of
   * Featherstone's book, we set \f$ a_0 = -g \f$ such that the resulting vector includes the graviational force.
   * Otherwise, we can set \f$ a_0 = 0 \f$.
   */
  void CCNCalc(VectorXd & cc);

  /*! -\brief Read the joint values and update the transform of each joint.
   */
  // void updateJointsAndTransform_();

  /*! \brief Computes the Jacobian for the com velocity \f$ \dot{c} = J_{c} \dot{q} \f$.
   * Note the difference: \f$ v^b_{O,c} = R^\top_{O,c} \dot{c} \f$
   */
  // void computeComJacobian(MatrixXd & comJac) const;

  /*! Compute the time derivative of the centroidal Momentum matrix
   */
  MatrixXd computeCMMDerivative() const;

  /*! Compute the centroidal Momentum matrix
   */
  MatrixXd computeCMM() const;

  // MatrixXd computeCMMTwo() const;

  /*! \brief Computes the Jacobian for the com velocity \f$ \dot{c} = J_{c} \dot{q} \f$.
   * Note the difference: \f$ v^b_{O,c} = R^\top_{O,c} \dot{c} \f$
   */
  MatrixXd computeComJac() const;

  /*! \brief compute the body Jacobian derivative w.r.t. a specific joint.
   * \param jointName
   * \param deltaJoint denotes the changing joint.
   */
  MatrixXd computeBodyJacobianDerivative(std::string const & jointName, std::string const deltaJoint) const;

  /*! \brief Attach an observer of the KinematicTree
   */
  void attachObserver(std::shared_ptr<KinematicTreeObserver> ob)
  {
    ktObservers_[ob->name()] = ob;
  }

  void deleteObserver(std::string const & key)
  {
    auto it = ktObservers_.find(key);

    if(it != ktObservers_.end())
    {
      ktObservers_.erase(it);
    }
  }
  void notifyObservers()
  {
    for(auto & o : ktObservers_)
    {
      o.second->updateKTreeObserver();
    }
  }

protected:
  void computeBodyJacobian_(std::string const & jointName, MatrixXd & jac, bool const fullSize = false) const;
  void computeBodyJacobianDerivative_(std::string const & jointName, MatrixXd & jac) const;
  void computeFullSizeBodyJacobianDerivative_(std::string const & jointName, MatrixXd & jac) const;

  /*! \brief A meta step to compute the body Jacobian derivative:
   * Derivative of a tc_dagger employing the 6D cross product.
   */
  void tc_dagger_dot_(MatrixXd const & jacobian,
                      size_t const & idx,
                      size_t const & startIdx,
                      size_t const & localDof,
                      VectorXd const & dq,
                      Vector6d & output) const;

  // void computeLinkComBodyJac_(MatrixXd & comJac, std::string const & linkName, bool const & fullJacobian = true)
  // const;

  std::shared_ptr<Model> modelPtr_;
  std::shared_ptr<Graph> kGraph_;

  // There might be less links than joints
  // std::map<std::string, std::shared_ptr<GeoLink>> links_;
  void readLinkNames_(std::vector<std::string> & names) const;
  // void readEdges_(std::vector<std::pair<int, int>> & edges) const;
  void addEdges_(std::map<BoostGraph::edge_descriptor, std::string> & ename, BoostGraph & g) const;

  // int diffNJNL_{0};

  std::unordered_map<std::string, int> const & linkIndices_() const
  {
    return graph()->linkIndices();
  }
  std::unordered_map<std::string, int> const & jointIndices_() const
  {
    return graph()->jointIndices();
  }
  // std::vector<Iso3d> initialTfs_;

  // std::vector<int> pred_; ///< Predecessor of each joint
  // std::vector<int> succ_; ///< Successor of each joint

  std::string name_{"robot"};
  bool floatingBase_{false};
  // Iso3d baseTransform_; ///< Transform from the inertial frame to the root link frame

  // std::map<std::string, std::shared_ptr<GeoLink>> links_;
  // std::map<std::string, std::shared_ptr<GeoJoint>> joints_;

  // std::shared_ptr<GeoLink> metaGetLink(const std::string & name) const;
  // std::shared_ptr<GeoJoint> metaGetJoint(const std::string & name) const;

  std::unordered_map<std::string, std::shared_ptr<KinematicChain>> chains_;

  /*! \brief Update the transforms of the articulated joints assuming the joint positions are already set.
   * This function is used whenever the joint positions are modified.
   */
  void updateTransform_();

  /*! \brief Updates ???
   * 1.
   * 2.
   * 3.
   */
  void updateTree_();

  /*! \brief Recursively updates the transform of each joint.
   */
  void transformDFSUpdate_(std::string const & currentNode,
                           std::string const & currentJoint,
                           Iso3d & iniTF,
                           std::unordered_set<std::string> & visited);

  void checkTransform_(std::unordered_set<std::string> & visited,
                       std::string const & currentNode,
                       std::string const & currentJoint,
                       Iso3d & iniTF);

  /*! \brief Check if j is an one-DOF articulated joint.
   */
  bool isArticulatedJoint_(int const & idx) const;

  /*! \brief Check if j is an one-DOF articulated joint.
   */
  bool isArticulatedJoint_(std::string const & jName) const;

  VectorXd q_, dq_, ddq_, tau_;
  // bGraph bGraph_;
  MatrixXd JSIM_;

  std::vector<Matrix6d> const & get_crb_links() const
  {
    return crb_links_;
  }
  std::vector<Matrix6d> crb_links_;
  Vector3d com_ = Vector3d::Zero();

  Iso3d cmmtf_ = Iso3d::Identity();
  double mass_ = 0.0;

  // std::unordered_map<std::string, std::shared_ptr<JointDependentFrame>> frames_;
  std::unordered_map<std::string, std::shared_ptr<GeoFrame>> frames_;

  std::unordered_map<std::string, std::shared_ptr<KinematicTreeObserver>> ktObservers_;
};

} // namespace GeoRobotics
