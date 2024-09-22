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
#include "GeometricRobotics/Utils.h"
#include "GeometricRobotics/urdf/Joint.h"
#include "GeometricRobotics/urdf/Link.h"
//#include <map>
#include <unordered_map>
namespace GeoRobotics
{

class Model
{
public:
  Model() {}
  ~Model() {}

  void reset()
  {
    name_.clear();
    links_.clear();
    joints_.clear();
    materials_.clear();
    rootLink_.reset();
  }

  const std::shared_ptr<Link> getRoot()
  {
    return rootLink_;
  }

  const std::shared_ptr<Link> getLink(const std::string & name) const
  {
    return metaGetLink(name);
  }

  /*! \brief Returns a valid shared_ptr if the link exists, otherwise an empty ptr
   */
  void getLink(const std::string & name, std::shared_ptr<Link> & link) const
  {
    link = metaGetLink(name);
  }

  const std::shared_ptr<Joint> getJoint(const std::string & name) const
  {
    if(this->joints_.find(name) == this->joints_.end())
    {
      std::shared_ptr<Joint> ptr;
      ptr.reset();
      return ptr;
    }
    else
      return joints_.find(name)->second;
  };

  const std::string & name() const
  {
    return name_;
  };
  void setName(const std::string & name)
  {
    name_ = name;
  };

  /*! \brief Returns a valid shared_ptr if the material exists, otherwise an empty ptr
   */
  std::shared_ptr<Material> getMaterial(const std::string & name) const
  {
    if(this->materials_.find(name) == this->materials_.end())
    {
      std::shared_ptr<Material> ptr;
      ptr.reset();
      return ptr;
    }
    else
      return materials_.find(name)->second;
  }

  bool addLink(std::shared_ptr<Link> linkPtr)
  {
    auto status = links_.insert(std::make_pair(linkPtr->name, linkPtr));
    return status.second;
  }
  bool addJoint(std::shared_ptr<Joint> jointPtr)
  {
    auto status = joints_.insert(std::make_pair(jointPtr->name, jointPtr));
    return status.second;
  }

  bool addMaterial(std::shared_ptr<Material> materialPtr)
  {
    auto status = materials_.insert(std::make_pair(materialPtr->name, materialPtr));
    return status.second;
  }
  std::size_t numJoints() const
  {
    return joints_.size();
  }
  std::size_t numLinks() const
  {
    return links_.size();
  }

  void initTree(std::unordered_map<std::string, std::string> & parent_link_tree);

  void initRoot(const std::unordered_map<std::string, std::string> & parent_link_tree);

  const std::unordered_map<std::string, std::shared_ptr<Link>> & getLinks()
  {

    return links_;
  }
  const std::unordered_map<std::string, std::shared_ptr<Joint>> & getJoints()
  {

    return joints_;
  }

  /*! \brief Merge the links connected by a fixed joint.
   */
  void merge();

protected:
  void removeLink_(const std::string & name);
  void removeJoint_(const std::string & name);
  /*! \brief Recursively go through all the links.
   */
  void visitDFS_(const std::string & currentNode,
                 // const std::string & currentJoint,
                 std::unordered_set<std::string> & visited);

  /*! \brief Merge the links connected by a fixed joint.
   * \param joint merge the two links connected by the fixed joint
   */
  void mergeFixedJoint_(std::shared_ptr<Joint> joint);

  /*! \brief Recursively merge the fixed joints until all the children joints are not fixed joint
   * \param linkName
   */
  void mergeLink_(const std::string & linkName);

  ///< The root link
  std::shared_ptr<Link> rootLink_;

  ///< Robot name
  std::string name_;

  std::unordered_map<std::string, std::shared_ptr<Link>> links_;
  std::unordered_map<std::string, std::shared_ptr<Joint>> joints_;
  std::unordered_map<std::string, std::shared_ptr<Material>> materials_;

  /*! \brief Merge the inertia matrices of two links as if they are a composite-rigid body.
   */
  void mergeInertiaMatrix_(std::shared_ptr<MetaInertia> pIM,
                           std::shared_ptr<MetaInertia> cIM,
                           const std::shared_ptr<Joint> & j);
  std::shared_ptr<Link> metaGetLink(const std::string & name) const
  {
    if(links_.find(name) == links_.end())
    {
      std::shared_ptr<Link> ptr;
      return ptr;
    }
    else
      return links_.find(name)->second;
  }
};

} // namespace GeoRobotics
