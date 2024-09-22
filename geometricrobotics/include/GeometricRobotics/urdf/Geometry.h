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
#include "GeometricRobotics/urdf/Quaternion.h"

namespace GeoRobotics
{

class Color
{
public:
  Color()
  {
    reset();
  };
  float r;
  float g;
  float b;
  float a;

  void reset()
  {
    r = g = b = 0.0f;
    a = 1.0f;
  }
  bool init(const std::string & vector_str)
  {
    reset();
    std::vector<std::string> pieces;
    std::vector<float> rgba;
    split_string(pieces, vector_str, " ");
    for(unsigned int i = 0; i < pieces.size(); ++i)
    {
      if(!pieces[i].empty())
      {
        try
        {
          double piece = strToDouble(pieces[i].c_str());
          if((piece < 0) || (piece > 1))
            RoboticsUtils::throw_runtime_error(
                "Component [" + pieces[i] + "] is outside the valid range for colors [0, 1]", __FILE__, __LINE__);
          rgba.push_back(static_cast<float>(piece));
        }
        catch(std::runtime_error & /*e*/)
        {
          RoboticsUtils::throw_runtime_error("Unable to parse component [" + pieces[i]
                                                 + "] to a double (while parsing a color value)",
                                             __FILE__, __LINE__);
        }
      }
    }

    if(rgba.size() != 4)
    {
      return false;
    }

    this->r = rgba[0];
    this->g = rgba[1];
    this->b = rgba[2];
    this->a = rgba[3];

    return true;
  };
};

class Geometry
{
public:
  enum
  {
    SPHERE,
    BOX,
    CYLINDER,
    MESH
  } type;

  virtual ~Geometry(void) {}
};

class Box : public Geometry
{
public:
  Box()
  {
    reset();
    type = BOX;
  };
  Vector3d dim = Vector3d::Zero();

  void reset()
  {
    dim = Vector3d::Zero();
  };
};

class Cylinder : public Geometry
{
public:
  Cylinder()
  {
    reset();
    type = CYLINDER;
  };
  double length = 0.0;
  double radius = 0.0;

  void reset()
  {
    length = 0.0;
    radius = 0.0;
  };
};

class Mesh : public Geometry
{
public:
  Mesh()
  {
    reset();
    type = MESH;
  };
  std::string filename;
  Vector3d scale = Vector3d::Ones();

  void reset()
  {
    filename.clear();
    // default scale
    scale = Vector3d::Ones();
  };
};

class Material
{
public:
  Material()
  {
    reset();
  };
  std::string name;
  std::string texture_filename;
  Color color;

  void reset()
  {
    color.reset();
    texture_filename.clear();
    name.clear();
  };
};

class Sphere : public Geometry
{
public:
  Sphere()
  {
    reset();
    type = SPHERE;
  };
  double radius = 0.0;

  void reset()
  {
    radius = 0;
  };
};

class MetaInertia
{
public:
  MetaInertia() : mass(0.0), ixx(0.0), ixy(0.0), ixz(0.0), iyy(0.0), iyz(0.0), izz(0.0)
  {
    reset();
  };
  ///< Position and orientation of the COM in its local body frame, i.e., w.r.t. the the parent joint frame.
  MetaPose origin;
  double mass;
  double ixx, ixy, ixz, iyy, iyz, izz;

  void reset()
  {
    origin.reset();
    // mass = 0.0;
    // ixx = ixy = ixz = iyy = iyz = izz = 0.0;
  };

  bool nonZeroMass() const
  {
    return (mass != 0.0);
  }

  Matrix6d constructInertiaTensor() const
  {
    Matrix6d m;

    m << Matrix3d::Identity() * mass, Matrix3d::Zero(), Matrix3d::Zero(), constructRotationalInertia();

    return m;
  }

  /*!  \brief compute the inertia matrix w.r.t. a reference frame
   *  \param transform: the COM(of the link)'s pose w.r.t. to the reference frame X, i.e., \f$ g_{X, com} \f$.
   */
  Matrix6d spatialInertia(const Iso3d & transform)
  {
    const Matrix6d & adgInvM = metaAdgInvMatrix(transform);
    return adgInvM.transpose() * constructInertiaTensor() * adgInvM;
  }
  Matrix3d constructRotationalInertia() const
  {
    Matrix3d rInertia = Matrix3d::Identity();
    rInertia(0, 0) = ixx;
    rInertia(0, 1) = ixy;
    rInertia(0, 2) = ixz;
    rInertia(1, 0) = ixy;
    rInertia(2, 0) = ixz;

    rInertia(1, 1) = iyy;
    rInertia(1, 2) = iyz;
    rInertia(2, 1) = iyz;

    rInertia(2, 2) = izz;

    return rInertia;
  }

  void randomize()
  {
    mass = grNumber(0.0, 3.0);
    ixx = grNumber(0.0, 0.3);
    ixy = grNumber(0.0, 0.3);
    ixz = grNumber(0.0, 0.3);
    iyy = grNumber(0.0, 0.3);
    iyz = grNumber(0.0, 0.3);
    izz = grNumber(0.0, 0.3);
    origin.randomize();
  }
};

class Visual
{
public:
  Visual()
  {
    reset();
  };
  MetaPose origin;
  std::shared_ptr<Geometry> geometry;

  std::string materialName;
  std::shared_ptr<Material> material;

  void reset()
  {
    origin.reset();
    materialName.clear();
    material.reset();
    geometry.reset();
    name.clear();
  };

  std::string name;
};

class Collision
{
public:
  Collision()
  {
    reset();
  };
  MetaPose origin;
  std::shared_ptr<Geometry> geometry;

  void reset()
  {
    origin.reset();
    geometry.reset();
    name.clear();
  };

  std::string name;
};

} // namespace GeoRobotics
