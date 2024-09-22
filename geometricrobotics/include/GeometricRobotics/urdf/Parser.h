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
#include "GeometricRobotics/urdf/Geometry.h"
#include "GeometricRobotics/urdf/Link.h"
#include "GeometricRobotics/urdf/Model.h"
#include "GeometricRobotics/urdf/Quaternion.h"
#include <fstream>
#include <tinyxml.h>

namespace GeoRobotics
{
class Parser
{
public:
  Parser() {}
  ~Parser() {}

  /*! \brief Parse a 3D vector from the string
   * \param vector_str
   */

  bool parseMetaPose(MetaPose & pose, TiXmlElement * xml);

  bool parseCollision(Collision & col, TiXmlElement * config);
  bool parseVisual(Visual & vis, TiXmlElement * config);
  bool parseMetaInertia(MetaInertia & i, TiXmlElement * config);

  bool parseMesh(Mesh & m, TiXmlElement * c);
  bool parseCylinder(Cylinder & y, TiXmlElement * c);
  bool parseSphere(Sphere & s, TiXmlElement * c);
  bool parseBox(Box & b, TiXmlElement * c);

  bool parseJoint(std::shared_ptr<Joint> joint, TiXmlElement * config);
  bool parseLink(Link & link, TiXmlElement * config);

  bool parseMaterial(Material & material, TiXmlElement * config, bool only_name_is_ok);
  bool parseJointDynamics(std::shared_ptr<JointDynamics> jd, TiXmlElement * config);
  bool parseJointLimits(std::shared_ptr<JointLimits> jl, TiXmlElement * config);

  std::shared_ptr<Geometry> parseGeometry(TiXmlElement * g);

  std::shared_ptr<Model> parseURDFFile(const std::string & path);
  std::shared_ptr<Model> parseURDF(const std::string & xml_string);

  bool assignMaterial(const std::shared_ptr<Visual> & visual, std::shared_ptr<Model> & model, const char * link_name);

protected:
  inline std::shared_ptr<Model> metaThrow(const std::string & msg, std::shared_ptr<Model> modelPtr)
  {
    RoboticsUtils::throw_runtime_error(msg, __FILE__, __LINE__);
    modelPtr.reset();
    return modelPtr;
  }

  ;
};

} // namespace GeoRobotics
