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
#include "GeometricRobotics/urdf/Parser.h"

namespace GeoRobotics
{

bool Parser::parseMetaPose(MetaPose & pose, TiXmlElement * xml)
{
  pose.reset();
  if(xml)
  {
    const char * xyz_str = xml->Attribute("xyz");
    if(xyz_str != NULL)
    {
      try
      {
        pose.position = parseVector3d(xyz_str);
      }
      catch(std::runtime_error & e)
      {
        RoboticsUtils::throw_runtime_error("GeoRobotics:: " + std::string(e.what()), __FILE__, __LINE__);
        return false;
      }
    }

    const char * rpy_str = xml->Attribute("rpy");
    if(rpy_str != NULL)
    {
      try
      {
        pose.rotation.init(rpy_str);
      }
      catch(std::runtime_error & e)
      {
        RoboticsUtils::throw_runtime_error("GeoRobotics:: " + std::string(e.what()), __FILE__, __LINE__);
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<Model> Parser::parseURDFFile(const std::string & path)
{
  std::ifstream stream(path.c_str());
  if(!stream)
  {
    RoboticsUtils::throw_runtime_error(("GeoRobotics::File " + path + " does not exist").c_str(), __FILE__, __LINE__);
    std::shared_ptr<Model> emptyPtr;
    return emptyPtr;
  }

  std::string xml_str((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  return parseURDF(xml_str);
}

std::shared_ptr<Model> Parser::parseURDF(const std::string & xml_string)
{
  std::shared_ptr<Model> modelPtr = std::make_shared<Model>();

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());
  if(xml_doc.Error())
  {
    return metaThrow(xml_doc.ErrorDesc(), modelPtr);
  }

  TiXmlElement * robot_xml = xml_doc.FirstChildElement("robot");
  if(!robot_xml)
  {
    metaThrow("Could not find the 'robot' element in the xml file", modelPtr);
  }

  // Get robot name
  const char * name = robot_xml->Attribute("name");
  if(!name)
  {
    metaThrow("No name given for the robot.", modelPtr);
  }

  modelPtr->setName(name);

  // Get all Material elements
  for(TiXmlElement * material_xml = robot_xml->FirstChildElement("material"); material_xml;
      material_xml = material_xml->NextSiblingElement("material"))
  {
    std::shared_ptr<Material> materialPtr;
    materialPtr.reset(new Material);

    try
    {
      parseMaterial(*materialPtr, material_xml, false); // material needs to be fully defined here

      if(modelPtr->getMaterial(materialPtr->name))
      {
        std::cout << RoboticsUtils::error << "The material " << materialPtr->name << " is not unique. "
                  << RoboticsUtils::reset << std::endl;
        materialPtr.reset();
        modelPtr.reset();
        return modelPtr;
      }
      else
      {
        if(modelPtr->addMaterial(materialPtr))
        {
          // std::cout << RoboticsUtils::info << "Successfully added a new material: " << materialPtr->name.c_str()
          //           << RoboticsUtils::reset << std::endl;
        }
        else
        {
          // std::cout << RoboticsUtils::info << "The material: " << materialPtr->name
          //           << " already exits in the model: " << modelPtr->name() << RoboticsUtils::reset << std::endl;
        }
      }
    }
    catch(std::runtime_error & /*e*/)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::material xml is not initialized correctly", __FILE__, __LINE__);
      materialPtr.reset();
      modelPtr.reset();
      return modelPtr;
    }
  }

  // Get all Link elements
  for(TiXmlElement * link_xml = robot_xml->FirstChildElement("link"); link_xml;
      link_xml = link_xml->NextSiblingElement("link"))
  {
    std::shared_ptr<Link> link;
    link.reset(new Link());

    try
    {
      parseLink(*link, link_xml);
      if(modelPtr->getLink(link->name))
      {
        RoboticsUtils::throw_runtime_error("GeoRobotics::link is not unique: " + link->name, __FILE__, __LINE__);
        modelPtr.reset();
        return modelPtr;
      }
      else
      {
        // set link visual(s) material
        // RoboticsUtils::quickInfo("GeoRobotics::Setting link material: " + link->name);
        if(link->visual)
        {
          assignMaterial(link->visual, modelPtr, link->name.c_str());
        }
        for(const auto & visual : link->visualArray)
        {
          assignMaterial(visual, modelPtr, link->name.c_str());
        }

        modelPtr->addLink(link);
        // RoboticsUtils::quickHL("GeoRobotics::Successfully added a new link: " + link->name);
      }
    }
    catch(std::runtime_error & /*e*/)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::link xml is not initialized correctly", __FILE__, __LINE__);
      modelPtr.reset();
      return modelPtr;
    }
  }
  if(modelPtr->numLinks() == 0)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::No link elements found in urdf file", __FILE__, __LINE__);
    modelPtr.reset();
    return modelPtr;
  }

  // Get all Joint elements
  for(TiXmlElement * joint_xml = robot_xml->FirstChildElement("joint"); joint_xml;
      joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    std::shared_ptr<Joint> joint = std::make_shared<Joint>();
    // joint.reset(new Joint());

    if(parseJoint(joint, joint_xml))
    {
      if(modelPtr->getJoint(joint->name))
      {
        RoboticsUtils::throw_runtime_error("GeoRobotics::joint is not unique: " + joint->name, __FILE__, __LINE__);
        modelPtr.reset();
        return modelPtr;
      }
      else
      {
        modelPtr->addJoint(joint);
        // RoboticsUtils::quickInfo("GeoRobotics::Successfully added a new joint: ", joint->name,
        //                          " (type: ", ToString(joint->type), ")");
      }
    }
    else
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::joint xml is not initialized correctly", __FILE__, __LINE__);
      modelPtr.reset();
      return modelPtr;
    }
  }

  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::unordered_map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  try
  {
    modelPtr->initTree(parent_link_tree);
  }
  catch(std::runtime_error & e)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Failed to build tree: " + std::string(e.what()), __FILE__,
                                       __LINE__);
    modelPtr.reset();
    return modelPtr;
  }

  // find the root link
  try
  {
    modelPtr->initRoot(parent_link_tree);
  }
  catch(std::runtime_error & e)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Failed to find root link: " + std::string(e.what()), __FILE__,
                                       __LINE__);
    modelPtr.reset();
    return modelPtr;
  }

  return modelPtr;
}

bool Parser::parseBox(Box & b, TiXmlElement * c)
{
  b.reset();

  b.type = Geometry::BOX;
  if(!c->Attribute("size"))
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Box shape has no size attribute", __FILE__, __LINE__);
    return false;
  }
  try
  {
    b.dim = parseVector3d(c->Attribute("size"));
  }
  catch(std::runtime_error & e)
  {
    b.reset();
    return false;
  }
  return true;
}

bool Parser::parseSphere(Sphere & s, TiXmlElement * c)
{
  s.reset();

  s.type = Geometry::SPHERE;
  if(!c->Attribute("radius"))
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Sphere shape must have a radius attribute", __FILE__, __LINE__);
    return false;
  }

  try
  {
    s.radius = strToDouble(c->Attribute("radius"));
  }
  catch(std::runtime_error &)
  {
    std::stringstream stm;
    stm << "GeoRobotics::radius [" << c->Attribute("radius") << "] is not a valid float";
    RoboticsUtils::quickInfo(stm.str().c_str());
    return false;
  }

  return true;
}

bool Parser::parseCylinder(Cylinder & y, TiXmlElement * c)
{
  y.reset();

  y.type = Geometry::CYLINDER;
  if(!c->Attribute("length") || !c->Attribute("radius"))
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Cylinder shape must have both length and radius attributes",
                                       __FILE__, __LINE__);
    return false;
  }

  try
  {
    y.length = strToDouble(c->Attribute("length"));
  }
  catch(std::runtime_error &)
  {
    std::stringstream stm;
    stm << "length [" << c->Attribute("length") << "] is not a valid float";
    RoboticsUtils::quickInfo(stm.str().c_str());
    return false;
  }

  try
  {
    y.radius = strToDouble(c->Attribute("radius"));
  }
  catch(std::runtime_error &)
  {
    std::stringstream stm;
    stm << "radius [" << c->Attribute("radius") << "] is not a valid float";
    RoboticsUtils::quickInfo(stm.str().c_str());
    return false;
  }

  return true;
}

bool Parser::parseMesh(Mesh & m, TiXmlElement * c)
{
  m.reset();

  m.type = Geometry::MESH;
  if(!c->Attribute("filename"))
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Mesh must contain a filename attribute", __FILE__, __LINE__);
    return false;
  }

  m.filename = c->Attribute("filename");

  if(c->Attribute("scale"))
  {
    try
    {
      m.scale = parseVector3d(c->Attribute("scale"));
    }
    catch(std::runtime_error & e)
    {
      m.scale = Vector3d::Zero();
      RoboticsUtils::quickError("GeoRobotics::Mesh scale was specified, but could not be parsed: "
                                + std::string(e.what()));
      return false;
    }
  }
  else
  {
    m.scale.x() = m.scale.y() = m.scale.z() = 1.0;
  }
  return true;
}

std::shared_ptr<Geometry> Parser::parseGeometry(TiXmlElement * g)
{
  std::shared_ptr<Geometry> geomPtr;
  if(!g) return geomPtr;

  TiXmlElement * shape = g->FirstChildElement();
  if(!shape)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Geometry tag contains no child element.", __FILE__, __LINE__);
    return geomPtr;
  }

  std::string type_name = shape->ValueStr();
  if(type_name == "sphere")
  {
    Sphere * s = new Sphere();
    geomPtr.reset(s);
    if(parseSphere(*s, shape)) return geomPtr;
  }
  else if(type_name == "box")
  {
    Box * b = new Box();
    geomPtr.reset(b);
    if(parseBox(*b, shape)) return geomPtr;
  }
  else if(type_name == "cylinder")
  {
    Cylinder * c = new Cylinder();
    geomPtr.reset(c);
    if(parseCylinder(*c, shape)) return geomPtr;
  }
  else if(type_name == "mesh")
  {
    Mesh * m = new Mesh();
    geomPtr.reset(m);
    if(parseMesh(*m, shape)) return geomPtr;
  }
  else
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Unknown geometry type: " + type_name, __FILE__, __LINE__);
    return geomPtr;
  }

  return std::shared_ptr<Geometry>();
}

bool Parser::parseMaterial(Material & material, TiXmlElement * config, bool only_name_is_ok)
{
  bool has_rgb = false;
  bool has_filename = false;

  material.reset();

  if(!config->Attribute("name"))
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Material must contain a name attribute", __FILE__, __LINE__);
    return false;
  }

  material.name = config->Attribute("name");

  // texture
  TiXmlElement * t = config->FirstChildElement("texture");
  if(t)
  {
    if(t->Attribute("filename"))
    {
      material.texture_filename = t->Attribute("filename");
      has_filename = true;
    }
  }

  // color
  TiXmlElement * c = config->FirstChildElement("color");
  if(c)
  {
    if(c->Attribute("rgba"))
    {

      try
      {
        material.color.init(c->Attribute("rgba"));
        has_rgb = true;
      }
      catch(std::runtime_error & e)
      {
        material.color.reset();
        RoboticsUtils::quickError(
            std::string("GeoRobotics::Material [" + material.name + "] has malformed color rgba values: " + e.what()));
      }
    }
  }

  if(!has_rgb && !has_filename)
  {
    if(!only_name_is_ok) // no need for an error if only name is ok
    {
      if(!has_rgb)
        RoboticsUtils::quickError(std::string("GeoRobotics::Material [" + material.name + "] color has no rgba"));
      if(!has_filename)
        RoboticsUtils::quickError(std::string("GeoRobotics::Material [" + material.name + "] not defined in file"));
    }
    return false;
  }
  return true;
}

bool Parser::parseMetaInertia(MetaInertia & i, TiXmlElement * config)
{
  i.reset();

  // Origin
  TiXmlElement * o = config->FirstChildElement("origin");
  if(o)
  {
    if(!parseMetaPose(i.origin, o)) return false;
  }

  TiXmlElement * mass_xml = config->FirstChildElement("mass");
  if(!mass_xml)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Inertial element must have a mass element", __FILE__, __LINE__);
    return false;
  }
  if(!mass_xml->Attribute("value"))
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Inertial: mass element must have value attribute", __FILE__,
                                       __LINE__);
    return false;
  }

  try
  {
    i.mass = strToDouble(mass_xml->Attribute("value"));
  }
  catch(std::runtime_error &)
  {
    std::stringstream stm;
    stm << "GeoRobotics::Inertial: mass [" << mass_xml->Attribute("value") << "] is not a float";
    RoboticsUtils::throw_runtime_error(stm.str());
    return false;
  }

  TiXmlElement * inertia_xml = config->FirstChildElement("inertia");
  if(!inertia_xml)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Inertial element must have inertia element");
    return false;
  }

  std::vector<std::pair<std::string, double>> attrs{std::make_pair("ixx", 0.0), std::make_pair("ixy", 0.0),
                                                    std::make_pair("ixz", 0.0), std::make_pair("iyy", 0.0),
                                                    std::make_pair("iyz", 0.0), std::make_pair("izz", 0.0)};

  for(auto & attr : attrs)
  {
    if(!inertia_xml->Attribute(attr.first))
    {
      std::stringstream stm;
      stm << "GeoRobotics::Inertial: inertia element missing " << attr.first << " attribute";
      RoboticsUtils::throw_runtime_error(stm.str());
      return false;
    }

    try
    {
      attr.second = strToDouble(inertia_xml->Attribute(attr.first.c_str()));
    }
    catch(std::runtime_error &)
    {
      std::stringstream stm;
      stm << "GeoRobotics::Inertial: inertia element " << attr.first << " is not a valid double";
      RoboticsUtils::throw_runtime_error(stm.str());
      return false;
    }
  }

  i.ixx = attrs[0].second;
  i.ixy = attrs[1].second;
  i.ixz = attrs[2].second;
  i.iyy = attrs[3].second;
  i.iyz = attrs[4].second;
  i.izz = attrs[5].second;

  return true;
}

bool Parser::parseVisual(Visual & vis, TiXmlElement * config)
{
  vis.reset();

  // Origin
  TiXmlElement * o = config->FirstChildElement("origin");
  if(o)
  {
    if(!parseMetaPose(vis.origin, o)) return false;
  }

  // Geometry
  TiXmlElement * geom = config->FirstChildElement("geometry");
  vis.geometry = parseGeometry(geom);
  if(!vis.geometry) return false;

  const char * name_char = config->Attribute("name");
  if(name_char) vis.name = name_char;

  // Material
  TiXmlElement * mat = config->FirstChildElement("material");
  if(mat)
  {
    // get material name
    if(!mat->Attribute("name"))
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::Visual material must contain a name attribute", __FILE__,
                                         __LINE__);
      return false;
    }
    vis.materialName = mat->Attribute("name");

    // try to parse material element in place
    vis.material.reset(new Material());
    if(!parseMaterial(*vis.material, mat, true))
    {
      vis.material.reset();
    }
  }

  return true;
}

bool Parser::parseCollision(Collision & col, TiXmlElement * config)
{
  col.reset();

  // Origin
  TiXmlElement * o = config->FirstChildElement("origin");
  if(o)
  {
    if(!parseMetaPose(col.origin, o)) return false;
  }

  // Geometry
  TiXmlElement * geom = config->FirstChildElement("geometry");
  col.geometry = parseGeometry(geom);
  if(!col.geometry) return false;

  const char * name_char = config->Attribute("name");
  if(name_char) col.name = name_char;

  return true;
}

bool Parser::parseLink(Link & link, TiXmlElement * config)
{

  link.reset();

  const char * name_char = config->Attribute("name");
  if(!name_char)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::No name given for the link.", __FILE__, __LINE__);
    return false;
  }
  link.name = std::string(name_char);

  // Inertial (optional)
  TiXmlElement * i = config->FirstChildElement("inertial");
  if(i)
  {
    link.metaInertia.reset(new MetaInertia());
    if(!parseMetaInertia(*link.metaInertia, i))
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::Could not parse inertial element for Link " + link.name,
                                         __FILE__, __LINE__);
      return false;
    }
  }

  // Multiple Visuals (optional)
  for(TiXmlElement * vis_xml = config->FirstChildElement("visual"); vis_xml;
      vis_xml = vis_xml->NextSiblingElement("visual"))
  {

    std::shared_ptr<Visual> vis;
    vis.reset(new Visual());
    if(parseVisual(*vis, vis_xml))
    {
      link.visualArray.push_back(vis);
    }
    else
    {
      vis.reset();
      RoboticsUtils::throw_runtime_error("GeoRobotics::Could not parse visual element for Link " + link.name, __FILE__,
                                         __LINE__);
      return false;
    }
  }

  // Visual (optional)
  // Assign the first visual to the .visual ptr, if it exists
  if(!link.visualArray.empty()) link.visual = link.visualArray[0];

  // Multiple Collisions (optional)
  for(TiXmlElement * col_xml = config->FirstChildElement("collision"); col_xml;
      col_xml = col_xml->NextSiblingElement("collision"))
  {
    std::shared_ptr<Collision> col;
    col.reset(new Collision());
    if(parseCollision(*col, col_xml))
    {
      link.collisionArray.push_back(col);
    }
    else
    {
      col.reset();
      RoboticsUtils::throw_runtime_error("GeoRobotics::Could not parse collision element for Link" + link.name,
                                         __FILE__, __LINE__);
      return false;
    }
  }

  // Collision (optional)
  // Assign the first collision to the .collision ptr, if it exists
  if(!link.collisionArray.empty()) link.collision = link.collisionArray[0];

  return true;
}

bool Parser::parseJoint(std::shared_ptr<Joint> joint, TiXmlElement * config)
{
  // joint.reset();

  // Get Joint Name
  const char * name = config->Attribute("name");
  // RoboticsUtils::quickHL("Processing joint: ", name);
  if(!name)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::unnamed joint found", __FILE__, __LINE__);
    return false;
  }
  joint->name = name;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement * origin_xml = config->FirstChildElement("origin");
  if(!origin_xml)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics:: " + joint->name
                                           + " joint missing origin tag under parent describing transform from Parent "
                                             "Link to Joint Frame, (using Identity transform).",
                                       __FILE__, __LINE__);
    joint->transform.reset();
  }
  else
  {
    if(!parseMetaPose(joint->transform, origin_xml))
    {
      joint->transform.reset();
      RoboticsUtils::throw_runtime_error("GeoRobotics::Malformed parent origin element for joint: " + joint->name,
                                         __FILE__, __LINE__);
      return false;
    }
  }

  // Get Parent Link
  TiXmlElement * parent_xml = config->FirstChildElement("parent");
  if(parent_xml)
  {
    const char * pname = parent_xml->Attribute("link");
    if(!pname)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::no parent link name specified for Joint link: " + joint->name,
                                         __FILE__, __LINE__);
    }
    else
    {
      joint->parentLinkName = std::string(pname);
    }
  }

  // Get Child Link
  TiXmlElement * child_xml = config->FirstChildElement("child");
  if(child_xml)
  {
    const char * cname = child_xml->Attribute("link");
    if(!cname)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::no child link name specified for Joint link: " + joint->name,
                                         __FILE__, __LINE__);
    }
    else
    {
      joint->childLinkName = std::string(cname);
    }
  }

  // Get Joint type
  const char * type_char = config->Attribute("type");
  if(!type_char)
  {
    RoboticsUtils::throw_runtime_error(
        "GeoRobotics:: " + joint->name + " joint has no type, check to see if it's a reference.", __FILE__, __LINE__);
    return false;
  }

  std::string type_str = type_char;
  if(type_str == "planar")
    joint->type = JointType::PLANAR;
  else if(type_str == "floating")
    joint->type = JointType::FLOATING;
  else if(type_str == "revolute")
    joint->type = JointType::REVOLUTE;
  else if(type_str == "continuous")
    joint->type = JointType::CONTINUOUS;
  else if(type_str == "prismatic")
    joint->type = JointType::PRISMATIC;
  else if(type_str == "fixed")
    joint->type = JointType::FIXED;
  else
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::" + joint->name + " joint  has no known type " + type_str);
    return false;
  }

  // Get Joint Axis
  if(joint->type != JointType::FLOATING && joint->type != JointType::FIXED)
  {
    // axis
    TiXmlElement * axis_xml = config->FirstChildElement("axis");
    if(!axis_xml)
    {
      RoboticsUtils::throw_runtime_error(
          "GeoRobotics::" + joint->name + " has no axis elemement.  Defaulting to (1,0,0) axis", __FILE__, __LINE__);
      joint->axis = Vector3d(1.0, 0.0, 0.0);
    }
    else
    {
      if(axis_xml->Attribute("xyz"))
      {
        try
        {
          joint->axis = parseVector3d(axis_xml->Attribute("xyz"));
        }
        catch(std::runtime_error & e)
        {
          joint->axis << 0.0, 0.0, 1.0;
          RoboticsUtils::throw_runtime_error(
              "GeoRobotics::Malformed axis element for joint: " + joint->name + ", " + e.what(), __FILE__, __LINE__);
          return false;
        }
      }
    }
  }

  // Get limit
  TiXmlElement * limit_xml = config->FirstChildElement("limit");
  if(limit_xml)
  {
    joint->limits = std::make_shared<JointLimits>();
    if(!parseJointLimits(joint->limits, limit_xml))
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::Could not parse limit element for joint " + joint->name,
                                         __FILE__, __LINE__);
      joint->limits.reset();
      return false;
    }
  }
  else if(joint->type == JointType::REVOLUTE)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::" + joint->name
                                           + " joint is of type REVOLUTE but it does not specify limits",
                                       __FILE__, __LINE__);
    return false;
  }
  else if(joint->type == JointType::PRISMATIC)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::" + joint->name
                                           + " joint is of type PRISMATIC but it does not specify limits",
                                       __FILE__, __LINE__);
    return false;
  }

  // Get Dynamics
  TiXmlElement * prop_xml = config->FirstChildElement("dynamics");
  if(prop_xml)
  {
    joint->dynamics = std::make_shared<JointDynamics>();
    if(!parseJointDynamics(joint->dynamics, prop_xml))
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::Could not parse joint_dynamics element for joint: "
                                         + joint->name);
      joint->dynamics.reset();
      return false;
    }
    // RoboticsUtils::quickHL("Configured dynamics of joint: ", joint->name);
  }

  // RoboticsUtils::quickHL("Read the configuration of joint: ", joint->name);
  return true;
}
bool Parser::parseJointDynamics(std::shared_ptr<JointDynamics> jd, TiXmlElement * config)
{
  // jd.reset();

  // Get joint damping
  const char * damping_str = config->Attribute("damping");
  if(damping_str == NULL)
  {

    RoboticsUtils::throw_runtime_error("GeoRobotics::Joint_dynamics: no damping, defaults to 0", __FILE__, __LINE__);
    jd->damping = 0;
  }
  else
  {
    try
    {
      jd->damping = strToDouble(damping_str);
    }
    catch(std::runtime_error &)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::" + std::string(damping_str) + " is not a valid damping value.",
                                         __FILE__, __LINE__);
      return false;
    }
  }

  // Get joint friction
  const char * friction_str = config->Attribute("friction");
  if(friction_str == NULL)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::joint_dynamics: no friction, defaults to 0", __FILE__, __LINE__);
    jd->friction = 0;
  }
  else
  {
    try
    {
      jd->friction = strToDouble(friction_str);
    }
    catch(std::runtime_error &)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics::" + std::string(friction_str)
                                         + " is not a valid friction value.");
      return false;
    }
  }

  if(damping_str == NULL && friction_str == NULL)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::joint dynamics element specified with no damping and no friction",
                                       __FILE__, __LINE__);
    return false;
  }
  else
  {
    RoboticsUtils::quickInfo("GeoRobotics::joint_dynamics: damping " + std::to_string(jd->damping) + " and friction "
                             + std::to_string(jd->friction));
    RoboticsUtils::quickHL("Configured dynamics ");
    return true;
  }
}

bool Parser::parseJointLimits(std::shared_ptr<JointLimits> jl, TiXmlElement * config)
{
  // jl.reset();

  // Get lower joint limit
  const char * lower_str = config->Attribute("lower");
  if(lower_str == NULL)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Joint_limit: no lower, defaults to 0");
    jl->lower = 0;
  }
  else
  {
    try
    {
      jl->lower = strToDouble(lower_str);
    }
    catch(std::runtime_error &)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics:: " + std::string(lower_str) + " is not a valid lower value.",
                                         __FILE__, __LINE__);
      return false;
    }
  }

  // Get upper joint limit
  const char * upper_str = config->Attribute("upper");
  if(upper_str == NULL)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::Joint_limit: no upper, , defaults to 0", __FILE__, __LINE__);
    jl->upper = 0;
  }
  else
  {
    try
    {
      jl->upper = strToDouble(upper_str);
    }
    catch(std::runtime_error &)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics:: " + std::string(upper_str) + " is not a valid upper value.",
                                         __FILE__, __LINE__);
      return false;
    }
  }

  // Get joint effort limit
  const char * effort_str = config->Attribute("effort");
  if(effort_str == NULL)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::joint limit: no effort", __FILE__, __LINE__);
    return false;
  }
  else
  {
    try
    {
      jl->effort = strToDouble(effort_str);
    }
    catch(std::runtime_error &)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics:: " + std::string(effort_str) + " is not a valid effort value.",
                                         __FILE__, __LINE__);
      return false;
    }
  }

  // Get joint velocity limit
  const char * velocity_str = config->Attribute("velocity");
  if(velocity_str == NULL)
  {
    RoboticsUtils::throw_runtime_error("GeoRobotics::joint limit: no velocity", __FILE__, __LINE__);
    return false;
  }
  else
  {
    try
    {
      jl->velocity = strToDouble(velocity_str);
    }
    catch(std::runtime_error &)
    {
      RoboticsUtils::throw_runtime_error("GeoRobotics:: " + std::string(velocity_str)
                                         + " is not a valid velocity value.");
      return false;
    }
  }

  return true;
}

bool Parser::assignMaterial(const std::shared_ptr<Visual> & visual,
                            std::shared_ptr<Model> & modelPtr,
                            const char * link_name)
{
  if(visual->materialName.empty()) return true;

  const std::shared_ptr<Material> & material = modelPtr->getMaterial(visual->materialName);
  if(material)
  {
    RoboticsUtils::quickInfo("GeoRobotics::Setting link: " + std::string(link_name)
                             + " to material: " + visual->materialName);
    visual->material = material;
  }
  else
  {
    if(visual->material)
    {
      RoboticsUtils::quickInfo("GeoRobotics::Link: " + std::string(link_name) + " material " + visual->materialName
                               + " defined in Visual.");
      modelPtr->addMaterial(visual->material);
    }
    else
    {
      RoboticsUtils::quickError("GeoRobotics::link: " + std::string(link_name) + " material " + visual->materialName
                                + "  undefined.");
      return false;
    }
  }
  return true;
}

} // namespace GeoRobotics
