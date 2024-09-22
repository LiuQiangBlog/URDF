// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <fstream>
#include <stack>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <rdl_dynamics/Model.hpp>
#include <rdl_urdfreader/urdfreader.hpp>

using namespace std;

namespace RobotDynamics
{
namespace Urdf
{
using namespace Math;

/**
 * @brief Helper method for getting xml elements like <origin> 1 2 3 </origin> into an stl container for
 * further operations
 * @param element
 * @return vector of doubles
 */
inline std::vector<double> xmlElementToVector(TiXmlElement* element)
{
    assert(element);
    std::istringstream stringStream(element->GetText());

    std::string numberAsString;

    std::vector<double> output;

    while (stringStream >> numberAsString)
    {
        output.push_back(std::stod(numberAsString));
    }

    return output;
}

typedef vector<urdf::LinkSharedPtr> URDFLinkVector;
typedef vector<urdf::JointSharedPtr> URDFJointVector;
typedef map<string, urdf::LinkSharedPtr> URDFLinkMap;
typedef map<string, urdf::JointSharedPtr> URDFJointMap;

bool construct_model(ModelPtr rdl_model, urdf::ModelInterfaceSharedPtr urdf_model, bool floating_base, const std::map<std::string, hydro::BodyHydroData>& hydroData,
                     JointType floating_base_rotation_joint_type, bool verbose)
{
    urdf::LinkSharedPtr urdf_root_link;

    URDFLinkMap link_map;
    link_map = urdf_model->links_;

    URDFJointMap joint_map;
    joint_map = urdf_model->joints_;

    vector<string> joint_names;

    stack<urdf::LinkSharedPtr> link_stack;
    stack<int> joint_index_stack;

    // add the bodies in a depth-first order of the model tree
    link_stack.push(link_map[(urdf_model->getRoot()->name)]);

    // add the root body
    urdf::LinkConstSharedPtr root = urdf_model->getRoot();

    if (root->inertial)
    {
        Vector3d root_inertial_position(root->inertial->origin.position.x, root->inertial->origin.position.y, root->inertial->origin.position.z);

        RobotDynamics::Math::Matrix3d root_inertia_in_com_frame(root->inertial->ixx, root->inertial->ixy, root->inertial->ixz, root->inertial->ixy, root->inertial->iyy,
                                                                root->inertial->iyz, root->inertial->ixz, root->inertial->iyz, root->inertial->izz);

        RigidBodyInertia I(root->inertial->mass, root_inertial_position, root_inertia_in_com_frame);
        // this transform here handles the case where the com frame has a non-trivial rotation
        I.transform(RobotDynamics::Math::XrotQuat(root->inertial->origin.rotation.x, root->inertial->origin.rotation.y, root->inertial->origin.rotation.z,
                                                  root->inertial->origin.rotation.w));
        Body root_link;
        hydro::BodyHydroData bodyHydroData = hydroData.at(root->name);
        if (bodyHydroData.hasHydro)
        {
            DragData dragData(bodyHydroData.linearDrag, bodyHydroData.quadraticDrag);
            root_link = Body(root->inertial->mass, root_inertial_position, Matrix3d(I.Ixx, I.Iyx, I.Izx, I.Iyx, I.Iyy, I.Izy, I.Izx, I.Izy, I.Izz),
                             bodyHydroData.centerOfBuoyancy, bodyHydroData.volume, bodyHydroData.addedMassMatrix, dragData);
        }
        else
        {
            root_link = Body(root->inertial->mass, root_inertial_position, Matrix3d(I.Ixx, I.Iyx, I.Izx, I.Iyx, I.Iyy, I.Izy, I.Izx, I.Izy, I.Izz));
        }

        Joint root_joint(JointTypeFixed);
        if (floating_base && floating_base_rotation_joint_type == JointTypeSpherical)
        {
            root_joint = Joint(JointTypeFloatingBase);
        }
        else if (floating_base && floating_base_rotation_joint_type != JointTypeSpherical)
        {
            Body null_body(0., Vector3d(0., 0., 0.), Vector3d(0., 0., 0.));
            null_body.mIsVirtual = true;

            rdl_model->appendBody(SpatialTransform(), Joint(JointTypeTranslationXYZ), null_body);

            if (floating_base_rotation_joint_type == JointTypeEulerZYX)
            {
                root_joint = Joint(JointTypeEulerZYX);
            }
            else if (floating_base_rotation_joint_type == JointTypeEulerXYZ)
            {
                root_joint = Joint(JointTypeEulerXYZ);
            }
            else
            {
                std::cerr << "floating base rotation joint type " << floating_base_rotation_joint_type << " not currently supported by urdfreader" << std::endl;
                return false;
            }
        }

        SpatialTransform root_joint_frame = SpatialTransform();

        if (verbose)
        {
            cout << "+ Adding Root Body " << endl;
            cout << "  joint frame: " << root_joint_frame << endl;
            if (floating_base)
            {
                cout << "  joint type : floating" << endl;
            }
            else
            {
                cout << "  joint type : fixed" << endl;
            }
            cout << "  body inertia: " << endl << root_link.mInertia << endl;
            cout << "  body mass   : " << root_link.mMass << endl;
            cout << "  body name   : " << root->name << endl;
        }

        rdl_model->appendBody(root_joint_frame, root_joint, root_link, root->name);
    }

    if (link_stack.top()->child_joints.size() > 0)
    {
        joint_index_stack.push(0);
    }

    while (!link_stack.empty())
    {
        urdf::LinkSharedPtr cur_link = link_stack.top();
        unsigned int joint_idx;
        if (joint_index_stack.size() > 0)
        {
            joint_idx = joint_index_stack.top();
        }
        else
        {
            joint_idx = 0;
        }

        if (joint_idx < cur_link->child_joints.size())
        {
            urdf::JointSharedPtr cur_joint = cur_link->child_joints[joint_idx];

            // increment joint index
            joint_index_stack.pop();
            joint_index_stack.push(joint_idx + 1);

            link_stack.push(link_map[cur_joint->child_link_name]);
            joint_index_stack.push(0);

            if (verbose)
            {
                for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++)
                {
                    cout << "  ";
                }
                cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
            }

            joint_names.push_back(cur_joint->name);
        }
        else
        {
            link_stack.pop();
            joint_index_stack.pop();
        }
    }

    for (size_t j = 0; j < joint_names.size(); j++)
    {
        urdf::JointSharedPtr urdf_joint = joint_map[joint_names[j]];
        urdf::LinkSharedPtr urdf_parent = link_map[urdf_joint->parent_link_name];
        urdf::LinkSharedPtr urdf_child = link_map[urdf_joint->child_link_name];

        // determine where to add the current joint and child body
        unsigned int rdl_parent_id = 0;
        if (rdl_model->mBodies.size() != 1 || rdl_model->mFixedBodies.size() != 0)
        {
            rdl_parent_id = rdl_model->GetBodyId(urdf_parent->name.c_str());
        }

        if (rdl_parent_id == std::numeric_limits<unsigned int>::max())
        {
            cerr << "Error while processing joint '" << urdf_joint->name << "': parent link '" << urdf_parent->name << "' could not be found." << endl;
        }

        // create the joint
        Joint rdl_joint;
        if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS)
        {
            rdl_joint = Joint(SpatialVector(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));
        }
        else if (urdf_joint->type == urdf::Joint::PRISMATIC)
        {
            rdl_joint = Joint(SpatialVector(0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        }
        else if (urdf_joint->type == urdf::Joint::FIXED)
        {
            rdl_joint = Joint(JointTypeFixed);
        }
        else if (urdf_joint->type == urdf::Joint::FLOATING)
        {
            // todo: what order of DoF should be used?
            rdl_joint = Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.), SpatialVector(0., 0., 0., 0., 0., 1.),
                              SpatialVector(1., 0., 0., 0., 0., 0.), SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));
        }
        else if (urdf_joint->type == urdf::Joint::PLANAR)
        {
            // todo: which two directions should be used that are perpendicular
            // to the specified axis?
            cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << endl;
            return false;
        }

        // compute the joint transformation
        Vector3d joint_rpy;
        Vector3d joint_translation;
        urdf_joint->parent_to_joint_origin_transform.rotation.getRPY(joint_rpy[0], joint_rpy[1], joint_rpy[2]);
        joint_translation.set(urdf_joint->parent_to_joint_origin_transform.position.x, urdf_joint->parent_to_joint_origin_transform.position.y,
                              urdf_joint->parent_to_joint_origin_transform.position.z);
        SpatialTransform rdl_joint_frame = Xrot(joint_rpy[0], Vector3d(1., 0., 0.)) * Xrot(joint_rpy[1], Vector3d(0., 1., 0.)) *
                                           Xrot(joint_rpy[2], Vector3d(0., 0., 1.)) * Xtrans(Vector3d(joint_translation));

        // assemble the body
        Vector3d link_inertial_position;
        Vector3d link_inertial_rpy;
        Matrix3d link_inertial_inertia = Matrix3d::Zero();
        double link_inertial_mass = 0.;

        // but only if we actually have inertial data
        if (urdf_child->inertial)
        {
            link_inertial_mass = urdf_child->inertial->mass;

            link_inertial_position.set(urdf_child->inertial->origin.position.x, urdf_child->inertial->origin.position.y, urdf_child->inertial->origin.position.z);
            urdf_child->inertial->origin.rotation.getRPY(link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

            RobotDynamics::Math::Matrix3d link_inertia_in_com_frame = RobotDynamics::Math::Matrix3dZero;
            link_inertia_in_com_frame(0, 0) = urdf_child->inertial->ixx;
            link_inertia_in_com_frame(0, 1) = urdf_child->inertial->ixy;
            link_inertia_in_com_frame(0, 2) = urdf_child->inertial->ixz;
            link_inertia_in_com_frame(1, 0) = urdf_child->inertial->ixy;
            link_inertia_in_com_frame(1, 1) = urdf_child->inertial->iyy;
            link_inertia_in_com_frame(1, 2) = urdf_child->inertial->iyz;
            link_inertia_in_com_frame(2, 0) = urdf_child->inertial->ixz;
            link_inertia_in_com_frame(2, 1) = urdf_child->inertial->iyz;
            link_inertia_in_com_frame(2, 2) = urdf_child->inertial->izz;

            RobotDynamics::Math::SpatialTransform X_body_to_inertial = RobotDynamics::Math::XeulerXYZ(link_inertial_rpy);
            link_inertial_inertia = link_inertial_rpy == Vector3d(0., 0., 0.) ? link_inertia_in_com_frame :
                                                                                X_body_to_inertial.E.transpose() * link_inertia_in_com_frame * X_body_to_inertial.E;
        }

        Body rdl_body;
        hydro::BodyHydroData bodyHydroData = hydroData.at(urdf_child->name);
        if (bodyHydroData.hasHydro)
        {
            DragData dragData(bodyHydroData.linearDrag, bodyHydroData.quadraticDrag);
            rdl_body = Body(link_inertial_mass, link_inertial_position, link_inertial_inertia, bodyHydroData.centerOfBuoyancy, bodyHydroData.volume,
                            bodyHydroData.addedMassMatrix, dragData);
        }
        else
        {
            rdl_body = Body(link_inertial_mass, link_inertial_position, link_inertial_inertia);
        }

        if (verbose)
        {
            cout << "+ Adding Body " << endl;
            cout << "  parent_id  : " << rdl_parent_id << endl;
            cout << "  joint frame: " << rdl_joint_frame << endl;
            cout << "  joint dofs : " << rdl_joint.mDoFCount << endl;
            for (unsigned int j = 0; j < rdl_joint.mDoFCount; j++)
            {
                cout << "    " << j << ": " << rdl_joint.mJointAxes[j].transpose() << endl;
            }
            cout << "  body inertia: " << endl << rdl_body.mInertia << endl;
            cout << "  body mass   : " << rdl_body.mMass << endl;
            cout << "  body name   : " << urdf_child->name << endl;
            cout << "  body has_hydro :" << bodyHydroData.hasHydro << std::endl;
            cout << "  body volume   : " << bodyHydroData.volume << endl;
            cout << "  body center_of_buoyancy   : " << bodyHydroData.centerOfBuoyancy.transpose() << endl;
        }

        if (urdf_joint->type == urdf::Joint::FLOATING)
        {
            Matrix3d zero_matrix = Matrix3d::Zero();
            Body null_body(0., Vector3d::Zero(3), zero_matrix);
            Joint joint_txtytz(JointTypeTranslationXYZ);
            string trans_body_name = urdf_child->name + "_Translate";
            rdl_model->addBody(rdl_parent_id, rdl_joint_frame, joint_txtytz, null_body, trans_body_name);

            Joint joint_euler_zyx(JointTypeEulerXYZ);
            rdl_model->appendBody(SpatialTransform(), joint_euler_zyx, rdl_body, urdf_child->name);
        }
        else
        {
            rdl_model->addBody(rdl_parent_id, rdl_joint_frame, rdl_joint, rdl_body, urdf_child->name);
        }
    }

    return true;
}

bool parseJointBodyNameMapFromFile(const std::string& filename, std::map<std::string, std::string>& jointBodyMap)
{
    ifstream model_file(filename);
    if (!model_file)
    {
        cerr << "Error opening file '" << filename << "'." << endl;
        return false;
    }

    // reserve memory for the contents of the file
    std::string model_xml_string;
    model_file.seekg(0, std::ios::end);
    model_xml_string.reserve(model_file.tellg());
    model_file.seekg(0, std::ios::beg);
    model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

    model_file.close();

    return parseJointBodyNameMapFromString(model_xml_string.c_str(), jointBodyMap);
}

bool parseJointAndBodyNamesFromString(const std::string& model_xml_string, std::vector<std::string>& joint_names, std::vector<std::string>& body_names)
{
    TiXmlDocument doc;

    // Check if contents are valid, if not, abort
    if (!doc.Parse(model_xml_string.c_str()) && doc.Error())
    {
        std::cerr << "Can't parse urdf. Xml is invalid" << std::endl;
        return false;
    }

    // Find joints in transmission tags
    TiXmlElement* root = doc.RootElement();

    for (TiXmlElement* joint = root->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
    {
        if (!std::strcmp(joint->Attribute("type"), "fixed") || !std::strcmp(joint->Attribute("type"), "floating"))
        {
            continue;
        }

        joint_names.push_back(joint->Attribute("name"));
        body_names.push_back(joint->FirstChildElement("child")->Attribute("link"));
    }

    return true;
}

bool parseJointBodyNameMapFromString(const std::string& model_xml_string, std::map<std::string, std::string>& jointBodyMap)
{
    std::vector<std::string> joint_names, body_names;
    if (!parseJointAndBodyNamesFromString(model_xml_string, joint_names, body_names))
    {
        return false;
    }

    jointBodyMap.clear();
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        jointBodyMap.insert(std::pair<std::string, std::string>(joint_names[i], body_names[i]));
    }

    return true;
}

bool parseJointAndQIndex(const RobotDynamics::Model& model, const std::vector<std::string>& body_names, std::vector<unsigned int>& q_indices)
{
    q_indices.clear();
    for (unsigned int i = 0; i < body_names.size(); i++)
    {
        q_indices.push_back(model.mJoints[model.GetBodyId(body_names[i].c_str())].q_index);
    }

    return true;
}

bool parseJointAndQIndex(const std::string& model_xml_string, std::vector<unsigned int>& q_indices)
{
    RobotDynamics::ModelPtr model(new RobotDynamics::Model());
    if (!urdfReadFromString(model_xml_string, model))
    {
        return false;
    }

    std::vector<std::string> joint_names, body_names;
    if (!parseJointAndBodyNamesFromString(model_xml_string, joint_names, body_names))
    {
        return false;
    }

    return parseJointAndQIndex(*model, body_names, q_indices);
}

bool urdfReadFromFile(const std::string& filename, ModelPtr model, JointType floating_base_rotation_joint_type, bool verbose)
{
    ifstream model_file(filename.c_str());
    if (!model_file)
    {
        cerr << "Error opening file '" << filename << "'." << endl;
        return false;
    }

    // reserve memory for the contents of the file
    std::string model_xml_string;
    model_file.seekg(0, std::ios::end);
    model_xml_string.reserve(model_file.tellg());
    model_file.seekg(0, std::ios::beg);
    model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

    model_file.close();

    return urdfReadFromString(model_xml_string, model, floating_base_rotation_joint_type, verbose);
}

bool urdfReadFromString(const std::string& model_xml_string, ModelPtr model, JointType floating_base_rotation_joint_type, bool verbose)
{
    if (!model)
    {
        cerr << "Rdl model is nullptr" << std::endl;
        return false;
    }

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(model_xml_string.c_str());

    bool floating_base = std::strcmp(urdf_model->getRoot()->name.c_str(), "world") ? true : false;

    std::map<std::string, hydro::BodyHydroData> bodyHydroData;
    if (!hydro::parseHydrodynamicsParameters(model_xml_string.c_str(), bodyHydroData, false))
    {
        std::cerr << "Failed to parse hydrodynamics terms from urdf" << std::endl;
        return false;
    }

    if (!construct_model(model, urdf_model, floating_base, bodyHydroData, floating_base_rotation_joint_type, verbose))
    {
        cerr << "Error constructing model from urdf file." << endl;
        return false;
    }

    model->gravity.SpatialVector::set(0., 0., 0., 0., 0., -9.81);

    return true;
}

bool hydro::parseHydrodynamicsParameters(const char* model_xml_string, std::map<std::string, hydro::BodyHydroData>& data, bool verbose)
{
    TiXmlDocument doc;
    doc.Parse(model_xml_string);
    if (doc.Error())
    {
        std::cerr << "Error encountered parsing xml urdf. Error desc: " << doc.ErrorDesc() << std::endl;
        return false;
    }

    TiXmlElement* root = doc.RootElement();
    TiXmlElement* link_it = nullptr;

    for (link_it = root->FirstChildElement("link"); link_it; link_it = link_it->NextSiblingElement("link"))
    {
        TiXmlElement* hydro;

        std::string link_name = link_it->Attribute("name");
        hydro::BodyHydroData hydro_data;
        hydro = link_it->FirstChildElement("hydro");

        if (verbose)
        {
            std::cout << "---- Parsing hydro parameters for body " << link_name << "----" << std::endl;
        }

        if (!hydro)
        {
            // Doesn't have a hydro tag
            hydro_data.hasHydro = false;
            data[link_name] = hydro_data;

            if (verbose)
            {
                std::cout << "No <hydro> block found, skipping" << std::endl;
            }

            continue;  // This link has no hydro parameters, so just move on to the next one
        }
        else
        {
            if (verbose)
            {
                std::cout << "Found <hydro> block, proceeding" << std::endl;
            }
            hydro_data.hasHydro = true;
        }

        TiXmlElement* buoyancy;
        buoyancy = hydro->FirstChildElement("buoyancy");

        if (!buoyancy)
        {
            if (verbose)
            {
                std::cout << "No <buoyancy> block found" << std::endl;
            }

            hydro_data.volume = 0.0;
            hydro_data.centerOfBuoyancy = Math::Vector3d(0., 0., 0.);
        }
        else
        {
            hydro_data.volume = strtod(buoyancy->FirstChildElement("volume")->GetText(), nullptr);
            std::string origin_str = buoyancy->FirstChildElement("origin")->Attribute("xyz");
            std::stringstream ss(origin_str);
            std::vector<double> xyz(3);
            for (unsigned int i = 0; i < xyz.size(); ++i)
            {
                ss >> xyz[i];
            }

            hydro_data.centerOfBuoyancy.x() = xyz[0];
            hydro_data.centerOfBuoyancy.y() = xyz[1];
            hydro_data.centerOfBuoyancy.z() = xyz[2];
        }

        if (verbose)
        {
            std::cout << "  Buoyancy params: " << std::endl;
            std::cout << "     volume  : " << hydro_data.volume << endl;
            std::cout << "     center of buoyancy: " << hydro_data.centerOfBuoyancy.x() << ", " << hydro_data.centerOfBuoyancy.y() << ", "
                      << hydro_data.centerOfBuoyancy.z() << endl;
        }

        TiXmlElement* addedMass = hydro->FirstChildElement("added_mass");
        if (addedMass)
        {
            std::vector<double> added_mass_vec;
            added_mass_vec = xmlElementToVector(addedMass);
            if (added_mass_vec.size() != 36)
            {
                std::cerr << "Added mass matrix for " << link_name << " only has " << added_mass_vec.size()
                          << " elements. It MUST be a 6x6 matrix so it needs 36 elements" << std::endl;
                return false;
            }

            RobotDynamics::Math::SpatialMatrix added_mass;
            for (unsigned int i = 0; i < added_mass.rows(); i++)
            {
                for (unsigned int j = 0; j < added_mass.rows(); j++)
                {
                    added_mass(i, j) = added_mass_vec[6 * i + j];
                }
            }

            /**
             * Flip the order bc in RDL everything is angular part first
             */
            hydro_data.addedMassMatrix.block<3, 3>(0, 0) = added_mass.block<3, 3>(3, 3);
            hydro_data.addedMassMatrix.block<3, 3>(3, 0) = added_mass.block<3, 3>(0, 3);
            hydro_data.addedMassMatrix.block<3, 3>(0, 3) = added_mass.block<3, 3>(3, 0);
            hydro_data.addedMassMatrix.block<3, 3>(3, 3) = added_mass.block<3, 3>(0, 0);
        }

        if (verbose)
        {
            std::cout << "  Added mass matrix: \n" << std::endl;
            std::cout << hydro_data.addedMassMatrix << std::endl;
        }

        TiXmlElement* drag = hydro->FirstChildElement("drag");
        if (!drag)
        {
            for (unsigned int i = 0; i < 6; i++)
            {
                hydro_data.linearDrag[i] = 0.;
                hydro_data.quadraticDrag[i] = 0.;
            }
        }
        else
        {
            TiXmlElement* linear_damping = drag->FirstChildElement("linear_damping");
            TiXmlElement* quadratic_damping = drag->FirstChildElement("quadratic_damping");

            if (linear_damping)
            {
                std::vector<double> linearDrag = xmlElementToVector(linear_damping);

                if (linearDrag.size() != 6)
                {
                    std::cerr << "Linear drag vector for " << link_name << " only has " << std::to_string(hydro_data.linearDrag.size()) << "elements, it must have 6!"
                              << std::endl;
                    return false;
                }

                /**
                 * RDL does everything with angular first, so these have to be flipped assuming they're
                 * put in the urdf in the order x y z r p y
                 */
                hydro_data.linearDrag[0] = linearDrag[3];
                hydro_data.linearDrag[1] = linearDrag[4];
                hydro_data.linearDrag[2] = linearDrag[5];
                hydro_data.linearDrag[3] = linearDrag[0];
                hydro_data.linearDrag[4] = linearDrag[1];
                hydro_data.linearDrag[5] = linearDrag[2];
            }

            if (quadratic_damping)
            {
                std::vector<double> quadraticDrag = xmlElementToVector(quadratic_damping);

                if (quadraticDrag.size() != 6)
                {
                    std::cerr << "Quadratic drag vector for " << link_name << " only has " << std::to_string(hydro_data.quadraticDrag.size())
                              << "elements, it must have 6!" << std::endl;
                    return false;
                }

                /**
                 * RDL does everything with angular first, so these have to be flipped assuming they're
                 * put in the urdf in the order x y z r p y
                 */
                hydro_data.quadraticDrag[0] = quadraticDrag[3];
                hydro_data.quadraticDrag[1] = quadraticDrag[4];
                hydro_data.quadraticDrag[2] = quadraticDrag[5];
                hydro_data.quadraticDrag[3] = quadraticDrag[0];
                hydro_data.quadraticDrag[4] = quadraticDrag[1];
                hydro_data.quadraticDrag[5] = quadraticDrag[2];
            }
        }

        if (verbose)
        {
            std::cout << "  Drag params: " << std::endl;
            std::cout << "     Linear drag  : " << hydro_data.linearDrag[0] << " " << hydro_data.linearDrag[1] << " " << hydro_data.linearDrag[2] << " "
                      << hydro_data.linearDrag[3] << " " << hydro_data.linearDrag[4] << " " << hydro_data.linearDrag[5] << endl;
            std::cout << "     Quadratic drag  : " << hydro_data.quadraticDrag[0] << " " << hydro_data.quadraticDrag[1] << " " << hydro_data.quadraticDrag[2] << " "
                      << hydro_data.quadraticDrag[3] << " " << hydro_data.quadraticDrag[4] << " " << hydro_data.quadraticDrag[5] << endl;
        }

        data[link_name] = hydro_data;
    }

    return true;
}
}  // namespace Urdf
}  // namespace RobotDynamics
