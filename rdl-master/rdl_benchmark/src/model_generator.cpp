// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#include <rdl_benchmark/model_generator.hpp>
#include <rdl_dynamics/Model.hpp>

using namespace RobotDynamics;
using namespace RobotDynamics::Math;

void generate_planar_tree_recursive(ModelPtr model, unsigned int parent_body_id, int depth, double length)
{
    if (depth == 0)
    {
        return;
    }

    // create left child
    Joint joint_rot_z(JointTypeRevolute, Vector3d(0., 0., 1.));
    Body body(length, Vector3d(0., -0.25 * length, 0.), Vector3d(length, length, length));

    Vector3d displacement(-0.5 * length, -0.25 * length, 0.);
    unsigned int child_left = model->addBody(parent_body_id, Xtrans(displacement), joint_rot_z, body);

    generate_planar_tree_recursive(model, child_left, depth - 1, length * 0.4);

    displacement.set(0.5 * length, -0.25 * length, 0.);
    unsigned int child_right = model->addBody(parent_body_id, Xtrans(displacement), joint_rot_z, body);

    generate_planar_tree_recursive(model, child_right, depth - 1, length * 0.4);
}

void generate_planar_tree(ModelPtr model, int depth)
{
    // we first add a single body that is hanging straight down from
    // (0, 0, 0). After that we generate the tree recursively such that each
    // call adds two children.
    //
    double length = 1.;

    Joint joint_rot_z(JointTypeRevolute, Vector3d(0., 0., 1.));
    Body body(length, Vector3d(0., -0.25 * length, 0.), Vector3d(length, length, length));

    unsigned int base_child = model->addBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

    generate_planar_tree_recursive(model, base_child, depth, length * 0.4);
}
