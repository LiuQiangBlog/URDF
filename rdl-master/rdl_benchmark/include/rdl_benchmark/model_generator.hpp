// Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
// Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
// RDL - Robot Dynamics Library
// Licensed under the zlib license. See LICENSE for more details.

#ifndef _MODEL_GENERATOR_H
#define _MODEL_GENERATOR_H

#include <rdl_dynamics/Model.hpp>

namespace RobotDynamics
{
class Model;
}

void generate_planar_tree(RobotDynamics::ModelPtr model, int depth);

/* _MODEL_GENERATOR_H */
#endif
