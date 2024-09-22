# Copyright (c) 2017 Jordan Lack <jlack1987@gmail.com>
# RDL - Robot Dynamics Library
# Licensed under the zlib license. See LICENSE for more details.

find_package(Boost REQUIRED COMPONENTS system thread)
list(APPEND rdl_dynamics_LIBRARIES ${Boost_LIBRARIES})
