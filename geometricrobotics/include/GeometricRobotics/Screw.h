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

#include <Eigen/Dense>
#include <iostream>

namespace GeoRobotics
{

struct ScrewProperties
{
  explicit ScrewProperties() {}
  ///< The point on the axis
  Vector3d q{Vector3d::Zero()};

  ///< The unit normal of the directed axis
  Vector3d direction{Vector3d::Zero()};

  ///< The direction writes: q + lambda * direction
  double lambda{1.0};

  ///< Ratio between translation and rotation: (1) 0.0: pure rotation; (2) finite: rotation and translation; (3)
  ///< infinite: pure translation
  double pitch{0.0};

  ///< The translation distance
  double translation{0.0};

  ///< The rotation angle
  double angle{0.0};
};

class Screw
{
public:
  Screw() {}
  ~Screw() {}

  const ScrewProperties & getProperties()
  {
    return properties_;
  }

protected:
  ScrewProperties properties_{};
};

} // namespace GeoRobotics
