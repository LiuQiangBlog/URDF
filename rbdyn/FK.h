/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "rbdyn/config.hh"

namespace rbd
{
class MultiBody;
struct MultiBodyConfig;

/**
 * Compute the forward kinematic of a MultiBody.
 * @param mb MultiBody used has model.
 * @param mbc Use q generalized position vector. Fill bodyPosW, jointConfig, motionSubspace
 * and parentToSon.
 */
void forwardKinematics(const MultiBody &mb, MultiBodyConfig &mbc);

/**
 * Safe version.
 * @see forwardKinematics.
 * @throw std::domain_error If there is a mismatch between mb and mbc.
 */
void sForwardKinematics(const MultiBody &mb, MultiBodyConfig &mbc);

} // namespace rbd
