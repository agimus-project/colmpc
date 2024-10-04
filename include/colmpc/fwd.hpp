// BSD 3-Clause License
// 
// Copyright (C) 2024, LAAS-CNRS.
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.

#ifndef COLMPC_FWDL_HPP_
#define COLMPC_FWDL_HPP_

#include <pinocchio/fwd.hpp>

namespace colmpc {

template <typename Scalar>
class ResidualDistanceCollisionTpl;
typedef ResidualDistanceCollisionTpl<double> ResidualDistanceCollision;
template <typename Scalar>
class ResidualDataDistanceCollisionTpl;
typedef ResidualDataDistanceCollisionTpl<double> ResidualDataDistanceCollision;

template <typename Scalar>
class ResidualModelVelocityAvoidanceTpl;
typedef ResidualModelVelocityAvoidanceTpl<double>
    ResidualModelVelocityAvoidance;
template <typename Scalar>
class ResidualDataVelocityAvoidanceTpl;
typedef ResidualDataVelocityAvoidanceTpl<double> ResidualDataVelocityAvoidance;

}  // namespace colmpc

#endif  // COLMPC_FWDL_HPP_
