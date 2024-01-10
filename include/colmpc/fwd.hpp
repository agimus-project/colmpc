
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
}


#endif
