#ifndef COLMPC_PYTHON_HPP_
#define COLMPC_PYTHON_HPP_

#include "colmpc/fwd.hpp"
// include fwd first
#include <eigenpy/eigenpy.hpp>

namespace colmpc {
namespace python {

void exposeResidualDistanceCollision();
void exposeResidualVelocityAvoidance();

}  // namespace python
}  // namespace colmpc

#endif  // COLMPC_PYTHON_HPP_
