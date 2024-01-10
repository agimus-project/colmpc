#ifndef __colmpc_python__
#define __colmpc_python__

#include <pinocchio/fwd.hpp>
// include pinocchio first
#include <eigenpy/eigenpy.hpp>

namespace colmpc {
namespace python {

void exposeResidualDistanceCollision();

}  // namespace python
}  // namespace colmpc

#endif  // #ifndef __colmpc_python__
