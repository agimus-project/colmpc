#ifndef __colmpc_python__
#define __colmpc_python__

#include <eigenpy/eigenpy.hpp>
#include <pinocchio/fwd.hpp>

namespace colmpc {
namespace python {

void exposeStdContainers();
void exposeResidualDistanceCollision();

}  // namespace python
}  // namespace colmpc

#endif  // #ifndef __colmpc_python__
