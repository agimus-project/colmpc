#ifndef __sobec_py2cpp__
#define __sobec_py2cpp__

#include <boost/smart_ptr.hpp>
#include <crocoddyl/core/fwd.hpp>
#include <sobec/fwd.hpp>

namespace sobec {
typedef boost::shared_ptr<crocoddyl::ShootingProblem> ShootingProblemPtr;
typedef boost::shared_ptr<sobec::MPCWalk> MPCWalkPtr;

ShootingProblemPtr initShootingProblem(const char* fileName);
MPCWalkPtr initMPCWalk(const char* fileName);
bool reprProblem(ShootingProblemPtr problem);

}  // namespace sobec

#endif  // #ifndef __sobec_py2cpp__
