#ifndef __sobec_python__
#define __sobec_python__

#include <eigenpy/eigenpy.hpp>
#include <pinocchio/fwd.hpp>

namespace sobec {
namespace python {

void exposeStdContainers();
void exposeResidualCoMVelocity();
void exposeResidualVelCollision();
void exposeResidualCenterOfPressure();
void exposeResidualFeetCollision();
void exposeResidualFlyHigh();
void exposeResidualFlyAngle();
void exposeResidualDCMPosition();
void exposeResidual2DSurface();
void exposeActivationQuadRef();
void exposeDesigner();
void exposeHorizonManager();
void exposeModelFactory();
void exposeIntegratedActionLPF();
void exposeStateLPF();
void exposeDAMContactFwdDyn();
void exposeResidualContactForce();
void exposeWBC();
void exposeWBCHorizon();
void exposeFootTrajectory();
void exposeFlex();
void exposeOCPWalk();
void exposeMPCWalk();

}  // namespace python
}  // namespace sobec

namespace sobec {
namespace newcontacts {
namespace python {

void exposeResidualContactForce();
void exposeDAMContactFwdDyn();
void exposeContact6D();
void exposeContact3D();
void exposeContact1D();
void exposeMultipleContacts();

}  // namespace python
}  // namespace newcontacts
}  // namespace sobec

#endif  // #ifndef __sobec_python__
