import numpy as np
import pinocchio as pin
import crocoddyl
from crocoddyl.utils import *
import hppfcl


class ResidualCollision(crocoddyl.ResidualModelAbstract):
    """Class computing the residual of the collision constraint. This residual is simply the signed distance between the two closest points of the 2 shapes."""

    def __init__(
        self,
        state,
        geom_model: pin.Model,
        geom_data,
        pair_id: int,
    ):
        """Class computing the residual of the collision constraint. This residual is simply the signed distance between the two closest points of the 2 shapes.

        Args:
            state (crocoddyl.StateMultibody): _description_
            geom_model (pin.Model): Collision model of pinocchio
            geom_data (_type_): Collision data of the collision model of pinocchio
            pair_id (int): ID of the collision pair
        """
        crocoddyl.ResidualModelAbstract.__init__(self, state, 1, True, True, True)

        # Pinocchio robot model
        self._pinocchio = self.state.pinocchio

        # Geometry model of the robot
        self._geom_model = geom_model

        # Geometry data of the robot
        self._geom_data = geom_data

        # Pair ID of the collisionPair
        self._pair_id = pair_id

        # Number of joints
        self._nq = self._pinocchio.nq

        # Making sure that the pair of collision exists
        assert self._pair_id <= len(self._geom_model.collisionPairs)

        # Collision pair
        self._collisionPair = self._geom_model.collisionPairs[self._pair_id]

        # Geometry ID of the shape 1 of collision pair
        self._shape1_id = self._collisionPair.first

        # Making sure that the frame exists
        assert self._shape1_id <= len(self._geom_model.geometryObjects)

        # Geometry object shape 1
        self._shape1 = self._geom_model.geometryObjects[self._shape1_id]

        # Shape 1 parent joint
        self._shape1_parentJoint = self._shape1.parentJoint

        # Geometry ID of the shape 2 of collision pair
        self._shape2_id = self._collisionPair.second

        # Making sure that the frame exists
        assert self._shape2_id <= len(self._geom_model.geometryObjects)

        # Geometry object shape 2
        self._shape2 = self._geom_model.geometryObjects[self._shape2_id]

        # Shape 2 parent joint
        self._shape2_parentJoint = self._shape2.parentJoint

        # Checking that shape 1 is belonging to the robot & shape 2 is the obstacle
        assert not "obstacle" in self._shape1.name
        assert "obstacle" in self._shape2.name

    def calc(self, data, x, u=None):
        data.r[:] = self.f(data, x[: self._nq])

    def f(self, data, q):
        # Storing q outside of the state vector
        self.q = q

        ### Computes the distance for the collision pair pair_id
        # Updating the position of the joints & the geometry objects.
        pin.updateGeometryPlacements(
            self._pinocchio,
            data.shared.pinocchio,
            self._geom_model,
            self._geom_data,
            self.q,
        )

        # Distance Request & Result from hppfcl / pydiffcol
        self._req = hppfcl.DistanceRequest()
        self._res = hppfcl.DistanceResult()

        # Getting the geometry of the shape 1
        self._shape1_geom = self._shape1.geometry

        # Getting its pose in the world reference
        self._shape1_placement = self._geom_data.oMg[self._shape1_id]

        # Doing the same for the second shape.
        self._shape2_geom = self._shape2.geometry
        self._shape2_placement = self._geom_data.oMg[self._shape2_id]

        # Computing the distance
        distance = hppfcl.distance(
            self._shape1_geom,
            hppfcl.Transform3f(
                self._shape1_placement.rotation, self._shape1_placement.translation
            ),
            self._shape2_geom,
            hppfcl.Transform3f(
                self._shape2_placement.rotation, self._shape2_placement.translation
            ),
            self._req,
            self._res,
        )

        return distance


    def calcDiff(self, data, x, u=None):
        # self.calcDiff_numdiff(data, x)
        # J_n = self._J

        self.calcDiff_ana(data, x)
        J_f = self._J

        # J_diff = J_f - J_n

        # for k in J_diff:
        #     if np.linalg.norm(k) > 1e-5:
        #         print(J_diff)
        #         print(x[:6].tolist())
        #         print('-----------')

        data.Rx[: self._nq] = self._J

    def calcDiff_numdiff(self, data, x):
        j_diff = np.zeros(self._nq)
        fx = self.f(data, x[: self._nq])
        for i in range(self._nq):
            e = np.zeros(self._nq)
            e[i] = 1e-6
            j_diff[i] = (self.f(data, x[: self._nq] + e) - fx) / e[i]
        self._J = j_diff
        self._distance_numdiff = fx

    def calcDiff_ana(self, data, x):
        jacobian1 = pin.computeFrameJacobian(
            self._pinocchio,
            data.shared.pinocchio,
            self.q,
            self._shape1.parentFrame,
            pin.LOCAL_WORLD_ALIGNED,
        )

        jacobian2 = pin.computeFrameJacobian(
            self._pinocchio,
            data.shared.pinocchio,
            self.q,
            self._shape2.parentFrame,
            pin.LOCAL_WORLD_ALIGNED,
        )

        # Computing the distance
        distance = hppfcl.distance(
            self._shape1.geometry,
            hppfcl.Transform3f(
                self._shape1_placement.rotation, self._shape1_placement.translation
            ),
            self._shape2.geometry,
            hppfcl.Transform3f(
                self._shape2_placement.rotation, self._shape2_placement.translation
            ),
            self._req,
            self._res,
        )
        cp1 = self._res.getNearestPoint1()
        cp2 = self._res.getNearestPoint2()

        ## Transport the jacobian of frame 1 into the jacobian associated to cp1
        # Vector from frame 1 center to p1
        f1p1 = cp1 - data.shared.pinocchio.oMf[self._shape1.parentFrame].translation
        # The following 2 lines are the easiest way to understand the transformation
        # although not the most efficient way to compute it.
        f1Mp1 = pin.SE3(np.eye(3), f1p1)
        jacobian1 = f1Mp1.actionInverse @ jacobian1

        ## Transport the jacobian of frame 2 into the jacobian associated to cp2
        # Vector from frame 2 center to p2
        f2p2 = cp2 - data.shared.pinocchio.oMf[self._shape2.parentFrame].translation
        # The following 2 lines are the easiest way to understand the transformation
        # although not the most efficient way to compute it.
        f2Mp2 = pin.SE3(np.eye(3), f2p2)
        jacobian2 = f2Mp2.actionInverse @ jacobian2

        CP1_SE3 = pin.SE3.Identity()
        CP1_SE3.translation = cp1

        CP2_SE3 = pin.SE3.Identity()
        CP2_SE3.translation = cp2
        self._J = (cp1 - cp2).T / distance @ (jacobian1[:3] - jacobian2[:3])



if __name__ == "__main__":
    pass
