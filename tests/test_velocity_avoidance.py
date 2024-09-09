import unittest

import crocoddyl
import hppfcl
import numdifftools as nd
import numpy as np
import pinocchio as pin
from scenes import Scene
from wrapper_panda import PandaWrapper

from colmpc import ResidualDataVelocityAvoidance, ResidualModelVelocityAvoidance


class TestResidualModelVelocityAvoidance:
    def __init__(
        self,
        state: ResidualDataVelocityAvoidance,
        geom_model: pin.Model,
        pair_id: int,
    ):
        self._pinocchio = state.pinocchio
        self._geom_model = geom_model
        self._pair_id = pair_id

        self._collisionPair = self._geom_model.collisionPairs[self._pair_id]
        self._shape1_id = self._collisionPair.first
        self._shape1 = self._geom_model.geometryObjects[self._shape1_id]
        self._shape2_id = self._collisionPair.second
        self._shape2 = self._geom_model.geometryObjects[self._shape2_id]

        self._di = 1e-2
        self._ds = 1e-5
        self._ksi = 0.01

    def calc(self, x: np.array, u: np.array) -> float:
        q = x[: self._pinocchio.nq]
        v = x[self._pinocchio.nq :]

        # Creating the data models
        rdata = self._pinocchio.createData()
        cdata = self._geom_model.createData()

        # Updating the position of the joints & the geometry objects.
        pin.forwardKinematics(self._pinocchio, rdata, q, v)
        pin.updateGeometryPlacements(self._pinocchio, rdata, self._geom_model, cdata)

        # Poses and geometries of the shapes
        shape1_placement = cdata.oMg[self._shape1_id]
        shape2_placement = cdata.oMg[self._shape2_id]

        req = hppfcl.DistanceRequest()
        res = hppfcl.DistanceResult()
        distance = hppfcl.distance(
            self._shape1.geometry,
            shape1_placement,
            self._shape2.geometry,
            shape2_placement,
            req,
            res,
        )
        x1 = res.getNearestPoint1()
        x2 = res.getNearestPoint2()

        c1 = shape1_placement.translation
        c2 = shape2_placement.translation

        v1 = pin.getFrameVelocity(
            self._pinocchio, rdata, self._shape1.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).linear
        v2 = pin.getFrameVelocity(
            self._pinocchio, rdata, self._shape2.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).linear
        w1 = pin.getFrameVelocity(
            self._pinocchio, rdata, self._shape1.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).angular
        w2 = pin.getFrameVelocity(
            self._pinocchio, rdata, self._shape2.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).angular

        Lc = (x1 - x2).T
        Lr1 = c1.T @ pin.skew(x2 - x1) + x2.T @ pin.skew(x1)
        Lr2 = c2.T @ pin.skew(x1 - x2) + x1.T @ pin.skew(x2)

        Ldot = Lc @ (v1 - v2) + Lr1 @ w1 + Lr2 @ w2
        d_dot = Ldot / distance
        return d_dot + self._ksi * (distance - self._ds) / (self._di - self._ds)


class TestVelocityAvoidance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load robot
        robot_wrapper = PandaWrapper(capsule=False)
        cls.rmodel, cmodel, _ = robot_wrapper()
        cls.rdata = cls.rmodel.createData()

        scene = Scene()
        # Update collision model
        cls.cmodel, _, q0 = scene.create_scene(cls.rmodel, cmodel, "box")
        cls.cdata = cls.cmodel.createData()

        # Create state and action model
        cls.state = crocoddyl.StateMultibody(cls.rmodel)
        actuation = crocoddyl.ActuationModelFull(cls.state)
        actuation_data = actuation.createData()
        shared_data = crocoddyl.DataCollectorActMultibody(cls.rdata, actuation_data)

        # Initialize C++ implementation
        cls.velocity_avoidance_residual = ResidualModelVelocityAvoidance(
            cls.state, 7, cls.cmodel, 0
        )
        cls.residual_data = cls.velocity_avoidance_residual.createData(shared_data)

        # Initialize Python test implementation
        cls.test_residual = TestResidualModelVelocityAvoidance(cls.state, cls.cmodel, 0)

    def _update_placement(self, q: np.array, v: np.array) -> None:
        pin.forwardKinematics(self.state.pinocchio, self.residual_data.pinocchio, q, v)
        pin.updateGeometryPlacements(
            self.state.pinocchio,
            self.residual_data.pinocchio,
            self.cmodel,
            self.cdata,
        )

    def test_calc(self):
        for _ in range(200):
            # Generate new random configuration
            q = pin.randomConfiguration(self.rmodel)
            v = np.random.rand(*q.shape)

            # Update poses and compute new residual value
            self._update_placement(q, v)
            # Create state vector
            x = np.concatenate((q, v))

            self.velocity_avoidance_residual.calc(
                self.residual_data, x, np.zeros(q.shape)
            )

            # Compute value of reference residual
            test_residual_result = self.test_residual.calc(x, np.zeros(q.shape))

            self.assertAlmostEqual(
                self.residual_data.r[0],
                test_residual_result,
                places=5,
                msg="Result missmatch in function ``calc`` between Python and C++ implementation!.",
            )

    def test_calc_diff_finite(self):
        def calc_wrapper(x: np.array) -> float:
            self._update_placement(x[: self.rmodel.nq], x[self.rmodel.nq :])
            self.velocity_avoidance_residual.calc(
                self.residual_data, x, np.zeros(len(x) // 2)
            )
            return self.residual_data.r[0]

        calcDiff_numerical = nd.Gradient(calc_wrapper)

        for _ in range(200):
            # Generate new random configuration
            q = pin.randomConfiguration(self.rmodel)
            v = np.random.rand(*q.shape)

            # Update poses and compute new residual value
            self._update_placement(q, v)
            # Create state vector
            x = np.concatenate((q, v))

            # Compute exact derivative
            self.velocity_avoidance_residual.calcDiff(
                self.residual_data, x, np.zeros(q.shape)
            )

            # Compute approximate derivative
            Rx_num = calcDiff_numerical(x)

            np.testing.assert_allclose(
                self.residual_data.Rx,
                Rx_num,
                rtol=1e-5,
                err_msg="Result missmatch in function ``calcDiff`` "
                "between C++ implementation and finite differences!.",
            )


if __name__ == "__main__":
    unittest.main()
