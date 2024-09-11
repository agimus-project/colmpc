import unittest

import crocoddyl
import hppfcl
import numdifftools as nd
import numpy as np
import pinocchio as pin
from scenes import Scene
from wrapper_panda import PandaWrapper

from colmpc import ResidualDataVelocityAvoidance, ResidualModelVelocityAvoidance

np.set_printoptions(precision=7, linewidth=350, suppress=True, threshold=1e6)


def numdiff(f, q, h=1e-6):
    # print(f"numdiff")
    # print(f"q: {q[:7]}")
    # print(f"v: {q[7:]}")
    # print("---------")

    j_diff = np.zeros(len(q))
    fx = f(q)
    for i in range(len(q)):
        e = np.zeros(len(q))
        e[i] = h
        j_diff[i] = (f(q + e) - fx) / e[i]
    return j_diff


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

        self._nq = self._pinocchio.nq

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

    def f(self, x):
        return self.calc(x, None)

    def calcDiff(self, data, x, u=None):
        ddistdot_dq_val = self.ddistdot_dq(self._pinocchio, self._geom_model, x)
        ddist_dq = np.r_[
            self.ddist_dq(self._pinocchio, self._geom_model, x), np.zeros(self._nq)
        ]
        nd = numdiff(self.f, x)
        return ddistdot_dq_val - ddist_dq * self._ksi / (self._di - self._ds)
        # print(f"ddotdq: {(data.Rx - nd)[:7]}")
        # print(f"ddotdvq: {(data.Rx - nd)[7:]}")

        # print(f"no nd: {np.linalg.norm(ddistdot_dq_val)}")
        # print(f"nd : {np.linalg.norm(nd)}")
        # print(np.max(nd -ddistdot_dq_val))
        # print("---------")
        # data.Rx[:] = nd

    def ddist_dq(self, rmodel, cmodel, x: np.ndarray):
        q = x[: rmodel.nq]
        v = x[rmodel.nq :]

        # Creating the data models
        rdata = rmodel.createData()
        cdata = cmodel.createData()
        # Updating the position of the joints & the geometry objects.
        pin.forwardKinematics(rmodel, rdata, q, v)
        pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata)

        # Poses and geometries of the shapes
        shape1_placement = cdata.oMg[self._shape1_id]
        shape2_placement = cdata.oMg[self._shape2_id]

        jacobian1 = pin.computeFrameJacobian(
            rmodel,
            rdata,
            q,
            self._shape1.parentFrame,
            pin.LOCAL_WORLD_ALIGNED,
        )

        jacobian2 = pin.computeFrameJacobian(
            rmodel,
            rdata,
            q,
            self._shape2.parentFrame,
            pin.LOCAL_WORLD_ALIGNED,
        )

        req = hppfcl.DistanceRequest()
        res = hppfcl.DistanceResult()
        # Computing the distance
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

        ## Transport the jacobian of frame 1 into the jacobian associated to x1
        # Vector from frame 1 center to p1
        f1p1 = x1 - rdata.oMf[self._shape1.parentFrame].translation
        # The following 2 lines are the easiest way to understand the transformation
        # although not the most efficient way to compute it.
        f1Mp1 = pin.SE3(np.eye(3), f1p1)
        jacobian1 = f1Mp1.actionInverse @ jacobian1

        ## Transport the jacobian of frame 2 into the jacobian associated to x2
        # Vector from frame 2 center to p2
        f2p2 = x2 - rdata.oMf[self._shape2.parentFrame].translation
        # The following 2 lines are the easiest way to understand the transformation
        # although not the most efficient way to compute it.
        f2Mp2 = pin.SE3(np.eye(3), f2p2)
        jacobian2 = f2Mp2.actionInverse @ jacobian2

        CP1_SE3 = pin.SE3.Identity()
        CP1_SE3.translation = x1

        CP2_SE3 = pin.SE3.Identity()
        CP2_SE3.translation = x2
        self._J = (x1 - x2).T / distance @ (jacobian1[:3] - jacobian2[:3])
        return self._J

    def ddistdot_dq(self, rmodel, cmodel, x: np.ndarray):
        q = x[: rmodel.nq]
        v = x[rmodel.nq :]

        # print(f"not numdiff")
        # print(f"q: {q}")
        # print(f"v: {v}")
        # print("---------")

        # Creating the data models
        rdata = rmodel.createData()
        cdata = cmodel.createData()

        # Updating the position of the joints & the geometry objects.
        pin.forwardKinematics(rmodel, rdata, q, v)
        pin.computeForwardKinematicsDerivatives(
            rmodel, rdata, q, v, np.zeros(rmodel.nq)
        )
        pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata)

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
            rmodel, rdata, self._shape1.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).linear
        v2 = pin.getFrameVelocity(
            rmodel, rdata, self._shape2.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).linear
        w1 = pin.getFrameVelocity(
            rmodel, rdata, self._shape1.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).angular
        w2 = pin.getFrameVelocity(
            rmodel, rdata, self._shape2.parentFrame, pin.LOCAL_WORLD_ALIGNED
        ).angular

        D1 = np.diagflat(self._shape1.geometry.radii)
        D2 = np.diagflat(self._shape2.geometry.radii)

        R1 = shape1_placement.rotation
        R2 = shape2_placement.rotation
        A1, A2 = R1 @ D1 @ R1.T, R2 @ D2 @ R2.T  # From pinocchio A = RDR.T

        sol_lam1, sol_lam2 = -(x1 - c1).T @ (x1 - x2), (x2 - c2).T @ (x1 - x2)

        theta_dot = np.r_[v1, v2, w1, w2]

        Ldot = (
            (x1 - x2) @ (v1 - v2)
            - np.cross(x1 - x2, x1 - c1) @ w1
            + np.cross(x1 - x2, x2 - c2) @ w2
        )
        dist_dot = Ldot / distance

        Lyy = np.r_[
            np.c_[np.eye(3) + sol_lam1 * A1, -np.eye(3), A1 @ (x1 - c1), np.zeros(3)],
            np.c_[-np.eye(3), np.eye(3) + sol_lam2 * A2, np.zeros(3), A2 @ (x2 - c2)],
            [np.r_[A1 @ (x1 - c1), np.zeros(3), np.zeros(2)]],
            [np.r_[np.zeros(3), A2 @ (x2 - c2), np.zeros(2)]],
        ]
        Lyc = np.r_[
            np.c_[-sol_lam1 * A1, np.zeros([3, 3])],
            np.c_[np.zeros([3, 3]), -sol_lam2 * A2],
            [np.r_[-A1 @ (x1 - c1), np.zeros(3)]],
            [np.r_[np.zeros(3), -A2 @ (x2 - c2)]],
        ]
        Lyr = np.r_[
            np.c_[
                sol_lam1 * (A1 @ pin.skew(x1 - c1) - pin.skew(A1 @ (x1 - c1))),
                np.zeros([3, 3]),
            ],
            np.c_[
                np.zeros([3, 3]),
                sol_lam2 * (A2 @ pin.skew(x2 - c2) - pin.skew(A2 @ (x2 - c2))),
            ],
            [np.r_[(x1 - c1) @ A1 @ pin.skew(x1 - c1), np.zeros(3)]],
            [np.r_[np.zeros(3), (x2 - c2) @ A2 @ pin.skew(x2 - c2)]],
        ]

        yc = -np.linalg.inv(Lyy) @ Lyc
        yr = -np.linalg.inv(Lyy) @ Lyr

        xc, xr = yc[:3], yr[:3]
        dx1 = np.c_[yc[:3], yr[:3]]
        dx2 = np.c_[yc[3:6], yr[3:6]]

        dL_dtheta = np.r_[
            x1 - x2,
            -(x1 - x2),
            -np.cross(x1 - x2, x1 - c1),
            np.cross(x1 - x2, x2 - c2),
        ]

        ddL_dtheta2 = (
            np.r_[
                dx1 - dx2,
                -dx1 + dx2,
                -pin.skew(x1 - x2) @ dx1 + pin.skew(x1 - c1) @ (dx1 - dx2),
                pin.skew(x1 - x2) @ dx2 - pin.skew(x2 - c2) @ (dx1 - dx2),
            ]
            + np.r_[
                np.zeros([6, 12]),
                np.c_[
                    pin.skew(x1 - x2), np.zeros([3, 9])
                ],  ###! Changed from np.c_[pin.skew(x1 - x2)
                np.c_[
                    np.zeros([3, 3]), -pin.skew(x1 - x2), np.zeros([3, 6])
                ],  ###!  - pin.skew(x1 - x2)
            ]
        )

        d_dist_dot_dtheta = (
            theta_dot.T @ ddL_dtheta2 / distance - dist_dot / distance**2 * dL_dtheta
        )

        # d_dist_dot_dtheta_dot = dL_dtheta / distance

        d_dist_dot_dtheta_dot = theta_dot.T @ ddL_dtheta2 / distance

        d_theta1_dq = pin.computeFrameJacobian(
            rmodel, rdata, q, self._shape1.parentFrame, pin.LOCAL_WORLD_ALIGNED
        )
        d_theta2_dq = pin.computeFrameJacobian(
            rmodel, rdata, q, self._shape2.parentFrame, pin.LOCAL_WORLD_ALIGNED
        )

        d_c1_dq = d_theta1_dq[:3]
        d_r1_dq = d_theta1_dq[3:]

        d_c2_dq = d_theta2_dq[:3]
        d_r2_dq = d_theta2_dq[3:]

        d_theta_dq = np.r_[d_c1_dq, d_c2_dq, d_r1_dq, d_r2_dq]

        d_theta1_dot_dq = pin.getFrameVelocityDerivatives(
            rmodel, rdata, frame_id=self._shape1.parentFrame, reference_frame=pin.WORLD
        )[0]
        d_theta2_dot_dq = pin.getFrameVelocityDerivatives(
            rmodel, rdata, frame_id=self._shape2.parentFrame, reference_frame=pin.WORLD
        )[0]

        d_v1_dq = d_theta1_dot_dq[:3, :]
        d_w1_dq = d_theta1_dot_dq[3:, :]

        d_v2_dq = d_theta2_dot_dq[:3, :]
        d_w2_dq = d_theta2_dot_dq[:3, :]

        d_theta_dot_dq = np.r_[d_v1_dq, d_v2_dq, d_w1_dq, d_w2_dq]
        d_dist_dot_dq = (
            d_dist_dot_dtheta @ d_theta_dq + d_dist_dot_dtheta_dot @ d_theta_dot_dq
        )
        return np.r_[d_dist_dot_dq, self.ddist_dq(rmodel, cmodel, x)]


class TestVelocityAvoidance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Create random number generator
        cls.rng = np.random.default_rng(2137)

        # Load robot
        robot_wrapper = PandaWrapper(capsule=False)
        cls.rmodel, cmodel, _ = robot_wrapper()
        cls.rdata = cls.rmodel.createData()

        scene = Scene()
        # Update collision model
        cls.cmodel = scene.create_scene(cmodel, scene=4)
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
            v = self.rng.random(q.shape)

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
            v = self.rng.random(q.shape)

            # Update poses and compute new residual value
            self._update_placement(q, v)
            # Create state vector
            x = np.concatenate((q, v))

            # Compute exact derivative
            self.velocity_avoidance_residual.calc(
                self.residual_data, x, np.zeros(q.shape)
            )
            self.velocity_avoidance_residual.calcDiff(
                self.residual_data, x, np.zeros(q.shape)
            )

            Rx_py = self.test_residual.calcDiff(
                self.residual_data, x, np.zeros(q.shape)
            )

            np.testing.assert_allclose(
                self.residual_data.Rx,
                Rx_py,
                rtol=1e-5,
                err_msg="Result missmatch in function ``calcDiff`` "
                "between C++ implementation and python implementation!.",
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
