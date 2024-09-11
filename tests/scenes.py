import hppfcl
import numpy as np
import pinocchio as pin


class Scene:
    def __init__(self) -> None:
        pass

    def create_scene(self, rmodel: pin.Model, cmodel: pin.Model, name_scene: str):
        """Create a scene amond the ones : "box"

        Args:
            rmodel (pin.Model): robot model
            cmodel (pin.Model): collision model of the robot
            name_scene (str): name of the scene
        """

        self._name_scene = name_scene
        self._cmodel = cmodel
        self._rmodel = rmodel

        self._target = pin.SE3.Identity()
        if self._name_scene == "box":
            self._target = pin.SE3(
                pin.utils.rotate("x", np.pi), np.array([0.0, 0.2, 0.8])
            )
            self._q0 = np.array(
                [6.2e-01, 1.7e00, 1.5e00, 6.9e-01, -1.3e00, 1.1e00, 1.5e-01]
            )
            OBSTACLE_HEIGHT = 0.85
            OBSTACLE_X = 2.0e-1
            OBSTACLE_Y = 0.5e-2
            OBSTACLE_Z = 0.5
            obstacles = [
                (
                    "obstacle1",
                    hppfcl.Box(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z),
                    pin.SE3(
                        pin.utils.rotate("y", np.pi / 2),
                        np.array([-0.0, -0.1, OBSTACLE_HEIGHT]),
                    ),
                ),
                (
                    "obstacle2",
                    hppfcl.Box(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z),
                    pin.SE3(
                        pin.utils.rotate("y", np.pi / 2),
                        np.array([-0.0, 0.4, OBSTACLE_HEIGHT]),
                    ),
                ),
                (
                    "obstacle3",
                    hppfcl.Box(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z),
                    pin.SE3(
                        pin.utils.rotate("y", np.pi / 2)
                        @ pin.utils.rotate("x", np.pi / 2),
                        np.array([0.25, 0.15, OBSTACLE_HEIGHT]),
                    ),
                ),
                (
                    "obstacle4",
                    hppfcl.Box(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z),
                    pin.SE3(
                        pin.utils.rotate("y", np.pi / 2)
                        @ pin.utils.rotate("x", np.pi / 2),
                        np.array([-0.25, 0.15, OBSTACLE_HEIGHT]),
                    ),
                ),
            ]
        elif self._name_scene == "ball":
            self._q0 = pin.neutral(self._rmodel)
            self._target.translation = np.array([0, -0.4, 1.5])
            OBSTACLE_RADIUS = 1.5e-1
            OBSTACLE_POSE = pin.SE3.Identity()
            OBSTACLE_POSE.translation = np.array([0.25, -0.4, 1.5])
            obstacles = [("obstacle1", hppfcl.Sphere(OBSTACLE_RADIUS), OBSTACLE_POSE)]
        elif self._name_scene == "wall":
            self._target = pin.SE3(
                pin.utils.rotate("x", np.pi), np.array([0.0, 0.2, 0.8])
            )
            self._q0 = np.array(
                [6.2e-01, 1.7e00, 1.5e00, 6.9e-01, -1.3e00, 1.1e00, 1.5e-01]
            )
            OBSTACLE_HEIGHT = 0.85
            OBSTACLE_X = 2.0e-0
            OBSTACLE_Y = 0.5e-2
            OBSTACLE_Z = 0.5
            obstacles = [
                (
                    "obstacle1",
                    hppfcl.Box(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z),
                    pin.SE3(
                        pin.utils.rotate("y", np.pi / 2),
                        np.array([-0.0, -0.1, OBSTACLE_HEIGHT]),
                    ),
                ),
            ]

        else:
            raise NotImplementedError(f"The input {name_scene} is not implemented.")

        # Adding all the obstacles to the geom model
        for obstacle in obstacles:
            name = obstacle[0]
            shape = obstacle[1]
            pose = obstacle[2]
            geom_obj = pin.GeometryObject(
                name,
                0,
                0,
                shape,
                pose,
            )
            self._cmodel.addGeometryObject(geom_obj)
        self._add_collision_pairs()
        return self._cmodel, self._target, self._q0

    def _add_collision_pairs(self):
        """Add the collision pairs in the collision model w.r.t to the chosen scene."""
        if self._name_scene == "box":
            obstacles = [
                "support_link_0",
                "obstacle1",
                "obstacle2",
                "obstacle3",
                "obstacle4",
            ]
            shapes_avoiding_collision = [
                "panda2_link7_sc_4",
                "panda2_link7_sc_1",
                "panda2_link6_sc_2",
                "panda2_link5_sc_3",
                "panda2_link5_sc_4",
                "panda2_rightfinger_0",
                "panda2_leftfinger_0",
            ]
        elif self._name_scene == "ball":
            obstacles = ["obstacle1"]
            shapes_avoiding_collision = [
                "support_link_0",
                "panda2_leftfinger_0",
                "panda2_rightfinger_0",
                "panda2_link5_sc_4",
            ]
        elif self._name_scene == "wall":
            obstacles = [
                "support_link_0",
                "obstacle1",
            ]
            shapes_avoiding_collision = [
                "panda2_link7_sc_4",
                "panda2_link7_sc_1",
                "panda2_link6_sc_2",
                "panda2_link5_sc_3",
                "panda2_link5_sc_4",
                "panda2_rightfinger_0",
                "panda2_leftfinger_0",
            ]
        for shape in shapes_avoiding_collision:
            for obstacle in obstacles:
                self._cmodel.addCollisionPair(
                    pin.CollisionPair(
                        self._cmodel.getGeometryId(shape),
                        self._cmodel.getGeometryId(obstacle),
                    )
                )
