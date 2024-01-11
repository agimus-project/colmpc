import unittest

from os.path import dirname, join, abspath
import numpy as np

import example_robot_data as robex
import pinocchio as pin
import hppfcl


class TestRobotsDistanceDerivativesCapsuleSpheres(unittest.TestCase):
    """This class is made to test the distances derivatives between primitives pairs such as sphere-sphere of panda robot & ur10. The collisions shapes are from hppfcl."""

    radius = 1.5e-1
    halfLength = 2e-1

    def load_panda(self):
        """Load the robot from the models folder.

        Returns:
            rmodel, vmodel, cmodel: Robot model, visual model & collision model of the robot.
        """

        ### LOADING THE ROBOT
        pinocchio_model_dir = join(
            dirname(dirname(dirname(str(abspath(__file__))))), "models"
        )
        model_path = join(pinocchio_model_dir, "franka_description/robots")
        mesh_dir = pinocchio_model_dir
        urdf_filename = "franka2.urdf"
        urdf_model_path = join(join(model_path, "panda"), urdf_filename)

        rmodel, cmodel, vmodel = pin.buildModelsFromUrdf(
            urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
        )

        q0 = pin.neutral(rmodel)

        rmodel, [vmodel, cmodel] = pin.buildReducedModel(
            rmodel, [vmodel, cmodel], [1, 9, 10], q0
        )

        ### CREATING THE SPHERE ON THE UNIVERSE
        CAPSULE_POSE = pin.SE3.Identity()
        CAPSULE_POSE.translation = np.array([0.0, 0.25, 1.5])
        CAPSULE = hppfcl.Capsule(self.radius, self.halfLength)
        CAPSULE_GEOM_OBJECT = pin.GeometryObject(
            "CAPSULE",
            rmodel.getFrameId("universe"),
            rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
            CAPSULE,
            CAPSULE_POSE,
        )
        self.ID_CAPSULE_PA = cmodel.addGeometryObject(CAPSULE_GEOM_OBJECT)

        ### CREATING THE SPHERE ON THE END EFFECTOR
        SPHERE2_POSE = pin.SE3.Identity()
        SPHERE2_POSE.translation = np.array([0.2, 0.0, 0.0])
        SPHERE2 = hppfcl.Sphere(self.radius)
        SPHERE2_GEOM_OBJECT = pin.GeometryObject(
            "SPHERE2",
            rmodel.getFrameId("panda2_leftfinger"),
            rmodel.frames[rmodel.getFrameId("panda2_leftfinger")].parentJoint,
            SPHERE2,
            SPHERE2_POSE,
        )
        self.ID_SPHERE2_PA = cmodel.addGeometryObject(SPHERE2_GEOM_OBJECT)

        ### CREATING THE SPHERE ON THE ROBOT
        SPHERE3_POSE = pin.SE3.Identity()
        SPHERE3_POSE.translation = np.array([0.0, 0.1, 0.2])
        SPHERE3 = hppfcl.Sphere(self.radius)
        SPHERE3_GEOM_OBJECT = pin.GeometryObject(
            "SPHERE3",
            rmodel.getFrameId("panda2_link3_sc_joint"),
            rmodel.frames[rmodel.getFrameId("panda2_link3_sc_joint")].parentJoint,
            SPHERE3,
            SPHERE3_POSE,
        )
        self.ID_SPHERE3_PA = cmodel.addGeometryObject(SPHERE3_GEOM_OBJECT)

        return rmodel, vmodel, cmodel

    def load_ur(self):
        robot = robex.load("ur10")
        rmodel = robot.model
        cmodel = robot.collision_model
        vmodel = robot.visual_model

        ### CREATING THE SPHERE ON THE UNIVERSE
        CAPSULE_POSE = pin.SE3.Identity()
        CAPSULE_POSE.translation = np.array([0.0, 0.25, 1.5])
        CAPSULE = hppfcl.Capsule(self.radius, self.halfLength)
        self.CAPSULE_GEOM_OBJECT = pin.GeometryObject(
            "CAPSULE",
            rmodel.getFrameId("universe"),
            rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
            CAPSULE,
            CAPSULE_POSE,
        )
        self.ID_CAPSULE_UR = cmodel.addGeometryObject(self.CAPSULE_GEOM_OBJECT)

        ### CREATING THE SPHERE ON THE END EFFECTOR
        SPHERE2_POSE = pin.SE3.Identity()
        SPHERE2_POSE.translation = np.array([0.2, 0.0, 0.0])
        SPHERE2 = hppfcl.Sphere(self.radius)
        self.SPHERE2_GEOM_OBJECT = pin.GeometryObject(
            "SPHERE2",
            rmodel.getFrameId("tool0"),
            rmodel.frames[rmodel.getFrameId("tool0")].parentJoint,
            SPHERE2,
            SPHERE2_POSE,
        )
        self.ID_SPHERE2_UR = cmodel.addGeometryObject(self.SPHERE2_GEOM_OBJECT)

        ### CREATING THE SPHERE ON THE ROBOT
        SPHERE3_POSE = pin.SE3.Identity()
        SPHERE3_POSE.translation = np.array([0.0, 0.3, 0.0])
        SPHERE3 = hppfcl.Sphere(self.radius)
        self.SPHERE3_GEOM_OBJECT = pin.GeometryObject(
            "SPHERE3",
            rmodel.getFrameId("wrist_2_joint"),
            rmodel.frames[rmodel.getFrameId("wrist_2_joint")].parentJoint,
            SPHERE3,
            SPHERE3_POSE,
        )
        self.ID_SPHERE3_UR = cmodel.addGeometryObject(self.SPHERE3_GEOM_OBJECT)

        return rmodel, vmodel, cmodel

    # def test_distance(self):
    #     # Loading the UR robot
    #     rmodel_ur, vmodel_ur, cmodel_ur = self.load_ur()

    #     rdata_ur = rmodel_ur.createData()
    #     cdata_ur = cmodel_ur.createData()

    #     q_ur = pin.neutral(rmodel_ur)

    #     # Loading the panda robot
    #     rmodel_pa, vmodel_pa, cmodel_pa = self.load_panda()

    #     rdata_pa = rmodel_pa.createData()
    #     cdata_pa = cmodel_pa.createData()

    #     # Number of joints
    #     nq_ur = rmodel_ur.nq
    #     nq_pa = rmodel_pa.nq

    #     # Distance & Derivative results from hppfcl
    #     req = hppfcl.DistanceRequest()
    #     res = hppfcl.DistanceResult()

    #     q_ur = pin.randomConfiguration(rmodel_ur)
    #     q_pa = pin.randomConfiguration(rmodel_pa)

    #     # Updating the models
    #     pin.forwardKinematics(rmodel_ur, rdata_ur, q_ur)
    #     pin.framesForwardKinematics(rmodel_ur, rdata_ur, q_ur)
    #     pin.updateGeometryPlacements(rmodel_ur, rdata_ur, cmodel_ur, cdata_ur, q_ur)

    #     pin.forwardKinematics(rmodel_pa, rdata_pa, q_pa)
    #     pin.framesForwardKinematics(rmodel_pa, rdata_pa, q_pa)
    #     pin.updateGeometryPlacements(rmodel_pa, rdata_pa, cmodel_pa, cdata_pa, q_pa)

    #     CAPSULE_placement_ur = cdata_ur.oMg[cmodel_ur.getGeometryId("CAPSULE")]
    #     sphere2_placement_ur = cdata_ur.oMg[cmodel_ur.getGeometryId("SPHERE2")]
    #     sphere3_placement_ur = cdata_ur.oMg[cmodel_ur.getGeometryId("SPHERE3")]

    #     CAPSULE_placement_pa = cdata_pa.oMg[cmodel_pa.getGeometryId("CAPSULE")]
    #     sphere2_placement_pa = cdata_pa.oMg[cmodel_pa.getGeometryId("SPHERE2")]
    #     sphere3_placement_pa = cdata_pa.oMg[cmodel_pa.getGeometryId("SPHERE3")]

    #     ### Distance between capsule (on universe) & sphere 2 (on tool0 / gripper)

    #     distance_ur = self.dist(
    #         rmodel_ur,
    #         rdata_ur,
    #         cmodel_ur,
    #         cdata_ur,
    #         self.ID_CAPSULE_UR,
    #         self.ID_SPHERE2_UR,
    #         res,
    #         req,
    #         q_ur,
    #     )
    #     distance_pa = self.dist(
    #         rmodel_pa,
    #         rdata_pa,
    #         cmodel_pa,
    #         cdata_pa,
    #         self.ID_CAPSULE_PA,
    #         self.ID_SPHERE2_PA,
    #         res,
    #         req,
    #         q_pa,
    #     )

    #     self.assertAlmostEqual(
    #         distance_ur,
    #         self.distance_sphere_capsule(sphere2_placement_ur, CAPSULE_placement_ur)
    #     )
    #     self.assertAlmostEqual(
    #         distance_pa,
    #         self.distance_sphere_capsule(sphere2_placement_pa, CAPSULE_placement_pa)

    #     )

    #     ### Distance between sphere 2 (on tool0 / gripper) & sphere 3 (on the robot)

    #     distance_ur = self.dist(
    #         rmodel_ur,
    #         rdata_ur,
    #         cmodel_ur,
    #         cdata_ur,
    #         self.ID_SPHERE3_UR,
    #         self.ID_SPHERE2_UR,
    #         res,
    #         req,
    #         q_ur,
    #     )
    #     distance_pa = self.dist(
    #         rmodel_pa,
    #         rdata_pa,
    #         cmodel_pa,
    #         cdata_pa,
    #         self.ID_SPHERE3_PA,
    #         self.ID_SPHERE2_PA,
    #         res,
    #         req,
    #         q_pa,
    #     )

    #     self.assertAlmostEqual(
    #         distance_ur,
    #         np.linalg.norm(
    #             (sphere3_placement_ur.inverse() * sphere2_placement_ur).translation
    #         )
    #         - 2 * self.radius,
    #     )
    #     self.assertAlmostEqual(
    #         distance_pa,
    #         np.linalg.norm(
    #             (sphere3_placement_pa.inverse() * sphere2_placement_pa).translation
    #         )
    #         - 2 * self.radius,
    #     )

    #     ### Distance between capsule 1 (on universe) & sphere 3 (on the robot)

    #     distance_ur = self.dist(
    #         rmodel_ur,
    #         rdata_ur,
    #         cmodel_ur,
    #         cdata_ur,
    #         self.ID_SPHERE3_UR,
    #         self.ID_CAPSULE_UR,
    #         res,
    #         req,
    #         q_ur,
    #     )
    #     distance_pa = self.dist(
    #         rmodel_pa,
    #         rdata_pa,
    #         cmodel_pa,
    #         cdata_pa,
    #         self.ID_SPHERE3_PA,
    #         self.ID_CAPSULE_PA,
    #         res,
    #         req,
    #         q_pa,
    #     )

    #     self.assertAlmostEqual(
    #         distance_ur,
    #         self.distance_sphere_capsule(sphere3_placement_ur, CAPSULE_placement_ur)

    #     )
    #     self.assertAlmostEqual(
    #         distance_pa,
    #         self.distance_sphere_capsule(sphere3_placement_pa, CAPSULE_placement_pa)

    #     )

    def test_distance_derivatives(self):
        # Loading the UR robot
        rmodel_ur, vmodel_ur, cmodel_ur = self.load_ur()

        rdata_ur = rmodel_ur.createData()
        cdata_ur = cmodel_ur.createData()

        q_ur = pin.neutral(rmodel_ur)

        # Loading the panda robot
        rmodel_pa, vmodel_pa, cmodel_pa = self.load_panda()

        rdata_pa = rmodel_pa.createData()
        cdata_pa = cmodel_pa.createData()

        q_pa = pin.neutral(rmodel_pa)

        # Number of joints
        nq_ur = rmodel_ur.nq
        nq_pa = rmodel_pa.nq

        # Updating the models
        pin.forwardKinematics(rmodel_ur, rdata_ur, q_ur)
        pin.framesForwardKinematics(rmodel_ur, rdata_ur, q_ur)
        pin.updateGeometryPlacements(rmodel_ur, rdata_ur, cmodel_ur, cdata_ur, q_ur)

        pin.forwardKinematics(rmodel_pa, rdata_pa, q_pa)
        pin.framesForwardKinematics(rmodel_pa, rdata_pa, q_pa)
        pin.updateGeometryPlacements(rmodel_pa, rdata_pa, cmodel_pa, cdata_pa, q_pa)

        # Distance & Derivative results from hppfcl
        req = hppfcl.DistanceRequest()
        res = hppfcl.DistanceResult()

        ### Distance derivatives between sphere 1 (on universe) & sphere 2 (on tool0 / gripper)

        distance_deriv_ana_ur = self.ddist_analytic(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )
        distance_deriv_numdiff_ur = self.ddist_numdiff(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )

        distance_deriv_ana_pa = self.ddist_analytic(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )
        distance_deriv_numdiff_pa = self.ddist_numdiff(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )

        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_ur),
            np.linalg.norm(distance_deriv_numdiff_ur),
        )
        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_pa),
            np.linalg.norm(distance_deriv_numdiff_pa),
        )

        ### Distance derivatives between sphere 2 (on tool0 / gripper) & sphere 3 (on the robot)

        distance_deriv_ana_ur = self.ddist_analytic(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_SPHERE3_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )
        distance_deriv_numdiff_ur = self.ddist_numdiff(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_SPHERE3_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )

        distance_deriv_ana_pa = self.ddist_analytic(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_SPHERE3_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )
        distance_deriv_numdiff_pa = self.ddist_numdiff(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_SPHERE3_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )

        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_ur),
            np.linalg.norm(distance_deriv_numdiff_ur),
        )
        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_pa),
            np.linalg.norm(distance_deriv_numdiff_pa),
        )

        ### Distance derivatives between sphere 1 (on universe) & sphere 3 (on the robot)

        distance_deriv_ana_ur = self.ddist_analytic(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE3_UR,
            res,
            req,
            q_ur,
        )
        distance_deriv_numdiff_ur = self.ddist_numdiff(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE3_UR,
            res,
            req,
            q_ur,
        )

        distance_deriv_ana_pa = self.ddist_analytic(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE3_PA,
            res,
            req,
            q_pa,
        )
        distance_deriv_numdiff_pa = self.ddist_numdiff(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE3_PA,
            res,
            req,
            q_pa,
        )

        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_ur),
            np.linalg.norm(distance_deriv_numdiff_ur),
        )
        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_pa),
            np.linalg.norm(distance_deriv_numdiff_pa),
        )

        ### Doing the same with random configuration
        q_ur = pin.randomConfiguration(rmodel_ur)
        q_pa = pin.randomConfiguration(rmodel_pa)

        # Updating the models
        pin.forwardKinematics(rmodel_ur, rdata_ur, q_ur)
        pin.framesForwardKinematics(rmodel_ur, rdata_ur, q_ur)
        pin.updateGeometryPlacements(rmodel_ur, rdata_ur, cmodel_ur, cdata_ur, q_ur)

        pin.forwardKinematics(rmodel_pa, rdata_pa, q_pa)
        pin.framesForwardKinematics(rmodel_pa, rdata_pa, q_pa)
        pin.updateGeometryPlacements(rmodel_pa, rdata_pa, cmodel_pa, cdata_pa, q_pa)

        ### Distance derivatives between sphere 1 (on universe) & sphere 2 (on tool0 / gripper)

        distance_deriv_ana_ur = self.ddist_analytic(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )
        distance_deriv_numdiff_ur = self.ddist_numdiff(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )

        distance_deriv_ana_pa = self.ddist_analytic(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )
        distance_deriv_numdiff_pa = self.ddist_numdiff(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )

        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_ur),
            np.linalg.norm(distance_deriv_numdiff_ur),
        )
        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_pa),
            np.linalg.norm(distance_deriv_numdiff_pa),
        )

        ### Distance derivatives between sphere 2 (on tool0 / gripper) & sphere 3 (on the robot)

        distance_deriv_ana_ur = self.ddist_analytic(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_SPHERE3_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )
        distance_deriv_numdiff_ur = self.ddist_numdiff(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_SPHERE3_UR,
            self.ID_SPHERE2_UR,
            res,
            req,
            q_ur,
        )

        distance_deriv_ana_pa = self.ddist_analytic(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_SPHERE3_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )
        distance_deriv_numdiff_pa = self.ddist_numdiff(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_SPHERE3_PA,
            self.ID_SPHERE2_PA,
            res,
            req,
            q_pa,
        )

        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_ur),
            np.linalg.norm(distance_deriv_numdiff_ur),
        )
        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_pa),
            np.linalg.norm(distance_deriv_numdiff_pa),
        )

        ### Distance derivatives between sphere 1 (on universe) & sphere 3 (on the robot)

        distance_deriv_ana_ur = self.ddist_analytic(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE3_UR,
            res,
            req,
            q_ur,
        )
        distance_deriv_numdiff_ur = self.ddist_numdiff(
            rmodel_ur,
            rdata_ur,
            cmodel_ur,
            cdata_ur,
            self.ID_CAPSULE_UR,
            self.ID_SPHERE3_UR,
            res,
            req,
            q_ur,
        )

        distance_deriv_ana_pa = self.ddist_analytic(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE3_PA,
            res,
            req,
            q_pa,
        )
        distance_deriv_numdiff_pa = self.ddist_numdiff(
            rmodel_pa,
            rdata_pa,
            cmodel_pa,
            cdata_pa,
            self.ID_CAPSULE_PA,
            self.ID_SPHERE3_PA,
            res,
            req,
            q_pa,
        )

        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_ur),
            np.linalg.norm(distance_deriv_numdiff_ur),
        )
        self.assertAlmostEqual(
            np.linalg.norm(distance_deriv_ana_pa),
            np.linalg.norm(distance_deriv_numdiff_pa),
        )

        ### Testing now the derivatives across a trajectory

        q0_ur = pin.neutral(rmodel_ur)
        q1_ur = pin.randomConfiguration(rmodel_ur)

        q0_pa = pin.neutral(rmodel_pa)
        q1_pa = pin.randomConfiguration(rmodel_pa)

        alpha_list = np.linspace(0, 1, 100)

        for alpha in alpha_list:
            q_ur = pin.interpolate(rmodel_ur, q0_ur, q1_ur, alpha)
            q_pa = pin.interpolate(rmodel_pa, q0_pa, q1_pa, alpha)

            # Updating the models
            pin.forwardKinematics(rmodel_ur, rdata_ur, q_ur)
            pin.framesForwardKinematics(rmodel_ur, rdata_ur, q_ur)
            pin.updateGeometryPlacements(rmodel_ur, rdata_ur, cmodel_ur, cdata_ur, q_ur)

            pin.forwardKinematics(rmodel_pa, rdata_pa, q_pa)
            pin.framesForwardKinematics(rmodel_pa, rdata_pa, q_pa)
            pin.updateGeometryPlacements(rmodel_pa, rdata_pa, cmodel_pa, cdata_pa, q_pa)

            ### Distance derivatives between sphere 1 (on universe) & sphere 2 (on tool0 / gripper)

            distance_deriv_ana_ur = self.ddist_analytic(
                rmodel_ur,
                rdata_ur,
                cmodel_ur,
                cdata_ur,
                self.ID_CAPSULE_UR,
                self.ID_SPHERE2_UR,
                res,
                req,
                q_ur,
            )
            distance_deriv_numdiff_ur = self.ddist_numdiff(
                rmodel_ur,
                rdata_ur,
                cmodel_ur,
                cdata_ur,
                self.ID_CAPSULE_UR,
                self.ID_SPHERE2_UR,
                res,
                req,
                q_ur,
            )

            distance_deriv_ana_pa = self.ddist_analytic(
                rmodel_pa,
                rdata_pa,
                cmodel_pa,
                cdata_pa,
                self.ID_CAPSULE_PA,
                self.ID_SPHERE2_PA,
                res,
                req,
                q_pa,
            )
            distance_deriv_numdiff_pa = self.ddist_numdiff(
                rmodel_pa,
                rdata_pa,
                cmodel_pa,
                cdata_pa,
                self.ID_CAPSULE_PA,
                self.ID_SPHERE2_PA,
                res,
                req,
                q_pa,
            )

            self.assertAlmostEqual(
                np.linalg.norm(distance_deriv_ana_ur),
                np.linalg.norm(distance_deriv_numdiff_ur),
            )
            self.assertAlmostEqual(
                np.linalg.norm(distance_deriv_ana_pa),
                np.linalg.norm(distance_deriv_numdiff_pa),
            )

            ### Distance derivatives between sphere 2 (on tool0 / gripper) & sphere 3 (on the robot)

            distance_deriv_ana_ur = self.ddist_analytic(
                rmodel_ur,
                rdata_ur,
                cmodel_ur,
                cdata_ur,
                self.ID_SPHERE3_UR,
                self.ID_SPHERE2_UR,
                res,
                req,
                q_ur,
            )
            distance_deriv_numdiff_ur = self.ddist_numdiff(
                rmodel_ur,
                rdata_ur,
                cmodel_ur,
                cdata_ur,
                self.ID_SPHERE3_UR,
                self.ID_SPHERE2_UR,
                res,
                req,
                q_ur,
            )

            distance_deriv_ana_pa = self.ddist_analytic(
                rmodel_pa,
                rdata_pa,
                cmodel_pa,
                cdata_pa,
                self.ID_SPHERE3_PA,
                self.ID_SPHERE2_PA,
                res,
                req,
                q_pa,
            )
            distance_deriv_numdiff_pa = self.ddist_numdiff(
                rmodel_pa,
                rdata_pa,
                cmodel_pa,
                cdata_pa,
                self.ID_SPHERE3_PA,
                self.ID_SPHERE2_PA,
                res,
                req,
                q_pa,
            )

            self.assertAlmostEqual(
                np.linalg.norm(distance_deriv_ana_ur),
                np.linalg.norm(distance_deriv_numdiff_ur),
            )
            self.assertAlmostEqual(
                np.linalg.norm(distance_deriv_ana_pa),
                np.linalg.norm(distance_deriv_numdiff_pa),
            )

            ### Distance derivatives between sphere 1 (on universe) & sphere 3 (on the robot)

            distance_deriv_ana_ur = self.ddist_analytic(
                rmodel_ur,
                rdata_ur,
                cmodel_ur,
                cdata_ur,
                self.ID_CAPSULE_UR,
                self.ID_SPHERE3_UR,
                res,
                req,
                q_ur,
            )
            distance_deriv_numdiff_ur = self.ddist_numdiff(
                rmodel_ur,
                rdata_ur,
                cmodel_ur,
                cdata_ur,
                self.ID_CAPSULE_UR,
                self.ID_SPHERE3_UR,
                res,
                req,
                q_ur,
            )

            distance_deriv_ana_pa = self.ddist_analytic(
                rmodel_pa,
                rdata_pa,
                cmodel_pa,
                cdata_pa,
                self.ID_CAPSULE_PA,
                self.ID_SPHERE3_PA,
                res,
                req,
                q_pa,
            )
            distance_deriv_numdiff_pa = self.ddist_numdiff(
                rmodel_pa,
                rdata_pa,
                cmodel_pa,
                cdata_pa,
                self.ID_CAPSULE_PA,
                self.ID_SPHERE3_PA,
                res,
                req,
                q_pa,
            )

            self.assertAlmostEqual(
                np.linalg.norm(distance_deriv_ana_ur),
                np.linalg.norm(distance_deriv_numdiff_ur),
            )
            self.assertAlmostEqual(
                np.linalg.norm(distance_deriv_ana_pa),
                np.linalg.norm(distance_deriv_numdiff_pa),
            )

    def dist(self, rmodel, rdata, cmodel, cdata, shape1_id, shape2_id, res, req, q):
        """Computes the distance with hppfcl.

        Args:
            q (np.ndarray): Configuration of the robot

        Returns:
            distance : distance between shape 1 & shape 2
        """
        # Computing the distance
        pin.framesForwardKinematics(rmodel, rdata, q)
        pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q)

        shape1_placement = cdata.oMg[shape1_id]
        shape2_placement = cdata.oMg[shape2_id]

        distance = hppfcl.distance(
            cmodel.geometryObjects[shape1_id].geometry,
            shape1_placement,
            cmodel.geometryObjects[shape2_id].geometry,
            shape2_placement,
            req,
            res,
        )
        return distance

    def distance_sphere_capsule(self, sphere_placement, capsule_placement):
            """Computes the signed distance between a sphere & a capsule.

            Args:
                sphere (pin.GeometryObject): Geometry object of pinocchio, stored in the geometry model.
                capsule (pin.GeometryObject): Geometry object of pinocchio, stored in the geometry model.

            Returns:
                disntance (float): Signed distance between the closest points of a capsule-sphere pair.
            """
            A, B = self.get_A_B_from_center_capsule(capsule_placement)

            # Position of the center of the sphere
            C = sphere_placement.translation

            AB = B - A
            AC = C - A

            # Project AC onto AB, but deferring divide by Dot(AB, AB)
            t = np.dot(AC, AB)
            if t <= 0.0:
                # C projects outside the [A, B] interval, on the A side; clamp to A
                t = 0.0
                closest_point = A
            else:
                denom = np.dot(AB, AB)  # Always nonnegative since denom = ||AB||^2
                if t >= denom:
                    # C projects outside the [A, B] interval, on the B side; clamp to B
                    t = 1.0
                    closest_point = B
                else:
                    # C projects inside the [A, B] interval; must do deferred divide now
                    t = t / denom
                    closest_point = A + t * AB

            # Calculate distance between C and the closest point on the segment
            distance = np.linalg.norm(C - closest_point) - 2 * self.radius

            return distance

    def get_A_B_from_center_capsule(self, capsule_placement):
        """Computes the points A & B of a capsule. The point A & B are the limits of the segment defining the capsule.

        Args:
            capsule (pin.GeometryObject): Geometry object of pinocchio, stored in the geometry model.
        """

        A = pin.SE3.Identity()
        A.translation = np.array([0, 0, -self.halfLength])
        B = pin.SE3.Identity()
        B.translation = np.array([0, 0, +self.halfLength])

        A *= capsule_placement
        B *= capsule_placement
        return (A.translation, B.translation)

    def ddist_analytic(
        self, rmodel, rdata, cmodel, cdata, shape1_id, shape2_id, res, req, q
    ):
        pin.forwardKinematics(rmodel, rdata, q)
        pin.computeJointJacobians(rmodel, rdata, q)
        pin.updateGeometryPlacements(
            rmodel,
            rdata,
            cmodel,
            cdata,
            q,
        )

        shape1 = cmodel.geometryObjects[shape1_id]
        shape2 = cmodel.geometryObjects[shape2_id]

        shape1_placement = cdata.oMg[shape1_id]
        shape2_placement = cdata.oMg[shape2_id]

        jacobian1 = pin.computeFrameJacobian(
            rmodel,
            rdata,
            q,
            shape1.parentFrame,
            pin.LOCAL_WORLD_ALIGNED,
        )

        jacobian2 = pin.computeFrameJacobian(
            rmodel,
            rdata,
            q,
            shape2.parentFrame,
            pin.LOCAL_WORLD_ALIGNED,
        )

        # Computing the distance
        distance = hppfcl.distance(
            shape1.geometry,
            hppfcl.Transform3f(shape1_placement.rotation, shape1_placement.translation),
            shape2.geometry,
            hppfcl.Transform3f(shape2_placement.rotation, shape2_placement.translation),
            req,
            res,
        )
        cp1 = res.getNearestPoint1()
        cp2 = res.getNearestPoint2()

        ## Transport the jacobian of frame 1 into the jacobian associated to cp1
        # Vector from frame 1 center to p1
        f1p1 = cp1 - rdata.oMf[shape1.parentFrame].translation
        # The following 2 lines are the easiest way to understand the transformation
        # although not the most efficient way to compute it.
        f1Mp1 = pin.SE3(np.eye(3), f1p1)
        jacobian1 = f1Mp1.actionInverse @ jacobian1

        ## Transport the jacobian of frame 2 into the jacobian associated to cp2
        # Vector from frame 2 center to p2
        f2p2 = cp2 - rdata.oMf[shape2.parentFrame].translation
        # The following 2 lines are the easiest way to understand the transformation
        # although not the most efficient way to compute it.
        f2Mp2 = pin.SE3(np.eye(3), f2p2)
        jacobian2 = f2Mp2.actionInverse @ jacobian2

        CP1_SE3 = pin.SE3.Identity()
        CP1_SE3.translation = cp1

        CP2_SE3 = pin.SE3.Identity()
        CP2_SE3.translation = cp2
        deriv = (cp1 - cp2).T / distance @ (jacobian1[:3] - jacobian2[:3])

        return deriv

    def ddist_numdiff(
        self, rmodel, rdata, cmodel, cdata, shape1_id, shape2_id, res, req, q
    ):
        """Finite derivative of the dist function.

        Args:
            q (np.ndarray): Configuration of the robot

        Returns:
            distance derivative: distance derivative between shape 1 & shape 2
        """
        j_diff = np.zeros(rmodel.nq)
        for i in range(rmodel.nq):
            e = np.zeros(rmodel.nq)
            e[i] = 1e-6
            j_diff[i] = (
                (
                    self.dist(
                        rmodel,
                        rdata,
                        cmodel,
                        cdata,
                        shape1_id,
                        shape2_id,
                        res,
                        req,
                        q + e,
                    )
                    - self.dist(
                        rmodel,
                        rdata,
                        cmodel,
                        cdata,
                        shape1_id,
                        shape2_id,
                        res,
                        req,
                        q - e,
                    )
                )
                / e[i]
                / 2
            )
        return j_diff


if __name__ == "__main__":
    unittest.main()
