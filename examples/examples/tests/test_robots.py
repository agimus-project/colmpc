from os.path import dirname, join, abspath
import time
import numpy as np
import matplotlib.pyplot as plt
import hppfcl
import example_robot_data as robex
import pinocchio as pin
import argparse

hppfcl.WITH_OCTOMAP = False

RED = np.array([249, 136, 126, 125]) / 255
RED_FULL = np.array([249, 136, 126, 255]) / 255

GREEN = np.array([170, 236, 149, 125]) / 255
GREEN_FULL = np.array([170, 236, 149, 255]) / 255

BLUE = np.array([144, 169, 183, 125]) / 255
BLUE_FULL = np.array([144, 169, 183, 255]) / 255

YELLOW = np.array([1, 1, 0, 0.5])
YELLOW_FULL = np.array([1, 1, 0, 1.0])

BLACK = np.array([0, 0, 0, 0.5])
BLACK_FULL = np.array([0, 0, 0, 1.0])

# PARSER

parser = argparse.ArgumentParser()
parser.add_argument(
    "-p",
    "--panda",
    help="Chose the panda to do the unit tests",
    action="store_true",
    default=False,
)
parser.add_argument(
    "-d",
    "--display",
    help="Display in a meshcat visualizer.",
    action="store_true",
    default=False,
)
args = parser.parse_args()

### HELPERS

YELLOW_FULL = np.array([1, 1, 0, 1.0])
BLUE_FULL = np.array([144, 169, 183, 255]) / 255

WITH_DISPLAY = args.display
PANDA = args.panda


def load_panda():
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
    SPHERE1_RADIUS = 1.5e-1
    CAPSULE_HALFLENGTH = 1e-1
    SPHERE1_POSE = pin.SE3.Identity()
    SPHERE1_POSE.translation = np.array([0.0, 0.25, 1.5])
    SPHERE1 = hppfcl.Capsule(SPHERE1_RADIUS, CAPSULE_HALFLENGTH)
    SPHERE1_GEOM_OBJECT = pin.GeometryObject(
        "SPHERE1",
        rmodel.getFrameId("universe"),
        rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
        SPHERE1,
        SPHERE1_POSE,
    )
    SPHERE1_GEOM_OBJECT.meshColor = YELLOW

    IG_SPHERE1 = cmodel.addGeometryObject(SPHERE1_GEOM_OBJECT)

    ### CREATING THE SPHERE ON THE END EFFECTOR
    SPHERE2_RADIUS = 1.5e-0
    SPHERE2_POSE = pin.SE3.Identity()
    SPHERE2_POSE.translation = np.array([0.2, 0.0, 0.0])
    SPHERE2 = hppfcl.Sphere(SPHERE2_RADIUS)
    SPHERE2_GEOM_OBJECT = pin.GeometryObject(
        "SPHERE2",
        rmodel.getFrameId("panda2_leftfinger"),
        rmodel.frames[rmodel.getFrameId("panda2_leftfinger")].parentJoint,
        SPHERE2,
        SPHERE2_POSE,
    )
    SPHERE2_GEOM_OBJECT.meshColor = GREEN
    IG_SPHERE2 = cmodel.addGeometryObject(SPHERE2_GEOM_OBJECT)

    ### CREATING THE SPHERE ON THE ROBOT
    SPHERE3_RADIUS = 1.5e-1
    SPHERE3_POSE = pin.SE3.Identity()
    SPHERE3_POSE.translation = np.array([0.0, 0.1, 0.2])
    SPHERE3 = hppfcl.Capsule(SPHERE3_RADIUS, CAPSULE_HALFLENGTH)
    SPHERE3_GEOM_OBJECT = pin.GeometryObject(
        "SPHERE3",
        rmodel.getFrameId("panda2_link3_sc_joint"),
        rmodel.frames[rmodel.getFrameId("panda2_link3_sc_joint")].parentJoint,
        SPHERE3,
        SPHERE3_POSE,
    )
    SPHERE3_GEOM_OBJECT.meshColor = BLUE
    IG_SPHERE3 = cmodel.addGeometryObject(SPHERE3_GEOM_OBJECT)

    return rmodel, vmodel, cmodel


def load_ur():
    robot = robex.load("ur10")
    rmodel = robot.model
    cmodel = robot.collision_model
    vmodel = robot.visual_model

    ### CREATING THE SPHERE ON THE UNIVERSE
    SPHERE1_RADIUS = 1.5e-1
    SPHERE1_POSE = pin.SE3.Identity()
    SPHERE1_POSE.translation = np.array([0.0, 0.25, 1.5])
    SPHERE1 = hppfcl.Sphere(SPHERE1_RADIUS)
    SPHERE1_GEOM_OBJECT = pin.GeometryObject(
        "SPHERE1",
        rmodel.getFrameId("universe"),
        rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
        SPHERE1,
        SPHERE1_POSE,
    )
    SPHERE1_GEOM_OBJECT.meshColor = YELLOW

    IG_SPHERE1 = cmodel.addGeometryObject(SPHERE1_GEOM_OBJECT)

    ### CREATING THE SPHERE ON THE END EFFECTOR
    SPHERE2_RADIUS = 1.5e-1
    SPHERE2_POSE = pin.SE3.Identity()
    SPHERE2_POSE.translation = np.array([0.2, 0.0, 0.0])
    SPHERE2 = hppfcl.Sphere(SPHERE2_RADIUS)
    SPHERE2_GEOM_OBJECT = pin.GeometryObject(
        "SPHERE2",
        rmodel.getFrameId("tool0"),
        rmodel.frames[rmodel.getFrameId("tool0")].parentJoint,
        SPHERE2,
        SPHERE2_POSE,
    )
    SPHERE2_GEOM_OBJECT.meshColor = GREEN
    IG_SPHERE2 = cmodel.addGeometryObject(SPHERE2_GEOM_OBJECT)

    ### CREATING THE SPHERE ON THE ROBOT
    SPHERE3_RADIUS = 1.5e-1
    SPHERE3_POSE = pin.SE3.Identity()
    SPHERE3_POSE.translation = np.array([0.0, 0.3, 0.0])
    SPHERE3 = hppfcl.Sphere(SPHERE3_RADIUS)
    SPHERE3_GEOM_OBJECT = pin.GeometryObject(
        "SPHERE3",
        rmodel.getFrameId("wrist_2_joint"),
        rmodel.frames[rmodel.getFrameId("wrist_2_joint")].parentJoint,
        SPHERE3,
        SPHERE3_POSE,
    )
    SPHERE3_GEOM_OBJECT.meshColor = BLUE
    IG_SPHERE3 = cmodel.addGeometryObject(SPHERE3_GEOM_OBJECT)

    return rmodel, vmodel, cmodel


######################################## DISTANCE & ITS DERIVATIVES COMPUTATION #######################################


def dist(q):
    """Computes the distance with diffcol

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
        shape1_geom,
        shape1_placement,
        shape2_geom,
        shape2_placement,
        req,
        res,
    )
    return distance


def ddist_numdiff(q):
    """Finite derivative of the dist function.

    Args:
        q (np.ndarray): Configuration of the robot

    Returns:
        distance derivative: distance derivative between shape 1 & shape 2
    """
    j_diff = np.zeros(nq)
    fx = dist(q)
    for i in range(nq):
        e = np.zeros(nq)
        e[i] = 1e-6
        j_diff[i] = (dist(q + e) - dist(q - e)) / e[i] / 2
    return j_diff


def ddist_analytic(q):
    pin.forwardKinematics(rmodel, rdata, q)
    pin.computeJointJacobians(rmodel, rdata, q)
    pin.updateGeometryPlacements(
        rmodel,
        rdata,
        cmodel,
        cdata,
        q,
    )
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


if __name__ == "__main__":
    # Loading the robot

    if PANDA:
        rmodel, vmodel, cmodel = load_panda()
    else:
        rmodel, vmodel, cmodel = load_ur()
    # Creating the datas model
    rdata = rmodel.createData()
    cdata = cmodel.createData()

    # Initial configuration
    q0 = pin.neutral(rmodel)

    # Number of joints
    nq = rmodel.nq

    # Updating the models
    pin.framesForwardKinematics(rmodel, rdata, q0)
    pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q0)

    # Creating the shapes for the collision detection.

    shape1_id = cmodel.getGeometryId("SPHERE1")
    shape2_id = cmodel.getGeometryId("SPHERE3")

    # Coloring the sphere
    shape1 = cmodel.geometryObjects[shape1_id]
    shape1.meshColor = BLUE_FULL

    # Getting the geometry of the shape 1
    shape1_geom = shape1.geometry

    # Getting its pose in the world reference
    shape1_placement = cdata.oMg[shape1_id]

    # Coloring the sphere
    shape2 = cmodel.geometryObjects[shape2_id]
    shape2.meshColor = YELLOW_FULL

    shape2_geom = shape2.geometry

    # Getting its pose in the world reference
    shape2_placement = cdata.oMg[shape2_id]

    if WITH_DISPLAY:
        from wrapper_meshcat import MeshcatWrapper

        # Generating the meshcat visualizer
        MeshcatVis = MeshcatWrapper()
        vis, meshcatVis = MeshcatVis.visualize(
            robot_model=rmodel,
            robot_collision_model=cmodel,
            robot_visual_model=cmodel,
        )
        # Displaying the initial
        vis.display(q0)

    # Distance & Derivative results from hppfcl
    req = hppfcl.DistanceRequest()
    res = hppfcl.DistanceResult()

    distance = dist(q0)

    distance_deriv_ana = ddist_analytic(
        q0,
    )
    distance_deriv_numdiff = ddist_numdiff(
        q0,
    )

    print(f"distance_deriv_ana : {distance_deriv_ana}")
    print(f"distance_deriv_numdiff : {distance_deriv_numdiff}")

    q1 = np.array([1, 1, 1, 1, 1, 1,1])

    print(dist(q1))
    distance_deriv_ana = ddist_analytic(
        q1,
    )
    distance_deriv_numdiff = ddist_numdiff(
        q1,
    )

    print(f"distance_deriv_ana : {distance_deriv_ana}")
    print(f"distance_deriv_numdiff : {distance_deriv_numdiff}")
    # vis.display(q0)
    # alpha = np.linspace(0, 1, 100)
    # for k in alpha:
    #     vis.display(pin.interpolate(rmodel, q0, q1, k))

    ######### TESTING
    def test():
        # Making sure the shapes exist
        assert shape1_id <= len(cmodel.geometryObjects) - 1
        assert shape2_id <= len(cmodel.geometryObjects) - 1

        # Making sure the shapes are spheres
        assert isinstance(shape1.geometry, hppfcl.Sphere)
        assert isinstance(shape2.geometry, hppfcl.Sphere)
