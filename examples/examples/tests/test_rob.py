import numpy as np
import hppfcl
import example_robot_data as robex
from os.path import dirname, join, abspath
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

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


    robot = pin.RobotWrapper.BuildFromURDF(
        urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
    )
    rmodel, cmodel, vmodel
    q0 = pin.neutral(rmodel)

    rmodel, [vmodel, cmodel] = pin.buildReducedModel(
        rmodel, [vmodel, cmodel], [1, 9, 10], q0
    )

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
    ID_SPHERE1 = cmodel.addGeometryObject(SPHERE1_GEOM_OBJECT)

    ### CREATING THE SPHERE ON THE END EFFECTOR
    SPHERE2_RADIUS = 1.5e-1
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
    ID_SPHERE2 = cmodel.addGeometryObject(SPHERE2_GEOM_OBJECT)

    ### CREATING THE SPHERE ON THE ROBOT
    SPHERE3_RADIUS = 1.5e-1
    SPHERE3_POSE = pin.SE3.Identity()
    SPHERE3_POSE.translation = np.array([0.0, 0.1, 0.2])
    SPHERE3 = hppfcl.Sphere(SPHERE3_RADIUS)
    SPHERE3_GEOM_OBJECT = pin.GeometryObject(
        "SPHERE3",
        rmodel.getFrameId("panda2_link3_sc_joint"),
        rmodel.frames[rmodel.getFrameId("panda2_link3_sc_joint")].parentJoint,
        SPHERE3,
        SPHERE3_POSE,
    )
    ID_SPHERE3 = cmodel.addGeometryObject(SPHERE3_GEOM_OBJECT)

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
        ID_SPHERE1_UR = cmodel.addGeometryObject(SPHERE1_GEOM_OBJECT)

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
        ID_SPHERE2_UR = cmodel.addGeometryObject(SPHERE2_GEOM_OBJECT)

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
        ID_SPHERE3_UR = cmodel.addGeometryObject(SPHERE3_GEOM_OBJECT)

        return rmodel, vmodel, cmodel

rmodel, vmodel, cmodel = load_panda()
rdata = rmodel.createData()
cdata = cmodel.createData()

q0 = pin.neutral(rmodel)
pin.forwardKinematics(rmodel, rdata, q0)
pin.framesForwardKinematics(rmodel, rdata, q0)
pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q0)

req = hppfcl.DistanceRequest()
res = hppfcl.DistanceResult()

shape1_placement = cdata.oMg[cmodel.getGeometryId("SPHERE3")]
shape2_placement = cdata.oMg[cmodel.getGeometryId("SPHERE1")]

distance1 = hppfcl.distance(
    cmodel.geometryObjects[cmodel.getGeometryId("SPHERE3")].geometry,
    shape1_placement,
    cmodel.geometryObjects[cmodel.getGeometryId("SPHERE1")].geometry,
    shape2_placement,
    req,
    res,
)
# tmp1 = shape1_placement.translation - (rdata.oMf[rmodel.getFrameId("tool0")] * shape2_placement).translation - (1.5e-1 *2)
tmp = (
    shape1_placement.inverse()
* shape2_placement)
print(f"distance hppfcl with oMg: {distance1}")

print(f"distance manually : {np.linalg.norm(tmp.translation) - (1.5e-1 *2)}")

print("-------------------------------")

q0 = pin.randomConfiguration(rmodel)
pin.forwardKinematics(rmodel, rdata, q0)
pin.framesForwardKinematics(rmodel, rdata, q0)
pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q0)

shape1_placement = cdata.oMg[cmodel.getGeometryId("SPHERE1")]
shape2_placement = cdata.oMg[cmodel.getGeometryId("SPHERE2")]

distance1 = hppfcl.distance(
    cmodel.geometryObjects[cmodel.getGeometryId("SPHERE1")].geometry,
    shape1_placement,
    cmodel.geometryObjects[cmodel.getGeometryId("SPHERE2")].geometry,
    shape2_placement,
    req,
    res,
)

tmp = (
    shape1_placement.inverse()
    * shape2_placement
)
print(f"distance hppfcl with oMg: {distance1}")

print(f"distance manually : {np.linalg.norm(tmp.translation)- (1.5e-1 *2)}")
