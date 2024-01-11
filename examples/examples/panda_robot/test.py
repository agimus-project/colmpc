from os.path import dirname, join, abspath
import time
import numpy as np

import pinocchio as pin
import hppfcl

from wrapper_meshcat import MeshcatWrapper
from wrapper_robot import RobotWrapper
from ocp_panda_reaching import OCPPandaReaching
from ocp_panda_reaching_obs_single_point import OCPPandaReachingColWithSingleCol

from utils import BLUE, YELLOW_FULL

### PARAMETERS
# Number of nodes of the trajectory
T = 200
# Time step between each node
dt = 0.001


### LOADING THE ROBOT
pinocchio_model_dir = join(dirname(dirname(dirname(str(abspath(__file__))))), "models")
model_path = join(pinocchio_model_dir, "franka_description/robots")
mesh_dir = pinocchio_model_dir
urdf_filename = "franka2.urdf"
urdf_model_path = join(join(model_path, "panda"), urdf_filename)
srdf_model_path = model_path + "/panda/demo.srdf"

# Creating the robot
robot_wrapper = RobotWrapper(
    urdf_model_path=urdf_model_path, mesh_dir=mesh_dir, srdf_model_path=srdf_model_path
)
rmodel, cmodel, vmodel = robot_wrapper()
rdata = rmodel.createData()
cdata = cmodel.createData()

### CREATING THE TARGET
TARGET_POSE = pin.SE3(pin.utils.rotate("x", np.pi), np.array([0, 0, 1.55]))
TARGET_POSE.translation = np.array([0, -0.4, 1.5])

### CREATING THE OBSTACLE
OBSTACLE_RADIUS = 1.5e-1
OBSTACLE_POSE = pin.SE3.Identity()
OBSTACLE_POSE.translation = np.array([0.25, -0.425, 1.5])
OBSTACLE = hppfcl.Sphere(OBSTACLE_RADIUS)
OBSTACLE_GEOM_OBJECT = pin.GeometryObject(
    "obstacle",
    rmodel.getFrameId("universe"),
    rmodel.frames[rmodel.getFrameId("universe")].parent,
    OBSTACLE,
    OBSTACLE_POSE,
)
OBSTACLE_GEOM_OBJECT.meshColor = BLUE

IG_OBSTACLE = cmodel.addGeometryObject(OBSTACLE_GEOM_OBJECT)

### INITIAL CONFIG OF THE ROBOT
INITIAL_CONFIG = pin.neutral(rmodel)

### ADDING THE COLLISION PAIR BETWEEN A LINK OF THE ROBOT & THE OBSTACLE
cmodel.geometryObjects[cmodel.getGeometryId("panda2_link6_sc_2")].meshColor = YELLOW_FULL
cmodel.addCollisionPair(
    pin.CollisionPair(cmodel.getGeometryId("panda2_link6_sc_2"), IG_OBSTACLE)
)
cdata = cmodel.createData()
print(f'shape 1 : {cmodel.getGeometryId("panda2_link6_sc_2")}')
print(f'shape 2 : {IG_OBSTACLE}')

# Generating the meshcat visualizer
MeshcatVis = MeshcatWrapper()
vis, meshcatVis = MeshcatVis.visualize(
    TARGET_POSE,
    robot_model=rmodel,
    robot_collision_model=cmodel,
    robot_visual_model=vmodel,
)

### INITIAL X0
q0 = INITIAL_CONFIG
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])
req = hppfcl.DistanceRequest()
res = hppfcl.DistanceResult()
q_test = np.array([ 2.02829e-01, 3.74201e-01, 3.06873e-01,-6.99667e-01,-8.18421e-02, 1.29950e+00, 1.43251e-02])
pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q_test)
print(
    hppfcl.distance(
        cmodel.geometryObjects[cmodel.getGeometryId("panda2_link6_sc_2")].geometry,
        cdata.oMg[cmodel.getGeometryId("panda2_link6_sc_2")],
        cmodel.geometryObjects[IG_OBSTACLE].geometry,
        cdata.oMg[IG_OBSTACLE],
        req, 
        res
    )
)
q_test = np.array([2.02997e-01, 3.71892e-01, 3.07120e-01,-6.98672e-01,-8.17971e-02, 1.29954e+00, 1.43237e-02])
pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q_test)
print(
    hppfcl.distance(
        cmodel.geometryObjects[cmodel.getGeometryId("panda2_link6_sc_2")].geometry,
        cdata.oMg[cmodel.getGeometryId("panda2_link6_sc_2")],
        cmodel.geometryObjects[IG_OBSTACLE].geometry,
        cdata.oMg[IG_OBSTACLE],
        req, 
        res
    )
)