from os.path import dirname, join, abspath
import time
import numpy as np

import pinocchio as pin
import hppfcl

from wrapper_meshcat import MeshcatWrapper
from wrapper_robot2 import RobotWrapper
from ocp_ur_reaching import OCPURReaching
from ocp_ur_reaching_obs_single_point import OCPURReachingColWithSingleCol

hppfcl.WITH_OCTOMAP = False

BLUE_FULL = np.array([144, 169, 183, 255]) / 255

YELLOW = np.array([1, 1, 0, 0.5])
YELLOW_FULL = np.array([1, 1, 0, 1.])

### PARAMETERS
# Number of nodes of the trajectory
T = 200
# Time step between each node
dt = 0.001


### LOADING THE ROBOT
# Creating the robot
robot_wrapper = RobotWrapper()
robot, rmodel, cmodel = robot_wrapper(target=True)

rdata = rmodel.createData()
cdata = cmodel.createData()

### CREATING THE TARGET
TARGET_POSE = robot_wrapper._M_target

### CREATING THE OBSTACLE
OBSTACLE_RADIUS = 2.0e-1
OBSTACLE_POSE = pin.SE3.Identity()
OBSTACLE_POSE.translation = np.array([0.25, -0.225, 0.3])
# OBSTACLE = hppfcl.Capsule(OBSTACLE_RADIUS, OBSTACLE_RADIUS)
OBSTACLE = hppfcl.Sphere(OBSTACLE_RADIUS)
OBSTACLE_GEOM_OBJECT = pin.GeometryObject(
    "obstacle",
    rmodel.getFrameId("universe"),
    rmodel.frames[rmodel.getFrameId("universe")].parent,
    OBSTACLE,
    OBSTACLE_POSE,
)
OBSTACLE_GEOM_OBJECT.meshColor = BLUE_FULL

IG_OBSTACLE = cmodel.addGeometryObject(OBSTACLE_GEOM_OBJECT)

### INITIAL CONFIG OF THE ROBOT
INITIAL_CONFIG = pin.neutral(rmodel)

### ADDING THE COLLISION PAIR BETWEEN A LINK OF THE ROBOT & THE OBSTACLE
cmodel.geometryObjects[cmodel.getGeometryId("endeff_geom")].meshColor = YELLOW_FULL
cmodel.addCollisionPair(
    pin.CollisionPair(cmodel.getGeometryId("endeff_geom"), IG_OBSTACLE)
)
cdata = cmodel.createData()

# Generating the meshcat visualizer
MeshcatVis = MeshcatWrapper()
vis, meshcatVis = MeshcatVis.visualize(
    TARGET_POSE,
    robot_model=rmodel,
    robot_collision_model=cmodel,
    robot_visual_model=cmodel,
)

### INITIAL X0
q0 = INITIAL_CONFIG
x0 = np.concatenate([q0, pin.utils.zero(rmodel.nv)])

### CREATING THE PROBLEM WITHOUT OBSTACLE
problem = OCPURReaching(
    rmodel,
    cmodel,
    TARGET_POSE,
    T,
    dt,
    x0,
    WEIGHT_GRIPPER_POSE=100,
    WEIGHT_xREG=1e-2,
    WEIGHT_uREG=1e-4,
)
ddp = problem()
# Solving the problem
ddp.solve()

XS_init = ddp.xs
US_init = ddp.us

vis.display(INITIAL_CONFIG)
input()
for xs in ddp.xs:
    vis.display(np.array(xs[:6].tolist()))
    time.sleep(1e-3)

### CREATING THE PROBLEM WITH WARM START
problem = OCPURReachingColWithSingleCol(
    rmodel,
    cmodel,
    TARGET_POSE,
    OBSTACLE_POSE,
    OBSTACLE_RADIUS,
    T,
    dt,
    x0,
    WEIGHT_GRIPPER_POSE=100,
    WEIGHT_xREG=1e-2,
    WEIGHT_uREG=1e-4,
    SAFETY_THRESHOLD=2e-2
)
ddp = problem()

XS_init = [x0] * (T+1)
US_init = [np.zeros(rmodel.nv)] * T
US_init = ddp.problem.quasiStatic(XS_init[:-1])

# Solving the problem
ddp.solve(XS_init, US_init)

print("End of the computation, press enter to display the traj if requested.")
### DISPLAYING THE TRAJ
while True:
    vis.display(INITIAL_CONFIG)
    input()
    for xs in ddp.xs:
        vis.display(np.array(xs[:6].tolist()))
        time.sleep(1e-3)
    input()
    print("replay")
