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
TARGET_POSE = pin.SE3(pin.utils.rotate("x", np.pi), np.array([-0.2, -0.0, 0.8]))

### CREATING THE OBSTACLE
OBSTACLE_RADIUS = 1.5e-1
OBSTACLE_HALFLENGTH = 2e-1
OBSTACLE_POSE = pin.SE3(pin.utils.rotate("x", np.pi/2), np.array([0.1, -0.0, 0.8]))
OBSTACLE = hppfcl.Capsule(OBSTACLE_RADIUS, OBSTACLE_HALFLENGTH)
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

### CREATING THE PROBLEM WITHOUT OBSTACLE
problem = OCPPandaReaching(
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
    vis.display(np.array(xs[:7].tolist()))
    time.sleep(1e-3)

### CREATING THE PROBLEM WITH WARM START
problem = OCPPandaReachingColWithSingleCol(
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
    SAFETY_THRESHOLD=1e-2
)
ddp = problem()

# Solving the problem
ddp.solve(XS_init, US_init)

print("End of the computation, press enter to display the traj if requested.")
### DISPLAYING THE TRAJ
while True:
    vis.display(INITIAL_CONFIG)
    input()
    for xs in ddp.xs:
        vis.display(np.array(xs[:7].tolist()))
        time.sleep(1e-3)
    input()
    print("replay")
