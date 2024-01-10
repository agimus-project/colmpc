from os.path import dirname, join, abspath
import time
import numpy as np

import pinocchio as pin
import hppfcl

from wrapper_meshcat import MeshcatWrapper
from wrapper_robot import RobotWrapper
from ocp_panda_reaching import OCPPandaReaching
from ocp_panda_reaching_obs_single_point import OCPPandaReachingColWithSingleCol

from ur_robot.utils import BLUE, YELLOW_FULL, get_transform

### PARAMETERS
# Number of nodes of the trajectory
T = 200
# Time step between each node
dt = 0.001


### LOADING THE ROBOT
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")
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
OBSTACLE_POSE_INIT = pin.SE3.Identity()
OBSTACLE_POSE_INIT.translation = np.array([0.25, -0.585, 1.5])
OBSTACLE = hppfcl.Sphere(OBSTACLE_RADIUS)
OBSTACLE_GEOM_OBJECT = pin.GeometryObject(
    "obstacle",
    rmodel.getFrameId("universe"),
    rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
    OBSTACLE,
    OBSTACLE_POSE_INIT,
)
OBSTACLE_GEOM_OBJECT.meshColor = BLUE

IG_OBSTACLE = cmodel.addGeometryObject(OBSTACLE_GEOM_OBJECT)

### INITIAL CONFIG OF THE ROBOT
INITIAL_CONFIG = pin.neutral(rmodel)

### ADDING THE COLLISION PAIR BETWEEN A LINK OF THE ROBOT & THE OBSTACLE
cmodel.geometryObjects[cmodel.getGeometryId("panda2_link5_sc_4")].meshColor = YELLOW_FULL
cmodel.addCollisionPair(
    pin.CollisionPair(cmodel.getGeometryId("panda2_link5_sc_4"), IG_OBSTACLE)
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
# input()
for xs in ddp.xs:
    break
    vis.display(np.array(xs[:7].tolist()))
    # time.sleep(1e-3)

start = 0
stop = 0.03
step = 0.001
theta_list = np.arange(start, stop, step)
results = {}

OBSTACLE_POSE = OBSTACLE_POSE_INIT.copy()

for k, theta in enumerate(theta_list):
    print(
        f"#################################################### ITERATION n°{k} out of {len(theta_list)-1}####################################################"
    )
    print(
        f"theta = {round(theta,3)} , step = {round(theta_list[0]-theta_list[1], 3)}, theta min = {round(theta_list[0],3)}, theta max = {round(theta_list[-1],3)} "
    )
    OBSTACLE_POSE.translation += np.array([0, theta, 0])
    cmodel.geometryObjects[
        cmodel.getGeometryId("obstacle")
    ].placement.translation = OBSTACLE_POSE.translation

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
        SAFETY_THRESHOLD=1e-2,
    )
    ddp = problem()

    # Solving the problem
    ddp.solve(XS_init, US_init)
    results[str(theta)] = ddp.xs.tolist()

    XS_init = ddp.xs.tolist()
    US_init = ddp.us.tolist()

print("End of the computation, press enter to display the traj if requested.")
### DISPLAYING THE TRAJ

rmodel, cmodel, vmodel = robot_wrapper()
rdata = rmodel.createData()
cdata = cmodel.createData()
cmodel.geometryObjects[cmodel.getGeometryId("panda2_link5_sc_4")].meshColor = YELLOW_FULL

MeshcatVis = MeshcatWrapper()
vis, meshcatvis = MeshcatVis.visualize(
    TARGET_POSE,
    OBSTACLE_POSE_INIT,
    robot_model=rmodel,
    robot_collision_model=cmodel,
    robot_visual_model=vmodel,
    obstacle_type="sphere",
    OBSTACLE_DIM=OBSTACLE_RADIUS,
)

while True:
    OBSTACLE_POSE = OBSTACLE_POSE_INIT.copy()
    meshcatvis["obstacle"].set_transform(get_transform(OBSTACLE_POSE))

    for k in range(len(theta_list)):
        print(f"theta : {theta_list[k]}, iteration n°{k + 1} out of {len(theta_list)}")
        OBSTACLE_POSE.translation += np.array([0, theta_list[k], 0])
        meshcatvis["obstacle"].set_transform(get_transform(OBSTACLE_POSE))
        for xs in results[str(theta_list[k])]:
            vis.display(np.array(xs[:7]))
            time.sleep(1e-3)
        input()
    print("replay?")
    input()
