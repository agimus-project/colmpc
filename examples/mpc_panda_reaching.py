"""
Example script : MPC simulation with PANDA arm
static target reaching task
Inspired example from Sebastien Kleff: https://github.com/machines-in-motion/minimal_examples_crocoddyl
"""

import time

import mpc_utils
import numpy as np
import pybullet
import pin_utils
import pinocchio as pin

import matplotlib.pyplot as plt
from env import BulletEnv
from ocp_panda_reaching_obs import OCPPandaReachingColWithMultipleCol
from panda_robot_loader import PandaRobot
from meshcat_wrapper import MeshcatWrapper
np.set_printoptions(precision=4, linewidth=180)

# # # # # # # #
### HELPERS  ##
# # # # # # # #

WITH_PLOTS = True
WITH_SAVING_RESULTS = False

# # # # # # # # # # # # # # # # # # #
### LOAD ROBOT MODEL and SIMU ENV ###
# # # # # # # # # # # # # # # # # # #

# Simulation environment
env = BulletEnv(server=pybullet.DIRECT)
# Robot simulator
robot_simulator = PandaRobot()
env.add_robot(robot_simulator)

# Extract robot model
nq = robot_simulator.pin_robot.model.nq
nv = robot_simulator.pin_robot.model.nv
nu = nq
nx = nq + nv
q0 = np.array([0.1, 0.7, 0.0, 0.7, -0.5, 1.5, 0.0])
v0 = np.zeros(nv)
x0 = np.concatenate([q0, v0])
# Add robot to simulation and initialize
robot_simulator.reset_state(q0, v0)
robot_simulator.forward_robot(q0, v0)
print("[PyBullet] Created robot (id = " + str(robot_simulator.robotId) + ")")

shapes_in_collision_with_obstacle = [
    "panda2_rightfinger_0",
    "panda2_link7_sc_1",
    "panda2_link7_sc_4",
    "panda2_link6_sc_2",
    "panda2_link5_sc_3",
]

id_list = []
for shape in shapes_in_collision_with_obstacle:
    id_shape = robot_simulator.pin_robot.collision_model.getGeometryId(shape)
    id_obstacle = robot_simulator.pin_robot.collision_model.getGeometryId("obstacle")
    robot_simulator.pin_robot.collision_model.addCollisionPair(
        pin.CollisionPair(
            id_shape,
            id_obstacle,
        )
    )
    id_list.append(id_shape)

# # # # # # # # # # # #
###  OCP CONSTANTS  ###
# # # # # # # # # # # #

TARGET_POSE1 = pin.SE3(pin.utils.rotate("x", np.pi), np.array([0, -0.4, 1.5]))
TARGET_POSE2 = pin.SE3(pin.utils.rotate("x", np.pi), np.array([0, -0.0, 1.5]))

# OBSTACLE_POSE = pin.SE3(pin.utils.rotate("x", np.pi), np.array([0, -0.2, 1.5]))
OBSTACLE_POSE = robot_simulator.pin_robot.collision_model.geometryObjects[
    robot_simulator.pin_robot.collision_model.getGeometryId("obstacle")
].placement
OBSTACLE_RADIUS = 1.0e-1

dt = 2e-2
T = 10

max_iter = 4  # Maximum iterations of the solver
max_qp_iters = (
    25  # Maximum iterations for solving each qp solved in one iteration of the solver
)

WEIGHT_GRIPPER_POSE = 1e2
WEIGHT_GRIPPER_POSE_TERM = 1e2
WEIGHT_xREG = 1e-2
WEIGHT_xREG_TERM = 1e-2
WEIGHT_uREG = 1e-4
max_qp_iters = 25
callbacks = False
safety_threshhold = 7e-2

# vis.display(robot_simulator.pin_robot.)


# # # # # # # # # # # # # # #
###  SETUP CROCODDYL OCP  ###
# # # # # # # # # # # # # # #
# State and actuation model


### CREATING THE PROBLEM WITH OBSTACLE

print("Solving the problem with collision")
problem = OCPPandaReachingColWithMultipleCol(
    robot_simulator.pin_robot.model,
    robot_simulator.pin_robot.collision_model,
    TARGET_POSE1,
    T,
    dt,
    x0,
    WEIGHT_GRIPPER_POSE=WEIGHT_GRIPPER_POSE,
    WEIGHT_GRIPPER_POSE_TERM=WEIGHT_GRIPPER_POSE_TERM,
    WEIGHT_xREG=WEIGHT_xREG,
    WEIGHT_xREG_TERM=WEIGHT_xREG_TERM,
    WEIGHT_uREG=WEIGHT_uREG,
    max_qp_iters=max_qp_iters,
    SAFETY_THRESHOLD=safety_threshhold,
    callbacks=callbacks,
)
ddp = problem()

xs_init = [x0 for i in range(T + 1)]
us_init = ddp.problem.quasiStatic(xs_init[:-1])

ddp.solve(xs_init, us_init, maxiter=100)
xs_init = ddp.xs
us_init = ddp.us

# # # # # # # # # # # #
###  MPC SIMULATION ###
# # # # # # # # # # # #
# OCP parameters
ocp_params = {}
ocp_params["N_h"] = T
ocp_params["dt"] = dt
ocp_params["maxiter"] = max_iter
ocp_params["pin_model"] = robot_simulator.pin_robot.model
ocp_params["armature"] = problem._runningModel.differential.armature
ocp_params["id_endeff"] = robot_simulator.pin_robot.model.getFrameId(
    "panda2_leftfinger"
)
ocp_params["active_costs"] = ddp.problem.runningModels[
    0
].differential.costs.active.tolist()
# Simu parameters
sim_params = {}
sim_params["sim_freq"] = int(1.0 / env.dt)
sim_params["mpc_freq"] = 1000
sim_params["T_sim"] = 1.0
log_rate = 100


# Initialize simulation data
sim_data = mpc_utils.init_sim_data(sim_params, ocp_params, x0)
# Display target
mpc_utils.display_ball(
    TARGET_POSE1.translation, RADIUS=0.05, COLOR=[1.0, 0.0, 0.0, 0.6]
)
mpc_utils.display_ball(
    TARGET_POSE2.translation, RADIUS=0.5e-1, COLOR=[1.0, 0.0, 0.0, 0.6]
)

mpc_utils.display_ball(
    OBSTACLE_POSE.translation, RADIUS=OBSTACLE_RADIUS, COLOR=[1.0, 1.0, 0.0, 0.6]
)

print(f"sim_data['N_sim'] : {sim_data['N_sim']}")

time_calc = []
u_list = []
# Simulate
mpc_cycle = 0
TARGET_POSE = TARGET_POSE1
for i in range(sim_data["N_sim"]):
    if i % 500 == 0 and i != 0:
        ### Changing from target pose 1 to target pose 2 or inversely
        if TARGET_POSE == TARGET_POSE1:
            TARGET_POSE = TARGET_POSE2
        else:
            TARGET_POSE = TARGET_POSE1

        problem = OCPPandaReachingColWithMultipleCol(
            robot_simulator.pin_robot.model,
            robot_simulator.pin_robot.collision_model,
            TARGET_POSE,
            T,
            dt,
            sim_data["state_mea_SIM_RATE"][i, :],
            WEIGHT_GRIPPER_POSE=WEIGHT_GRIPPER_POSE,
            WEIGHT_GRIPPER_POSE_TERM=WEIGHT_GRIPPER_POSE_TERM,
            WEIGHT_xREG=WEIGHT_xREG,
            WEIGHT_xREG_TERM=WEIGHT_xREG_TERM,
            WEIGHT_uREG=WEIGHT_uREG,
            max_qp_iters=max_qp_iters,
            SAFETY_THRESHOLD=safety_threshhold,
            callbacks=callbacks,
        )
        ddp = problem()

    if i % log_rate == 0:
        print("\n SIMU step " + str(i) + "/" + str(sim_data["N_sim"]) + "\n")

    # Solve OCP if we are in a planning cycle (MPC/planning frequency)
    if i % int(sim_params["sim_freq"] / sim_params["mpc_freq"]) == 0:
        # Set x0 to measured state
        ddp.problem.x0 = sim_data["state_mea_SIM_RATE"][i, :]
        # Warm start using previous solution
        xs_init = [*list(ddp.xs[1:]), ddp.xs[-1]]
        xs_init[0] = sim_data["state_mea_SIM_RATE"][i, :]
        us_init = [*list(ddp.us[1:]), ddp.us[-1]]

        # Solve OCP & record MPC predictions
        start = time.process_time()
        ddp.solve(xs_init, us_init, maxiter=ocp_params["maxiter"])
        t_solve = time.process_time() - start
        time_calc.append(t_solve)
        sim_data["state_pred"][mpc_cycle, :, :] = np.array(ddp.xs)
        sim_data["ctrl_pred"][mpc_cycle, :, :] = np.array(ddp.us)
        # Extract relevant predictions for interpolations
        x_curr = sim_data["state_pred"][
            mpc_cycle, 0, :
        ]  # x0* = measured state    (q^,  v^ )
        x_pred = sim_data["state_pred"][
            mpc_cycle, 1, :
        ]  # x1* = predicted state   (q1*, v1*)
        u_curr = sim_data["ctrl_pred"][
            mpc_cycle, 0, :
        ]  # u0* = optimal control   (tau0*)
        # Record costs references
        q = sim_data["state_pred"][mpc_cycle, 0, : sim_data["nq"]]
        sim_data["ctrl_ref"][mpc_cycle, :] = pin_utils.get_u_grav(
            q,
            ddp.problem.runningModels[0].differential.pinocchio,
            ocp_params["armature"],
        )
        sim_data["state_ref"][mpc_cycle, :] = (
            ddp.problem.runningModels[0]
            .differential.costs.costs["stateReg"]
            .cost.residual.reference
        )
        sim_data["lin_pos_ee_ref"][mpc_cycle, :] = (
            ddp.problem.runningModels[0]
            .differential.costs.costs["gripperPoseRM"]
            .cost.residual.reference
        )

        # Select reference control and state for the current MPC cycle
        x_ref_MPC_RATE = x_curr + sim_data["ocp_to_mpc_ratio"] * (x_pred - x_curr)
        u_ref_MPC_RATE = u_curr
        if mpc_cycle == 0:
            sim_data["state_des_MPC_RATE"][mpc_cycle, :] = x_curr
        sim_data["ctrl_des_MPC_RATE"][mpc_cycle, :] = u_ref_MPC_RATE
        sim_data["state_des_MPC_RATE"][mpc_cycle + 1, :] = x_ref_MPC_RATE

        # Increment planning counter
        mpc_cycle += 1

        # Select reference control and state for the current SIMU cycle
        x_ref_SIM_RATE = x_curr + sim_data["ocp_to_mpc_ratio"] * (x_pred - x_curr)
        u_ref_SIM_RATE = u_curr

        # First prediction = measurement = initialization of MPC
        if i == 0:
            sim_data["state_des_SIM_RATE"][i, :] = x_curr
        sim_data["ctrl_des_SIM_RATE"][i, :] = u_ref_SIM_RATE
        sim_data["state_des_SIM_RATE"][i + 1, :] = x_ref_SIM_RATE

        # Send torque to simulator & step simulator
        robot_simulator.send_joint_command(u_ref_SIM_RATE)
        env.step()
        # Measure new state from simulator
        q_mea_SIM_RATE, v_mea_SIM_RATE = robot_simulator.get_state()
        # Update pinocchio model
        robot_simulator.forward_robot(q_mea_SIM_RATE, v_mea_SIM_RATE)
        # Record data
        x_mea_SIM_RATE = np.concatenate([q_mea_SIM_RATE, v_mea_SIM_RATE]).T
        sim_data["state_mea_SIM_RATE"][i + 1, :] = x_mea_SIM_RATE
        u_list.append(u_curr.tolist())

if WITH_PLOTS:
    distances = {}
    for shape in shapes_in_collision_with_obstacle:
        distances[shape] = []
    for q in sim_data["state_mea_SIM_RATE"]:
        for shape, id_shape in zip(shapes_in_collision_with_obstacle, id_list):
            dist = pin_utils.compute_distance_between_shapes(robot_simulator.pin_robot.model,robot_simulator.pin_robot.collision_model,id_shape, id_obstacle, q[:7])
            distances[shape].append(dist)
    
    for key, value in distances.items():
        plt.plot(value, label = key)
    plt.plot(np.zeros(len(value)), label = "collision line")
    plt.legend()
    plt.show()
    plot_data = mpc_utils.extract_plot_data_from_sim_data(sim_data)

    mpc_utils.plot_mpc_results(
        plot_data,
        which_plots=["all"],
        PLOT_PREDICTIONS=True,
        pred_plot_sampling=int(sim_params["mpc_freq"] / 10),
    )


# Generating the meshcat visualizer
MeshcatVis = MeshcatWrapper()
vis, meshcatVis = MeshcatVis.visualize(
    TARGET_POSE2,
    robot_model=robot_simulator.pin_robot.model,
    robot_collision_model=robot_simulator.pin_robot.collision_model,
    robot_visual_model=robot_simulator.pin_robot.collision_model,
)

while True:
    vis.display(q0)
    input()
    for q in sim_data["state_mea_SIM_RATE"]:
        vis.display(np.array(q[:7].tolist()))
        time.sleep(2e-2)
    input()
    print("replay")


    