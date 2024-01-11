'''
Example script : MPC simulation with KUKA arm 
static target reaching task
'''
import numpy as np
np.set_printoptions(precision=4, linewidth=180)

import pinocchio as pin
import pin_utils, mpc_utils

from mim_robots.pybullet.env import BulletEnvWithGround
from panda_robot_loader import PandaRobot
from ocp_panda_reaching import OCPPandaReaching
from ocp_panda_reaching_obs import OCPPandaReachingColWithMultipleCol
import pybullet as p


# # # # # # # # # # # # # # # # # # #
### LOAD ROBOT MODEL and SIMU ENV ### 
# # # # # # # # # # # # # # # # # # # 
# Simulation environment
env = BulletEnvWithGround(p.GUI, dt=1e-3)
# Robot simulator 
robot_simulator = PandaRobot()
env.add_robot(robot_simulator)

# Extract robot model
nq = robot_simulator.pin_robot.model.nq
nv = robot_simulator.pin_robot.model.nv
nu = nq; nx = nq+nv
q0 = np.array([0.1, 0.7, 0., 0.7, -0.5, 1.5, 0.])
v0 = np.zeros(nv)
x0 = np.concatenate([q0, v0])
# Add robot to simulation and initialize
robot_simulator.reset_state(q0, v0)
robot_simulator.forward_robot(q0, v0)
print("[PyBullet] Created robot (id = "+str(robot_simulator.robotId)+")")

TARGET_POSE = pin.SE3(pin.utils.rotate("x", np.pi), np.array([0, -0.4, 1.5]))
OBSTACLE_POSE = pin.SE3(pin.utils.rotate("x", np.pi), np.array([0.25, -0.425, 1.5]))
OBSTACLE_RADIUS = 1.5e-1
dt = 1e-2
T = 100

# # # # # # # # # # # # # # #
###  SETUP CROCODDYL OCP  ###
# # # # # # # # # # # # # # #
# State and actuation model


### CREATING THE PROBLEM WITHOUT OBSTACLE
problem = OCPPandaReaching(
    robot_simulator.pin_robot.model,
    robot_simulator.pin_robot.collision_model,
    TARGET_POSE,
    T,
    dt,
    x0,
    WEIGHT_GRIPPER_POSE=1e2,
    WEIGHT_GRIPPER_POSE_TERM=1e2,
    WEIGHT_xREG=1e-2,
    WEIGHT_xREG_TERM=1e-2,
    WEIGHT_uREG=1e-4,
)
ddp = problem()

xs_init = [x0 for i in range(T+1)]
us_init = ddp.problem.quasiStatic(xs_init[:-1])
# Solve
ddp.solve(xs_init, us_init, maxiter=100)

### CREATING THE PROBLEM WITH OBSTACLE

# Obstacle cost with hard constraint
robot_simulator.pin_robot.collision_model.addCollisionPair(
    pin.CollisionPair(robot_simulator.pin_robot.collision_model.getGeometryId("panda2_link6_sc_2"), robot_simulator.pin_robot.collision_model.getGeometryId("obstacle"))
)

print(robot_simulator.pin_robot.collision_model.geometryObjects[robot_simulator.pin_robot.collision_model.getGeometryId("panda2_link6_sc_2")].parentFrame)

problem = OCPPandaReachingColWithMultipleCol(
    robot_simulator.pin_robot.model,
    robot_simulator.pin_robot.collision_model,
    TARGET_POSE,
    OBSTACLE_POSE,
    OBSTACLE_RADIUS,
    T,
    dt,
    x0,
    WEIGHT_GRIPPER_POSE=1e2,
    WEIGHT_GRIPPER_POSE_TERM=1e2,
    WEIGHT_xREG=1e-2,
    WEIGHT_xREG_TERM=1e-2,
    WEIGHT_uREG=1e-4,
)
ddp = problem()

xs_init = ddp.xs
us_init = ddp.us
# Solve

ddp.solve(xs_init, us_init, maxiter=100)


# # # # # # # # # # # #
###  MPC SIMULATION ###
# # # # # # # # # # # #
# OCP parameters
ocp_params = {}
ocp_params['N_h']          = T
ocp_params['dt']           = dt
ocp_params['maxiter']      = 10 
ocp_params['pin_model']    = robot_simulator.pin_robot.model
ocp_params['armature']     = problem._runningModel.differential.armature
ocp_params['id_endeff']    = robot_simulator.pin_robot.model.getFrameId("panda2_leftfinger")
ocp_params['active_costs'] = ddp.problem.runningModels[0].differential.costs.active.tolist()
# Simu parameters
sim_params = {}
sim_params['sim_freq']  = int(1./env.dt)
sim_params['mpc_freq']  = 1000
sim_params['T_sim']     = 2.
log_rate = 100
# Initialize simulation data 
sim_data = mpc_utils.init_sim_data(sim_params, ocp_params, x0)
# Display target 
mpc_utils.display_ball(TARGET_POSE.translation, RADIUS=.05, COLOR=[1.,0.,0.,.6])
mpc_utils.display_ball(OBSTACLE_POSE.translation, RADIUS=1.5e-1, COLOR=[1.,1.,0.,.6])

# Simulate
mpc_cycle = 0
for i in range(sim_data['N_sim']): 

    if(i%log_rate==0): 
        print("\n SIMU step "+str(i)+"/"+str(sim_data['N_sim'])+"\n")

    # Solve OCP if we are in a planning cycle (MPC/planning frequency)
    if(i%int(sim_params['sim_freq']/sim_params['mpc_freq']) == 0):
        # Set x0 to measured state 
        ddp.problem.x0 = sim_data['state_mea_SIM_RATE'][i, :]
        # Warm start using previous solution
        xs_init = list(ddp.xs[1:]) + [ddp.xs[-1]]
        xs_init[0] = sim_data['state_mea_SIM_RATE'][i, :]
        us_init = list(ddp.us[1:]) + [ddp.us[-1]] 
        
        # Solve OCP & record MPC predictions
        ddp.solve(xs_init, us_init, maxiter=ocp_params['maxiter'])
        sim_data['state_pred'][mpc_cycle, :, :]  = np.array(ddp.xs)
        sim_data['ctrl_pred'][mpc_cycle, :, :]   = np.array(ddp.us)
        # Extract relevant predictions for interpolations
        x_curr = sim_data['state_pred'][mpc_cycle, 0, :]    # x0* = measured state    (q^,  v^ )
        x_pred = sim_data['state_pred'][mpc_cycle, 1, :]    # x1* = predicted state   (q1*, v1*) 
        u_curr = sim_data['ctrl_pred'][mpc_cycle, 0, :]     # u0* = optimal control   (tau0*)
        # Record costs references
        q = sim_data['state_pred'][mpc_cycle, 0, :sim_data['nq']]
        sim_data['ctrl_ref'][mpc_cycle, :]       = pin_utils.get_u_grav(q, ddp.problem.runningModels[0].differential.pinocchio, ocp_params['armature'])
        sim_data['state_ref'][mpc_cycle, :]      = ddp.problem.runningModels[0].differential.costs.costs['stateReg'].cost.residual.reference
        sim_data['lin_pos_ee_ref'][mpc_cycle, :] = ddp.problem.runningModels[0].differential.costs.costs['gripperPoseRM'].cost.residual.reference


        # Select reference control and state for the current MPC cycle
        x_ref_MPC_RATE  = x_curr + sim_data['ocp_to_mpc_ratio'] * (x_pred - x_curr)
        u_ref_MPC_RATE  = u_curr 
        if(mpc_cycle==0):
            sim_data['state_des_MPC_RATE'][mpc_cycle, :]   = x_curr  
        sim_data['ctrl_des_MPC_RATE'][mpc_cycle, :]    = u_ref_MPC_RATE   
        sim_data['state_des_MPC_RATE'][mpc_cycle+1, :] = x_ref_MPC_RATE    
        
        # Increment planning counter
        mpc_cycle += 1
        

        # Select reference control and state for the current SIMU cycle
        x_ref_SIM_RATE  = x_curr + sim_data['ocp_to_mpc_ratio'] * (x_pred - x_curr)
        u_ref_SIM_RATE  = u_curr 

        # First prediction = measurement = initialization of MPC
        if(i==0):
            sim_data['state_des_SIM_RATE'][i, :]   = x_curr  
        sim_data['ctrl_des_SIM_RATE'][i, :]    = u_ref_SIM_RATE  
        sim_data['state_des_SIM_RATE'][i+1, :] = x_ref_SIM_RATE 

        # Send torque to simulator & step simulator
        robot_simulator.send_joint_command(u_ref_SIM_RATE)
        env.step()
        # Measure new state from simulator 
        q_mea_SIM_RATE, v_mea_SIM_RATE = robot_simulator.get_state()
        # Update pinocchio model
        robot_simulator.forward_robot(q_mea_SIM_RATE, v_mea_SIM_RATE)
        # Record data 
        x_mea_SIM_RATE = np.concatenate([q_mea_SIM_RATE, v_mea_SIM_RATE]).T 
        sim_data['state_mea_SIM_RATE'][i+1, :] = x_mea_SIM_RATE


plot_data = mpc_utils.extract_plot_data_from_sim_data(sim_data)

mpc_utils.plot_mpc_results(plot_data, which_plots=['all'], PLOT_PREDICTIONS=True, pred_plot_sampling=int(sim_params['mpc_freq']/10))
