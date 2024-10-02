import pinocchio as pin
import crocoddyl
import colmpc as col
import numpy as np
import mim_solvers

def create_ocp_velocity(rmodel, gmodel, param_parser):
    # Stat and actuation model
    state = crocoddyl.StateMultibody(rmodel)
    actuation = crocoddyl.ActuationModelFull(state)

    # Running & terminal cost models
    runningCostModel = crocoddyl.CostModelSum(state)
    terminalCostModel = crocoddyl.CostModelSum(state)

    ### Creation of cost terms

    # State Regularization cost
    xResidual = crocoddyl.ResidualModelState(state, param_parser.get_X0())
    xRegCost = crocoddyl.CostModelResidual(state, xResidual)

    # Control Regularization cost
    uResidual = crocoddyl.ResidualModelControl(state)
    uRegCost = crocoddyl.CostModelResidual(state, uResidual)

    # End effector frame cost
    framePlacementResidual = crocoddyl.ResidualModelFramePlacement(
        state,
        rmodel.getFrameId("panda2_hand_tcp"),
        param_parser.get_target_pose(),
    )


    goalTrackingCost = crocoddyl.CostModelResidual(
        state, framePlacementResidual
    )

    # Obstacle cost with hard constraint
    runningConstraintModelManager = crocoddyl.ConstraintModelManager(
        state, actuation.nu
    )
    terminalConstraintModelManager = crocoddyl.ConstraintModelManager(
        state, actuation.nu
    )
    # Creating the residual

    for col_idx, col_pair in enumerate(gmodel.collisionPairs):
        obstacleVelocityResidual = col.ResidualModelVelocityAvoidance(
        state , gmodel , col_idx, param_parser.get_di(), param_parser.get_ds(),param_parser.get_ksi() 
        )
        # Creating the inequality constraint
        constraint = crocoddyl.ConstraintModelResidual(
            state,
            obstacleVelocityResidual,
            np.array([0]),
            np.array([np.inf]),
        )

        # Adding the constraint to the constraint manager
        runningConstraintModelManager.addConstraint(
            "col_" + str(col_idx), constraint
        )
        terminalConstraintModelManager.addConstraint(
            "col_term_" + str(col_idx), constraint
        )


    # Adding costs to the models
    runningCostModel.addCost("stateReg", xRegCost, param_parser.get_W_xREG())
    runningCostModel.addCost("ctrlRegGrav", uRegCost, param_parser.get_W_uREG())
    runningCostModel.addCost(
        "gripperPoseRM", goalTrackingCost, param_parser.get_W_gripper_pose()
    )
    terminalCostModel.addCost("stateReg", xRegCost, param_parser.get_W_xREG())
    terminalCostModel.addCost(
        "gripperPose", goalTrackingCost, param_parser.get_W_gripper_pose_term()
    )

    # Create Differential Action Model (DAM), i.e. continuous dynamics and cost functions
    running_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state,
        actuation,
        runningCostModel,
        runningConstraintModelManager,
    )
    terminal_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state,
        actuation,
        terminalCostModel,
        terminalConstraintModelManager,
    )

    runningModel = crocoddyl.IntegratedActionModelEuler(
        running_DAM, param_parser.get_dt()
    )
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        terminal_DAM, 0.0
    )

    runningModel.differential.armature = np.array(
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
    )
    terminalModel.differential.armature = np.array(
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
    )

    problem = crocoddyl.ShootingProblem(
        param_parser.get_X0(), [runningModel] * param_parser.get_T(), terminalModel
    )
    # Create solver + callbacks
    # Define mim solver with inequalities constraints
    ocp = mim_solvers.SolverCSQP(problem)

    # Merit function
    ocp.use_filter_line_search = False

    # Parameters of the solver
    ocp.termination_tolerance = 1e-3
    ocp.max_qp_iters = 1000
    ocp.eps_abs = 1e-6
    ocp.eps_rel = 0

    ocp.with_callbacks = True
    
    return ocp

def create_ocp_distance(rmodel, gmodel, param_parser):
     # Stat and actuation model
    state = crocoddyl.StateMultibody(rmodel)
    actuation = crocoddyl.ActuationModelFull(state)

    # Running & terminal cost models
    runningCostModel = crocoddyl.CostModelSum(state)
    terminalCostModel = crocoddyl.CostModelSum(state)

    ### Creation of cost terms

    # State Regularization cost
    xResidual = crocoddyl.ResidualModelState(state, param_parser.get_X0())
    xRegCost = crocoddyl.CostModelResidual(state, xResidual)

    # Control Regularization cost
    uResidual = crocoddyl.ResidualModelControl(state)
    uRegCost = crocoddyl.CostModelResidual(state, uResidual)

    # End effector frame cost
    framePlacementResidual = crocoddyl.ResidualModelFramePlacement(
        state,
        rmodel.getFrameId("panda2_hand_tcp"),
        param_parser.get_target_pose(),
    )


    goalTrackingCost = crocoddyl.CostModelResidual(
        state, framePlacementResidual
    )

    # Obstacle cost with hard constraint
    runningConstraintModelManager = crocoddyl.ConstraintModelManager(
        state, actuation.nu
    )
    terminalConstraintModelManager = crocoddyl.ConstraintModelManager(
        state, actuation.nu
    )
    # Creating the residual

    for col_idx, col_pair in enumerate(gmodel.collisionPairs):
        obstacleDistanceResidual = col.ResidualDistanceCollision(
        state ,7, gmodel, col_idx
        )
        # Creating the inequality constraint
        constraint = crocoddyl.ConstraintModelResidual(
            state,
            obstacleDistanceResidual,
            np.array([param_parser.get_safety_threshold()]),
            np.array([np.inf]),
        )

        # Adding the constraint to the constraint manager
        runningConstraintModelManager.addConstraint(
            "col_" + str(col_idx), constraint
        )
        terminalConstraintModelManager.addConstraint(
            "col_term_" + str(col_idx), constraint
        )


       # Adding costs to the models
    runningCostModel.addCost("stateReg", xRegCost, param_parser.get_W_xREG())
    runningCostModel.addCost("ctrlRegGrav", uRegCost, param_parser.get_W_uREG())
    runningCostModel.addCost(
        "gripperPoseRM", goalTrackingCost, param_parser.get_W_gripper_pose()
    )
    terminalCostModel.addCost("stateReg", xRegCost, param_parser.get_W_xREG())
    terminalCostModel.addCost(
        "gripperPose", goalTrackingCost, param_parser.get_W_gripper_pose_term()
    )

    # Create Differential Action Model (DAM), i.e. continuous dynamics and cost functions
    running_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state,
        actuation,
        runningCostModel,
        runningConstraintModelManager,
    )
    terminal_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state,
        actuation,
        terminalCostModel,
        terminalConstraintModelManager,
    )

    runningModel = crocoddyl.IntegratedActionModelEuler(
        running_DAM, param_parser.get_dt()
    )
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        terminal_DAM, 0.0
    )

    runningModel.differential.armature = np.array(
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
    )
    terminalModel.differential.armature = np.array(
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
    )

    problem = crocoddyl.ShootingProblem(
        param_parser.get_X0(), [runningModel] * param_parser.get_T(), terminalModel
    )
    # Create solver + callbacks
    # Define mim solver with inequalities constraints
    ocp = mim_solvers.SolverCSQP(problem)

    # Merit function
    ocp.use_filter_line_search = False

    # Parameters of the solver
    ocp.termination_tolerance = 1e-3
    ocp.max_qp_iters = 1000
    ocp.eps_abs = 1e-6
    ocp.eps_rel = 0

    ocp.with_callbacks = True
    
    return ocp



def create_ocp_nocol(rmodel, param_parser):
     # Stat and actuation model
    state = crocoddyl.StateMultibody(rmodel)
    actuation = crocoddyl.ActuationModelFull(state)

    # Running & terminal cost models
    runningCostModel = crocoddyl.CostModelSum(state)
    terminalCostModel = crocoddyl.CostModelSum(state)

    ### Creation of cost terms

    # State Regularization cost
    xResidual = crocoddyl.ResidualModelState(state, param_parser.get_X0())
    xRegCost = crocoddyl.CostModelResidual(state, xResidual)

    # Control Regularization cost
    uResidual = crocoddyl.ResidualModelControl(state)
    uRegCost = crocoddyl.CostModelResidual(state, uResidual)

    # End effector frame cost
    framePlacementResidual = crocoddyl.ResidualModelFramePlacement(
        state,
        rmodel.getFrameId("panda2_hand_tcp"),
        param_parser.get_target_pose(),
    )


    goalTrackingCost = crocoddyl.CostModelResidual(
        state, framePlacementResidual
    )

       # Adding costs to the models
    runningCostModel.addCost("stateReg", xRegCost, param_parser.get_W_xREG())
    runningCostModel.addCost("ctrlRegGrav", uRegCost, param_parser.get_W_uREG())
    runningCostModel.addCost(
        "gripperPoseRM", goalTrackingCost, param_parser.get_W_gripper_pose()
    )
    terminalCostModel.addCost("stateReg", xRegCost, param_parser.get_W_xREG())
    terminalCostModel.addCost(
        "gripperPose", goalTrackingCost, param_parser.get_W_gripper_pose_term()
    )

    # Create Differential Action Model (DAM), i.e. continuous dynamics and cost functions
    running_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state,
        actuation,
        runningCostModel,
    )
    terminal_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
        state,
        actuation,
        terminalCostModel,
    )

    runningModel = crocoddyl.IntegratedActionModelEuler(
        running_DAM, param_parser.get_dt()
    )
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        terminal_DAM, 0.0
    )

    runningModel.differential.armature = np.array(
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
    )
    terminalModel.differential.armature = np.array(
        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0]
    )

    problem = crocoddyl.ShootingProblem(
        param_parser.get_X0(), [runningModel] * param_parser.get_T(), terminalModel
    )
    # Create solver + callbacks
    # Define mim solver with inequalities constraints
    ocp = mim_solvers.SolverCSQP(problem)

    # Merit function
    ocp.use_filter_line_search = False

    # Parameters of the solver
    ocp.termination_tolerance = 1e-3
    ocp.max_qp_iters = 1000
    ocp.eps_abs = 1e-6
    ocp.eps_rel = 0

    ocp.with_callbacks = True