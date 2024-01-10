## Class heavily inspired by the work of Sebastien Kleff : https://github.com/machines-in-motion/minimal_examples_crocoddyl

from typing import Any
import numpy as np
import crocoddyl
import pinocchio as pin
import mim_solvers

from residualDistanceCollision import ResidualCollision


class OCPURReachingColWithSingleCol:
    """This class is creating a optimal control problem of a UR robot reaching for a target while taking a collision between a given previously given shape of the robot and an obstacle into consideration"""

    def __init__(
        self,
        rmodel: pin.Model,
        cmodel: pin.GeometryModel,
        TARGET_POSE: pin.SE3,
        OBSTACLE_POSE: pin.SE3,
        OBSTACLE_RADIUS: float,
        T: int,
        dt: float,
        x0: np.ndarray,
        WEIGHT_xREG=1e-1,
        WEIGHT_uREG=1e-4,
        WEIGHT_GRIPPER_POSE=10,
        SAFETY_THRESHOLD=1e-2,
    ) -> None:
        """Creating the class for optimal control problem of a UR robot reaching for a target while taking a collision between a given previously given shape of the robot and an obstacle into consideration.

        Args:
            rmodel (pin.Model): pinocchio Model of the robot
            cmodel (pin.GeometryModel): Collision model of the robot
            TARGET_POSE (pin.SE3): Pose of the target in WOLRD ref
            OBSTACLE_POSE (pin.SE3): Pose of the obstacle in the universe ref
            OBSTACLE_RADIUS (float): Radius of the obstacle
            T (int): Number of nodes in the trajectory
            dt (float): Time step between each node
            x0 (np.ndarray): Initial state of the problem
            WEIGHT_xREG (float, optional): State regulation weight. Defaults to 1e-1.
            WEIGHT_uREG (float, optional): Command regulation weight. Defaults to 1e-4.
            WEIGHT_GRIPPER_POSE (float, optional): End effector pose weight. Defaults to 10.
            SAFETY_THRESHOLD (float, optional): Safety threshold of collision avoidance. Defaults to 1e-2.
        """
        # Models of the robot
        self._rmodel = rmodel
        self._cmodel = cmodel

        # Poses & dimensions of the target & obstacle
        self._TARGET_POSE = TARGET_POSE
        self._OBSTACLE_RADIUS = OBSTACLE_RADIUS
        self._OBSTACLE_POSE = OBSTACLE_POSE
        self._SAFETY_THRESHOLD = SAFETY_THRESHOLD

        # Params of the problem
        self._T = T
        self._dt = dt
        self._x0 = x0

        # Weights
        self._WEIGHT_xREG = WEIGHT_xREG
        self._WEIGHT_uREG = WEIGHT_uREG
        self._WEIGHT_GRIPPER_POSE = WEIGHT_GRIPPER_POSE

        # Data models
        self._rdata = rmodel.createData()
        self._cdata = cmodel.createData()

        # Frames
        self._endeff_frame = self._rmodel.getFrameId("tool0")

        # Making sure that the frame exists
        assert self._endeff_frame <= len(self._rmodel.frames)

        # Collision pair id
        k = 0

        # Making sure that the pair of collision exists
        assert k <= len(self._cmodel.collisionPairs)

        # Collision pair
        self._collisionPair = self._cmodel.collisionPairs[k]

        # Geometry ID of the shape 1 of collision pair
        self._id_shape1 = self._collisionPair.first

        # Making sure that the frame exists
        assert self._id_shape1 <= len(self._cmodel.geometryObjects)

        # Geometry object shape 1
        self._shape1 = self._cmodel.geometryObjects[self._id_shape1]

        # Shape 1 parent joint
        self._shape1_parentJoint = self._shape1.parentJoint

        # Geometry ID of the shape 2 of collision pair
        self._id_shape2 = self._collisionPair.second

        # Making sure that the frame exists
        assert self._id_shape2 <= len(self._cmodel.geometryObjects)

        # Geometry object shape 2
        self._shape2 = self._cmodel.geometryObjects[self._id_shape2]

        # Shape 2 parent joint
        self._shape2_parentJoint = self._shape2.parentJoint

        # Checking that shape 1 is belonging to the robot & shape 2 is the obstacle
        assert not "obstacle" in self._shape1.name
        assert "obstacle" in self._shape2.name

    def __call__(self) -> Any:
        "Setting up croccodyl OCP"

        # Stat and actuation model
        self._state = crocoddyl.StateMultibody(self._rmodel)
        self._actuation = crocoddyl.ActuationModelFull(self._state)

        # Running & terminal cost models
        self._runningCostModel = crocoddyl.CostModelSum(self._state)
        self._terminalCostModel = crocoddyl.CostModelSum(self._state)

        ### Creation of cost terms

        # State Regularization cost
        xResidual = crocoddyl.ResidualModelState(self._state, self._x0)
        xRegCost = crocoddyl.CostModelResidual(self._state, xResidual)

        # Control Regularization cost
        uResidual = crocoddyl.ResidualModelControl(self._state)
        uRegCost = crocoddyl.CostModelResidual(self._state, uResidual)

        # End effector frame cost
        framePlacementResidual = crocoddyl.ResidualModelFrameTranslation(
            self._state,
            self._endeff_frame,
            self._TARGET_POSE.translation,
        )

        goalTrackingCost = crocoddyl.CostModelResidual(
            self._state, framePlacementResidual
        )

        # Obstacle cost with hard constraint
        self._runningConstraintModelManager = crocoddyl.ConstraintModelManager(
            self._state, self._actuation.nu
        )
        self._terminalConstraintModelManager = crocoddyl.ConstraintModelManager(
            self._state, self._actuation.nu
        )
        # Creating the residual
        obstacleDistanceResidual = ResidualCollision(
            self._state, self._cmodel, self._cdata, 0
        )

        # Creating the inequality constraint
        constraint = crocoddyl.ConstraintModelResidual(
            self._state,
            obstacleDistanceResidual,
            np.array([self._SAFETY_THRESHOLD]),
            np.array([np.inf]),
        )

        # Adding the constraint to the constraint manager
        self._runningConstraintModelManager.addConstraint("col", constraint)
        self._terminalConstraintModelManager.addConstraint("col_term", constraint)

        # Adding costs to the models
        self._runningCostModel.addCost("stateReg", xRegCost, self._WEIGHT_xREG)
        self._runningCostModel.addCost("ctrlRegGrav", uRegCost, self._WEIGHT_uREG)
        self._runningCostModel.addCost(
            "gripperPoseRM", goalTrackingCost, self._WEIGHT_GRIPPER_POSE
        )
        self._terminalCostModel.addCost("stateReg", xRegCost, self._WEIGHT_xREG)
        self._terminalCostModel.addCost(
            "gripperPose", goalTrackingCost, self._WEIGHT_GRIPPER_POSE
        )

        # Create Differential Action Model (DAM), i.e. continuous dynamics and cost functions
        self._running_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self._state,
            self._actuation,
            self._runningCostModel,
            self._runningConstraintModelManager,
        )
        self._terminal_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self._state,
            self._actuation,
            self._terminalCostModel,
            self._terminalConstraintModelManager,
        )

        # Create Integrated Action Model (IAM), i.e. Euler integration of continuous dynamics and cost
        self._runningModel = crocoddyl.IntegratedActionModelEuler(
            self._running_DAM, self._dt
        )
        self._terminalModel = crocoddyl.IntegratedActionModelEuler(
            self._terminal_DAM, 0.0
        )

        self._runningModel.differential.armature = np.array(
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        )
        self._terminalModel.differential.armature = np.array(
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        )

        problem = crocoddyl.ShootingProblem(
            self._x0, [self._runningModel] * self._T, self._terminalModel
        )
        # Create solver + callbacks
        # ddp = crocoddyl.SolverSQP(problem)

        # ddp.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose()])

        # Define mim solver with inequalities constraints
        ddp = mim_solvers.SolverCSQP(problem)

        # Merit function
        ddp.use_filter_line_search = True

        # Parameters of the solver
        ddp.termination_tolerance = 1e-3
        ddp.max_qp_iters = 10000
        ddp.eps_abs = 1e-6
        ddp.eps_rel = 0

        ddp.with_callbacks = True

        return ddp
