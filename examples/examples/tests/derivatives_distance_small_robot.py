from os.path import dirname, join, abspath
import numpy as np
import matplotlib.pyplot as plt
import hppfcl

import pinocchio as pin
import pydiffcol


### HELPERS

YELLOW_FULL = np.array([1, 1, 0, 1.0])
BLUE_FULL = np.array([144, 169, 183, 255]) / 255

WITH_DISPLAY = True


def select_strategy(strat: str, verbose: bool = False):
    req = hppfcl.DistanceRequest()
    req.gjk_initial_guess = hppfcl.GJKInitialGuess.CachedGuess
    req.gjk_convergence_criterion = hppfcl.GJKConvergenceCriterion.DualityGap
    req.gjk_convergence_criterion_type = hppfcl.GJKConvergenceCriterionType.Absolute
    req.gjk_tolerance = 1e-8
    req.epa_tolerance = 1e-8
    req.epa_max_face_num = 1000
    req.epa_max_vertex_num = 1000
    req.epa_max_iterations = 1000
    req_diff = pydiffcol.DerivativeRequest()
    req_diff.warm_start = np.array([1.0, 0.0, 0.0])
    req_diff.support_hint = np.array([0, 0], dtype=np.int32)
    req_diff.use_analytic_hessians = False

    if strat == "finite_differences":
        req_diff.derivative_type = pydiffcol.DerivativeType.FiniteDifferences
    elif strat == "zero_order_gaussian":
        req_diff.derivative_type = pydiffcol.DerivativeType.ZeroOrderGaussian
    elif strat == "first_order_gaussian":
        req_diff.derivative_type = pydiffcol.DerivativeType.FirstOrderGaussian
    elif strat == "first_order_gumbel":
        req_diff.derivative_type = pydiffcol.DerivativeType.FirstOrderGumbel
    else:
        raise NotImplementedError

    if verbose:
        print("Strategy: ", req_diff.derivative_type)
        print("Noise: ", req_diff.noise)
        print("Num samples: ", req_diff.num_samples)

    return req, req_diff


def create_robot(M_target=pin.SE3.Identity()):
    rmodel = pin.Model()
    gmodel = pin.GeometryModel()
    r1 = 0.5  # Radii of the sphere


    # Creation of the joint models variables used to setup the robot

    revolut_joint_y = pin.JointModelRY()
    revolut_joint_z = pin.JointModelRZ()
    revolut_joint_x = pin.JointModelRX()

    # Id of the universe
    joint_universe = rmodel.getJointId("universe")

    # Creation of the robot

    # Creation of the joint 1 (a revolut around the z one)
    Mj1 = pin.SE3.Identity()  # Pose of the joint in the universe
    joint1_id = rmodel.addJoint(joint_universe, revolut_joint_z, Mj1, "joint1")

    # Creation of the frame F1
    joint1_frame = pin.Frame("joint1_frame", joint1_id, pin.SE3.Identity(), pin.BODY)
    joint1_frame_id = rmodel.addFrame(joint1_frame, False)

    # Creation of the shape
    joint1_shape = hppfcl.Sphere(r1)
    joint1_geom = pin.GeometryObject(
        "joint1_geom", joint1_id, joint1_frame_id, Mj1, joint1_shape)
    id_joint1_geom = gmodel.addGeometryObject(joint1_geom)


    # Creation of the joint 2 (a revolut one around the y axis)
    Mj2 = pin.SE3.Identity()
    Mj2.translation = np.array([0, 0, r1])

    joint2_id = rmodel.addJoint(joint1_id, revolut_joint_y, Mj2, "joint2")

    # Creation of the frame F2
    joint2_frame = pin.Frame("joint2_frame", joint2_id, Mj2, pin.BODY)
    joint2_frame_id = rmodel.addFrame(joint2_frame, False)

    # Creation of the shape
    joint2_shape = hppfcl.Sphere(r1)
    joint2_geom = pin.GeometryObject(
        "joint2_geom", joint2_id, joint2_frame_id, Mj2, joint2_shape)
    id_joint2_geom = gmodel.addGeometryObject(joint2_geom)

    # Creation of the frame for the target object

    target_frame = pin.Frame("target", rmodel.getJointId(
        "universe"), M_target, pin.BODY)
    target = rmodel.addFrame(target_frame, False)

    # Creation of the shape of the target
    T3 = rmodel.frames[target].placement
    target_shape = hppfcl.Sphere(r1)
    target_geom = pin.GeometryObject(
        "target_geom", rmodel.getJointId("universe"), target, T3, target_shape)
    id_target_geom = gmodel.addGeometryObject(target_geom)

    return rmodel, gmodel

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

    distance = pydiffcol.distance(
        shape1_geom,
        shape1_placement,
        shape2_geom,
        shape2_placement,
        req,
        res,
    )
    return distance


def dist_numdiff(q):
    """Finite derivative of the dist function at q.

    Args:
        q (np.ndarray): Configuration of the robot

    Returns:
        distance derivative: distance derivative between shape 1 & shape 2
    """
    j_diff = np.zeros(nq)
    fx = dist(q)
    for i in range(nq):
        e = np.zeros(nq)
        e[i] = 1e-5
        j_diff[i] = (dist(q + e) - dist(q)) / e[i]
    return j_diff


def ddist(q, FRAME=pin.LOCAL):
    """Diffcol derivative of the dist function with regards to .

    Args:
        q (np.ndarray): Configuration of the robot

    Returns:
        distance derivative: distance derivative between shape 1 & shape 2
    """

    # Computing the distance
    pin.framesForwardKinematics(rmodel, rdata, q)
    pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q)

    shape1_placement = cdata.oMg[shape1_id]
    shape2_placement = cdata.oMg[shape2_id]

    pin.computeJointJacobians(rmodel, rdata, q)

    pydiffcol.distance_derivatives(
        shape1_geom,
        shape1_placement,
        shape2_geom,
        shape2_placement,
        req,
        res,
        req_diff,
        res_diff,
    )

    jacobian = pin.computeFrameJacobian(
        rmodel,
        rdata,
        q,
        shape1.parentFrame,
        FRAME,
    )
    return np.dot(res_diff.ddist_dM1, jacobian)


def derivative_distance_sphere_sphere(FRAME=pin.LOCAL):
    pin.forwardKinematics(rmodel, rdata, q)
    pin.computeJointJacobians(rmodel, rdata, q)
    pin.updateGeometryPlacements(
        rmodel,
        rdata,
        cmodel,
        cdata,
        q,
    )
    jacobian = pin.computeFrameJacobian(
        rmodel,
        rdata,
        q,
        shape1.parentFrame,
        FRAME,
    )

    # Computing the distance
    distance = hppfcl.distance(
        shape1_geom,
        shape1_placement,
        shape2_geom,
        shape2_placement,
        req,
        res,
    )

    cp1 = res.getNearestPoint1()
    cp2 = res.getNearestPoint2()

    deriv = (cp1 - cp2).T / np.linalg.norm(cp1 - cp2) @ jacobian[:3]

    return deriv


def derivative_distance_sphere_sphere3(FRAME=pin.LOCAL):
    pin.forwardKinematics(rmodel, rdata, q)
    pin.computeJointJacobians(rmodel, rdata, q)
    pin.updateGeometryPlacements(
        rmodel,
        rdata,
        cmodel,
        cdata,
        q,
    )
    jacobian = pin.computeFrameJacobian(
        rmodel,
        rdata,
        q,
        shape1.parentFrame,
        FRAME,
    )

    # Computing the distance
    distance = hppfcl.distance(
        shape1_geom,
        shape1_placement,
        shape2_geom,
        shape2_placement,
        req,
        res,
    )

    cp1 = res.getNearestPoint1()
    cp2 = res.getNearestPoint2()

    CP1_SE3 = pin.SE3.Identity()
    CP1_SE3.translation = cp1

    CP2_SE3 = pin.SE3.Identity()
    CP2_SE3.translation = cp2

    deriv = (cp1 - cp2) / distance @ jacobian[:3]
    return deriv


def dist_numdiff(q):
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
        e[i] = 1e-5
        j_diff[i] = (dist(q + e) - dist(q)) / e[i]
    return j_diff


if __name__ == "__main__":
    # Loading the robot
    target = pin.SE3.Identity()
    target.translation = np.array([1,1,1])
    rmodel, cmodel = create_robot(target)
    # Creating the datas model


    ### CREATING THE OBSTACLE
    OBSTACLE_RADIUS = 1.5e-1
    OBSTACLE_POSE = pin.SE3.Identity()
    OBSTACLE_POSE.translation = np.array([0.25, -0.625, 1.5])
    # OBSTACLE = hppfcl.Capsule(OBSTACLE_RADIUS, OBSTACLE_RADIUS)
    OBSTACLE = hppfcl.Sphere(OBSTACLE_RADIUS)
    OBSTACLE_GEOM_OBJECT = pin.GeometryObject(
        "obstacle",
        rmodel.getFrameId("universe"),
        rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
        OBSTACLE,
        OBSTACLE_POSE,
    )

    IG_OBSTACLE = cmodel.addGeometryObject(OBSTACLE_GEOM_OBJECT)


    rdata = rmodel.createData()
    cdata = cmodel.createData()

    # Initial configuration
    q0 = pin.neutral(rmodel)
    q = np.array([1,1])
    # q = pin.neutral(rmodel)

    # Number of joints
    nq = rmodel.nq

    # Updating the models
    pin.framesForwardKinematics(rmodel, rdata, q0)
    pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q0)

    # Creating the shapes for the collision detection.
    shape1_id = cmodel.getGeometryId("joint2_geom")

    # Making sure the shape exists
    assert shape1_id <= len(cmodel.geometryObjects) -1

    # Coloring the sphere
    shape1 = cmodel.geometryObjects[shape1_id]
    shape1.meshColor = BLUE_FULL

    # Getting the geometry of the shape 1
    shape1_geom = shape1.geometry

    # Making sure the shape is a sphere
    assert isinstance(shape1_geom, hppfcl.Sphere)

    # Getting its pose in the world reference
    shape1_placement = cdata.oMg[shape1_id]

    # Doing the same for the second shape.
    shape2_id = cmodel.getGeometryId("target_geom")
    assert shape2_id <= len(cmodel.geometryObjects) -1

    # Coloring the sphere
    shape2 = cmodel.geometryObjects[shape2_id]
    shape2.meshColor = YELLOW_FULL

    shape2_geom = shape2.geometry
    assert isinstance(shape2_geom, hppfcl.Sphere)

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

    # Distance & Derivative results from diffcol
    req, req_diff = select_strategy("finite_differences")
    res = pydiffcol.DistanceResult()
    res_diff = pydiffcol.DerivativeResult()

    # Computing the distance between shape 1 & shape 2 at q0 & comparing with the distance anatically computed
    print(f"dist(q) : {round(dist(q0),6)}")
    dist2 = np.linalg.norm(shape1_placement.translation - shape2_placement.translation)
    print(
        f"np.linalg.norm(dist2) : {dist2 - shape1_geom.radius - shape2_geom.radius:.6f}"
    )

    deriv0 = ddist(q)
    deriv = derivative_distance_sphere_sphere()
    deriv3 = derivative_distance_sphere_sphere3()
    deriv_numdiff = dist_numdiff(q)
    np.set_printoptions(precision=3)
    print(f"deriv diffcol : {deriv0}")
    print(f"deriv : {deriv}")
    # print(f"deriv2 : {deriv2}")
    print(f"deriv3 : {deriv3}")
    print(f"numdif : {deriv_numdiff}")

    # Varying k from -pi to pi for a single joint to explore the derivatives of the distance through its full rotation.
    distance_list = []
    ddist_list_local = []
    ddist_numdiff_list_local = []
    ddist_list_florent_local = []
    ddist_list_analytical_local = []

    ddist_list_lwa = []
    ddist_numdiff_list_lwa = []
    ddist_list_florent_lwa = []
    ddist_list_analytical_lwa = []

    ddist_list_w = []
    ddist_numdiff_list_w = []
    ddist_list_florent_w = []
    ddist_list_analytical_w = []

    theta = np.linspace(-np.pi, np.pi, 1000)
    for k in theta:
        q = np.array([0, k])
        d = dist(q)
        FRAME = pin.LOCAL
        distance_list.append(d)
        ddist_diffcol = ddist(q, FRAME)
        ddist_numdiff = dist_numdiff(q)
        ddist_ana = derivative_distance_sphere_sphere3(FRAME)
        ddist_florent = derivative_distance_sphere_sphere(FRAME)
        ddist_list_local.append(ddist_diffcol)
        ddist_numdiff_list_local.append(ddist_numdiff)
        ddist_list_analytical_local.append(ddist_ana)
        ddist_list_florent_local.append(ddist_florent)

        FRAME = pin.LOCAL_WORLD_ALIGNED
        ddist_diffcol = ddist(q, FRAME)
        ddist_numdiff = dist_numdiff(q)
        ddist_ana = derivative_distance_sphere_sphere3(FRAME)
        ddist_florent = derivative_distance_sphere_sphere(FRAME)
        ddist_list_lwa.append(ddist_diffcol)
        ddist_numdiff_list_lwa.append(ddist_numdiff)
        ddist_list_analytical_lwa.append(ddist_ana)
        ddist_list_florent_lwa.append(ddist_florent)

        vis.display(q)

    plots = [331, 332, 333, 334, 335, 336, 337]
    plt.figure()

    for k in range(nq):
        plt.subplot(plots[k])
        plt.plot(
            theta,
            np.array(ddist_list_local)[:, k],
            label="diffcol_local",
            marker=".",
            markersize=5,
        )
        plt.plot(theta, np.array(ddist_numdiff_list_local)[:, k], "--", label="numdiff")
        plt.plot(
            theta,
            np.array(ddist_list_florent_local)[:, k],
            label="florent_local",
            marker=".",
            markersize=5,
        )
        plt.plot(
            theta,
            np.array(ddist_list_analytical_local)[:, k],
            "--",
            label="analytical_local",
        )
        plt.title("joint" + str(k))
        plt.ylabel(f"Distance derivative w.r.t. joint {k}")
    plt.legend()
    plt.suptitle(
        f"pin.LOCAL"
    )
    plt.figure()

    for k in range(nq):
        plt.subplot(plots[k])
        plt.plot(
            theta,
            np.array(ddist_list_lwa)[:, k],
            label="diffcol_lwa",
            marker=".",
            markersize=5,
        )
        plt.plot(theta, np.array(ddist_numdiff_list_local)[:, k], "--", label="numdiff")
        plt.plot(
            theta,
            np.array(ddist_list_florent_lwa)[:, k],
            label="florent_lwa",
            marker=".",
            markersize=5,
        )
        plt.plot(
            theta,
            np.array(ddist_list_analytical_lwa)[:, k],
            "--",
            label="analytical_lwa",
        )

        plt.title("joint" + str(k))
        plt.ylabel(f"Distance derivative w.r.t. joint {k}")
    plt.legend()

    plt.suptitle(
        f"LWA"    )

    plt.show()
