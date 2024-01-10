import hppfcl
import pinocchio as pin
import numpy as np
import time
import copy
import matplotlib.pyplot as plt
import meshcat

import meshcat.geometry as g
import meshcat.transformations as tf

RED = np.array([249, 136, 126, 125]) / 255
RED_FULL = np.array([249, 136, 126, 255]) / 255

GREEN = np.array([170, 236, 149, 125]) / 255
GREEN_FULL = np.array([170, 236, 149, 255]) / 255

BLUE = np.array([144, 169, 183, 125]) / 255
BLUE_FULL = np.array([144, 169, 183, 255]) / 255

YELLOW = np.array([1, 1, 0, 0.5])
YELLOW_FULL = np.array([1, 1, 0, 1.])

BLACK = np.array([0, 0, 0, 0.5])
BLACK_FULL = np.array([0, 0, 0, 1.])


pairs_to_avoid = (
    ("panda2_link0_sc", "panda2_link1_sc"),
    ("panda2_link0_sc", "panda2_link2_sc"),
    ("panda2_link1_sc","panda2_link2_sc"),
    ("panda2_link1_sc","panda2_link3_sc"),
    ("panda2_link2_sc","panda2_link3_sc"),
    ("panda2_link3_sc","panda2_link4_sc"),
    ("panda2_link3_sc","panda2_link5_sc"),
    ("panda2_link4_sc","panda2_link5_sc"),
    ("panda2_link5_sc","panda2_link6_sc"),
    ("panda2_link5_sc","panda2_link7_sc"),
    ("panda2_link6_sc","panda2_link7_sc"),
    ("panda2_link6_sc","panda2_hand_sc"),
    ("panda2_link7_sc","panda2_hand_sc"),
    ("panda2_link6_sc","panda2_leftfinger"),
    ("panda2_link7_sc","panda2_leftfinger"),
    ("panda2_hand_sc","panda2_leftfinger"),
    ("panda2_link6_sc","panda2_rightfinger"),
    ("panda2_link7_sc","panda2_rightfinger"),
    ("panda2_hand_sc","panda2_rightfinger"),
    ("panda2_leftfinger","panda2_rightfinger"),
)

def get_transform(T_: hppfcl.Transform3f):
    """Returns a np.ndarray instead of a pin.SE3 or a hppfcl.Transform3f

    Args:
        T_ (hppfcl.Transform3f): transformation to change into a np.ndarray. Can be a pin.SE3 as well

    Raises:
        NotADirectoryError: _description_

    Returns:
        _type_: _description_
    """
    T = np.eye(4)
    if isinstance(T_, hppfcl.Transform3f):
        T[:3, :3] = T_.getRotation()
        T[:3, 3] = T_.getTranslation()
    elif isinstance(T_, pin.SE3):
        T[:3, :3] = T_.rotation
        T[:3, 3] = T_.translation
    else:
        raise NotADirectoryError
    return T

def get_transform_from_list(T_ : list):
    T = pin.SE3.Identity()
    T.translation = np.array([T_[0][3],T_[1][3],T_[2][3]])
    T_rot = np.eye(3)
    T_rot[0][:] = T_[0][:3]
    T_rot[1][:] = T_[1][:3]
    T_rot[2][:] = T_[2][:3]
    T.rotation = T_rot
    return T

def get_q_iter_from_Q(Q: np.ndarray, iter: int, nq: int):
    """Returns the iter-th configuration vector q_iter in the Q array.

    Args:
        Q (np.ndarray): Optimization vector.
        iter (int): Index of the q_iter desired.
        nq (int): size of q_iter

    Returns:
        q_iter (np.ndarray): Array of the configuration of the robot at the iter-th step.
    """
    q_iter = np.array((Q[nq * iter : nq * (iter + 1)]))
    return q_iter


def get_difference_between_q_iter(Q: np.ndarray, iter: int, nq: int):
    """Returns the difference between the q_iter and q_iter-1 in the array self.Q

    Parameters
    ----------
    Q : np.ndarray
        Optimization vector.
    iter : int
        Index of the q_iter desired.
    nq : int
        Length of a configuration vector.

    Returns:
        q_iter+1 - q_iter (np.ndarray): Difference of the arrays of the configuration of the robot at the iter-th and ither +1 -th steps.

    """
    return get_q_iter_from_Q(Q, iter, nq) - get_q_iter_from_Q(Q, iter - 1, nq)


def get_difference_between_q_iter_sup(Q: np.ndarray, iter: int, nq: int):
    """Returns the difference between the q_iter + 1 and q_iter in the array self.Q

    Parameters
    ----------
    Q : np.ndarray
        Optimization vector.
    iter : int
        Index of the q_iter desired.
    nq : int
        Length of a configuration vector.

    Returns:
        q_iter+1 - q_iter (np.ndarray): Difference of the arrays of the configuration of the robot at the iter-th and ither +1 -th steps.

    """
    return get_q_iter_from_Q(Q, iter + 1, nq) - get_q_iter_from_Q(Q, iter, nq)


def display_last_traj(vis, Q: np.ndarray, q0: np.ndarray, T: int, dt=None):
    """Display the trajectory computed by the solver

    Parameters
    ----------
    vis : Meshcat.Visualizer
        Meshcat visualizer
    Q : np.ndarray
        Optimization vector.
    q0 : np.ndarray
        Initial configuration vector
    nq : int
        size of q_iter
    """
    for q_iter in [q0] + np.split(Q, T):
        vis.display(q_iter)
        if dt is None:
            input()
        else:
            time.sleep(dt)


def display_last_traj_with_obstacle_moving(
    vis, meshcatvis, Q: np.ndarray, q0: np.ndarray, T: int, theta_list, TARGET, dt=None
):
    """Display the trajectory computed by the solver

    Parameters
    ----------
    vis : Meshcat.Visualizer
        Meshcat visualizer
    Q : np.ndarray
        Optimization vector.
    q0 : np.ndarray
        Initial configuration vector
    nq : int
        size of q_iter
    """
    for q_iter in [q0] + np.split(Q, T + 1):
        vis.display(q_iter)
        for theta in theta_list:
            OBSTACLE_translation = TARGET.translation / 2 + [
                0.2 + theta,
                0 + theta,
                0.8 + theta,
            ]
            rotation = np.identity(3)
            rotation[1, 1] = 0
            rotation[2, 2] = 0
            rotation[1, 2] = -1
            rotation[2, 1] = 1
            OBSTACLE_rotation = rotation
            OBSTACLE = TARGET.copy()
            OBSTACLE.translation = OBSTACLE_translation
            OBSTACLE.rotation = OBSTACLE_rotation
            meshcatvis["obstacle"].set_transform(get_transform(OBSTACLE))
            time.sleep(dt)
        if dt is None:
            input()
        else:
            time.sleep(dt)


def numdiff(f, x, eps=1e-8):
    """Estimate df/dx at x with finite diff of step eps

    Parameters
    ----------
    f : function handle
        Function evaluated for the finite differente of its gradient.
    x : np.ndarray
        Array at which the finite difference is calculated
    eps : float, optional
        Finite difference step, by default 1e-6

    Returns
    -------
    jacobian : np.ndarray
        Finite difference of the function f at x.
    """
    xc = np.copy(x)
    f0 = np.copy(f(x))
    res = []
    for i in range(len(x)):
        xc[i] += eps
        res.append(copy.copy(f(xc) - f0) / eps)
        xc[i] = x[i]
    return np.array(res).T


def generate_reachable_target(
    rmodel, rdata=None, frameName="endeff", returnConfiguration=False
):
    """
    Sample a random configuration, then returns the forward kinematics
    for this configuration rdata.oMf[frameId].
    If rdata is None, create it on the flight (warning emitted)
    """
    q_target = pin.randomConfiguration(rmodel)

    # Creation of a temporary model.Data, to have access to the forward kinematics.
    if rdata is None:
        rdata = rmodel.createData()
        print("Warning: pin.Data create for a simple kinematic, please avoid")

    # Updating the model.Data with the framesForwardKinematics
    pin.framesForwardKinematics(rmodel, rdata, q_target)

    # Get and check Frame Id
    fid = rmodel.getFrameId(frameName)
    assert fid < len(rmodel.frames)

    if returnConfiguration:
        return rdata.oMf[fid].copy(), q_target
    return rdata.oMf[fid].copy()


# def plot_end_effector_positions(
#     rmodel: pin.Model,
#     cmodel: pin.Model,
#     rdata: pin.Data,
#     Q: np.ndarray,
#     T: int,
#     nq,
#     TARGET: pin.SE3,
#     TARGET_SHAPE: hppfcl.ShapeBase,
# ):
#     px_l, py_l, pz_l = [], [], []
#     resx_l, resy_l, resz_l = [], [], []

#     for t in range(T):
#         q_t = get_q_iter_from_Q(Q, t, nq)
#         req = hppfcl.DistanceRequest()
#         res = pydiffcol.DistanceResult()
#         pin.framesForwardKinematics(rmodel, rdata, q_t)

#         endeff_Shape = cmodel.geometryObjects[
#             cmodel.getGeometryId("panda2_link7_sc_5")
#         ].geometry
#         endeff_Transform = rdata.oMf[rmodel.getFrameId("panda2_joint7")]

#         dist_endeff_target = pydiffcol.distance(
#             endeff_Shape,
#             endeff_Transform,
#             TARGET_SHAPE,
#             TARGET,
#             req,
#             res,
#         )

#         px, py, pz = endeff_Transform.translation

#         px_l.append(px)
#         py_l.append(py)
#         pz_l.append(pz)

#         resx_l.append(res.w[0])
#         resy_l.append(res.w[1])
#         resz_l.append(res.w[2])

#     px_t = np.ones(len(px_l)) * TARGET.translation[0]
#     py_t = np.ones(len(py_l)) * TARGET.translation[1]
#     pz_t = np.ones(len(pz_l)) * TARGET.translation[2]

#     goal = np.zeros((len(resx_l)))
#     plt.figure()
#     plt.subplot(311)
#     plt.plot(px_l, "-o", label="End effector pose")
#     plt.plot(px_t, "-", label="Target position")
#     plt.legend()
#     plt.ylabel("px (m)")
#     plt.subplot(312)
#     plt.plot(py_l, "-o", label="End effector pose")
#     plt.plot(py_t, "-", label="Target position")
#     plt.ylabel("py (m)")
#     plt.legend()
#     plt.subplot(313)
#     plt.plot(pz_l, "-o", label="End effector pose")
#     plt.plot(pz_t, "-", label="Target position")
#     plt.ylabel("pz (m)")
#     plt.xlabel("Iterations")
#     plt.suptitle("3D Position of the end effector through iterations")
#     plt.legend()

#     plt.figure()
#     plt.subplot(311)
#     plt.plot(resx_l, "-o", label="End effector pose")
#     plt.plot(goal, label="Goal")
#     plt.legend()
#     plt.ylabel("px (m)")
#     plt.subplot(312)
#     plt.plot(resy_l, "-o", label="End effector pose")
#     plt.plot(goal, label="Goal")
#     plt.legend()
#     plt.ylabel("py (m)")
#     plt.subplot(313)
#     plt.plot(resz_l, "-o", label="End effector pose")
#     plt.plot(goal, label="Goal")
#     plt.legend()
#     plt.ylabel("pz (m)")
#     plt.xlabel("Iterations")
#     plt.suptitle(
#         "Vector separating witness points on the target and the end effector through iterations"
#     )


def RGB_to_hex(RGB):
    """[255,255,255] -> "#FFFFFF" """
    # Components need to be integers for hex to make sense
    RGB = [int(x) for x in RGB]
    return "#" + "".join(
        ["0{0:x}".format(v) if v < 16 else "{0:x}".format(v) for v in RGB]
    )


def hex_to_RGB(hex_str):
    """#FFFFFF -> [255,255,255]"""
    # Pass 16 to the integer function for change of base
    return [int(hex_str[i : i + 2], 16) for i in range(1, 6, 2)]


def color_dict(gradient):
    """Takes in a list of RGB sub-lists and returns dictionary of
    colors in RGB and hex form for use in a graphing function
    defined later on"""
    return {
        "hex": [RGB_to_hex(RGB) for RGB in gradient],
        "r": [RGB[0] for RGB in gradient],
        "g": [RGB[1] for RGB in gradient],
        "b": [RGB[2] for RGB in gradient],
    }


def linear_gradient(start_hex, finish_hex="#FFFFFF", n=10):
    """returns a gradient list of (n) colors between
    two hex colors. start_hex and finish_hex
    should be the full six-digit color string,
    inlcuding the number sign ("#FFFFFF")"""
    # Starting and ending colors in RGB form
    s = hex_to_RGB(start_hex)
    f = hex_to_RGB(finish_hex)
    # Initilize a list of the output colors with the starting color
    RGB_list = [s]
    # Calcuate a color at each evenly spaced value of t from 1 to n
    for t in range(1, n):
        # Interpolate RGB vector for color at the current value of t
        curr_vector = [
            int(s[j] + (float(t) / (n - 1)) * (f[j] - s[j])) for j in range(3)
        ]
        # Add it to our list of output colors
        RGB_list.append(curr_vector)

    return color_dict(RGB_list)



def check_limits(rmodel : pin.Model,rdata : pin.Data,  Q : np.ndarray, CHECK_POS = True, CHECK_SPEED = True, CHECK_ACCEL = True):
    """Checks whether the trajectory in input respects the limits given by the URDF and translated into the pinocchio model.

    Args:
        rmodel (pin.Model): Pinocchio model of the robot.
        rdata (pin.Data) : Data of the pinocchio model.
        Q (np.ndarray): Array describing the trajectory of the robot.
        CHECK_POS (bool, optional): Checking the positions limits of each joint. Defaults to True.
        CHECK_SPEED (bool, optional): Checking the speed limit of each joint. Defaults to True.
        CHECK_ACCEL (bool, optional): Checking the accel limit of each joint thanks to the effort limit. Defaults to False. # TO DO

    Returns:
        _type_: _description_
    """
    # Going through all the configurations in Q

    pos_respect = True
    vel_respect = True
    accel_respect = True
    pos_defect = []
    vel_defect = []
    accel_defect = []
    k_pos_defect = []
    k_vel_defect = []
    k_accel_deffect = []
    for k in range(int(len(Q)/rmodel.nq)):
        # Obtaining q_k & q_k+1
        q_k = get_q_iter_from_Q(Q, k, rmodel.nq)
        # Checking the positions limits if requested
        if CHECK_POS:
            for i, (q, q_min, q_max) in enumerate(zip(q_k, rmodel.lowerPositionLimit, rmodel.upperPositionLimit)):
                if q > q_max or q < q_min:
                    pos_respect = False
                    pos_defect.append(q)
                    k_pos_defect.append(k * rmodel.nq + i)
        # Checking the speed limits if requested
        if CHECK_SPEED:
            if k == int(len(Q)/rmodel.nq) -1:
                break
            q_k_next = get_q_iter_from_Q(Q, k+1, rmodel.nq)
            vel_k = q_k_next - q_k
            for ii, (vel, vel_max) in enumerate(zip(vel_k, rmodel.velocityLimit)):
                if abs(vel) > vel_max:
                    vel_respect = False
                    vel_defect.append(vel)
                    k_vel_defect.append(k * rmodel.nq + ii)
        if CHECK_ACCEL:
            if k == int(len(Q)/rmodel.nq) -2:
                break
            q_k_next = get_q_iter_from_Q(Q, k+1, rmodel.nq)
            q_k_next_next = get_q_iter_from_Q(Q, k+2, rmodel.nq)
            vel_k = q_k_next - q_k
            vel_k_next = q_k_next_next - q_k_next
            a_k = vel_k_next - vel_k
            M = pin.crba(rmodel, rdata, q_k)
            a_max_k = np.linalg.pinv(pin.crba(rmodel, rdata, q_k)) @ rmodel.effortLimit
            print(a_max_k)
            for ii, (a, a_max) in enumerate(zip(a_k, a_max_k)):
                if abs(a) > abs(a_max):
                    accel_respect = True
                    accel_defect.append(vel)
                    k_accel_deffect.append(k * rmodel.nq + ii)

    return "Respect the limits of positions ?",pos_respect, "values of the defect :", pos_defect, "positions of the defect", k_pos_defect,"Respect the limits of speed ?", vel_respect,"values of the defect :", vel_defect,"positions of the defect", k_vel_defect,"Respect the limits of accel ?", accel_respect,"values of the defect :", accel_defect,"positions of the defect", accel_defect

# def check_auto_collisions(rmodel : pin.Model, rdata : pin.Data, cmodel : pin.GeometryModel, cdata : pin.Data):
#     """Check whether the model is in auto-collision.

#     Args:
#         rmodel (pin.Model): Pinocchio model of the robot
#         rdata (pin.Data): Data of the model
#         cmodel (pin.GeometryModel): Collision model of the robot
#         cdata (pin.Data): collision data of the robot
#     """
#     pairs_to_avoid = (
#         ("panda2_link0_sc", "panda2_link1_sc"),
#         ("panda2_link0_sc", "panda2_link2_sc"),
#         ("panda2_link1_sc","panda2_link2_sc"),
#         ("panda2_link1_sc","panda2_link3_sc"),
#         ("panda2_link2_sc","panda2_link3_sc"),
#         ("panda2_link3_sc","panda2_link4_sc"),
#         ("panda2_link3_sc","panda2_link5_sc"),
#         ("panda2_link4_sc","panda2_link5_sc"),
#         ("panda2_link5_sc","panda2_link6_sc"),
#         ("panda2_link5_sc","panda2_link7_sc"),
#         ("panda2_link6_sc","panda2_link7_sc"),
#         ("panda2_link6_sc","panda2_hand_sc"),
#         ("panda2_link7_sc","panda2_hand_sc"),
#         ("panda2_link6_sc","panda2_leftfinger"),
#         ("panda2_link7_sc","panda2_leftfinger"),
#         ("panda2_hand_sc","panda2_leftfinger"),
#         ("panda2_link6_sc","panda2_rightfinger"),
#         ("panda2_link7_sc","panda2_rightfinger"),
#         ("panda2_hand_sc","panda2_rightfinger"),
#         ("panda2_leftfinger","panda2_rightfinger"),
#     )
#     oMg_list = []
#     geometry_objects_name = []
#     geometry_objects_geom = []
#     collision_pairs = []
#     # Distance request for pydiffcol
#     req, req_diff = select_strategy("first_order_gaussian")
#     res = pydiffcol.DistanceResult()
#     res_diff = pydiffcol.DerivativeResult()

#     for oMg, geometry_objects in zip(cdata.oMg, cmodel.geometryObjects):
#         # Only selecting the cylinders
#         if isinstance(geometry_objects.geometry, hppfcl.Cylinder):
#             oMg_list.append(oMg)
#             geometry_objects_name.append(geometry_objects.name[:-2])
#             geometry_objects_geom.append(geometry_objects.geometry)

#     # Going through all the geometry objects of the collision model
#     for oMg, geometry_objects in zip(cdata.oMg, cmodel.geometryObjects):
#         # Only selecting the cylinders
#         if isinstance(geometry_objects.geometry, hppfcl.Cylinder):
#             for oMg_ref, geometry_objects_name_ref, geometry_objects_geom_ref in zip(oMg_list, geometry_objects_name, geometry_objects_geom):
#                 if (geometry_objects_name_ref, geometry_objects.name[:-2]) not in pairs_to_avoid and (geometry_objects.name[:-2],geometry_objects_name_ref ) not in pairs_to_avoid and not geometry_objects_name_ref==geometry_objects.name[:-2]:
#                     dist = pydiffcol.distance(
#                         hppfcl.Capsule(geometry_objects_geom_ref.radius,geometry_objects_geom_ref.halfLength),
#                         oMg_ref,
#                         hppfcl.Capsule(geometry_objects.geometry.radius,geometry_objects.geometry.halfLength),
#                         oMg,
#                         req,
#                         res
#                     )
#                     if dist < 0:
#                         collision_pairs.append((geometry_objects.name, geometry_objects_name_ref))

#     return collision_pairs

def rgbToHex(color):
    if len(color) == 4:
        c = color[:3]
        opacity = color[3]
    else:
        c = color
        opacity = 1.0
    hex_color = "0x%02x%02x%02x" % (int(c[0] * 255), int(c[1] * 255), int(c[2] * 255))
    return hex_color, opacity


def meshcat_material(r, g, b, a):
    material = meshcat.geometry.MeshPhongMaterial()
    material.color = int(r * 255) * 256**2 + int(g * 255) * 256 + int(b * 255)
    material.opacity = a
    return material


# Building the meshcat materials
red = meshcat_material(RED[0], RED[1], RED[2], RED[3])
green = meshcat_material(GREEN[0], GREEN[1], GREEN[2], GREEN[3])
yellow = meshcat_material(YELLOW[0], YELLOW[1], YELLOW[2], YELLOW[3])
blue = meshcat_material(BLUE[0], BLUE[1], BLUE[2], BLUE[3])


if __name__ == "__main__":
    import example_robot_data as robex

    robot = robex.load("ur10")
    p = generate_reachable_target(robot.model, robot.data, "tool0")

    assert np.all(np.isfinite(p.translation))
