# 2-Clause BSD License

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import numpy as np
import pinocchio as pin
import meshcat
import example_robot_data as robex
import hppfcl

# This class is for unwrapping an URDF and converting it to a model. It is also possible to add objects in the model,
# such as a ball at a specific position.


YELLOW_FULL = np.array([1, 1, 0, 1.0])
BLUE_FULL = np.array([144, 169, 183, 255]) / 255

np.set_printoptions(precision=3)

class RobotWrapper:
    def __init__(self, scale=1.0, name_robot="ur10"):
        """Initialize the wrapper with a scaling number of the target and the name of the robot wanted to get unwrapped.

        Parameters
        ----------
        _scale : float, optional
            Scale of the target, by default 1.0
        name_robot : str, optional
            Name of the robot wanted to get unwrapped, by default "ur10"
        """

        self._scale = scale
        self._robot = robex.load(name_robot)
        self._rmodel = self._robot.model
        self._color = np.array([249, 136, 126, 255]) / 255

    def __call__(self, target=False):
        """Create a robot with a new frame at the end effector position and place a hppfcl: ShapeBase cylinder at this position.

        Parameters
        ----------
        target : bool, optional
            Boolean describing whether the user wants a target or not, by default False

        Returns
        -------
        _robot
            Robot description of the said robot
        _rmodel
            Model of the robot
        _gmodel
            Geometrical model of the robot


        """

        # Creation of the frame for the end effector by using the frame tool0, which is at the end effector pose.
        # This frame will be used for the position of the cylinder at the end of the effector.
        # The cylinder is used to have a HPPFCL shape at the end of the robot to make contact with the target

        # Obtaining the frame ID of the frame tool0
        ID_frame_tool0 = self._rmodel.getFrameId("tool0")
        # Obtaining the frame tool0
        frame_tool0 = self._rmodel.frames[ID_frame_tool0]
        # Obtaining the parent joint of the frame tool0
        parent_joint = frame_tool0.parent
        # Obtaining the placement of the frame tool0
        Mf_endeff = frame_tool0.placement

        # Creation of the geometrical model
        self._gmodel = self._robot.visual_model

        # Creation of the cylinder at the end of the end effector

        # Setting up the raddi of the cylinder
        endeff_radii = 1e-2
        # Creating a HPPFCL shape
        endeff_shape = hppfcl.Sphere(endeff_radii)
        # Creating a pin.GeometryObject for the model of the _robot
        geom_endeff = pin.GeometryObject(
            "endeff_geom", ID_frame_tool0, parent_joint, endeff_shape, Mf_endeff
        )
        geom_endeff.meshColor = self._color
        # Add the geometry object to the geometrical model
        self._gmodel.addGeometryObject(geom_endeff)

        if target:
            self._create_target()

        return self._robot, self._rmodel, self._gmodel

    def _create_target(self):
        """Updates the version of the robot models with a sphere that can be used as a target.

        Returns
        -------
        _robot
            Robot description of the said robot
        _rmodel
            Model of the robot
        _gmodel
            Geometrical model of the robot
        """

        # Setup of the shape of the target (a sphere here)
        r_target = 5e-2 * self._scale

        # Creation of the target

        # Creating the frame of the target

        self._M_target = self._generate_reachable_SE3_vector()

        target_frame = pin.Frame(
            "target", 0, self._rmodel.getFrameId("universe"), self._M_target, pin.BODY
        )
        target = self._rmodel.addFrame(target_frame, False)
        T_target = self._rmodel.frames[target].placement
        target_shape = hppfcl.Sphere(r_target)
        geom_target = pin.GeometryObject(
            "target_geom",
            self._rmodel.getFrameId("universe"),
            self._rmodel.getJointId("universe"),
            target_shape,
            T_target,
        )

        geom_target.meshColor = self._color
        self._gmodel.addGeometryObject(geom_target)

    def _generate_reachable_SE3_vector(self):
        """Generate a SE3 vector that can be reached by the robot.

        Returns
        -------
        Reachable_SE3_vector
            SE3 Vector describing a reachable position by the robot
        """

        # Generate a random configuration of the robot, with consideration to its limits
        self._q_target = pin.randomConfiguration(self._rmodel)
        # Creation of a temporary model.Data, to have access to the forward kinematics.
        ndata = self._rmodel.createData()
        # Updating the model.Data with the framesForwardKinematics
        pin.framesForwardKinematics(self._rmodel, ndata, self._q_target)

        return ndata.oMf[self._rmodel.getFrameId("tool0")]


def create_visualizer(robot, grid=False, axes=False):
    """Create a visualizer using meshcat, allowing a robot to be visualized.

    Parameters
    ----------
    robot :
        Robot unwrapped from pinocchio
    grid : bool, optional
        Whether the user wants the grid, by default False
    axes : bool, optional
        Whether the user wants the axes, by default False

    Returns
    -------
    vis
        A vis object, can be updated with vis.dispay(q), where q is an np.ndarray of robot.nq dimensions.
    """
    Viewer = pin.visualize.MeshcatVisualizer
    vis = Viewer(robot.model, robot.collision_model, robot.visual_model)
    vis.initViewer(viewer=meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000"))
    vis.viewer.delete()
    vis.loadViewerModel()
    if not grid:
        vis.viewer["/Grid"].set_property("visible", False)
    if not axes:
        vis.viewer["/Axes"].set_property("visible", False)
    return vis


def dist(q):
    """Computes the distance with hppfcl. Updates the hppfcl.distanceResult as well with hppfcl.distanceResult.getNearestPoint1() for instance.

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

    # try:
    #     distance = hppfcl.distance(
    #         shape1.geometry,
    #         shape1_placement,
    #         shape2.geometry,
    #         shape2_placement,
    #         req,
    #         res,
    #     )

    distance = hppfcl.distance(
        shape1.geometry,
        hppfcl.Transform3f(shape1_placement.rotation, shape1_placement.translation),
        shape2.geometry,
        hppfcl.Transform3f(shape2_placement.rotation, shape2_placement.translation),
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
        e[i] = 1e-12
        j_diff[i] = (dist(q + e) - dist(q)) / e[i]
    return j_diff

#      The derivative is computed page 3 and the result is :
#  $\ frac{\ partial d_{ij}}{\ partial q} (q) = \ frac{( R(q) - O(q))^{T} }{ || O(q) - R(q) ||} \ frac{\ partial R_{\in body}}{\ partial q}$


def derivative_distance_sphere_sphere_florent():
    """Distance derivatives found with the demonstration of florent : https://homepages.laas.fr/florent/publi/05icra1.pdf
    Returns:
        distance derivative: distance derivative between shape 1 & shape 2
    """
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
        pin.LOCAL_WORLD_ALIGNED,
    )

    # Computing the distance
    distance = hppfcl.distance(
        shape1.geometry,
        hppfcl.Transform3f(shape1_placement.rotation, shape1_placement.translation),
        shape2.geometry,
        hppfcl.Transform3f(shape2_placement.rotation, shape2_placement.translation),
        req,
        res,
        )

    cp1 = res.getNearestPoint1()
    cp2 = res.getNearestPoint2()

    deriv = (cp1 - cp2).T / np.linalg.norm(cp1 - cp2) @ jacobian[:3]

    return deriv

# Let S1 & S2 spheres of radius r1 & r2. As spheres are invariable by rotations, let's only work with translations t1 & t2 here. t1 = (x1,x2,x3), where xi $\in$ R3 & t2 = (y1,y2,y3).
# The distance can be written as : $     d (S_1 + t_1, S_2 + t_2) = || t_2 - t_1 || - (r_1 + r_2) $.
# Hence, the derivative : $     \ frac{\ partial d}{\ partial t_1} (S_1 + t_1, S_2 + t_2) = \ frac{\ partial}{\ partial t_1} || t_2 - t_1 || $,

# The distance can also be written as : $   d (S_1 + t_1, S_2 + t_2) = \sqrt{\sum ^3 _{i = 1} (y_i - x_i)^{2}} $.
# Now, it's only a simple vector derivatives.
# Hence :   $\ frac{\ partial d}{\partial t_1} (S_1 + t_1, S_2 + t_2) = \sum ^{3}_{i = 1} \ frac{(x_i - y_i) e_i }{d(S_1 + t_1, S_2 + t_2)} $ where $ (e_1,e_2,e_3)$ base of $R^3$.

def derivative_distance_sphere_sphere_analytics():
    """
    Distance derivatives found with the analytical demonstration of the derivative of distance with regards to the translation of the closest point of the shape 1.

    Returns:
        distance derivative: distance derivative between shape 1 & shape 2
    """
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
        pin.LOCAL_WORLD_ALIGNED,
    )

    # Computing the distance
    distance = hppfcl.distance(
        shape1.geometry,
        hppfcl.Transform3f(shape1_placement.rotation, shape1_placement.translation),
        shape2.geometry,
        hppfcl.Transform3f(shape2_placement.rotation, shape2_placement.translation),
        req,
        res,
        )
    cp1 = res.getNearestPoint1()
    cp2 = res.getNearestPoint2()

    CP1_SE3 = pin.SE3.Identity()
    CP1_SE3.translation = cp1

    CP2_SE3 = pin.SE3.Identity()
    CP2_SE3.translation = cp2

    deriv = (cp1 - cp2).T / distance @ jacobian[:3]
    return deriv


def dist_numdiff(q):
    """Finite derivative of the dist function with regards to the configuration vector of the robot.

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
    robot_wrapper = RobotWrapper()
    robot, rmodel, cmodel = robot_wrapper(target=True)

    rdata = rmodel.createData()
    cdata = cmodel.createData()

    vis = create_visualizer(robot)

    nq = rmodel.nq
    q0 = pin.neutral(rmodel)
    q = pin.randomConfiguration(rmodel)

    vis.display(q)

    # Updating the models
    pin.framesForwardKinematics(rmodel, rdata, q)
    pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata, q)

    # Creating the shapes for the collision detection.
    shape1_id = cmodel.getGeometryId("endeff_geom")

    # Coloring the sphere
    shape1 = cmodel.geometryObjects[shape1_id]
    shape1.meshColor = BLUE_FULL

    # Getting its pose in the world reference
    shape1_placement = cdata.oMg[shape1_id]

    # Doing the same for the second shape.
    shape2_id = cmodel.getGeometryId("target_geom")

    # Coloring the sphere
    shape2 = cmodel.geometryObjects[shape2_id]
    shape2.meshColor = YELLOW_FULL

    # Getting its pose in the world reference
    shape2_placement = cdata.oMg[shape2_id]

    # Hppfcl distance result & request
    req = hppfcl.DistanceRequest()
    res = hppfcl.DistanceResult()

    ### DISTANCE & DERIVATIVES

        # Computing the distance between shape 1 & shape 2 at q0 & comparing with the distance anatically computed
    print(f"dist(q) : {dist(q):.6f}")
    dist2 = np.linalg.norm(shape1_placement.translation - shape2_placement.translation)
    print(
        f"np.linalg.norm(dist2) : {dist2 - shape1.geometry.radius - shape2.geometry.radius:.6f}"
    )

    deriv_ana = derivative_distance_sphere_sphere_analytics()
    deriv_florent = derivative_distance_sphere_sphere_florent()
    deriv_numdiff = dist_numdiff(q)

    print(f"deriv ana: {deriv_ana}")
    print(f"deriv florent: {deriv_florent}")
    print(f"numdif : {deriv_numdiff}")


    ######### TESTING
    def test():
        # Making sure the shapes exist
        assert shape1_id <= len(cmodel.geometryObjects) - 1
        assert shape2_id <= len(cmodel.geometryObjects) - 1

        # Making sure the shapes are spheres
        assert isinstance(shape1.geometry, hppfcl.Sphere)
        assert isinstance(shape2.geometry, hppfcl.Sphere)

    test()
