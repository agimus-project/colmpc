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
import hppfcl

from utils import RED

# This class is for unwrapping an URDF and converting it to a model. It is also possible to add objects in the model,
# such as a ball at a specific position.


class RobotWrapper:
    def __init__(
        self,
        scale=1.0,
        urdf_model_path=None,
        mesh_dir=None,
        srdf_model_path = None,
        auto_col = False,
        capsule = False,
    ):
        """Initialize the wrapper with a scaling number of the target and the name of the robot wanted to get unwrapped.

        Args:
            scale (float, optional): Scale of the target, by default 1.0. Defaults to 1.0.
            name_robot (str, optional): Name of the robot wanted to get unwrapped. Defaults to "franka".
            urdf_model_path (_type_, optional): _description_. Defaults to None.
            mesh_dir (_type_, optional): _description_. Defaults to None.
        """

        self._scale = scale
        self._urdf_model_path = urdf_model_path
        self._mesh_dir = mesh_dir
        self._srdf_model_path = srdf_model_path
        self._color = np.array([249, 136, 126, 255]) / 255
        self._auto_col = auto_col
        self._capsule = capsule

    def __call__(self):
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
        (
            self._rmodel,
            self._collision_model,
            self._visual_model,
        ) = pin.buildModelsFromUrdf(
            self._urdf_model_path, self._mesh_dir, pin.JointModelFreeFlyer()
        )

        q0 = pin.neutral(self._rmodel)

        jointsToLock = [
            "root_joint:",
            "panda2_finger_joint1:",
            "panda2_finger_joint2:",
            "universe",
        ]

        jointsToLockIDs = [1,9,10]

        geom_models = [self._visual_model, self._collision_model]
        self._model_reduced, geometric_models_reduced = pin.buildReducedModel(
            self._rmodel,
            list_of_geom_models=geom_models,
            list_of_joints_to_lock=jointsToLockIDs,
            reference_configuration=q0,
        )

        self._visual_model_reduced, self._collision_model_reduced = (
            geometric_models_reduced[0],
            geometric_models_reduced[1],
        )

        # Modifying the collision model to add the capsules
        rdata = self._model_reduced.createData()
        cdata = self._collision_model_reduced.createData()
        q0 = pin.neutral(self._model_reduced)

        # Updating the models
        pin.framesForwardKinematics(self._model_reduced, rdata, q0)
        pin.updateGeometryPlacements(
            self._model_reduced, rdata, self._collision_model_reduced, cdata, q0
        )

        # Adding the capsules to the collision model

        collision_model_reduced_copy = self._collision_model_reduced.copy()

        # Replacing the cylinders by capsules
        if self._capsule:
            list_names_capsules = []
            for i, geometry_object in enumerate(collision_model_reduced_copy.geometryObjects):

                if isinstance(geometry_object.geometry, hppfcl.Sphere):
                    self._collision_model_reduced.removeGeometryObject(geometry_object.name)
                # Only selecting the cylinders
                if isinstance(geometry_object.geometry, hppfcl.Cylinder):
                    if (geometry_object.name[:-4] + "capsule") in list_names_capsules:
                        capsule = pin.GeometryObject(
                        geometry_object.name[:-4] + "capsule" + "1",
                        geometry_object.parentJoint,
                        geometry_object.parentFrame,
                        geometry_object.placement,
                        hppfcl.Capsule(geometry_object.geometry.radius, geometry_object.geometry.halfLength),
                        )
                        capsule.meshColor = RED
                        self._collision_model_reduced.addGeometryObject(capsule)
                        self._collision_model_reduced.removeGeometryObject(geometry_object.name)
                        list_names_capsules.append(geometry_object.name[:-4] + "capsule" + "1" )
                    else:
                        capsule = pin.GeometryObject(
                        geometry_object.name[:-4] + "capsule",
                        geometry_object.parentJoint,
                        geometry_object.parentFrame,
                        geometry_object.placement,
                        hppfcl.Capsule(geometry_object.geometry.radius, geometry_object.geometry.halfLength),
                        )
                        capsule.meshColor = RED
                        self._collision_model_reduced.addGeometryObject(capsule)
                        self._collision_model_reduced.removeGeometryObject(geometry_object.name)
                        list_names_capsules.append(geometry_object.name[:-4] + "capsule")

        # Removing the geometry objects that aren't Capsule / Box and disabling the collisions for the finger and the camera
        for geometry_object in self._collision_model_reduced.geometryObjects:
            # Disabling the collisions for the fingers
            # weird utf-8 encoding shit
            try:
                if "finger" in geometry_object.name or "camera" in geometry_object.name or "support" in geometry_object.name:
                    geometry_object.disableCollision = True
            except:
                pass
            # Getting rid of the cylinders in cmodel
            if isinstance(geometry_object.geometry, hppfcl.Cylinder):
                self._collision_model_reduced.removeGeometryObject(geometry_object.name)


        # For some reasons, the following cylinders aren't removed with the loop from before.
        # self._collision_model_reduced.removeGeometryObject("panda2_link0_sc_0")
        # self._collision_model_reduced.removeGeometryObject('panda2_link7_sc_3')
        # self._collision_model_reduced.removeGeometryObject('panda2_link5_sc_0')
        # self._collision_model_reduced.removeGeometryObject('panda2_link4_sc_0')
        # self._collision_model_reduced.removeGeometryObject('panda2_link2_sc_0')

        if self._auto_col:
            self._collision_model_reduced.addAllCollisionPairs()
            # self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_link2_capsule37"),self._collision_model_reduced.getGeometryId("panda2_link6_capsule22") ))
            # self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_link4_capsule31"),self._collision_model_reduced.getGeometryId("panda2_link6_capsule22") ))
            # self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_link4_capsule31"),self._collision_model_reduced.getGeometryId("panda2_link5_capsule28") ))
            # self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_link4_capsule31"),self._collision_model_reduced.getGeometryId("panda2_link5_capsule25") ))
            # self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_link4_capsule31"),self._collision_model_reduced.getGeometryId("panda2_link3_capsule34") ))
            self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_leftfinger_0"),self._collision_model_reduced.getGeometryId("panda2_link6_capsule22") ))
            self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_leftfinger_0"),self._collision_model_reduced.getGeometryId("panda2_link6_capsule22") ))
            # self._collision_model_reduced.addCollisionPair(pin.CollisionPair(self._collision_model_reduced.getGeometryId("panda2_leftfinger_0"),self._collision_model_reduced.getGeometryId("support_link_0") ))


        pin.removeCollisionPairs(self._model_reduced, self._collision_model_reduced, self._srdf_model_path)

        return (
            self._model_reduced,
            self._collision_model_reduced,
            self._visual_model_reduced,
        )




if __name__ == "__main__":
    from os.path import dirname, join, abspath
    from wrapper_meshcat import MeshcatWrapper
    pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

    model_path = join(pinocchio_model_dir, "franka_description/robots")
    mesh_dir = pinocchio_model_dir
    urdf_filename = "franka2.urdf"
    urdf_model_path = join(join(model_path, "panda"), urdf_filename)
    srdf_model_path = model_path + "/panda/demo.srdf"

    # Creating the robot
    robot_wrapper = RobotWrapper(
        urdf_model_path=urdf_model_path,
        mesh_dir=mesh_dir,
        srdf_model_path=srdf_model_path
    )
    rmodel, cmodel, vmodel = robot_wrapper()
    rdata = rmodel.createData()
    cdata = cmodel.createData()
        # Generating the meshcat visualizer
    MeshcatVis = MeshcatWrapper()
    vis = MeshcatVis.visualize(robot_model=rmodel, robot_visual_model=vmodel, robot_collision_model=cmodel)
    vis[0].display(pin.neutral(rmodel))
    pin.computeCollisions(rmodel, rdata, cmodel, cdata, pin.neutral(rmodel), False)
    for k in range(len(cmodel.collisionPairs)):
        cr = cdata.collisionResults[k]
        cp = cmodel.collisionPairs[k]
        print("collision pair:",cp.first,",",cp.second,"- collision:","Yes" if cr.isCollision() else "No")
