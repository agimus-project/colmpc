"""wrapper

Pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""
try:
    # use standard Python importlib if available (Python>3.7)
    import importlib.resources as importlib_resources
except ImportError:
    import importlib_resources
import pybullet
import time


class BulletEnv(object):
    """This class manages a PyBullet simulation environment and provides utility functions to interact with :py:obj:`PinBulletWrapper` objects.

    Attributes:
        dt (float): The length of the simulation integration step.
        objects (list): The list of the PyBullet ids for all the non-robot objects.
        robots (list): The list of the robot wrapper of all added robots.
    """

    def __init__(self, server=pybullet.GUI, dt=0.001):
        """Initializes the PyBullet client.

        Args:
            server (int, optional): PyBullet server mode. pybullet.GUI creates a graphical frontend using OpenGL while pybullet.DIRECT does not. Defaults to pybullet.GUI.
            dt (float, optional): The length of the simulation integration step.. Defaults to 0.001.
        """
        self.dt = dt
        self.objects = []
        self.robots = []

        self.physics_client = pybullet.connect(server)
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)

    def add_robot(self, robot):
        self.robots.append(robot)
        return robot

    def add_object_from_urdf(
        self, urdf_path, pos=[0, 0, 0], orn=[0, 0, 0, 1], useFixedBase=True
    ):
        """Adds an object described by a URDF file.

        Args:
            urdf_path (str): The absolute path of the URDF file
            pos (list, optional): The initial position of the object in the world frame. Defaults to [0, 0, 0].
            orn (list, optional): The initial orientation of the object in the world frame, expressed in quaternions. Defaults to [0, 0, 0, 1].
            useFixedBase (bool, optional): Determines if the robot base is fixed or not. Defaults to True.

        Returns:
            [int]: The PyBullet id of the object if added successfully.
        """
        # Load the object.
        object_id = pybullet.loadURDF(urdf_path, useFixedBase=useFixedBase)
        pybullet.resetBasePositionAndOrientation(object_id, pos, orn)
        self.objects.append(object_id)
        return object_id

    def start_video_recording(self, file_name):
        """Starts video recording and save as a mp4 file.

        Args:
            file_name (str): The absolute path of the file to be saved.
        """
        self.file_name = file_name
        pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

    def stop_video_recording(self):
        """Stops video recording if any."""
        if hasattr(self, "file_name"):
            pybullet.stopStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

    def step(self, sleep=False):
        """Integrates the simulation one step forward.

        Args:
            sleep (bool, optional): Determines if the simulation sleeps for :py:attr:`~dt` seconds at each step. Defaults to False.
        """
        if sleep:
            time.sleep(self.dt)
        pybullet.stepSimulation()

        for robot in self.robots:
            robot.compute_numerical_quantities(self.dt)

    def print_physics_engine_params(self):
        """Prints the parametes of the physics engine."""
        params = pybullet.getPhysicsEngineParameters(self.physicsClient)
        print("physics_engine_params:")
        for key in params:
            print("    - ", key, ": ", params[key])

