import yaml
import numpy as np
import pinocchio as pin
import hppfcl
from wrapper_panda import PandaWrapper
from visualizer import create_viewer
from param_parsers import ParamParser

# Creating the robot
robot_wrapper = PandaWrapper(capsule=False)
rmodel, cmodel, vmodel = robot_wrapper()

pp = ParamParser("scenes.yaml", 3)

cmodel = pp.add_collisions(rmodel, cmodel)
print(cmodel)
vis = create_viewer(
    rmodel, cmodel, cmodel
)

vis.display(pp.get_initial_config())