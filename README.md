# COLMPC: Collision Avoidance for MPC

This repo is mainly a addon residual for Crocoddyl for collision avoidance for trajectory optimisation and model predictive control (MPC).
It has several dependencies: 

# Dependencies 

## For OCP & MPC scripts: 

- HPPFCL : https://github.com/humanoid-path-planner/hpp-fcl/tree/hppfcl3x **(HPPFCL3X BRANCH REQUIERED)** for collision computations.
- Pinocchio: https://github.com/stack-of-tasks/pinocchio fast rigid body dynamics.
- Crocoddyl: https://github.com/loco-3d/crocoddyl framework for the solver.

## For visualization: 
- Pybullet: https://pybullet.org/wordpress/ 

## For the examples: 

- MiM Solvers: https://github.com/machines-in-motion/mim_solvers solver.
- Mim Robots: https://github.com/machines-in-motion/mim_robots pybullet env.

# Installations

HPP-FCL & Pinocchio must be built from sources. Don't forget to checkout the hppfcl3x branch. Build pinocchio with the flag : WITH_COLLISION_SUPPORT=ON. 
Mim Robot is built with pip.

# Usage

Before trying the scripts, test your hppfcl installation. To do this and make sure the hppfcl librairy works well in your computer, run : 
``` python tests/__init__.py```.

## For the MPC part:

Simply run ```python examples/mpc_panda_reaching.py```

As the code is still in developpement, the code is constantly moving and sometimes, examples do not work. Hence, do not hesitate to contact me at ahaffemaye@laas.fr. 

# Credits

The examples are based on https://github.com/machines-in-motion/minimal_examples_crocoddyl/tree/master from Sebastien Kleff. 

