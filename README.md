# COLMPC: Collision Avoidance for MPC

This repo is mainly a addon residual for Crocoddyl for collision avoidance for trajectory optimisation and model predictive control (MPC).
It has several dependencies: 

# Dependencies 

## For OCP & MPC scripts: 

- HPPFCL : https://github.com/humanoid-path-planner/hpp-fcl  (commit: 65fb435b44a1bbd5059347d7a311cc7c7aa1349e) for collision computations.
- Pinocchio: https://github.com/stack-of-tasks/pinocchio (v2.7.0) fast rigid body dynamics.
- Crocoddyl: https://github.com/loco-3d/crocoddyl (commit: 5b415a16138d)framework for the solver.

## For visualization: 
- Pybullet: https://pybullet.org/wordpress/ 

## For the examples: 

- MiM Solvers: https://github.com/machines-in-motion/mim_solvers solver for the SQP and Constrained-SQP solver.

# Installations

HPP-FCL & Pinocchio must be built from sources. Don't forget to checkout to the right commits. Build pinocchio with the flag : WITH_COLLISION_SUPPORT=ON. 

# Usage

Before trying the scripts, test your hppfcl installation. To do this and make sure the hppfcl librairy works well in your computer, run : 
``` python tests/__init__.py```.

If you have a problem with ``` FakeCollisionGeometry```, it is likely that the linking of Pinocchio with HPPFCL wasn't done properly. Verify that you have the right commits & the right compilation flags.
If the unit tests don't pass, it is likely that you don't have the right HPPFCL version.  

## For the MPC part:

Simply run ```python examples/mpc_panda_reaching.py```

As the code is still in developpement, the code is constantly moving and sometimes, examples do not work. Hence, do not hesitate to contact me at [ahaffemaye@laas.fr](mailto:ahaffemaye@laas.fr). 

# Credits

The examples are based on https://github.com/machines-in-motion/minimal_examples_crocoddyl/tree/master from Sebastien Kleff. 


# Citation

Please, if you use this library, please cite this paper: https://laas.hal.science/hal-04425002. 
