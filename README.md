# COLMPC: Collision Avoidance for MPC

This repo is mainly an addon residual for Crocoddyl for collision avoidance for trajectory optimisation and model predictive control (MPC).

Here's a video of the addon residual in a real time torque controlled 7-DoF manipulator robot:

<div align="center">
    <video src="https://www.youtube.com/watch?v=81bagcv4PUc" width=400/>
</div>

## Dependencies

### For OCP & MPC scripts:

- [HPPFCL](https://github.com/humanoid-path-planner/hpp-fcl)  (commit: 65fb435b44a1bbd5059347d7a311cc7c7aa1349e) for collision computations.
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (v2.7.0) fast rigid body dynamics.
- [Crocoddyl](https://github.com/loco-3d/crocoddyl) (commit: 5b415a16138d)framework for the solver.

### For visualization:
- [Pybullet](https://pybullet.org/wordpress/).
- [Meshcat](https://github.com/meshcat-dev/meshcat-python).

### For the examples:

- [MiM Solvers](https://github.com/machines-in-motion/mim_solvers) solver for the SQP and Constrained-SQP solver, and [Mim Robot](https://github.com/machines-in-motion/mim_robots/tree/main).

## Installation
HPP-FCL & Pinocchio must be built from sources. Build pinocchio with the flag : WITH_COLLISION_SUPPORT=ON.
> [!NOTE]
> Don't forget to switch to the right commits!

## Usage
Before trying the examples, test your hppfcl installation. To do this and make sure the hppfcl librairy works well on your computer, run in the test folder :
``` python -m unittest```.

### Possible issue
If you have a problem with ``` FakeCollisionGeometry```, it is likely that the linking of Pinocchio with HPPFCL wasn't done properly. Verify that you have the right commits & the right compilation flags.

### For the MPC part:

Create a meshcat-server by writting in a terminal ```meshcat-server```. Then, to see the example with hard constraints simply run in the main directory ```python examples/mpc_panda_reaching.py```. To compare it with the standard MPC without hard constraint but collision avoidance written as soft constraint, run  ```python examples/mpc_panda_reaching_soft_constraint.py```.


As the code is still in developpement, the code is constantly moving and sometimes, examples do not work. Hence, do not hesitate to contact me at [ahaffemaye@laas.fr](mailto:ahaffemaye@laas.fr).

# Citation
To cite **COLMPC** in your academic research, please use the following bibtex entry:
```bibtex
@inproceedings{haffemayer_model_2024,
	title = {Model predictive control under hard collision avoidance constraints for a robotic arm},
	author = {Haffemayer, Arthur and Jordana, Armand and Fourmy, Médéric and Wojciechowski, Krzysztof and Saurel, Guilhem and Petrík, Vladimír and Lamiraux, Florent and Mansard, Nicolas},
    booktitle={Ubiquitous Robots (UR)}
	year = {2024},
}
```

# Credits
The examples are based on [the examples](https://github.com/machines-in-motion/minimal_examples_crocoddyl/tree/master) of Sebastien Kleff.
