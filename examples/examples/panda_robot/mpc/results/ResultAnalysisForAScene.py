from os import listdir
from os.path import dirname, join, abspath, isfile
import json, codecs

import numpy as np
import pinocchio as pin
import hppfcl
import matplotlib.pyplot as plt

from Result import Result

def load_model():
    """Load the pinocchio model
    """
    pinocchio_model_dir = join(dirname(
    dirname(dirname(dirname(dirname(str(abspath(__file__))))))), "models"
    )
    model_path = join(pinocchio_model_dir, "franka_description/robots")
    mesh_dir = pinocchio_model_dir
    urdf_filename = "franka2.urdf"
    urdf_model_path = join(join(model_path, "panda"), urdf_filename)

    robot = pin.RobotWrapper.BuildFromURDF(
        urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
    )
    rmodel1, [vmodel1, cmodel1] = robot.model, [robot.visual_model, robot.collision_model]
    q0 = pin.neutral(rmodel1)

    rmodel, [vmodel, cmodel] = pin.buildReducedModel(
        rmodel1, [vmodel1, cmodel1], [1, 9, 10], q0
    )
            
    ### CREATING THE SPHERE ON THE UNIVERSE
    OBSTACLE_RADIUS = 1.0e-1
    # OBSTACLE_POSE = pin.SE3.Identity()
    # OBSTACLE_POSE.translation = np.array([0.25, -0.425, 1.5])
    OBSTACLE_POSE = pin.SE3(pin.utils.rotate("x", np.pi), np.array([0, -0.2, 1.5]))
    OBSTACLE = hppfcl.Sphere(OBSTACLE_RADIUS)
    OBSTACLE_GEOM_OBJECT = pin.GeometryObject(
        "obstacle",
        rmodel.getFrameId("universe"),
        rmodel.frames[rmodel.getFrameId("universe")].parentJoint,
        OBSTACLE,
        OBSTACLE_POSE,
    )
    ID_OBSTACLE = cmodel.addGeometryObject(OBSTACLE_GEOM_OBJECT)
    
    return rmodel, cmodel
    
class ResultAnalysisForAScene():
    def __init__(self, list_of_results) -> None:
        self._list_of_results = list_of_results
    
    def get_all_time_calc(self):
        
        self._time_calc_dict = {}
        for res in self._list_of_results:
            result_name = str(res.get_Nnodes()) + " nodes " + str(res.get_dt()) + " dt " + str(res.get_max_iter())+ " maxit " + str(res.get_max_qp_iters()) + " maxqpit"
            self._time_calc_dict[result_name] = res.get_time_calc()
            
    def plot_time_calc(self):
        
        self.get_all_time_calc()
        for key, val in self._time_calc_dict.items():
            plt.plot(val,"o-" ,label = key)
        plt.legend()
        plt.xlabel("Time steps (ms)")
        plt.ylabel("Time taken for solving the OCP (s)")
        plt.title("Time taken for solving the OCP through time")
        plt.show()
        
    def get_all_min_distances(self, rmodel, cmodel):
        self._min_distance_dict = {}
        self._dict_of_collisions_objects = {}
        for res in self._list_of_results:
            self._min_distance_dict[res.get_name()] = res.compute_minimal_distances_between_collision_pairs(rmodel, cmodel)
            for key in self._min_distance_dict[res.get_name()].keys():
                if key not in self._dict_of_collisions_objects.keys():
                    self._dict_of_collisions_objects[key] = res.get_name()
        
    def plot_min_distances(self, rmodel, cmodel):
        subplot_dict = {
            1: None, 2: [121,122], 3 :[221, 222,223], 4 :[221, 222,223, 224], 5: [321, 322, 323, 324, 325],
            6: [321, 322, 323, 324, 325, 326], 7 : [331, 332, 333, 334, 335, 336, 337], 8 : [331, 332, 333, 334, 335, 336, 337, 338],
            9 : [331, 332, 333, 334, 335, 336, 337, 338, 339]
        }
        self.get_all_min_distances(rmodel, cmodel)
        subplot_number = subplot_dict[len(self._dict_of_collisions_objects)]
        for i, key in enumerate(self._dict_of_collisions_objects.keys()):
            plt.subplot(subplot_number[i])
            for kkey, value in self._min_distance_dict.items():
                if key in value:
                    plt.plot(value[key], label = kkey)
            plt.legend()
            plt.xlabel("Time steps (ms)")
            plt.ylabel("Distance (m)")
            plt.title(key)
        plt.suptitle("Distances minimal to the obstacle through time steps")
        plt.show()
        
    def plot_config(self):
        
        subplot = [331, 332, 333, 334, 335, 336, 337]
        Q1, Q2, Q3, Q4, Q5, Q6, Q7 = {}, {}, {}, {}, {}, {}, {}
        for res in self._list_of_results:
            Q1[res.get_name()] = [q[0] for q in res.get_Q()]
            Q2[res.get_name()] = [q[1] for q in res.get_Q()]
            Q3[res.get_name()] = [q[2] for q in res.get_Q()]
            Q4[res.get_name()] = [q[3] for q in res.get_Q()]
            Q5[res.get_name()] = [q[4] for q in res.get_Q()]
            Q6[res.get_name()] = [q[5] for q in res.get_Q()]
            Q7[res.get_name()] = [q[6] for q in res.get_Q()]

        for i,Q in enumerate([Q1, Q2, Q3, Q4, Q5, Q6, Q7]):
            plt.subplot(subplot[i])
            for key, value in Q.items():
                plt.plot(value, label = key)
            plt.legend()
            plt.xlabel("Time steps (ms)")
            plt.ylabel("Configurations (rad)")
            plt.title(key)
        plt.suptitle("Configurations through time steps")
        plt.show()
        
    def plot_velocities(self):
        subplot = [331, 332, 333, 334, 335, 336, 337]
        V1, V2, V3, V4, V5, V6, V7 = {}, {}, {}, {}, {}, {}, {}
        for res in self._list_of_results:
            V1[res.get_name()] = [V[0] for V in res.get_V()]
            V2[res.get_name()] = [V[1] for V in res.get_V()]
            V3[res.get_name()] = [V[2] for V in res.get_V()]
            V4[res.get_name()] = [V[3] for V in res.get_V()]
            V5[res.get_name()] = [V[4] for V in res.get_V()]
            V6[res.get_name()] = [V[5] for V in res.get_V()]
            V7[res.get_name()] = [V[6] for V in res.get_V()]

        for i,V in enumerate([V1, V2, V3, V4, V5, V6, V7]):
            plt.subplot(subplot[i])
            for key, value in V.items():
                plt.plot(value, label = key)
            plt.legend()
            plt.xlabel("Time steps (ms)")
            plt.ylabel("Velocities (rad/s)")
            plt.title(key)
        plt.suptitle("Velocities through time steps")
        plt.show()
        
    def plot_controls(self):
        subplot = [331, 332, 333, 334, 335, 336, 337]
        U1, U2, U3, U4, U5, U6, U7 = {}, {}, {}, {}, {}, {}, {}
        for res in self._list_of_results:
            U1[res.get_name()] = [U[0] for U in res.get_U()]
            U2[res.get_name()] = [U[1] for U in res.get_U()]
            U3[res.get_name()] = [U[2] for U in res.get_U()]
            U4[res.get_name()] = [U[3] for U in res.get_U()]
            U5[res.get_name()] = [U[4] for U in res.get_U()]
            U6[res.get_name()] = [U[5] for U in res.get_U()]
            U7[res.get_name()] = [U[6] for U in res.get_U()]

        for i,U in enumerate([U1, U2, U3, U4, U5, U6, U7]):
            plt.subplot(subplot[i])
            for key, value in U.items():
                plt.plot(value, label = key)
            plt.legend()
            plt.xlabel("Time steps (ms)")
            plt.ylabel("Torque (Nm)")
            plt.title(key)
        plt.suptitle("Torque through time steps")
        plt.show()
        
    
if __name__=="__main__":

    COMPARING_NODES = True     
    SCENE1 = True
    rmodel, cmodel = load_model()
    curr_path = dirname(str(abspath(__file__)))

    list_analysis = []
    if SCENE1:
        path = curr_path + "/scene1"
        if COMPARING_NODES:
            path += "/comparingnodes"
    for filename in listdir(path):
        f = join(path, filename)
        # checking if it is a file
        if isfile(f):
            list_analysis.append(Result(f))

    ana = ResultAnalysisForAScene(list_analysis)    
    ana.plot_time_calc()
    ana.plot_min_distances(rmodel, cmodel)
    ana.plot_config()
    ana.plot_velocities()
    ana.plot_controls()
    
    