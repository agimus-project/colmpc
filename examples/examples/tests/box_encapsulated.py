import hppfcl
import pinocchio as pin
import numpy as np
from pinocchio.visualize import MeshcatVisualizer

### FROM THIS EXAMPLE, WE CAN SEE THAT THE CLOSEST POINTS IN PENETRATION AREN'T WELL COMPUTED FOR SOME HPPFCL VERSIONS.


RED = np.array([249, 136, 126, 125]) / 255
GREEN = np.array([170, 236, 149, 125]) / 255
BLACK_FULL = np.array([0, 0, 0, 1.0])
colors = [RED, GREEN]

l1 = 0.5
l2 = 0.8
l3 = 0.3

rmodel = pin.Model()

cmodel = pin.GeometryModel()
geometries = [
    hppfcl.Box(l1,l2,l3),
    hppfcl.Capsule(l3, l2/2),
]
placement1 = pin.SE3(pin.utils.rotate("z", np.pi/2), np.array([0, 0, 0]))
try:
    geom_obj1 = pin.GeometryObject("obj0", 0, 0, placement1, geometries[0])
except:
    geom_obj1 = pin.GeometryObject("obj0", 0, 0, geometries[0], placement1)

geom_obj1.meshColor = RED
cmodel.addGeometryObject(geom_obj1)

placement2 = pin.SE3(pin.utils.rotate("y", np.pi/2), np.array([0, 0, 0]))
try:
    geom_obj2 = pin.GeometryObject("obj1", 0, 0, placement2, geometries[1])
except:
    geom_obj2 = pin.GeometryObject("obj1", 0, 0, geometries[1], placement2)
geom_obj2.meshColor = GREEN

cmodel.addGeometryObject(geom_obj2)

viz = MeshcatVisualizer(
    model=rmodel,
    collision_model=cmodel,
    visual_model=cmodel,
)

# Initialize the viewer.
viz.initViewer(open=False)
viz.loadViewerModel("shapes")

viz.display(np.zeros(0))
input()
