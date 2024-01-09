import numpy as np
import pinocchio as pin
import hppfcl
from pinocchio.visualize import MeshcatVisualizer

### FROM THIS EXAMPLE, WE CAN SEE THAT THE CLOSEST POINTS IN PENETRATION AREN'T WELL COMPUTED FOR SOME HPPFCL VERSIONS.


RED = np.array([249, 136, 126, 125]) / 255
GREEN = np.array([170, 236, 149, 125]) / 255
BLACK_FULL = np.array([0, 0, 0, 1.0])

colors = [RED, GREEN]

rmodel = pin.Model()

cmodel = pin.GeometryModel()
geometries = [
    hppfcl.Sphere(0.7),
    hppfcl.Sphere(0.5),
]
for i, geom in enumerate(geometries):
    placement = pin.SE3(np.eye(3), np.array([i, 0, 0]))
    geom_obj = pin.GeometryObject("obj{}".format(i), 0, 0, geom, placement)
    geom_obj.meshColor = colors[i]
    cmodel.addGeometryObject(geom_obj)


rdata = rmodel.createData()
cdata = cmodel.createData()

pin.updateGeometryPlacements(rmodel, rdata, cmodel, cdata)
# Creating the shapes for the collision detection.
shape1_id = cmodel.getGeometryId("obj0")

# Coloring the sphere
shape1 = cmodel.geometryObjects[shape1_id]

# Getting its pose in the world reference
shape1_placement = cdata.oMg[shape1_id]

# Doing the same for the second shape.
shape2_id = cmodel.getGeometryId("obj1")

# Coloring the sphere
shape2 = cmodel.geometryObjects[shape2_id]

# Getting its pose in the world reference
shape2_placement = cdata.oMg[shape2_id]

req = hppfcl.DistanceRequest()
res = hppfcl.DistanceResult()

distance = hppfcl.distance(
    shape1.geometry,
    hppfcl.Transform3f(shape1_placement.rotation, shape1_placement.translation),
    shape2.geometry,
    hppfcl.Transform3f(shape2_placement.rotation, shape2_placement.translation),
    req,
    res,
)


print(distance)
print(f"res.getNearestPoint1() : {res.getNearestPoint1()}")
print(f"res.getNearestPoint2() : {res.getNearestPoint2()}")

placement1 = pin.SE3(np.eye(3), res.getNearestPoint1())
placement2 = pin.SE3(np.eye(3), res.getNearestPoint2())
geom = hppfcl.Sphere(0.05)
geom_obj1 = pin.GeometryObject("cp1", 0, 0, geom, placement1)
geom_obj2 = pin.GeometryObject("cp2", 0, 0, geom, placement2)
geom_obj1.meshColor = BLACK_FULL
geom_obj2.meshColor = BLACK_FULL
cmodel.addGeometryObject(geom_obj1)
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
