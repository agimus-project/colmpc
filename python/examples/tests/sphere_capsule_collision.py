import numpy as np
import pinocchio as pin
import hppfcl
from pinocchio.visualize import MeshcatVisualizer

### FROM THIS EXAMPLE, WE CAN SEE THAT THE CLOSEST POINTS IN PENETRATION AREN'T WELL COMPUTED FOR SOME HPPFCL VERSIONS.


RED = np.array([249, 136, 126, 125]) / 255
GREEN = np.array([170, 236, 149, 125]) / 255
BLACK_FULL = np.array([0, 0, 0, 1.0])
colors = [RED, GREEN]


def distance_capsule_sphere(capsule, sphere):
    # Storing the constants of the sphere & capsule
    l1 = capsule.geometry.halfLength
    r1 = capsule.geometry.radius
    r2 = sphere.geometry.radius

    A, B = get_A_B_from_center_capsule(capsule)

    # Position of the center of the sphere
    C = sphere.placement.translation

    AB = B - A
    AC = C - A

    # Project AC onto AB, but deferring divide by Dot(AB, AB)
    t = np.dot(AC, AB)
    if t <= 0.0:
        # C projects outside the [A, B] interval, on the A side; clamp to A
        t = 0.0
        closest_point = A
        print("A")
    else:
        denom = np.dot(AB, AB)  # Always nonnegative since denom = ||AB||^2
        if t >= denom:
            # C projects outside the [A, B] interval, on the B side; clamp to B
            t = 1.0
            closest_point = B
            print("B")
        else:
            # C projects inside the [A, B] interval; must do deferred divide now
            t = t / denom
            closest_point = A + t * AB
            print("C")

    # Calculate distance between C and the closest point on the segment
    distance = np.linalg.norm(C - closest_point) - (r1 + r2)

    return distance


def get_A_B_from_center_capsule(capsule):
    A = pin.SE3.Identity()
    A.translation = np.array([0, 0, -capsule.geometry.halfLength])
    B = pin.SE3.Identity()
    B.translation = np.array([0, 0, +capsule.geometry.halfLength])

    A = A * capsule.placement
    B *= capsule.placement
    return (A.translation, B.translation)


######################## TEST OF DISTANCE COMPUTATION #########################################

rmodel = pin.Model()

cmodel = pin.GeometryModel()
geometries = [
    hppfcl.Sphere(0.7),
    hppfcl.Capsule(0.5, 0.7),
]
placement1 = pin.SE3(pin.utils.rotate("y", np.pi), np.array([1, 0, 0]))
try:
    geom_obj1 = pin.GeometryObject("obj0", 0, 0, placement1, geometries[0])
except:
    geom_obj1 = pin.GeometryObject("obj0", 0, 0, geometries[0], placement1)

geom_obj1.meshColor = RED
cmodel.addGeometryObject(geom_obj1)

placement2 = pin.SE3(pin.utils.rotate("y", np.pi), np.array([0, 0, 0]))
try:
    geom_obj2 = pin.GeometryObject("obj1", 0, 0, placement2, geometries[1])
except:
    geom_obj2 = pin.GeometryObject("obj1", 0, 0, geometries[1], placement2)
geom_obj2.meshColor = GREEN

cmodel.addGeometryObject(geom_obj2)

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


result = distance_capsule_sphere(
    shape2,
    shape1,
)
print("Distance between capsule and sphere:", result)

######################################## VISUALIZER ##############################################

try:
    geom_obj1 = pin.GeometryObject("cp1", 0, 0, placement1, geom)
    geom_obj2 = pin.GeometryObject("cp2", 0, 0, placement2, geom)
except:
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

input()
