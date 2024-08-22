import time
import zmq
import sys
import numpy as np
import open3d as o3d

context = zmq.Context()
colorSocket = context.socket(zmq.PULL)
colorSocket.bind("tcp://*:5555")

positionSocket = context.socket(zmq.PULL)
positionSocket.bind("tcp://*:5556")
pcd = o3d.geometry.PointCloud()
while True:
    color_message = colorSocket.recv()
    position_message = positionSocket.recv()

    number_of_points = len(position_message) // (4 * 3)

    positions = np.frombuffer(position_message, dtype=np.float32).reshape(-1, 3)
    colors = np.frombuffer(color_message, dtype=np.float32).reshape(-1, 3)


    pcd.points = o3d.utility.Vector3dVector(positions)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    break

# Get bounidng box
print(pcd.get_min_bound())
print(pcd.get_max_bound())

# draw point of min bound and max bound
min_bound = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
min_bound.compute_vertex_normals()
min_bound.paint_uniform_color([1, 1, 0])
min_bound.translate(pcd.get_min_bound())
max_bound = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
max_bound.compute_vertex_normals()
max_bound.paint_uniform_color([0, 1, 1])
max_bound.translate(pcd.get_max_bound())

#draw xyz axis which y is up
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
mesh_frame.rotate([[1, 0, 0], [0, 0, 1], [0, 1, 0]])
mesh_frame.translate([0, 0, 0])
mesh_frame.paint_uniform_color([0, 0, 1])

# draw fix bounding box
a = (-325, 0, 77)
b = (275, 692, 677)

min_bound_fix_box = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
min_bound_fix_box.compute_vertex_normals()
min_bound_fix_box.paint_uniform_color([1, 0, 1])
min_bound_fix_box.translate([a[0], a[1], a[2]])

max_bound_fix_box = o3d.geometry.TriangleMesh.create_sphere(radius=5.0)
max_bound_fix_box.compute_vertex_normals()
max_bound_fix_box.paint_uniform_color([1, 0, 1])
max_bound_fix_box.translate([b[0], b[1], b[2]])

fixbox = o3d.geometry.AxisAlignedBoundingBox((a), (b))
fixbox.color = (0, 1, 0)

# draw bounding box
bbox = o3d.geometry.AxisAlignedBoundingBox(pcd.get_min_bound(), pcd.get_max_bound())
# set color to red
bbox.color = (1, 0, 0)


o3d.visualization.draw_geometries([pcd, 
                                    bbox, min_bound, max_bound, 
                                    fixbox, min_bound_fix_box, max_bound_fix_box,
                                    mesh_frame])
