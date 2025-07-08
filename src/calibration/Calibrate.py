import numpy as np  
import open3d as o3d
import os
import copy
import json
import pyk4a

from pyk4a import Config, PyK4A
from k4a_handler import k4a_Grabber
from rs2_handler import rs2_Grabber, rs
# from utils import *
# from TuningTransformMatrix import TuningTransformMatrix

def coord_transform_to_matrix(coord_transform_dict):
    """
    Convert coord_transform dictionary to a 4x4 numpy matrix.
    
    Args:
        coord_transform_dict: Dictionary with keys "0" to "15" representing matrix elements
        
    Returns:
        numpy.ndarray: 4x4 transformation matrix
    """
    matrix = np.zeros((4, 4))
    for i in range(16):
        row = i // 4
        col = i % 4
        matrix[row, col] = coord_transform_dict[str(i)]
    return matrix

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def draw_registration_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])

def calib(source, target):
    picked_id_source = pick_points(source)
    # picked_id_target = pick_points(target)
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.01  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    return reg_p2p.transformation 


def k4a_save(transform_matrices, ROI_per_device):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(current_dir + r'\template.json'), 'r') as template_file:
            json_template = json.load(template_file)

        for serial, matrix in transform_matrices.items():
            print(f"Serial: {serial}")
            # device = PyK4A(
            # Config(
            #     color_resolution=pyk4a.ColorResolution.RES_1536P,
            #     color_format=pyk4a.ImageFormat.COLOR_BGRA32,
            #     camera_fps=pyk4a.FPS.FPS_15,
            #     depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            #     synchronized_images_only=True,
            # ),
            # device_id=serial
            # )
            # device.open()
            
            json_template['devices_config'].update({serial: {
                'disabled': False, 
                'ROI': {
                    "start_x": ROI_per_device[serial][0],
                    "start_y": ROI_per_device[serial][1],
                    "width": ROI_per_device[serial][2],
                    "height": ROI_per_device[serial][3]
                },
                'coord_transform': {}
                }})
            
            for i in range(0, 16):
                json_template['devices_config'][serial]['coord_transform'].update({f"{i}": matrix.flatten()[i]})

            # device.close()

        # Save the updated JSON to a new file
        print(current_dir + r'\cameraconfig.json')
        with open(os.path.join(current_dir + r'\cameraconfig.json'), 'w') as output_file:
            json.dump(json_template, output_file, indent=4)

        print(f"Camera information added to the JSON template and saved to cameraconfig.json.")

def rs2_save(transform_matrices, ROI_per_device):
    current_dir = os.path.dirname(os.path.realpath(__file__))
    with open(os.path.join(current_dir + r'\template.json'), 'r') as template_file:
        json_template = json.load(template_file)

    print(f"Size of transform_matrices: {len(transform_matrices)}")

    for serial, matrix in transform_matrices.items():
        # ctx = rs.context()
        # devices = ctx.query_devices()
        # if device_id >= len(devices):
        #     print("Index out of range")
        #     exit()
        # serial_id = devices[serial].get_info(rs.camera_info.serial_number)
        # print(f"Serial ID: {serial_id}")
        # json_template['devices_config'].update({serial_id: {
        #     'disabled': False, 
        #     'ROI': {
        #         "start_x": ROI_per_device[serial][0],
        #         "start_y": ROI_per_device[serial][1],
        #         "width": ROI_per_device[serial][2],
        #         "height": ROI_per_device[serial][3]
        #     },
        #     'coord_transform': {}
        #     }})
        # for i in range(0, 16):
        #     json_template['devices_config'][serial_id]['coord_transform'].update({f"{i}": matrix.flatten()[i]})

        print(f"Serial ID: {serial}")
        json_template['devices_config'].update({serial: {
            'disabled': False, 
            'ROI': {
                "start_x": ROI_per_device[serial][0],
                "start_y": ROI_per_device[serial][1],
                "width": ROI_per_device[serial][2],
                "height": ROI_per_device[serial][3]
            },
            'coord_transform': {}
            }})
        
        for i in range(0, 16):
            json_template['devices_config'][serial]['coord_transform'].update({f"{i}": matrix.flatten()[i]})

    # Save the updated JSON to a new file
    with open(os.path.join(current_dir + r'\cameraconfig.json'), 'w') as output_file:
        json.dump(json_template, output_file, indent=4)

    print(f"Camera information added to the JSON template and saved to cameraconfig.json.")

def load_ply_from_folder(folder_path):
    ply_files = [file for file in os.listdir(folder_path) if file.endswith('.ply')]
    pointclouds = {}
    
    for file in ply_files:
        file_path = os.path.join(folder_path, file)
        device_id = os.path.splitext(file)[0]
        pointcloud = o3d.io.read_point_cloud(file_path)
        pointclouds[device_id] =  pointcloud
    
    return pointclouds

def tuning_tranform_matrix(source_l, target_l, transform_matrix):
    source = o3d.t.geometry.PointCloud.from_legacy(source_l)
    target = o3d.t.geometry.PointCloud.from_legacy(target_l)

    # Initial alignment or source to target transform.
    init_source_to_target = o3d.core.Tensor(transform_matrix)

    ################################################################################################
    print("Pre-compute transform fucntion using P2P Estimation")
    estimation = o3d.t.pipelines.registration.TransformationEstimationPointToPoint()

    # Search distance for Nearest Neighbour Search [Hybrid-Search is used].
    max_correspondence_distance = 0.01

    # Convergence-Criteria for Vanilla ICP
    criteria = o3d.t.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000001,
                                        relative_rmse=0.0000001,
                                        max_iteration=30)

    # Down-sampling voxel-size. If voxel_size < 0, original scale is used.
    voxel_size = -1

    reg_point_to_point = o3d.t.pipelines.registration.icp(source, target, max_correspondence_distance,
                              init_source_to_target, estimation, criteria,
                              voxel_size, None)
    
    ###################################################################################################
    print("Tuning the transform fucntion using Multi-ICP")
    init_source_to_target = (reg_point_to_point.transformation)

    criteria_list = [
    o3d.t.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0001,
                                relative_rmse=0.0001,
                                max_iteration=50),
    o3d.t.pipelines.registration.ICPConvergenceCriteria(0.00001, 0.00001, 30),
    o3d.t.pipelines.registration.ICPConvergenceCriteria(0.000001, 0.000001, 15)
    ]
    # `max_correspondence_distances` for Multi-Scale ICP (o3d.utility.DoubleVector):
    max_correspondence_distances = o3d.utility.DoubleVector([0.03, 0.014, 0.007]) 
    voxel_sizes = o3d.utility.DoubleVector([0.01, 0.005, 0.0025])

    # Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
    callback_after_iteration = lambda loss_log_map : print("Iteration Index: {}, Scale Index: {}, Scale Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
        loss_log_map["iteration_index"].item(),
        loss_log_map["scale_index"].item(),
        loss_log_map["scale_iteration_index"].item(),
        loss_log_map["fitness"].item(),
        loss_log_map["inlier_rmse"].item()))

    registration_ms_icp = o3d.t.pipelines.registration.multi_scale_icp(source, target, voxel_sizes,
                                           criteria_list,
                                           max_correspondence_distances,
                                           init_source_to_target, estimation,
                                           callback_after_iteration)
    
    testpc = o3d.geometry.PointCloud()
    temp = source_l
    testpc += temp.transform(registration_ms_icp.transformation.numpy())
    testpc += target_l
    o3d.visualization.draw_geometries([testpc])
    return registration_ms_icp.transformation.numpy()

def test():
    current_dir = os.path.dirname(os.path.realpath(__file__))

    k4a_grabber = k4a_Grabber()
    k4a_device_ids = k4a_grabber.get_grabber_list()

    rs_grabber = rs2_Grabber()
    rs_device_ids = rs_grabber.get_grabber_list()
    if not rs_device_ids and not k4a_device_ids:
        print("No devices available")
        exit()

    # Read the cameraconfig.json file
    with open(os.path.join(current_dir + r'\cameraconfig.json'), 'r') as config_file:
        config = json.load(config_file)

    merged_pc = o3d.geometry.PointCloud()
    for device_id in k4a_device_ids:
        print(f"Calibrating Kinect device {device_id}")
        # grabber.setup_cam(device_id)
        pc, serial = k4a_grabber.capture_point_cloud(device_id)

        # Transform the point cloud using coord_transform from cameraconfig.json
        coord_transform_dict = config['devices_config'][serial]['coord_transform']
        transform_matrix = coord_transform_to_matrix(coord_transform_dict)
        pc = pc.transform(transform_matrix)

        merged_pc += pc

    # Show the merged point cloud
    o3d.visualization.draw_geometries([merged_pc])

if __name__ == "__main__":
    # Get current running directory
    current_dir = os.path.dirname(os.path.realpath(__file__))

    pointclouds = {}
    ROI_per_device = {}
    transform_matrices = {}

    k4a_grabber = k4a_Grabber()
    k4a_device_ids = k4a_grabber.get_grabber_list()

    rs_grabber = rs2_Grabber()
    rs_device_ids = rs_grabber.get_grabber_list()
    if not rs_device_ids and not k4a_device_ids:
        print("No devices available")
        exit()

    for device_id in k4a_device_ids:
        print(f"Calibrating Kinect device {device_id}")
        # grabber.setup_cam(device_id)
        pc, _ = k4a_grabber.capture_point_cloud(device_id)
        serial = k4a_grabber.get_camera_serial()
        pointclouds[serial] = pc
        ROI_per_device[serial] = k4a_grabber.get_ROI()
    
    for device_id in rs_device_ids:
        print(f"Calibrating Realsense device {device_id}")
        # grabber.setup_cam(device_id)
        pc = rs_grabber.capture_point_cloud(device_id)
        serial = rs_grabber.get_camera_serial()
        pointclouds[serial] = pc
        ROI_per_device[serial] = rs_grabber.get_ROI()
    
    print(f"Size of pointclouds: {len(pointclouds)}")

    transform_matrices = {}
    origin = o3d.io.read_point_cloud(os.path.join(current_dir, "origin.ply"))
    picked_id_target = pick_points(origin)

    first_key, first_pc = next(iter(pointclouds.items()))
    maxtrix = calib(first_pc, origin)
    transform_matrices[first_key] = maxtrix
    first_pc = first_pc.transform(maxtrix)
    picked_id_target = pick_points(first_pc)

    # Print size of pointclouds

    if first_key in pointclouds:
        del pointclouds[first_key]

    print(f"Size of pointclouds: {len(pointclouds)}")

    for device_id, pointcloud in pointclouds.items():
        maxtrix = calib(pointcloud, first_pc)
        matrix_temp = maxtrix
        tuned_matrix = tuning_tranform_matrix(pointcloud, first_pc, maxtrix)
        
        while True:
            user_input = input("Are you happy with the tuning result? (y: Get tune result || n: Get result without tuning  || p: Tune again): ")
            if user_input.lower() == 'y':
                print(f"Get tuned matrix for {device_id}")
                confirm = True
                transform_matrices[device_id] = tuned_matrix
                break
            elif user_input.lower() == 'n':
                print(f"Get matrix without tuning for {device_id}")
                confirm = True
                transform_matrices[device_id] = matrix_temp
                break
            elif user_input.lower() == 'p':
                print(f"Get matrix without tuning for {device_id}")
                maxtrix = calib(pointcloud, first_pc)
                tuned_matrix = tuning_tranform_matrix(pointcloud, first_pc, matrix_temp)
            else:
                print("Invalid input. Please enter 'y' to confirm or 'n' to retry.")

    print("Merging transformed points clouds")

    for device_id, pointcloud in pointclouds.items():
        first_pc += pointcloud#.transform(transform_matrices[device_id])
        print(transform_matrices[device_id])

    # o3d.visualization.draw_geometries([merged_pc])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(first_pc)
    vis.run()  # user picks points
    vis.destroy_window()

    k4a_save(transform_matrices, ROI_per_device)
    # rs2_save(transform_matrices, ROI_per_device)

    test()
