import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import os
import copy
import json
import time
import cv2
import argparse

# ==========================================
# RealSense Hardware Handler
# ==========================================
class rs2_Grabber:
    def __init__(self):
        self.ctx = rs.context()
        self.device_count = self.ctx.query_devices().size()
        print(f"Number of connected devices: {self.device_count}")
        if not self.device_count:
            print("No devices available (skip this if running with --load).")

        self.device_ids = list(range(self.device_count))
        self.align = rs.align(rs.stream.color)
        
    def setup_cam(self, device_id=0):
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
        
        time.sleep(1.5)
        
        devices = self.ctx.query_devices()
        if device_id >= len(devices):
            print(f"Index out of range {device_id} / {len(devices)}")
            exit()
            
        serial = devices[device_id].get_info(rs.camera_info.serial_number)
        config.enable_device(serial)
        print(f"Setup Camera {device_id} / {len(devices)}: {serial}")
        return config, serial

    def capture_data(self, device_id=0, num_frames=30):
        """Captures color, depth, generates Point Cloud and gets intrinsics."""
        config, serial = self.setup_cam(device_id)
        pipeline = rs.pipeline(self.ctx)
        profile = pipeline.start(config)

        # Get Intrinsics
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        K = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.array(intr.coeffs, dtype=np.float32)

        # Allow auto-exposure to settle
        time.sleep(0.5)
        
        color_frame, depth_frame = None, None
        for _ in range(num_frames):
            frames = pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            d_frame = aligned_frames.get_depth_frame()
            c_frame = aligned_frames.get_color_frame()
            
            if not d_frame or not c_frame:
                continue
                
            color_frame = c_frame
            depth_frame = d_frame

        color_image = np.copy(np.asanyarray(color_frame.get_data()))
        
        # Calculate point cloud
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        
        # Fast vectorization mapping texture to vertices
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        tex_coords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
        
        # Filter valid points
        valid = verts[:, 2] > 0
        verts = verts[valid]
        tex_coords = tex_coords[valid]
        
        h, w = color_image.shape[:2]
        u = np.clip((tex_coords[:, 0] * w).astype(int), 0, w - 1)
        v = np.clip((tex_coords[:, 1] * h).astype(int), 0, h - 1)
        
        # Extract colors and convert BGR -> RGB for Open3D
        colors = color_image[v, u][:, ::-1] / 255.0

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(verts)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        pipeline.stop()
        return serial, color_image, pcd, K, dist_coeffs

    def get_grabber_list(self):
        return self.device_ids

# ==========================================
# File I/O for Camera Data
# ==========================================
def save_camera_data(data_dir, serial, img, pcd, K, dist_coeffs):
    os.makedirs(data_dir, exist_ok=True)
    cv2.imwrite(os.path.join(data_dir, f"color_{serial}.png"), img)
    o3d.io.write_point_cloud(os.path.join(data_dir, f"pc_{serial}.ply"), pcd)
    
    intr_data = {"K": K.tolist(), "dist_coeffs": dist_coeffs.tolist()}
    with open(os.path.join(data_dir, f"intrinsics_{serial}.json"), 'w') as f:
        json.dump(intr_data, f, indent=4)
    print(f"  Saved data for {serial} -> {data_dir}")

def load_camera_data(data_dir):
    images, point_clouds, intrinsics, dist_coeffs_dict = {}, {}, {}, {}
    if not os.path.exists(data_dir):
        print(f"[Error] Directory {data_dir} does not exist.")
        return images, point_clouds, intrinsics, dist_coeffs_dict

    for f in os.listdir(data_dir):
        if f.startswith("color_") and f.endswith(".png"):
            serial = f.replace("color_", "").replace(".png", "")
            img_path = os.path.join(data_dir, f)
            pcd_path = os.path.join(data_dir, f"pc_{serial}.ply")
            intr_path = os.path.join(data_dir, f"intrinsics_{serial}.json")
            
            if os.path.exists(pcd_path) and os.path.exists(intr_path):
                images[serial] = cv2.imread(img_path)
                point_clouds[serial] = o3d.io.read_point_cloud(pcd_path)
                
                with open(intr_path, 'r') as jf:
                    intr_data = json.load(jf)
                    intrinsics[serial] = np.array(intr_data["K"], dtype=np.float32)
                    dist_coeffs_dict[serial] = np.array(intr_data["dist_coeffs"], dtype=np.float32)
                    
                print(f"  Loaded data for {serial}")
    return images, point_clouds, intrinsics, dist_coeffs_dict

# ==========================================
# Calibration Core
# ==========================================
def coord_transform_to_matrix(coord_transform_dict):
    matrix = np.zeros((4, 4))
    for i in range(16):
        matrix[i // 4, i % 4] = coord_transform_dict[str(i)]
    return matrix

def calibrate_camera_charuco(images, charuco_board, apply_clahe=True, debug=False):
    """ Intrinsic calibration from a set of images """
    print("--- Running Intrinsic Camera Calibration ---")
    all_obj_points, all_img_points = [], []
    img_size = None

    try:
        params = cv2.aruco.DetectorParameters()
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    for img in images:
        if img_size is None:
            img_size = (img.shape[1], img.shape[0])
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if apply_clahe:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
            
        if hasattr(cv2.aruco, 'CharucoDetector'):
            detector = cv2.aruco.CharucoDetector(charuco_board, cv2.aruco.CharucoParameters(), params)
            charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
        else:
            dictionary = charuco_board.getDictionary()
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
            if marker_ids is not None and len(marker_ids) > 0:
                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, gray, charuco_board)
            else:
                charuco_ids = None

        if charuco_ids is not None and len(charuco_ids) >= 4:
            if hasattr(charuco_board, 'matchImagePoints'):
                obj_points, img_points = charuco_board.matchImagePoints(charuco_corners, charuco_ids)
            else:
                obj_points = charuco_board.chessboardCorners[charuco_ids]
                img_points = charuco_corners
                
            all_obj_points.append(obj_points)
            all_img_points.append(img_points)

    if not all_obj_points:
        print("[Error] Not enough corners found to intrinsically calibrate.")
        return None, None

    print(f"Calibrating with {len(all_img_points)} valid image(s)...")
    ret, K, dist_coeffs, _, _ = cv2.calibrateCamera(all_obj_points, all_img_points, img_size, None, None)
    print(f"Calibration complete. RMS Error: {ret:.4f}")
    return K, dist_coeffs

def auto_calib_charuco(img, K, dist_coeffs, charuco_board, apply_clahe=False, debug=False):
    """ Estimates camera extrinsic pose given the ChArUco board """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if apply_clahe:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

    try:
        params = cv2.aruco.DetectorParameters()
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    charuco_corners, charuco_ids = None, None
    
    #cv2.imshow('Grayscale', gray)
    #cv2.waitKey(0)

    if hasattr(cv2.aruco, 'CharucoDetector'):
        detector = cv2.aruco.CharucoDetector(charuco_board, cv2.aruco.CharucoParameters(), params)
        charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
    else:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, charuco_board.getDictionary(), parameters=params)
        if marker_ids is not None and len(marker_ids) > 0:
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, charuco_board)

    if debug:
        debug_img = img.copy()
        if charuco_corners is not None and len(charuco_corners) > 0:
            cv2.aruco.drawDetectedCornersCharuco(debug_img, charuco_corners, charuco_ids, (255, 0, 0))
        cv2.imshow("ChArUco Detection", cv2.resize(debug_img, (0,0), fx=1.5, fy=1.5))
        cv2.waitKey(0)

    if charuco_ids is None or len(charuco_ids) < 4:
        print("   [Warning] Board not detected. Reverting to identity.")
        return np.eye(4)

    if hasattr(charuco_board, 'matchImagePoints'):
        obj_points, img_points = charuco_board.matchImagePoints(charuco_corners, charuco_ids)
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, dist_coeffs)
    else:
        success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, charuco_board, K, dist_coeffs, None, None)
        
    if success:
        R, _ = cv2.Rodrigues(rvec)
        # R, t maps Board -> Camera. We want Camera -> Board (Global)
        transform = np.eye(4)
        transform[:3, :3] = R.T
        transform[:3, 3] = (-R.T @ tvec).flatten()

        # -------------------------------------------------------------------
        # Custom Origin Alignment: Loads the transformation matrix extracted
        # from your origin.ply to perfectly match your historical coordinate space.
        # -------------------------------------------------------------------
        origin_transform_file = "origin_transform.json"
        if os.path.exists(origin_transform_file):
            with open(origin_transform_file, "r") as f:
                data = json.load(f)
                T_board_to_origin = np.array(data["T_board_to_origin"], dtype=np.float64)
            transform = T_board_to_origin @ transform
        else:
            # Fallback to standard Right-Handed flip if the extraction script hasn't been run
            axis_flip = np.array([
                [ 1,  0,  0,  0],
                [ 0, -1,  0,  0],
                [ 0,  0, -1,  0],
                [ 0,  0,  0,  1]
            ], dtype=np.float64)
            
            transform = axis_flip @ transform

        return transform

    print("   [Warning] solvePnP failed. Reverting to identity.")
    return np.eye(4)

def rs2_save(transform_matrices, ROI_per_device=None, config_path='cameraconfig.json'):
    json_template = {'devices_config': {}}
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                data = json.load(f)
                if isinstance(data, dict):
                    json_template = data
                    if 'devices_config' not in json_template:
                        json_template['devices_config'] = {}
        except json.JSONDecodeError:
            pass

    for serial, matrix in transform_matrices.items():
        if serial not in json_template['devices_config']:
            json_template['devices_config'][serial] = {'disabled': False, 'coord_transform': {}}

        if ROI_per_device is not None:
            roi = ROI_per_device[serial]
            json_template['devices_config'][serial]['ROI'] = {
                "start_x": roi[0],
                "start_y": roi[1],
                "width": roi[2],
                "height": roi[3]
            }

        json_template['devices_config'][serial]['coord_transform'] = {
            str(i): float(matrix.flatten()[i]) for i in range(16)
        }

    with open(config_path, 'w') as f:
        json.dump(json_template, f, indent=4)
    print(f"\nCamera transformations saved to {config_path}")


# ==========================================
# GUI / Fine-Tuning with Point Clouds
# ==========================================
class ManualTuningWindow:
    def __init__(self, transform_matrices, point_clouds, marker_length=0.10, rs_grabber=None):
        self.rs_grabber = rs_grabber
        self.transform_matrices = copy.deepcopy(transform_matrices)
        self.deltas = {cid: [0.0]*6 for cid in transform_matrices.keys()} 
        self.full_point_clouds = point_clouds
        
        # Downsample point clouds specifically for the UI to remain highly responsive
        self.point_clouds_down = {}
        for cid, pc in point_clouds.items():
            self.point_clouds_down[cid] = pc #.voxel_down_sample(voxel_size=0.01)
            
        self.active_camera = None
        self.sliders = {cid: [] for cid in transform_matrices.keys()}
        self.base_point_size = 2.0
        self.camera_visible = {cid: True for cid in transform_matrices.keys()}
        
        self.colorize_clouds = False
        palette = [[1, 0, 0], [0, 1, 0], [0, 0.5, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 0.5, 0], [0.5, 0, 1]]
        self.cid_to_color = {cid: palette[i % len(palette)] for i, cid in enumerate(transform_matrices.keys())}
        
        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window("Manual Fine Tuning (Point Cloud)", 1280, 800)
        
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        
        # Visualize the global coordinate frame (Board origin)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=marker_length * 2)
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        self.scene.scene.add_geometry("global_axes", axes, mat)
        
        em = self.window.theme.font_size
        self.panel = gui.ScrollableVert(0, gui.Margins(em, em, em, em))
        
        global_ctrls = gui.Horiz(0.5 * em)
        
        self.cb_color = gui.Checkbox("Distinct Colors")
        self.cb_color.set_on_checked(self._on_color_toggled)
        global_ctrls.add_child(self.cb_color)
        
        self.btn_reset = gui.Button("Reset All")
        self.btn_reset.set_on_clicked(self._on_reset)
        global_ctrls.add_child(self.btn_reset)
        
        self.panel.add_child(global_ctrls)
        
        # Point Size Slider
        ps_layout = gui.Horiz(0.5 * em)
        ps_layout.add_child(gui.Label("Point Size:"))
        self.ps_slider = gui.Slider(gui.Slider.DOUBLE)
        self.ps_slider.set_limits(1.0, 10.0)
        self.ps_slider.double_value = self.base_point_size
        self.ps_slider.set_on_value_changed(self._on_point_size_changed)
        ps_layout.add_child(self.ps_slider)
        self.panel.add_child(ps_layout)
        
        self.panel.add_child(gui.Label(""))
        self.panel.add_child(gui.Label("Press 'C' to capture new point clouds."))
        self.panel.add_child(gui.Label("Press 'M' to save merged point cloud."))
        self.panel.add_child(gui.Label("Press 'X' to crop point clouds."))
        self.panel.add_child(gui.Label("Press 'O' to pick a new navigation origin."))
        self.panel.add_child(gui.Label("Press 'WASD' to move selected camera."))
        self.panel.add_child(gui.Label("Press 'Q/E' to move selected camera Up/Down."))
        self.panel.add_child(gui.Label(""))
        
        for cid in transform_matrices.keys():
            collapsable = gui.CollapsableVert(f"Camera {cid}", 0.25 * em, gui.Margins(em, 0, 0, 0))
            
            cb_visible = gui.Checkbox("Show Pointcloud")
            cb_visible.checked = True
            cb_visible.set_on_checked(self._make_on_visible_toggled(cid))
            collapsable.add_child(cb_visible)
            
            for i, axis in enumerate(['X', 'Y', 'Z']):
                horiz = self._create_slider_row(cid, f"Trans {axis} (m)", i, -4.0, 4.0, 0.005)
                collapsable.add_child(horiz)
                
            for i, axis in enumerate(['rX', 'rY', 'rZ']):
                horiz = self._create_slider_row(cid, f"Rot {axis} (deg)", i + 3, -180.0, 180.0, 0.1)
                collapsable.add_child(horiz)
                
            self.panel.add_child(collapsable)
            self._update_geometry(cid)
            
        btn = gui.Button("Save & Finish")
        btn.set_on_clicked(self._on_finish)
        self.panel.add_child(gui.Label("")) 
        self.panel.add_child(btn)

        self.window.set_on_layout(self._on_layout)
        self.window.set_on_key(self._on_key)
        self.window.add_child(self.scene)
        self.window.add_child(self.panel)
        
        bounds = self.scene.scene.bounding_box
        self.scene.setup_camera(60, bounds, (0.0, 0.0, 0.0))

    def _on_color_toggled(self, is_checked):
        self.colorize_clouds = is_checked
        for cid in self.transform_matrices.keys():
            self._update_geometry(cid)

    def _on_point_size_changed(self, value):
        self.base_point_size = value
        for cid in self.transform_matrices.keys():
            self._update_geometry(cid)

    def _on_reset(self):
        self.active_camera = None
        for cid in self.transform_matrices.keys():
            self.deltas[cid] = [0.0] * 6
            for slider in self.sliders[cid]:
                slider.double_value = 0.0
            self._update_geometry(cid)

    def _create_slider_row(self, cid, label_text, index, min_val, max_val, step):
        slider = gui.Slider(gui.Slider.DOUBLE)
        slider.set_limits(min_val, max_val)
        slider.double_value = 0.0
        on_change = self._make_on_slider_changed(cid, index)
        slider.set_on_value_changed(on_change)
        self.sliders[cid].append(slider)
        
        btn_minus = gui.Button("-")
        btn_plus = gui.Button("+")
        
        # Closures with default arguments prevent late-binding issues in loops
        def on_minus(s=slider, oc=on_change, st=step, m_val=min_val):
            new_val = max(m_val, s.double_value - st)
            s.double_value = new_val
            oc(new_val)
            
        def on_plus(s=slider, oc=on_change, st=step, m_val=max_val):
            new_val = min(m_val, s.double_value + st)
            s.double_value = new_val
            oc(new_val)
            
        btn_minus.set_on_clicked(on_minus)
        btn_plus.set_on_clicked(on_plus)
        
        em = self.window.theme.font_size
        horiz = gui.Horiz(0.25 * em)
        horiz.add_child(gui.Label(label_text))
        horiz.add_child(btn_minus)
        horiz.add_child(slider)
        horiz.add_child(btn_plus)
        
        return horiz

    def _make_on_visible_toggled(self, cid):
        def on_checked(is_checked):
            self.camera_visible[cid] = is_checked
            self._update_geometry(cid)
        return on_checked

    def _make_on_slider_changed(self, cid, index):
        def on_changed(val):
            self.deltas[cid][index] = val
            old_active = self.active_camera
            self.active_camera = cid
            
            if old_active is not None and old_active != cid:
                self._update_geometry(old_active)
            self._update_geometry(cid)
        return on_changed

    def _draw_camera_marker(self, cid, transform_mat):
        frame_name = f"cam_marker_{cid}"
        if self.scene.scene.has_geometry(frame_name):
            self.scene.scene.remove_geometry(frame_name)
            
        # Draw a coordinate frame / cone marker for the camera
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
        
        if self.active_camera == cid:
            # Add a highlighted yellow sphere to the active camera
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.06)
            sphere.paint_uniform_color([1.0, 1.0, 0.0]) # Yellow
            frame += sphere
            
        frame.transform(transform_mat)
        
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        self.scene.scene.add_geometry(frame_name, frame, mat)

    def _update_geometry(self, cid):
        dx, dy, dz, rx_deg, ry_deg, rz_deg = self.deltas[cid]
        rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
        
        delta_mat = np.eye(4)
        delta_mat[:3, :3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz((rx, ry, rz))
        delta_mat[0, 3] = dx
        delta_mat[1, 3] = dy
        delta_mat[2, 3] = dz
        
        final_mat = self.transform_matrices[cid] @ delta_mat
        
        # Always update the physical camera representation marker
        self._draw_camera_marker(cid, final_mat)
        
        pc_name = f"pc_{cid}"
        if self.scene.scene.has_geometry(pc_name):
            self.scene.scene.remove_geometry(pc_name)
            
        if not self.camera_visible.get(cid, True):
            return
            
        # Copy original downsampled point cloud and map to Global frame
        pc_copy = copy.deepcopy(self.point_clouds_down[cid])
        pc_copy.transform(final_mat)
        
        if getattr(self, 'colorize_clouds', False):
            pc_copy.paint_uniform_color(self.cid_to_color[cid])
        
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        
        # Emphasize the currently active camera with a slightly altered rendering
        if self.active_camera == cid:
            mat.point_size = self.base_point_size * 1.5
        else:
            mat.point_size = self.base_point_size
            
        self.scene.scene.add_geometry(pc_name, pc_copy, mat)

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        width = 350
        self.scene.frame = gui.Rect(r.x, r.y, r.width - width, r.height)
        self.panel.frame = gui.Rect(r.x + r.width - width, r.y, width, r.height)

    def _on_key(self, event):
        if event.type == gui.KeyEvent.Type.DOWN:
            k = event.key
            if k in [gui.KeyName.C, ord('c'), ord('C')]:
                self._on_capture()
                return True
            elif k in [gui.KeyName.M, ord('m'), ord('M')]:
                self._on_save_pc()
                return True
            elif k in [gui.KeyName.X, ord('x'), ord('X')]:
                self._on_crop()
                return True
            elif k in [gui.KeyName.O, ord('o'), ord('O')]:
                self._on_set_origo()
                return True

            # WASD movement for active camera (Z is forward/backward, X is sideways)
            if self.active_camera is not None:
                cid = self.active_camera
                step = 0.01  # 1 cm step
                handled = False
                
                if k in [gui.KeyName.W, ord('w'), ord('W')]:
                    self.deltas[cid][2] += step
                    self.sliders[cid][2].double_value = self.deltas[cid][2]
                    handled = True
                elif k in [gui.KeyName.S, ord('s'), ord('S')]:
                    self.deltas[cid][2] -= step
                    self.sliders[cid][2].double_value = self.deltas[cid][2]
                    handled = True
                elif k in [gui.KeyName.A, ord('a'), ord('A')]:
                    self.deltas[cid][0] -= step
                    self.sliders[cid][0].double_value = self.deltas[cid][0]
                    handled = True
                elif k in [gui.KeyName.D, ord('d'), ord('D')]:
                    self.deltas[cid][0] += step
                    self.sliders[cid][0].double_value = self.deltas[cid][0]
                    handled = True
                elif k in [gui.KeyName.Q, ord('q'), ord('Q')]: # up
                    self.deltas[cid][1] += step
                    self.sliders[cid][1].double_value = self.deltas[cid][1]
                    handled = True
                elif k in [gui.KeyName.E, ord('e'), ord('E')]: # down
                    self.deltas[cid][1] -= step
                    self.sliders[cid][1].double_value = self.deltas[cid][1]
                    handled = True
                    
                if handled:
                    self._update_geometry(cid)
                    return True
            else:
                if k in [gui.KeyName.W, ord('w'), ord('W'), gui.KeyName.S, ord('s'), ord('S'), gui.KeyName.A, ord('a'), ord('A'), gui.KeyName.D, ord('d'), ord('D')]:
                    print("Please select a camera first! (drag any slider of the camera you want to move)")
                    
        return False

    def _on_capture(self):
        if self.rs_grabber is None:
            print("Cannot capture: rs_grabber is not available (running with --load?)")
            return
            
        print("Capturing new point clouds...")
        for device_id in self.rs_grabber.get_grabber_list():
            serial, img, pcd, K, dist = self.rs_grabber.capture_data(device_id, num_frames=15)
            if serial in self.point_clouds_down:
                self.full_point_clouds[serial] = pcd
                self.point_clouds_down[serial] = pcd #.voxel_down_sample(voxel_size=0.01)
                self._update_geometry(serial)
        print("Capture complete.")

    def _on_save_pc(self):
        print("\nSaving combined point cloud...")
        combined = o3d.geometry.PointCloud()
        for cid in self.transform_matrices.keys():
            dx, dy, dz, rx_deg, ry_deg, rz_deg = self.deltas[cid]
            rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
            
            delta_mat = np.eye(4)
            delta_mat[:3, :3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz((rx, ry, rz))
            delta_mat[0, 3] = dx
            delta_mat[1, 3] = dy
            delta_mat[2, 3] = dz
            
            final_mat = self.transform_matrices[cid] @ delta_mat
            
            pc_copy = copy.deepcopy(self.full_point_clouds[cid])
            pc_copy.transform(final_mat)
            combined += pc_copy
            
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"merged_pc_{timestamp}.ply"
        o3d.io.write_point_cloud(filename, combined)
        print(f"Saved merged point cloud to '{filename}'\n")

    def _on_crop(self):
        import subprocess
        import sys
        
        print("\nOpening cropping window...")
        combined = o3d.geometry.PointCloud()
        for cid in self.transform_matrices.keys():
            dx, dy, dz, rx_deg, ry_deg, rz_deg = self.deltas[cid]
            rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
            
            delta_mat = np.eye(4)
            delta_mat[:3, :3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz((rx, ry, rz))
            delta_mat[0, 3] = dx
            delta_mat[1, 3] = dy
            delta_mat[2, 3] = dz
            
            final_mat = self.transform_matrices[cid] @ delta_mat
            
            pc_copy = copy.deepcopy(self.full_point_clouds[cid])
            pc_copy.transform(final_mat)
            combined += pc_copy

        # Save to temp file
        import tempfile
        temp_ply = tempfile.mktemp(suffix=".ply")
        temp_json = tempfile.mktemp(suffix=".json")
        temp_py = tempfile.mktemp(suffix=".py")
        
        o3d.io.write_point_cloud(temp_ply, combined)
        
        script_code = f"""
import open3d as o3d
import json
import numpy as np
import os

pcd = o3d.io.read_point_cloud(r"{temp_ply}")
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window(window_name='Crop the Pointcloud (Press K to select, C to crop, then close)', width=1260, height=720)
vis.add_geometry(pcd)
vis.run()
vis.destroy_window()

cropped_pc = vis.get_cropped_geometry()
if cropped_pc is not None and len(cropped_pc.points) > 0:
    bbox = cropped_pc.get_axis_aligned_bounding_box()
    min_bound = bbox.get_min_bound().tolist()
    max_bound = bbox.get_max_bound().tolist()
    with open(r"{temp_json}", "w") as f:
        json.dump({{"min": min_bound, "max": max_bound}}, f)
"""
        with open(temp_py, "w") as f:
            f.write(script_code)
            
        print("Launching cropping view in a separate process to avoid OpenGL/GLFW context conflicts...")
        subprocess.run([sys.executable, temp_py])
        
        if os.path.exists(temp_json):
            print("Applying crop to individual clouds...")
            with open(temp_json, "r") as f:
                bbox_data = json.load(f)
            
            min_bound = np.array(bbox_data["min"], dtype=np.float64)
            max_bound = np.array(bbox_data["max"], dtype=np.float64)
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            
            for cid in self.transform_matrices.keys():
                dx, dy, dz, rx_deg, ry_deg, rz_deg = self.deltas[cid]
                rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
                
                delta_mat = np.eye(4)
                delta_mat[:3, :3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz((rx, ry, rz))
                delta_mat[0, 3] = dx
                delta_mat[1, 3] = dy
                delta_mat[2, 3] = dz
                
                final_mat = self.transform_matrices[cid] @ delta_mat
                inv_mat = np.linalg.inv(final_mat)
                
                # Transform to global, crop using the bbox we got from the combined cloud, and back to local
                pc = self.full_point_clouds[cid]
                pc.transform(final_mat)
                cropped_indiv = pc.crop(bbox)
                cropped_indiv.transform(inv_mat)
                
                self.full_point_clouds[cid] = cropped_indiv
                self.point_clouds_down[cid] = cropped_indiv #.voxel_down_sample(voxel_size=0.005)
                
                self._update_geometry(cid)
            print("Crop applied successfully.")
            os.remove(temp_json)
        else:
            print("Crop cancelled or empty.")
            
        if os.path.exists(temp_ply):
            os.remove(temp_ply)
        if os.path.exists(temp_py):
            os.remove(temp_py)

    def _on_set_origo(self):
        import subprocess
        import sys
        
        print("\nOpening picking window to set new navigation origin...")
        combined = o3d.geometry.PointCloud()
        for cid in self.transform_matrices.keys():
            dx, dy, dz, rx_deg, ry_deg, rz_deg = self.deltas[cid]
            rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
            
            delta_mat = np.eye(4)
            delta_mat[:3, :3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz((rx, ry, rz))
            delta_mat[0, 3] = dx
            delta_mat[1, 3] = dy
            delta_mat[2, 3] = dz
            
            final_mat = self.transform_matrices[cid] @ delta_mat
            
            pc_copy = copy.deepcopy(self.full_point_clouds[cid])
            pc_copy.transform(final_mat)
            combined += pc_copy

        import tempfile
        temp_ply = tempfile.mktemp(suffix=".ply")
        temp_json = tempfile.mktemp(suffix=".json")
        temp_py = tempfile.mktemp(suffix=".py")
        
        o3d.io.write_point_cloud(temp_ply, combined)
        
        script_code = f"""
import open3d as o3d
import json
import numpy as np

pcd = o3d.io.read_point_cloud(r"{temp_ply}")
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window(window_name='Pick 1 Point for new Camera Center (Shift+Click to pick, then close)', width=1260, height=720)
vis.add_geometry(pcd)
vis.run()
vis.destroy_window()

picked = vis.get_picked_points()
if len(picked) > 0:
    idx = picked[0]
    pt = np.asarray(pcd.points)[idx]
    with open(r"{temp_json}", "w") as f:
        json.dump({{"center": pt.tolist()}}, f)
"""
        with open(temp_py, "w") as f:
            f.write(script_code)
            
        print("Launching picking view in a separate process...")
        subprocess.run([sys.executable, temp_py])
        
        if os.path.exists(temp_json):
            with open(temp_json, "r") as f:
                data = json.load(f)
            
            new_center = tuple(data["center"])
            print(f"Setting new camera center of rotation to {new_center}")
            
            bounds = self.scene.scene.bounding_box
            self.scene.setup_camera(60, bounds, new_center)
            
            os.remove(temp_json)
        else:
            print("Picking cancelled.")
            
        if os.path.exists(temp_ply):
            os.remove(temp_ply)
        if os.path.exists(temp_py):
            os.remove(temp_py)

    def _on_finish(self):
        for cid in self.transform_matrices.keys():
            dx, dy, dz, rx_deg, ry_deg, rz_deg = self.deltas[cid]
            rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
            
            delta_mat = np.eye(4)
            delta_mat[:3, :3] = o3d.geometry.PointCloud().get_rotation_matrix_from_xyz((rx, ry, rz))
            delta_mat[0, 3] = dx
            delta_mat[1, 3] = dy
            delta_mat[2, 3] = dz
            
            self.transform_matrices[cid] = self.transform_matrices[cid] @ delta_mat
            
        gui.Application.instance.quit()

    def run(self):
        gui.Application.instance.run()

# ==========================================
# Main Execution
# ==========================================
def main():
    parser = argparse.ArgumentParser(description="ChArUco calibration & 3D Point Cloud merge tool.")
    
    parser.add_argument("--load", action="store_true", help="Load captured image/PC data instead of connecting to realsense.")
    parser.add_argument("--calibrate", action="store_true", help="Override factory intrinsics by running ChArUco calibration on the captured frames.")
    parser.add_argument("--config", type=str, default=None, help="Path to cameraconfig.json to load transform matrices from, skipping charuco pose estimation.")
    parser.add_argument("--data_dir", type=str, default="./camera_data", help="Directory to save/load camera captures.")
    parser.add_argument("--debug", action="store_true", help="Show detection debug visualizations.")
    
    # ChArUco Params
    parser.add_argument("--dict", type=str, default="DICT_4X4_50")
    parser.add_argument("--squares_x", type=int, default=4)
    parser.add_argument("--squares_y", type=int, default=3)
    parser.add_argument("--charuco_square_len", type=float, default=0.187)
    parser.add_argument("--charuco_marker_len", type=float, default=0.1247)

    # ROI written into cameraconfig.json for every camera
    parser.add_argument("--roi", type=int, nargs=4, default=[15, 18, 1207, 621],
                        metavar=("start_x", "start_y", "width", "height"),
                        help="ROI (start_x start_y width height) stored per camera in the output config.")

    args = parser.parse_args()

    # Init ChArUco Board
    try:
        dict_type = getattr(cv2.aruco, args.dict)
    except AttributeError:
        dict_type = cv2.aruco.DICT_4X4_50

    try:
        dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
        charuco_board = cv2.aruco.CharucoBoard(
            (args.squares_x, args.squares_y), args.charuco_square_len, args.charuco_marker_len, dictionary)
    except AttributeError:
        dictionary = cv2.aruco.Dictionary_get(dict_type)
        charuco_board = cv2.aruco.CharucoBoard_create(
            args.squares_x, args.squares_y, args.charuco_square_len, args.charuco_marker_len, dictionary)

    # 1. Acquire Data (Load or Capture exactly once per device)
    images, point_clouds, intrinsics, dist_coeffs_dict = {}, {}, {}, {}
    rs_grabber = None

    if args.load:
        print(f"\n--- Loading Offline Data from {args.data_dir} ---")
        images, point_clouds, intrinsics, dist_coeffs_dict = load_camera_data(args.data_dir)
        if not images:
            print("No data found to load. Exiting.")
            return
    else:
        print(f"\n--- Capturing Data From Realsense Devices ---")
        rs_grabber = rs2_Grabber()
        for device_id in rs_grabber.get_grabber_list():
            print(f"Capturing Frame & Point Cloud from device index {device_id}...")
            serial, img, pcd, K, dist = rs_grabber.capture_data(device_id, num_frames=30)
            
            images[serial] = img
            point_clouds[serial] = pcd
            intrinsics[serial] = K
            dist_coeffs_dict[serial] = dist
            
            # Save for offline reuse
            save_camera_data(args.data_dir, serial, img, pcd, K, dist)

    # 2. Intrinsic Calibration (Optional)
    if args.calibrate:
        print("\n--- Performing Intrinsic Calibration Over Capture Set ---")
        K_new, dist_coeffs_new = calibrate_camera_charuco(list(images.values()), charuco_board, debug=args.debug)
        if K_new is not None:
            # Overwrite factory intrinsics
            for serial in intrinsics.keys():
                intrinsics[serial] = K_new
                dist_coeffs_dict[serial] = dist_coeffs_new

    # 3. Extrinsic Pose Estimation
    transform_matrices = {}
    if args.config and os.path.exists(args.config):
        print(f"\n--- Loading Extrinsics from {args.config} ---")
        with open(args.config, 'r') as f:
            config_data = json.load(f)
        for serial in images.keys():
            if serial in config_data.get('devices_config', {}):
                transform_matrices[serial] = coord_transform_to_matrix(config_data['devices_config'][serial].get('coord_transform', {}))
                print(f"Loaded matrix for camera {serial}")
            else:
                print(f"Warning: camera {serial} not found in config. Using identity matrix.")
                transform_matrices[serial] = np.eye(4)
    else:
        for serial, img in images.items():
            print(f"\n--- Solving Pose for Camera {serial} ---")
            T = auto_calib_charuco(
                img=img, 
                K=intrinsics[serial], 
                dist_coeffs=dist_coeffs_dict[serial], 
                charuco_board=charuco_board, 
                debug=args.debug
            )
            transform_matrices[serial] = T

    # 4. Preview and Fine Tuning with True Point Clouds
    print("\nLaunching Manual Tuning UI for visual verification & tuning with Dense Point Clouds...")
    tuning_app = ManualTuningWindow(transform_matrices, point_clouds, marker_length=args.charuco_square_len, rs_grabber=rs_grabber)
    tuning_app.run()
    
    # 5. Save output
    transform_matrices = tuning_app.transform_matrices
    ROI_per_device = {serial: args.roi for serial in transform_matrices.keys()}
    rs2_save(transform_matrices, ROI_per_device)

if __name__ == "__main__":
    main()
