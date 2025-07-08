import open3d as o3d
import numpy as np
import time

import cv2 

import pyk4a
from pyk4a import Config, PyK4A, connected_device_count
from pyk4a import transformation, depth_image_to_point_cloud, depth_image_to_color_camera

class k4a_Grabber:
    def __init__(self):
        self.device_count = connected_device_count()
        if not self.device_count:
            print("No devices available")
            # exit()
        print(f"Connected devices: {self.device_count}")
        self.device_ids = list(range(self.device_count))
        self.device = None
        self.serial = None
        self.point_cloud = None
        self.patterns_dict = {}
        self.camera_corners_ids = {}

    def setup_cam(self, device_id):
        self.device = PyK4A(
            Config(
                color_resolution=pyk4a.ColorResolution.RES_1536P,
                color_format=pyk4a.ImageFormat.COLOR_BGRA32,
                camera_fps=pyk4a.FPS.FPS_15,
                depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
                synchronized_images_only=True,
            ),
            device_id=device_id
        )
    
    def capture_point_cloud(self, device_id):
        self.device_id = device_id
        self.setup_cam(device_id)

        self.device.open()
        print(f"\nGrab point cloud from Camera {device_id}: {self.device.serial}")
        self.serial = self.device.serial
        self.device.close()

        time.sleep(0.5)
        self.device.start()
        time.sleep(2)

        capture = self.device.get_capture()
        time.sleep(0.5)

        while True:
            capture = self.device.get_capture()
            if np.any(capture.depth) and np.any(capture.color):
                self.manual_ROI(capture.color)

                depth_cropped = capture.transformed_depth_point_cloud[int(self.roi[1]):int(self.roi[1]+self.roi[3]), int(self.roi[0]):int(self.roi[0]+self.roi[2])]
                color_cropped = capture.color[int(self.roi[1]):int(self.roi[1]+self.roi[3]), int(self.roi[0]):int(self.roi[0]+self.roi[2])]

                print(f"ROI values: {self.roi}")
                reshaped_roi = depth_cropped.reshape((-1, 3))
                print("Reshaped ROI shape:", reshaped_roi.shape)

                points = depth_cropped.reshape((-1, 3)) / 1000
                colors = color_cropped[..., (2, 1, 0)].reshape((-1, 3))

                self.point_cloud = o3d.geometry.PointCloud()
                self.point_cloud.points = o3d.utility.Vector3dVector(points)
                self.point_cloud.colors = o3d.utility.Vector3dVector(colors / 255)

                # Use the mouse to create a bounding box for cropping
                bounding_box = o3d.visualization.VisualizerWithEditing()
                bounding_box.create_window(window_name='Crop the Pointcloud', width=1260, height=720)
                bounding_box.add_geometry(self.point_cloud)
                bounding_box.run()  # This will open the visualization window
                bounding_box.destroy_window()

                # Crop the point cloud using the bounding box
                self.point_cloud = bounding_box.get_cropped_geometry()

                # draw the 3D border box surrounding the cropped point cloud
                border_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=self.point_cloud.get_min_bound(), max_bound=self.point_cloud.get_max_bound())
                border_box.color = (1, 0, 0)
                border_box = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(border_box)
                border_box = o3d.geometry.PointCloud()
                border_box.points = o3d.utility.Vector3dVector(border_box.points)
                border_box.colors = o3d.utility.Vector3dVector(border_box.colors)
                self.point_cloud += border_box
                
                if self.show_point_cloud():
                    break

        self.device.stop()
        return self.point_cloud, self.serial

    def get_grabber_list(self):
        return self.device_ids
    
    def get_camera_serial(self):
        return self.serial
    
    def manual_ROI(self, color_image):
        self.roi = cv2.selectROI('Color Image', color_image, fromCenter=False, showCrosshair=True)
        cv2.destroyAllWindows()

    def get_ROI(self):
        return self.roi

    def show_point_cloud(self):
        print("Please check the quality of the Point Cloud. Press Q/ESQ to close the visualizer.")
        # o3d.visualization.draw_geometries([self.point_cloud])

        while(True):
            key = input("Press Y to confirm, N to retry.\nEnter your choice (N/Y): ")
            if key.upper() == 'N':
                return False
            elif key.upper() == 'Y':
                return True
            else:
                print("Invalid input. Please try again.")


if __name__ == "__main__":
    grabber = k4a_Grabber()
    device_ids = grabber.get_grabber_list()
    if not device_ids:
        print("No devices available")
        exit()
    
    device_id = device_ids[0]  # Select the first device
    grabber.setup_cam(device_id)
    point_cloud, _ = grabber.capture_point_cloud(device_id)
