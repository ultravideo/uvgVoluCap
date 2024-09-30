import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import time

import cv2

class rs2_Grabber:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)
        self.depth_scale = 0.001
        self.point_cloud = None
        self.patterns_dict = {}
        self.camera_corners_ids = {}
        self.device_count = rs.context().query_devices().size()
        if not self.device_count:
            print("No devices available")
            exit()
        self.device_ids = list(range(self.device_count))

    def setup_cam(self):
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    def capture_point_cloud(self):
        self.setup_cam()

        profile = self.pipeline.start(self.config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        time.sleep(0.5)
        for i in range(30):
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
   
            if not depth_frame or not color_frame:
                continue

            pc = rs.pointcloud()
            pc.map_to(color_frame)

            points = pc.calculate(depth_frame)
            texture_coordinates = np.asanyarray(points.get_texture_coordinates())
            vertices = np.asanyarray(points.get_vertices())

            print(f"Texture Coordinates: {texture_coordinates.shape}")
            print(f"Vertices: {vertices.shape}")

            color_image = np.asanyarray(color_frame.get_data())
            texture_width = color_frame.get_width()
            texture_height = color_frame.get_height()
            texture_x_step = color_frame.get_bytes_per_pixel()
            texture_y_step = color_frame.get_stride_in_bytes()
            texture_data = color_image.tobytes()

            points = []
            colors = []

            for i in range(len(vertices)):

                x, y, z = vertices[i]
                if z == 0:
                    continue

                u = texture_coordinates[i][0]
                v = texture_coordinates[i][1]

                texture_x = int(u * texture_width)
                texture_y = int(v * texture_height)

                if (texture_x <= 0 or texture_x >= texture_width-1): continue
                if (texture_y <= 0 or texture_y >= texture_height-1): continue
                
                idx = texture_x * texture_x_step + texture_y * texture_y_step
                b = texture_data[idx]
                g = texture_data[idx + 1]
                r = texture_data[idx + 2]

                color = [r/255, g/255 , b/255]

                points.append([x, y, z])
                colors.append(color)

            self.point_cloud = o3d.geometry.PointCloud()
            self.point_cloud.points = o3d.utility.Vector3dVector(points)
            self.point_cloud.colors = o3d.utility.Vector3dVector(colors)



            self.pipeline.stop()
            break

        return self.point_cloud

    def manual_ROI(self, color_image):
        print("Select ROI")
        r = cv2.selectROI("ROI", color_image)
        cv2.destroyAllWindows()
        self.roi = r
        print(f"ROI: {self.roi}")

        return self.roi
    
    def get_grabber_list(self):
        return self.device_ids
    
    def get_camera_serial(self):
        return self.serial
    
    def get_ROI(self):
        return self.roi
    
    def show_point_cloud(self):
        print("Please check the quality of the Point Cloud. Press Q/ESQ to close the visualizer.")
        o3d.visualization.draw_geometries([self.point_cloud])

        # while(True):
        #     key = input("Press Y to confirm, N to retry.\nEnter your choice (N/Y): ")
        #     if key.upper() == 'N':
        #         return False
        #     elif key.upper() == 'Y':
        #         return True
        #     else:
        #         print("Invalid input. Please try again.")

    
if __name__ == "__main__":
    grabber = rs2_Grabber()
    grabber.setup_cam()
    grabber.capture_point_cloud()
    grabber.show_point_cloud()

   

