#ifndef UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP
#define UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP

#include "based_camera.hpp"	
#include "uvgvolucap/log.hpp"
#include "uvgvolucap/threadqueue.hpp"
#include "geometry/point_cloud.hpp"
#include "geometry/grid.hpp"
#include <nlohmann/json.hpp>
#include <k4a/k4a.hpp>

namespace uvgvolucap {
    namespace camera{
        #define CAPTURE_TIMEOUT 500 /**< Capture timeout */
        /**
         * @brief Struct for main setting of the camera.
         * @details This struct contains the main setting of the camera.
         */
        struct MainSetting {
            mutable int color_resolution = 1536;
            mutable int depth_resolution = 576;
            mutable int fps = 30;
            mutable bool depth_to_color = true;
            mutable bool voxelized = false;
            mutable int voxelized_mode = 0;
            mutable int subsample_row = 0;
            mutable int subsample_col = 0;
        };

        /**
         * @brief Struct for filter configuration.
         * @details This struct contains the filter configuration.
         */
        struct FilterConfig {
            mutable float max_xy = 0;
            mutable float min_xy = 0;
            mutable float max_z = 0;
            mutable float min_z = 0;
        };

        /**
         * @brief Struct for ROI configuration.
         * @details This struct contains the ROI configuration for limiting the area in the images for converting to point cloud.
         */
        struct ROIConfig {
            mutable size_t start_x = 0;
            mutable size_t start_y = 0;
            mutable size_t width = 0;
            mutable size_t height = 0;
        };

        /**
         * @brief Struct for synchronization manager.
         * @details This struct contains the set of functions pointers for synchronization 
         * and data structre for final merge point cloud before sending to encoder/visualizer.
         */
        struct SyncManager {
            std::shared_ptr<std::function<void(int, bool)>> update_device_ready_fptr = nullptr;
            std::shared_ptr<std::function<void(int, bool)>> update_device_capture_fptr = nullptr;
            std::shared_ptr<std::function<int()>> get_num_ready_cam_fptr = nullptr;
            std::shared_ptr<std::function<int()>> get_num_cap_cam_fptr = nullptr;

            std::mutex sync_mx; /**< Mutex for synchronization. -public usage */
            std::condition_variable Cap_permission_cv; /**< Condition variable for synchronization. - internal usage */
        
            size_t count_pcl = 0; //For control based on user input
            std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer = nullptr; /**< Point cloud buffer */
            std::shared_ptr<uvgvolucap::Job> _job = nullptr; 
        };

        /**
         * @brief Struct for final point cloud configuration.
         * @details This struct contains information for grid to voxelized the data of the point cloud,
         * as well as the bounding box to filter the point cloud in the grid world.
         */
        struct PointCloudConfig {
            mutable size_t geometry_precision = 0;
            mutable int min_bound[3] = {0, 0, 0};
            mutable int max_bound[3] = {0, 0, 0};
            mutable int number_of_slices = 1;
        };
        
        /**
         * @brief Struct for camera information.
         * @details index is the index USB port of the camera to open the device correctly.
         *          sync_index is the index of the camera use for update the sync manager.
         *          The reason for initializing the index and sync_index is in some case, we need to disable some cameras which lead to mismatch between the index and sync_index.
         *          Therefore, we need to keep the original index and sync_index seperately to prevent the mismatch.
         */
        struct KinectCameraInfo {
            mutable std::string serial_number = "";
            mutable int index = 0;
            mutable int sync_index = 0;

            MainSetting system_config;
            FilterConfig filter_config;
            PointCloudConfig pointcloud_config;
            ROIConfig roi;

            mutable std::array<float, 16> transformation_matrix;
        };

        /**
         * @brief Class for frame.
         * @details This class is the data structure for the pointcloud factory. 
         * We can use this frame for generating the point cloud from the depth and color images in 2 modes: normal and voxelized.
         */
        class Frame {
        private:
            int id = 0; /**< Frame ID */
            k4a_image_t depth_image = NULL; /**< Depth image */
            k4a_image_t color_image = NULL; /**< Color image */
            
            std::shared_ptr<geometry::PclFragment> fragment_pcl = std::make_shared<geometry::PclFragment>(); /**< Point cloud */
            std::mutex subspace_mx;
            std::shared_ptr<std::vector<std::shared_ptr<geometry::PclFragment>>> subspace_fragments = std::make_shared<std::vector<std::shared_ptr<geometry::PclFragment>>>();

            int min_bound[3] = {0, 0 ,0};
            int max_bound[3] = {0, 0, 0};
            int number_of_slices = 8;

        public:
            /**
             * @brief Constructor for Data_package.
             * @param depth_ The depth image.
             * @param color_ The color image.
             */
            Frame(int _id, k4a_image_t depth_, k4a_image_t color_, int min_bound_[3], int max_bound_[3], int number_of_slices_ = 8);

            /**
             * @brief Get the number of slices.
             * @return int The number of slices.
             */
            int get_number_of_slices();

            /**
             * @brief Get the max bound.
             * @param index The index of the bound.
             * @return int The max bound.
             */
            int get_max_bound(int index);

            /**
             * @brief Get the min bound.
             * @param index The index of the bound.
             * @return int The min bound.
             */
            int get_min_bound(int index);
            
            /**
             * @brief Get the depth image.
             * @return k4a_image_t The depth image.
             */
            k4a_image_t get_depth_image();
            
            /**
             * @brief Get the color image.
             * @return k4a_image_t The color image.
             */
            k4a_image_t get_color_image();
            
            /**
             * @brief Get the subspace fragments.
             * @return geometry::_slice_fragments_ptr The subspace fragments.
             */
            geometry::_slice_fragments_ptr get_subspace_fragments();

            /**
             * @brief Get the frame point cloud.
             * @return std::shared_ptr<geometry::PclFragment> The frame point cloud.
             */
            std::shared_ptr<geometry::PclFragment> get_frame_pointcloud();
            
            /**
             * @brief Set the depth image.
             * @param depth_ The depth image.
             */
            void set_depth_image(k4a_image_t depth_);
            
            /**
             * @brief Set the color image.
             * @param color_ The color image.
             */
            void set_color_image(k4a_image_t color_);
            
            /**
             * @brief Set the min bound.
             * @param x The x value.
             * @param y The y value.
             * @param z The z value.
             */
            void set_min_bound(int x, int y, int z);
            
            /**
             * @brief Set the max bound.
             * @param x The x value.
             * @param y The y value.
             * @param z The z value.
             */
            void set_max_bound(int x, int y, int z) ;
            
            /**
             * @brief Set the number of slices.
             * @param num The number of slices.
             */
            void set_number_of_slices(int num) ;
        };
        
        /**
         * @brief Class for Kinect.
         * @details This class is the implementation of the Kinect camera.
         */
        class Kinect : public BasedCamera<uint32_t, uint32_t, std::string, nlohmann::json> {
        private:
            k4a_device_t m_device;                                                      // Device handler in Azure Kinect SDK
            k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;   // Device configuration in Azure Kinect SDK
            k4a_calibration_t m_calibration;                                            // Device calibration in Azure Kinect SDK                      
            k4a_transformation_t transformation_handle = nullptr;                       // Transformation handler in Azure Kinect SDK
            k4a_image_t xy_table = NULL;                                                // Pre-defined table for fast transformation between depth and color image
            k4a_float2_t *xy_table_data = NULL;                                         // Data of the xy_table after getting from depth/color image                      
            KinectCameraInfo device_info;                                               // Device information  including the setting, filter, and point cloud configuration                

            // Get depth image size for Color2Depth
            int target_viewpoint_width = 0;                                              // Depending on the transformation mode, this could be the width of color or depth image
            int target_viewpoint_height = 0;                                             // Depending on the transformation mode, this could be the height of color or depth image

            std::shared_ptr<std::thread> capture_thread_ptr;                             // Capture thread pointer
            std::function<void()> capture_function;                                      // Capture function
            std::shared_ptr<camera::SyncManager> sync_manager = nullptr;                 // Pointer to the universal sync manager which is declear from the PointCloudFactory class
            std::shared_ptr<geometry::Grid> grid_ptr = nullptr;                          // Pointer to the grid object for converting the point cloud to the grid world

        public:
            Kinect(uint32_t _index, uint32_t sync_index, std::string _serial, nlohmann::json _config);
            ~Kinect();

            /**
             * @brief Get the serial number using the Azure Kinect SDK.
             */
            std::string get_serial_number();

            /**
             * @brief Warm up the devices to redeay for synchronization and capture.
             */
            void warm_up() override;

            /**
             * @brief Stop the device.
             */
            void stop() override;
            
            /**
             * @brief Start the capture thread.
             * @param _thread_queue The thread queue.
             * @param _sync_manager The sync manager.
             * @details This function is used to start the capture thread for the camera, which also used to setup jobs with dependency and set the universal sync manager to sync_manager pointer.
             */
            void start_capture(std::shared_ptr<ThreadQueue> _thread_queue, std::shared_ptr<SyncManager> _sync_manager);
        
        private:
            /**
             * @brief Setup the configuration of the device based on the input configuration.
             */
            void setup_device_config();

            /**
             * @brief Create the xy table for fast transformation between depth and color image.
             * @param calibration The calibration of the device.
             */
            void createXYTable(const k4a_calibration_t *calibration);

            /**
             * @brief Transform the view point.
             * @param frame The frame.
             * @details This function is a function job which used to transform the view point of the depth image to the color image or vice versa.
             */
            void transform_view_point(std::shared_ptr<Frame> frame);

            /**
             * @brief Process the frame.
             * @param frame The frame.
             * @details This function is a function job which used to process the frame to generate the point cloud in normal mode.
             */
            void process_frame(std::shared_ptr<Frame> frame);

            /**
             * @brief Process the frame in subspace mode.
             * @param frame The frame.
             * @details This function is a function job which used to process the frame to generate the point cloud in subspace mode.
             */
            void process_frame_voxel_subspace(std::shared_ptr<Frame> frame);

            /**
             * @brief Pack the fragment point cloud and merge buffer point cloud.
             * @param fragment_pcl The fragment point cloud.
             * @param _asisgned_merge_buffer The merge buffer point cloud.
             * @details This function is a function job which used to pack the data from the fragment point cloud and merge buffer point cloud (Only use in normal mode).
             */
            void pack_fragment(std::shared_ptr<geometry::PclFragment> fragment_pcl, std::shared_ptr<geometry::MergeBufferPointCloud> _asisgned_merge_buffer);

            /**
             * @brief  Setup the job dependency for the point cloud production line in normal mode.
             * @details This function is used to capture frame and notify the sync manager that the frame is captured.
             * Also the threadqueue jobs also is managed in this function.
             * Jobs: TransformViewPoint, ProcessFrame, PackFragment
            */
            void pointcloud_production_line();

            /**
             * @brief  Setup the job dependency for the point cloud production line in subspace mode.
             * @details This function is used to capture frame and notify the sync manager that the frame is captured.
             * Also the threadqueue jobs also is managed in this function.
             * Jobs: TransformViewPoint, ProcessFramewithSubspace
            */
            void pointcloud_production_line_with_subsapce();

        protected:
            /**
             * @brief Initialize the camera.
             * @param _index The index of the camera.
             * @param _sync_index The sync index of the camera.
             * @param _serial The serial number of the camera.
             * @param _config The configuration of the camera.
             * @details This function is used to initialize the camera with the input configuration. 
             * The json is parsed here to get the corresponding setting, filter, and point cloud configuration.
             */
            void init(uint32_t _index, uint32_t _sync_index, std::string _serial, nlohmann::json _config) const override;

            /**
             * @brief Open the camera.
             * @details This function is used to open the camera with the configuration setup using Azure Kinect SDK.
             */
            void open() override;

            /**
             * @brief Close the camera.
             * @details This function is used to close the camera using Azure Kinect SDK.
             */
            void close() override;
            
            /**
             * @brief Start the camera.
             * @details This function is used to start the camera to capture the frame after opening the camera.
             */
            void start() override;
        };

        typedef std::shared_ptr<Kinect> _kinect_device_ptr; /**< Shared pointer to the Kinect camera */
    } // namespace camera
} // namespace uvgvolucap

#endif // UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP