#ifndef UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP
#define UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP

#include "based_camera.hpp"	
#include <k4a/k4a.hpp>

namespace uvgvolucap {
    namespace camera {  
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
        class Kinect_Frame : public BasedFrame {
        private:
            k4a_image_t depth_image = NULL; /**< Depth image */
            k4a_image_t color_image = NULL; /**< Color image */

        public:
            /**
             * @brief Constructor for Data_package.
             * @param depth_ The depth image.
             * @param color_ The color image.
             */
            Kinect_Frame(int _id, k4a_image_t depth_, k4a_image_t color_, int min_bound_[3], int max_bound_[3], int number_of_slices_ = 8);

            /**
             * @brief Destructor for Data_package.
             */
            ~Kinect_Frame() = default;

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
             * @brief Set the depth image.
             * @param depth_ The depth image.
             */
            void set_depth_image(k4a_image_t depth_);
            
            /**
             * @brief Set the color image.
             * @param color_ The color image.
             */
            void set_color_image(k4a_image_t color_);
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
            void transform_view_point(std::shared_ptr<Kinect_Frame> frame);

            /**
             * @brief Process the frame.
             * @param frame The frame.
             * @details This function is a function job which used to process the frame to generate the point cloud in normal mode.
             */
            void process_frame(std::shared_ptr<Kinect_Frame> frame);

            /**
             * @brief Process the frame in subspace mode.
             * @param frame The frame.
             * @details This function is a function job which used to process the frame to generate the point cloud in subspace mode.
             */
            void process_frame_voxel_subspace(std::shared_ptr<Kinect_Frame> frame);

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