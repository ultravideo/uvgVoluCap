#ifndef REALSENSE_CAMERA_HPP
#define REALSENSE_CAMERA_HPP

#include <librealsense2/rs.hpp>
#include "based_camera.hpp"	

namespace uvgvolucap {
    namespace camera {  
        struct RealsenseCameraInfo {
            mutable std::string serial_number = "";
            mutable int index = 0;
            mutable int sync_index = 0;

            MainSetting system_config;
            FilterConfig filter_config;
            PointCloudConfig pointcloud_config;
            ROIConfig roi;

            mutable std::array<float, 16> transformation_matrix;
        };

        class Realsense_Frame : public BasedFrame {
        private:
            rs2::pointcloud frame_pointcloud; /**< Frame point cloud */
            rs2::depth_frame depth_frame = NULL; /**< Depth frame */
            rs2::video_frame color_frame = NULL; /**< Color frame */
        
        public:
            Realsense_Frame(int _id, rs2::depth_frame depth_, rs2::video_frame color_, int min_bound_[3], int max_bound_[3], int number_of_slices_ = 8);
            ~Realsense_Frame() = default;

            rs2::depth_frame get_depth_frame();
            rs2::video_frame get_color_frame();
            rs2::pointcloud get_rs_pointcloud();

            void set_depth_frame(rs2::depth_frame depth_);
            void set_color_frame(rs2::video_frame color_);

            void transform_view_point();
        };

        /* ############################################################################################## */
        typedef struct rs2_depth_mode_t{
            int width = 640; /**< Width */
            int height = 480; /**< Height */
            rs2_format format = RS2_FORMAT_Z16; /**< Format */
        } rs2_depth_mode_t; /**< Depth mode */

        typedef struct rs2_color_mode_t{
            int width = 640; /**< Width */
            int height = 480; /**< Height */
            rs2_format format = RS2_FORMAT_RGB8; /**< Format */
        } rs2_color_mode_t; /**< Color mode */

        typedef int rs2_fps_t; /**< Frame per second */

        class Realsense : public BasedCamera<uint32_t, uint32_t, std::string, nlohmann::json> {
        private: 
            rs2::pipeline m_pipeline; /**< Pipeline */
            rs2::config m_config; /**< Configuration */
            rs2::device m_device; /**< Device */
            RealsenseCameraInfo device_info; /**< Device information */

            rs2::threshold_filter m_threshold_filter; /**< Threshold filter */
            rs2::spatial_filter m_spatial_filter; /**< Spatial filter */
            rs2::decimation_filter m_decimation_filter; /**< Decimation filter */
            rs2::temporal_filter m_temporal_filter; /**< Temporal filter */
            rs2::hole_filling_filter m_hole_filling_filter; /**< Hole filling filter */

            std::shared_ptr<std::thread> capture_thread_ptr; /**< Capture thread pointer */
            std::function<void()> capture_function; /**< Capture function */
            std::shared_ptr<camera::SyncManager> sync_manager = nullptr; /**< Pointer to the universal sync manager which is declear from the PointCloudFactory class */
            std::shared_ptr<geometry::Grid> grid_ptr = nullptr; /**< Pointer to the grid object for converting the point cloud to the grid world */
        
        public:
            Realsense(uint32_t _index, uint32_t sync_index, std::string _serial, nlohmann::json _config);
            ~Realsense();

            /**
             * @brief Get the serial number using the Azure Kinect SDK.
             */
            std::string get_serial_number();

            void open() override;

            void close() override;

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
             * @brief Transform the view point.
             * @param frame The frame.
             * @details This function is a function job which used to transform the view point of the depth image to the color image or vice versa.
             */
            void transform_view_point(std::shared_ptr<Realsense_Frame> frame);

            /**
             * @brief Process the frame.
             * @param frame The frame.
             * @details This function is a function job which used to process the frame to generate the point cloud in subspace mode.
             */
            void process_frame_voxel_subspace(std::shared_ptr<Realsense_Frame> frame);

            /**
             * @brief Process the frame.
             * @param frame The frame.
             * @details This function is a function job which used to process the frame to generate the point cloud in normal mode.
             */
            void process_frame(std::shared_ptr<Realsense_Frame> frame);

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
             * @brief Start the camera.
             * @details This function is used to start the camera to capture the frame after opening the camera.
             */
            void start() override;
        };

        typedef std::shared_ptr<Realsense> _realsense_device_ptr; /**< Shared pointer to the Realsense camera */
    } // namespace camera
} // namespace uvgvolucap
#endif // REALSENSE_CAMERA_HPP