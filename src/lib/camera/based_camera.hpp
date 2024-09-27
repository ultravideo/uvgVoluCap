#pragma once
#ifndef BASED_CAMERA_HPP
#define BASED_CAMERA_HPP

#include "uvgvolucap/log.hpp"
#include "uvgvolucap/threadqueue.hpp"
#include "geometry/point_cloud.hpp"
#include "geometry/grid.hpp"
#include <nlohmann/json.hpp>

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
         * @brief Enum for voxelizer mode.
         * @details This enum contains the voxelizer mode. Set as enum for position extension.
         */
        enum voxelizer_mode {
            VOXELIZER_SUBSPACE
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

        template <typename... Args>
        class BasedCamera {
        protected:
            bool is_opened_flag = false;
            bool is_started_flag = false;
            bool is_voxelized_flag = false;
            std::shared_ptr<uvgvolucap::ThreadQueue> thread_queue;

            // Get depth image size for Color2Depth
            int target_viewpoint_width = 0;                                              // Depending on the transformation mode, this could be the width of color or depth image
            int target_viewpoint_height = 0;                                             // Depending on the transformation mode, this could be the height of color or depth image

            std::shared_ptr<std::thread> capture_thread_ptr;                             // Capture thread pointer
            std::function<void()> capture_function;                                      // Capture function
            std::shared_ptr<camera::SyncManager> sync_manager = nullptr;                 // Pointer to the universal sync manager which is declear from the PointCloudFactory class
            std::shared_ptr<geometry::Grid> grid_ptr = nullptr;                          // Pointer to the grid object for converting the point cloud to the grid world

        /* ####################################################### */
        
        protected:
            /**
             * @brief Setup the device configuration.
             * @details Different cameras have different configurations. So, providing a virtual method to 
             * setup the device configuration based on the camera API.
             */  
            virtual void init(Args... args) const = 0;    

            /**
             * @brief Open the device.
             * @details Different cameras have different open methods. So, providing a virtual method to open
             * the device based on the camera API.
             */  
            virtual void open() = 0;

            /**
             * @brief Close the device.
             * @details Different cameras have different close methods. So, providing a virtual method to close
             * the device based on the camera API.
             */
            virtual void close() = 0;
            
            /**
             * @brief Start the device.
             * @details Different cameras have different start methods. So, providing a virtual method to start
             * the device based on the camera API.
             */
            virtual void start() = 0;

        public:
            BasedCamera() = default;
            ~BasedCamera() = default;

            /**
             * @brief Warm up the device.
             * @details Depending on the camera, the warm up process can be different.
             */
            virtual void warm_up() = 0;

            /**
             * @brief Stop the device.
             * @details Different cameras have different stop methods. So, providing a virtual method to stop
             * the device based on the camera API.
             */
            virtual void stop() = 0;
        };

        /**
         * @brief Class for frame.
         * @details This class is the data structure for the pointcloud factory. 
         * We can use this frame for generating the point cloud from the depth and color images in 2 modes: normal and voxelized.
         */
        class BasedFrame {
        protected:
            int id = 0; /**< Frame ID */
            std::shared_ptr<geometry::PclFragment> fragment_pcl = std::make_shared<geometry::PclFragment>(); /**< Point cloud */
            std::mutex subspace_mx;
            std::shared_ptr<std::vector<std::shared_ptr<geometry::PclFragment>>> subspace_fragments = std::make_shared<std::vector<std::shared_ptr<geometry::PclFragment>>>();

            int min_bound[3] = {0, 0 ,0};
            int max_bound[3] = {0, 0, 0};
            int number_of_slices = 8;

        public:
            /**
             * @brief Constructor for Data_package.
             */
            BasedFrame(int _id, int min_bound_[3], int max_bound_[3], int number_of_slices_);

            /**
             * @brief Destruction for Data_package.
             */
            ~BasedFrame();

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

    } // namespace camera
} // namespace uvgvolucap


#endif // BASED_CAMERA_HPP