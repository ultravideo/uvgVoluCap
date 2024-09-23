#pragma once
#ifndef UVGVOLUCAP_HPP
#define UVGVOLUCAP_HPP

#include "log.hpp"
#include "threadqueue.hpp"
#include "camera/kinect_utilities.hpp"
#include "camera/debug_macro.hpp"
#include <zmq.hpp>

#define RESET 99

namespace uvgvolucap {
    namespace API {
        enum CameraType {
            KINECT,
            REALSENSE
        };

        class PointCloudFactory {
        private:
            std::shared_ptr<uvgvolucap::ThreadQueue> thread_queue = std::make_shared<uvgvolucap::ThreadQueue>(40);
            size_t total_cams = 0;
            int gen_ID = 0;
            int limit = 0;
            int ready_Cam = 0; /**< The number of ready cameras. */
			int cap_Cam = 0; /**< The number of cameras being captured. */
            std::condition_variable main_cv; /**< Condition variable for synchronization. */
            std::shared_ptr<camera::SyncManager> sync_manager_handler = std::make_shared<camera::SyncManager>();
            std::mutex readyCam_mx; /**< Mutex to synchronize access to ready_Cam.  - internal usage */
            std::mutex capcam_mx; /**< Mutex to synchronize access to ready_Cam. - internal usage */
            
            bool stop_flag = false;
            bool is_voxelize_mode = false;
            std::string disconnet_msg = "DISCONNECT";
            std::string color_address = "";
            std::string position_address = "";

        public:
            PointCloudFactory();
            ~PointCloudFactory();

            /**
             * @brief Execute the point cloud factory.
             * @details This function is used to execute the point cloud factory including the voxelization process.
             */
            void execute_sync_with_voxelize();

            /**
             * @brief Execute the point cloud factory.
             * @details This function is used to execute the point cloud factory without the voxelization process.
             */
            void execute_sync();

            /**
             * @brief Set the voxelization mode.
             * @details This function is used to set the voxelization mode.
             * 
             * @param mode The boolean value for the voxelization mode.
             */
            void set_voxelization_mode(bool mode);

            /**
             * @brief Set the sync limit.
             * @details This function is used to set the sync limit.
             * 
             * @param total_cams The total number of cameras.
             */
            void set_sync_limit(size_t total_cams);

            /**
             * @brief Stop the point cloud factory.
             * @details This function is used for camera thread to notify synchronization manager that they are ready for the next frame.
             */
            void update_device_ready(int camera_index, bool reset);

            /**
             * @brief Update the device capture.
             * @details This function is for camera thread to notify synchronization manager that the current frame is captured.
             * 
             * @param camera_index The index of the camera.
             * @param reset The boolean value for reset.
             */
            void update_device_capture(int camera_index, bool reset);

            /**
             * @brief Set the zmq address.
             * @details This function is used to set the zmq address for sending the color and position data to encoder/visualizer.
             * 
             * @param i_color_address The color address.
             * @param i_position_address The position address.
             */
            void set_zmq_address(std::string i_color_address, std::string i_position_address);

            /**
             * @brief Get the number of ready cameras.
             * @details This function is used to get the number of ready cameras.
             * 
             * @return int The number of ready cameras.
             */
            int get_num_ready_cam();

            /**
             * @brief Get the number of captured cameras.
             * @details This function is used to get the number of captured cameras that are connected to the PC.
             * 
             * @return int The number of captured cameras.
             */
            int get_num_cap_cam();

            /**
             * @brief Start producing.
             * @details This function is used to start producing the point cloud.
             * 
             * @tparam Func The function.
             * @tparam Args The arguments.
             * @param func The function.
             * @param args The arguments.
             */
            template <typename Func, typename... Args>
            void start_producing(Func&& func, Args&&... args);

            /**
             * @brief Get the sync manager.
             * @details This function is used to get the sync manager for working in camera threads.
             * 
             * @return std::shared_ptr<camera::SyncManager> The sync manager.
             */
            std::shared_ptr<camera::SyncManager> get_sync_manager();
        
        private:
            /**
             * @brief Pack data.
             * @details This function is used to pack the data. This is the function job for the thread queue to gather the data from 
             * the fragment point cloud and merge buffer point cloud.
             * 
             * @param fragment_pcl The fragment point cloud.
             * @param m_merge_buffer The merge buffer point cloud.
             */
            void pack_data(std::shared_ptr<geometry::PclFragment> fragment_pcl, std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer);
        
            /**
             * @brief Voxelize data.
             * @details This function is used to voxelize the data. This is the function job for the thread queue to voxelize the data from 
             * the fragment point cloud and merge buffer point cloud.
             * 
             * @param m_merge_buffer The merge buffer point cloud.
             * @param slice_index The index of the slice.
             */
            void voxelize_data(std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer, int slice_index);
        };

        /**
         * @brief Input configuration.
         * @details This struct contains the input configuration for the point cloud factory.
         */
        struct input_config {
            std::string config_path;
            std::string color_address;
            std::string position_address;
        };

        /**
         * @brief Setup configuration.
         * @details This struct contains the setup configuration for the point cloud factory.
         */
        struct setup_config {
            std::shared_ptr<std::vector<uvgvolucap::camera::_kinect_device_ptr>> k4a_devices = nullptr;
            bool is_voxelized = false;
        };

        /**
         * @brief Setup the connected devices.
         * @details This function is used to setup the connected devices based on the configuration file.
         * 
         * @param config The input configuration.
         * @param setup_config The setup configuration.
         */
        void setup_k4a_devices(input_config &config, setup_config &setup_config);

        /**
         * @brief Run the point cloud factory.
         * @details This function is used to run the point cloud factory from the provided configuration file and zmq addresses from user.
         * 
         * @param config_path The path of the configuration file.
         * @param color_address The color address.
         * @param position_address The position address.
         */
        void k4a_run(input_config &config, setup_config &setup_config);
    }
}

#endif // UVGVOLUCAP_HPP