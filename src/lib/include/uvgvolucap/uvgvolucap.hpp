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
    namespace core {
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

            void execute_sync_with_voxelize();
            void execute_sync();
            void set_voxelization_mode(bool mode);
            void set_sync_limit(size_t total_cams);
            void update_device_ready(int camera_index, bool reset);
            void update_device_capture(int camera_index, bool reset);
            void set_zmq_address(std::string i_color_address, std::string i_position_address);
            int get_num_ready_cam();
            int get_num_cap_cam();

            template <typename Func, typename... Args>
            void start_producing(Func&& func, Args&&... args);

            std::shared_ptr<camera::SyncManager> get_sync_manager();
        
        private:
            void pack_data(std::shared_ptr<geometry::PclFragment> fragment_pcl, std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer);
        };
    }

    namespace API {
        struct input_config {
            std::string config_path;
            std::string color_address;
            std::string position_address;
        };

        void run(std::string config_path, std::string color_address, std::string position_address);
    }
}

#endif // UVGVOLUCAP_HPP