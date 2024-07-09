#pragma once
#ifndef UVGVOLUCAP_HPP
#define UVGVOLUCAP_HPP

#include "log.hpp"
#include "threadqueue.hpp"
#include "camera/kinect_utilities.hpp"

#include <zmq.hpp>

#define RESET 99

namespace uvgvolucap {
    namespace core {
        class PointCloudFactory {
        private:
            std::shared_ptr<uvgvolucap::ThreadQueue> thread_queue = std::make_shared<uvgvolucap::ThreadQueue>(20);

            int gen_ID = 0;
            int limit = 0;
            int ready_Cam = 0; /**< The number of ready cameras. */
			int cap_Cam = 0; /**< The number of cameras being captured. */
			// std::shared_ptr<std::mutex> sync_mx = std::make_shared<std::mutex>(); /**< Mutex for synchronization. -public usage */
            std::condition_variable main_cv; /**< Condition variable for synchronization. */
			// std::shared_ptr<std::condition_variable> Cap_permission_cv = std::make_shared<std::condition_variable>(); /**< Condition variable for synchronization. */
            std::shared_ptr<camera::SyncManager> sync_manager_handler = std::make_shared<camera::SyncManager>();
            // std::atomic<int> ready_Cam = 0; //test
            // std::atomic<int> cap_Cam = 0; // test
            std::mutex readyCam_mx; /**< Mutex to synchronize access to ready_Cam.  - internal usage */
            std::mutex capcam_mx; /**< Mutex to synchronize access to ready_Cam. - internal usage */
            
            bool stop_flag = false;


            /* Test */
            struct VoxelData {
                size_t index;
                int count;
            };
            using VoxelCoord = glm::vec3;
            // Define a hash function for VoxelCoord
            struct VoxelCoordHash {
                std::size_t operator()(const VoxelCoord& coord) const {
                    std::size_t hx = std::hash<std::size_t>()(static_cast<std::size_t>(coord.x));
                    std::size_t hy = std::hash<std::size_t>()(static_cast<std::size_t>(coord.y));
                    std::size_t hz = std::hash<std::size_t>()(static_cast<std::size_t>(coord.z));
                    return hx ^ (hy << 1) ^ (hz << 2);  // Combine the hashes
                }
            };

        public:
            PointCloudFactory();
            ~PointCloudFactory();

            void execute_sync();
            void set_sync_limit(size_t total_cams);
            void update_device_ready(int camera_index, bool reset);
            void update_device_capture(int camera_index, bool reset);
            int get_num_ready_cam();
            int get_num_cap_cam();
            // std::shared_ptr<std::condition_variable> get_Cap_permission_cv();
            // std::shared_ptr<std::mutex> get_sync_mx();

            template <typename Func, typename... Args>
            void start_producing(Func&& func, Args&&... args);

            std::shared_ptr<camera::SyncManager> get_sync_manager();
        
        private:

        };
    }

    namespace API {
        void test();
    }
}

#endif // UVGVOLUCAP_HPP