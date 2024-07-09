#include "uvgvolucap/uvgvolucap.hpp"
#include <unordered_map>

namespace uvgvolucap {
    namespace core {
        PointCloudFactory::PointCloudFactory() {
            sync_manager_handler->update_device_ready_fptr.reset(new std::function<void(int, bool)>(std::bind(&PointCloudFactory::update_device_ready, this, std::placeholders::_1, std::placeholders::_2)));
            sync_manager_handler->update_device_capture_fptr.reset(new std::function<void(int, bool)>(std::bind(&PointCloudFactory::update_device_capture, this, std::placeholders::_1, std::placeholders::_2)));
            sync_manager_handler->get_num_ready_cam_fptr.reset(new std::function<int()>(std::bind(&PointCloudFactory::get_num_ready_cam, this)));
            sync_manager_handler->get_num_cap_cam_fptr.reset(new std::function<int()>(std::bind(&PointCloudFactory::get_num_cap_cam, this)));
        }

        PointCloudFactory::~PointCloudFactory() {
            
        }

        void PointCloudFactory::execute_sync() {
            zmq::context_t context{1};

            zmq::socket_t colorSocket(context, ZMQ_PUSH);
            colorSocket.connect("tcp://localhost:5555");

            zmq::socket_t positionSocket(context, ZMQ_PUSH);
            positionSocket.connect("tcp://localhost:5556");

            std::unordered_map<VoxelCoord, VoxelData, VoxelCoordHash> voxelMap;

            //Lamda function for sending data
            auto send_data = [&](std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer) {
                
                geometry::_points_vec3 tmp_positions;
                geometry::_attributes_vec3 tmp_attributes;

                voxelMap.clear();
                size_t index = 0;
                for (size_t i = 0; i < m_merge_buffer->curr_index; i++)
                {
                    VoxelCoord coord = m_merge_buffer->positions[i];
                    auto voxel = voxelMap.find(coord);
                    if (voxel == voxelMap.end()) {

                        tmp_positions.push_back(m_merge_buffer->positions[i]);
                        tmp_attributes.push_back(m_merge_buffer->attributes[i]);

                        VoxelData data = {index, 1};
                        voxelMap.insert({coord, data});
                        index++;
                    }
                    else {
                        // float avg_r = (m_merge_buffer->attributes[voxel->second.index].x * voxel->second.count + m_merge_buffer->attributes[i].x) / (voxel->second.count + 1);
                        // float avg_g = (m_merge_buffer->attributes[voxel->second.index].y * voxel->second.count + m_merge_buffer->attributes[i].y) / (voxel->second.count + 1);
                        // float avg_b = (m_merge_buffer->attributes[voxel->second.index].z * voxel->second.count + m_merge_buffer->attributes[i].z) / (voxel->second.count + 1);

                        // glm::vec3 avg_color = glm::vec3(avg_r, avg_g, avg_b);

                        // tmp_attributes[voxel->second.index] = avg_color;
                        // voxel->second.count++;
                    }
                }
                Logger::log(LogLevel::INFO, "VoxelMap", std::to_string(voxelMap.size()) + "\n");
                zmq_send(positionSocket, tmp_positions.data(), voxelMap.size() * sizeof(glm::vec3), 0);
                zmq_send(colorSocket, tmp_attributes.data(), voxelMap.size() * sizeof(glm::vec3), 0);

                // zmq_send(colorSocket, m_merge_buffer->attributes, m_merge_buffer->curr_index * sizeof(glm::vec3), 0);
                // zmq_send(positionSocket, m_merge_buffer->positions, m_merge_buffer->curr_index * sizeof(glm::vec3), 0);
                Logger::log(LogLevel::ERROR, "Sender Zmq", std::to_string(m_merge_buffer->curr_index) + "\n");
            };

            std::unique_lock<std::mutex> lock(sync_manager_handler->sync_mx);
            std::shared_ptr<uvgvolucap::Job> pre_send_job = nullptr;

            while (!stop_flag)
            {
                sync_manager_handler->m_merge_buffer = std::make_shared<geometry::MergeBufferPointCloud>(); 
                std::shared_ptr<uvgvolucap::Job> curr_send_job = std::make_shared<uvgvolucap::Job>("SendJob", 3, send_data, sync_manager_handler->m_merge_buffer);
                sync_manager_handler->send_job  = curr_send_job;
                main_cv.wait(lock, [&]
                        { return ready_Cam == limit; });

                update_device_ready(RESET, true);
                update_device_capture(RESET, true);

                sync_manager_handler->Cap_permission_cv.notify_all();

                main_cv.wait(lock, [&]
                        { return cap_Cam == 0; });

                thread_queue->submitJob(sync_manager_handler->send_job);
                sync_manager_handler->count_pcl++;
                sync_manager_handler->m_merge_buffer->id = static_cast<int>(sync_manager_handler->count_pcl);

                if (pre_send_job == nullptr)
                {
                    pre_send_job = curr_send_job;
                    continue;
                }
                curr_send_job->addDependency(pre_send_job);
                pre_send_job = curr_send_job;
            }
        }

        void PointCloudFactory::set_sync_limit(size_t total_cams) {
            limit = static_cast<int>((1 << total_cams) - 1);
        }

        void PointCloudFactory::update_device_ready(int camera_index, bool reset) {
            std::lock_guard<std::mutex> lock(readyCam_mx); // Lock the mutex for the duration of the function
            if (reset)
            {
                ready_Cam = 0;
            }
            else
            {
                ready_Cam += (1 << camera_index);
                main_cv.notify_one();
            }
        }

        void PointCloudFactory::update_device_capture(int camera_index, bool reset) {
            std::lock_guard<std::mutex> lock(capcam_mx); // Lock the mutex for the duration of the function
            if (reset)
            {
                cap_Cam = limit;
            }
            else
            {
                cap_Cam -= (1 << camera_index);
                main_cv.notify_one();
            }
        }

        int PointCloudFactory::get_num_ready_cam() {
            std::lock_guard<std::mutex> lock(readyCam_mx);
            return ready_Cam;
        }

        int PointCloudFactory::get_num_cap_cam() {
            std::lock_guard<std::mutex> lock(capcam_mx);
            return cap_Cam;
        }

        std::shared_ptr<camera::SyncManager> PointCloudFactory::get_sync_manager() {
            return sync_manager_handler;
        }

        template <typename Func, typename... Args>
        void PointCloudFactory::start_producing(Func&& func, Args&&... args) {
            std::function<void()> f = std::bind(std::forward<Func>(func), std::forward<Args>(args)..., thread_queue, sync_manager_handler);
            f();
            execute_sync();
        }
    }

    namespace API {
        void test() {
            std::cout << "Hello, World!" << std::endl;
            std::shared_ptr<std::vector<camera::_kinect_device_ptr>> devices = std::make_shared<std::vector<camera::_kinect_device_ptr>>();
            // bool init_success = camera::init_connected_device(devices, "C:/Users/Guillaume/workspace/uvgvolucap/asset/cameraconfig.json");

            bool init_success = camera::init_connected_device(devices, "C:/Users/Guillaume/workspace/Testing/ROI/cameraconfig.json");
  
  
            if (!init_success) { return; }

            core::PointCloudFactory factory;
            factory.set_sync_limit(devices->size()); 
            factory.start_producing(camera::start_capture, devices);
            std::cout << "Goodbye, World!" << std::endl;
        }
    } // namespace API
} // namespace uvgvolucap
