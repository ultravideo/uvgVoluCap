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

        void PointCloudFactory::pack_data(std::shared_ptr<geometry::PclFragment> fragment_pcl, std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer) {
            if (fragment_pcl->prep_to_merge_buffer(m_merge_buffer)) {
                fragment_pcl->copy_to_merge_buffer();
            }
        }

        void PointCloudFactory::set_zmq_address(std::string i_color_address, std::string i_position_address) {
            this->color_address = i_color_address;
            this->position_address = i_position_address;

            if (color_address.empty() || position_address.empty()) {
                Logger::log(LogLevel::ERROR, "ZMQ", "Please set color and address address\n");
                exit(EXIT_FAILURE);
            }
        }

        void PointCloudFactory::voxelize_data(std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer, int slice_index) {
            std::shared_ptr<geometry::PclFragment> subspace_slice = std::make_shared<geometry::PclFragment>();
            
            for (size_t i = 0; i < m_merge_buffer->slice_components->size(); i++) {
                for (size_t j = 0; j < m_merge_buffer->slice_components->at(i)->at(slice_index)->max_size(); j++) {
                    auto point = m_merge_buffer->slice_components->at(i)->at(slice_index)->get_position_by_index(j);
                    auto color = m_merge_buffer->slice_components->at(i)->at(slice_index)->get_attribute_by_index(j);
                    subspace_slice->voxlelize(point.x, point.y, point.z, color.x, color.y, color.z);
                }
                subspace_slice->finallized();
                m_merge_buffer->slice_components->at(i)->at(slice_index)->clear();
            }
            pack_data(subspace_slice, m_merge_buffer);
            subspace_slice->clear();
        }

        void PointCloudFactory::execute_sync_with_voxelize() {
            zmq::context_t context{1};

            zmq::socket_t colorSocket(context, ZMQ_PUSH);
            colorSocket.connect(color_address);

            zmq::socket_t positionSocket(context, ZMQ_PUSH);
            positionSocket.connect(position_address);

            size_t sent_frame_count = 0;

            auto dummy_funct = [&]() { return; };

            std::condition_variable disconnet_cv;
            auto send_data = [&](std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer) {
                //Send data
#ifdef SENDER_TIMER
                auto start_time = std::chrono::high_resolution_clock::now();
#endif

#ifdef POINT_UINT16
                zmq_send(positionSocket, m_merge_buffer->positions, m_merge_buffer->curr_index * sizeof(geometry::vect3u16), 0);
#else
                zmq_send(positionSocket, m_merge_buffer->positions, m_merge_buffer->curr_index * sizeof(glm::vec3), 0);
#endif

#ifdef COLOR_UCHAR
                zmq_send(colorSocket, m_merge_buffer->attributes, m_merge_buffer->curr_index * sizeof(geometry::vec3u8), 0);
#else
                zmq_send(colorSocket, m_merge_buffer->attributes, m_merge_buffer->curr_index * sizeof(glm::vec3), 0);
#endif

                sent_frame_count++;
                disconnet_cv.notify_one();

#ifdef FINAL_NUMBER_DEBUG
                Logger::log(LogLevel::INFO, "Pts_nb", "Sent: frame No." + std::to_string(sent_frame_count-1) + " - number: " + std::to_string(m_merge_buffer->curr_index) + "\n");
#endif

#ifdef SENDER_TIMER
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_time = end_time - start_time;
                Logger::log(LogLevel::INFO, "Sender Zmq", "Elapsed time: " + std::to_string(elapsed_time.count()) + "s\n");
#endif
                m_merge_buffer->curr_index = 0;
                m_merge_buffer->slice_components->clear();
                
            };

            std::unique_lock<std::mutex> lock(sync_manager_handler->sync_mx);
            std::shared_ptr<uvgvolucap::Job> pre_send_job = nullptr;
#ifdef FPS_MEASURE
            auto start_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double>  elapsed_time = start_time - start_time;
#endif
            while (!stop_flag)
            {
#ifdef FPS_MEASURE
                auto end_time = std::chrono::high_resolution_clock::now();
                elapsed_time = end_time - start_time;
                if (elapsed_time.count() > RUNNINT_TIME && !stop_flag)
                {
                    stop_flag = true;
                    break;
                }
#endif

                sync_manager_handler->m_merge_buffer = std::make_shared<geometry::MergeBufferPointCloud>(this->total_cams); 
                std::shared_ptr<uvgvolucap::Job> dummy_job = std::make_shared<uvgvolucap::Job>("DummyJob", 3, dummy_funct);
                std::shared_ptr<uvgvolucap::Job> curr_send_job = std::make_shared<uvgvolucap::Job>("SendJob", 5, send_data, sync_manager_handler->m_merge_buffer);

                std::vector<std::shared_ptr<uvgvolucap::Job>> voxel_jobs;
                for (int i = 0; i < sync_manager_handler->m_merge_buffer->step; i++) {
                    std::shared_ptr<uvgvolucap::Job> voxel_job = std::make_shared<uvgvolucap::Job>("VoxelJob", 4, &PointCloudFactory::voxelize_data, this, sync_manager_handler->m_merge_buffer, i);
                    voxel_job->addDependency(dummy_job);
                    curr_send_job->addDependency(voxel_job);
                    voxel_jobs.push_back(voxel_job);
                }

                sync_manager_handler->_job  = dummy_job;

                main_cv.wait(lock, [&]
                        { return ready_Cam == limit; });

                update_device_ready(RESET, true);
                update_device_capture(RESET, true);

                sync_manager_handler->count_pcl++;
                sync_manager_handler->Cap_permission_cv.notify_all();

                main_cv.wait(lock, [&]
                        { return cap_Cam == 0; });

                thread_queue->submitJob(dummy_job);
                for (auto& job : voxel_jobs) {
                    thread_queue->submitJob(job);
                }
                thread_queue->submitJob(curr_send_job);

                sync_manager_handler->m_merge_buffer->id = static_cast<int>(sync_manager_handler->count_pcl);

                if (pre_send_job == nullptr)
                {
                    pre_send_job = curr_send_job;
                    continue;
                }
                curr_send_job->addDependency(pre_send_job);
                pre_send_job = curr_send_job;
            }

            std::mutex disconnet_mx;
            std::unique_lock<std::mutex> disconnect_lock(disconnet_mx);
            disconnet_cv.wait(disconnect_lock, [&]
                    { return sync_manager_handler->count_pcl == sent_frame_count; });
            
            Logger::log(LogLevel::INFO, "System", "FPS: " + std::to_string((sent_frame_count-1) / elapsed_time.count()) + "\n");
            Logger::log(LogLevel::INFO, "System", "Created Total Frames: " + std::to_string(sync_manager_handler->count_pcl) + "\n");
            Logger::log(LogLevel::INFO, "System", "Sent Total Frames: " + std::to_string(sent_frame_count) + "\n");

            colorSocket.disconnect(color_address);      
            positionSocket.disconnect(position_address);
            colorSocket.close();
            positionSocket.close();
            context.close();
            Logger::log(LogLevel::INFO, "System", "Disconnected to hosts\n");
        }

        void PointCloudFactory::execute_sync() {
            zmq::context_t context{1};

            zmq::socket_t colorSocket(context, ZMQ_PUSH);
            colorSocket.connect(color_address);

            zmq::socket_t positionSocket(context, ZMQ_PUSH);
            positionSocket.connect(position_address);

            size_t sent_frame_count = 0;
            std::condition_variable disconnet_cv;

            auto send_data = [&](std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer) {
                //Send data
#ifdef SENDER_TIMER
                auto start_time = std::chrono::high_resolution_clock::now();
#endif
                zmq_send(colorSocket, m_merge_buffer->attributes, m_merge_buffer->curr_index * sizeof(glm::vec3), 0);
                zmq_send(positionSocket, m_merge_buffer->positions, m_merge_buffer->curr_index * sizeof(glm::vec3), 0);
                sent_frame_count++;
#ifdef FINAL_NUMBER_DEBUG
                Logger::log(LogLevel::INFO, "Pts_nb", "number: " + std::to_string(m_merge_buffer->curr_index) + "\n");
#endif

#ifdef SENDER_TIMER
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_time = end_time - start_time;
                Logger::log(LogLevel::INFO, "Sender Zmq", "Elapsed time: " + std::to_string(elapsed_time.count()) + "s\n");
#endif
                m_merge_buffer->curr_index = 0;
                m_merge_buffer->slice_components->clear();
                disconnet_cv.notify_one();
                
            };

            std::unique_lock<std::mutex> lock(sync_manager_handler->sync_mx);
            std::shared_ptr<uvgvolucap::Job> pre_send_job = nullptr;
#ifdef FPS_MEASURE
            auto start_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double>  elapsed_time = start_time - start_time;
#endif
            while (!stop_flag)
            {
#ifdef FPS_MEASURE
                auto end_time = std::chrono::high_resolution_clock::now();
                elapsed_time = end_time - start_time;
                if (elapsed_time.count() > RUNNINT_TIME)
                {
                    stop_flag = true;
                }
#endif
                sync_manager_handler->m_merge_buffer = std::make_shared<geometry::MergeBufferPointCloud>(this->total_cams); 
                std::shared_ptr<uvgvolucap::Job> curr_send_job = std::make_shared<uvgvolucap::Job>("SendJob", 3, send_data, sync_manager_handler->m_merge_buffer);
                sync_manager_handler->_job  = curr_send_job;


                main_cv.wait(lock, [&]
                        { return ready_Cam == limit; });

                update_device_ready(RESET, true);
                update_device_capture(RESET, true);

                sync_manager_handler->count_pcl++;
                sync_manager_handler->Cap_permission_cv.notify_all();

                main_cv.wait(lock, [&]
                        { return cap_Cam == 0; });

                thread_queue->submitJob(curr_send_job);

                sync_manager_handler->m_merge_buffer->id = static_cast<int>(sync_manager_handler->count_pcl);

                if (pre_send_job == nullptr)
                {
                    pre_send_job = curr_send_job;
                    continue;
                }
                curr_send_job->addDependency(pre_send_job);
                pre_send_job = curr_send_job;
            }

            std::mutex disconnet_mx;
            std::unique_lock<std::mutex> disconnect_lock(disconnet_mx);
            disconnet_cv.wait(disconnect_lock, [&]
                    { return sync_manager_handler->count_pcl == sent_frame_count; });
            
            Logger::log(LogLevel::INFO, "System", "FPS: " + std::to_string((sent_frame_count-1) / elapsed_time.count()) + "\n");
            Logger::log(LogLevel::INFO, "System", "Created Total Frames: " + std::to_string(sync_manager_handler->count_pcl) + "\n");
            Logger::log(LogLevel::INFO, "System", "Sent Total Frames: " + std::to_string(sent_frame_count) + "\n");

            colorSocket.disconnect(color_address);      
            positionSocket.disconnect(position_address);
            colorSocket.close();
            positionSocket.close();
            context.close();
            Logger::log(LogLevel::INFO, "System", "Disconnected to hosts\n");
        }

        void PointCloudFactory::set_sync_limit(size_t total_cams) {
            limit = static_cast<int>((1 << total_cams) - 1);
            this->total_cams = total_cams;
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
            if (is_voxelize_mode) {
                execute_sync_with_voxelize();
            }
            else {
                execute_sync();
            }
            thread_queue->stop();
        }

        void PointCloudFactory::set_voxelization_mode(bool mode) {
            is_voxelize_mode = mode;
        }
    }

    namespace API {
        void k4a_run(input_config config) {
            _kinect_device_ptr_vector k4a_devices = std::make_shared<std::vector<camera::_kinect_device_ptr>>();

            bool is_voxelized = false;
            bool init_success = camera::init_connected_k4a_device(k4a_devices, config.config_path, is_voxelized);  
            if (!init_success) { 
                Logger::log(LogLevel::ERROR, "INIT", "Initialization failed\n");
                return; 
            }

            core::PointCloudFactory factory;
            factory.set_zmq_address(config.color_address, config.position_address);
            factory.set_sync_limit(k4a_devices->size()); 
            factory.set_voxelization_mode(is_voxelized);
            factory.start_producing(camera::kinect_start_capture, k4a_devices);

            for (auto& kdevice : *k4a_devices) {
                kdevice->stop();
            }

            //Sleep for 2s to allow the device threads to finish
            std::this_thread::sleep_for(std::chrono::seconds(2));

            Logger::log(LogLevel::INFO, "APP", "CAPTURE FINISHED\n");
            exit(EXIT_SUCCESS);
        }
    } // namespace API
} // namespace uvgvolucap
