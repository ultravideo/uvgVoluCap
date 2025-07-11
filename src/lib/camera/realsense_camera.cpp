#include "realsense_camera.hpp"
#include "camera_utilities.hpp"

namespace uvgvolucap {
    namespace camera {
        Realsense_Frame::Realsense_Frame(int _id, rs2::depth_frame depth_, rs2::video_frame color_, int min_bound_[3], int max_bound_[3], int number_of_slices_)
            :   BasedFrame(_id, min_bound_, max_bound_, number_of_slices_),
                depth_frame(depth_), 
                color_frame(color_) {
            for (size_t i = 0; i < number_of_slices; i++)
            {
                std::shared_ptr<geometry::PclFragment> subspace_slice = std::make_shared<geometry::PclFragment>();
                subspace_slice->set_min_bound(min_bound[0], i * (max_bound[1] - min_bound[1]) / number_of_slices, min_bound[2]);
                subspace_slice->set_max_bound(max_bound[0], (i + 1) * (max_bound[1] - min_bound[1]) / number_of_slices, max_bound[2]);
                subspace_fragments->push_back(subspace_slice);
            }
        }

        rs2::depth_frame Realsense_Frame::get_depth_frame() {
            return depth_frame;
        }

        rs2::video_frame Realsense_Frame::get_color_frame() {
            return color_frame;
        }

        void Realsense_Frame::set_depth_frame(rs2::depth_frame depth_) {
            depth_frame = depth_;
        }

        void Realsense_Frame::set_color_frame(rs2::video_frame color_) {
            color_frame = color_;
        }

        rs2::pointcloud Realsense_Frame::get_rs_pointcloud() {
            return frame_pointcloud;
        }

        void Realsense_Frame::transform_view_point() {
            frame_pointcloud.map_to(color_frame);
        }

        /* ############################################################################################## */    

        Realsense::Realsense(uint32_t _index, uint32_t _sync_index, std::string _serial, nlohmann::json _config) : BasedCamera()
        {
            init(_index, _sync_index, _serial, _config);
            setup_device_config();

            auto grid_attribute = _config["grid"];
            grid_ptr = std::make_shared<geometry::Grid>(grid_attribute["geometry_precision"].get<int>());
            grid_ptr->set_real_world_params(device_info.filter_config.max_xy, device_info.filter_config.min_xy, device_info.filter_config.max_z, device_info.filter_config.min_z);
        }

        Realsense::~Realsense()
        {
            if (capture_thread_ptr->joinable())
            {
                capture_thread_ptr->join();
            }
            m_pipeline.stop();
        }

        void Realsense::init(uint32_t _index, uint32_t _sync_index, std::string _serial, nlohmann::json _config) const
        {
            device_info.index = _index;
            device_info.sync_index = _sync_index;
            device_info.serial_number = _serial;

            device_info.system_config.color_resolution = _config["setting"]["realsense"]["color_resolution"].get<int>();
            device_info.system_config.depth_resolution = _config["setting"]["realsense"]["depth_resolution"].get<int>();
            device_info.system_config.fps = _config["setting"]["realsense"]["fps"].get<int>();
            device_info.system_config.depth_to_color = _config["setting"]["depth_to_color"].get<bool>();
            device_info.system_config.voxelized = _config["setting"]["voxelized"].get<bool>();
            device_info.system_config.voxelized_mode = _config["setting"]["voxelized_mode"].get<int>();
            device_info.system_config.subsample_row = _config["setting"]["subsample_row"].get<int>();
            device_info.system_config.subsample_col = _config["setting"]["subsample_col"].get<int>();

            device_info.filter_config.max_xy = _config["filter"]["max_xy"].get<float>();
            device_info.filter_config.min_xy = _config["filter"]["min_xy"].get<float>();
            device_info.filter_config.max_z = _config["filter"]["max_z"].get<float>();
            device_info.filter_config.min_z = _config["filter"]["min_z"].get<float>();

            int precision = _config["grid"]["geometry_precision"].get<int>();
            device_info.pointcloud_config.geometry_precision = static_cast<size_t>(std::pow(2, precision - 1));
            device_info.pointcloud_config.min_bound[0] = _config["grid"]["min_bound"]["x"].get<int>();
            device_info.pointcloud_config.min_bound[1] = _config["grid"]["min_bound"]["y"].get<int>();
            device_info.pointcloud_config.min_bound[2] = _config["grid"]["min_bound"]["z"].get<int>();
            device_info.pointcloud_config.max_bound[0] = _config["grid"]["max_bound"]["x"].get<int>();
            device_info.pointcloud_config.max_bound[1] = _config["grid"]["max_bound"]["y"].get<int>();
            device_info.pointcloud_config.max_bound[2] = _config["grid"]["max_bound"]["z"].get<int>();
            device_info.pointcloud_config.number_of_slices = _config["grid"]["number_of_slices"].get<int>();

            auto device_attribute = _config["devices_config"].find(device_info.serial_number);
            device_info.roi.start_x = device_attribute->at("ROI").at("start_x").get<size_t>();
            device_info.roi.start_y = device_attribute->at("ROI").at("start_y").get<size_t>();
            device_info.roi.width = device_attribute->at("ROI").at("width").get<size_t>();
            device_info.roi.height = device_attribute->at("ROI").at("height").get<size_t>();

            for (size_t i = 0; i < device_info.transformation_matrix.size(); i++)
            {
                device_info.transformation_matrix[i] = device_attribute->at("coord_transform").at(std::to_string(i)).get<float>();
            }
        }

        void Realsense::setup_device_config() {
            m_config.disable_all_streams();
            rs2_depth_mode_t depth_mode = utils_rs2::get_rs2_depth_mode(device_info.system_config.depth_resolution);
            rs2_color_mode_t color_mode = utils_rs2::get_rs2_color_format(device_info.system_config.color_resolution);
            rs2_fps_t fps = utils_rs2::get_rs2_fps(device_info.system_config.fps);
            m_config.enable_stream(RS2_STREAM_DEPTH, depth_mode.width, depth_mode.height, depth_mode.format, fps);
            m_config.enable_stream(RS2_STREAM_COLOR, color_mode.width, color_mode.height, color_mode.format, fps);
            m_config.enable_device(device_info.serial_number);

            // Setup the filters
            // m_threshold_filter.set_option(RS2_OPTION_MIN_DISTANCE, device_info.filter_config.min_z);
            // m_threshold_filter.set_option(RS2_OPTION_MAX_DISTANCE, device_info.filter_config.max_z);
            // m_spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
            // m_spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
            // m_spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
            // m_spatial_filter.set_option(RS2_OPTION_HOLES_FILL, 2.0f);

            // m_decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
            // m_temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
        }

        void Realsense::start() {
            if (!is_started_flag){
                m_pipeline.start(m_config);
                is_started_flag = true;
            }
            else {
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Device is already started\n");
                return;
            }
        }

        void Realsense::open() {
            
        }

        void Realsense::close() {
            
        }

        void Realsense::stop() {
            if (is_started_flag){
                m_pipeline.stop();
                is_started_flag = false;
                sync_manager->Cap_permission_cv.notify_all();
            }
            else {
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Device is not started\n");
                return;
            }
        }

        void Realsense::warm_up() {
            if (!is_started_flag){
                m_pipeline.start(m_config);
                is_started_flag = true;
            }
            else {
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Device is already started\n");
                return;
            }

            // Warm up the device
            int try_count = 0;
            bool warmup_success = false;
            int warmup_frame_count = 0;
            while (true) {
                rs2::frameset frames = m_pipeline.wait_for_frames();
                rs2::depth_frame depth_frame = frames.get_depth_frame();
                rs2::video_frame color_frame = frames.get_color_frame();
                warmup_frame_count++;
                if (depth_frame && color_frame && warmup_frame_count > 31)
                {
                    warmup_success = true;
                    break;
                }
                else {
                    // try_count++;
                    // if (try_count > 5) {
                    //     break;
                    // }
                }
            }

            if (!warmup_success)
            {
                stop();
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Fail to get first capture\n");
                exit(EXIT_FAILURE);
            }
            Logger::log(LogLevel::INFO, device_info.serial_number, "Device is ready\n");
        }

        std::string Realsense::get_serial_number()
        {
            return device_info.serial_number;
        }

        void Realsense::start_capture(std::shared_ptr<ThreadQueue> _thread_queue, std::shared_ptr<SyncManager> _sync_manager)
        {
            thread_queue = _thread_queue;
            sync_manager = _sync_manager;
            capture_function = [this]()
            {
                if (!device_info.system_config.voxelized) {
                    this->pointcloud_production_line();
                }
                else {
                    // switch (get_voxelizer_mode(device_info.system_config.voxelized_mode))
                    // {
                    // case VOXELIZER_SUBSPACE:
                        this->pointcloud_production_line_with_subsapce();
                        // break;
                    
                    // default:
                    //     Logger::log(LogLevel::ERROR, device_info.serial_number, "Invalid voxelizer mode\n");
                    //     exit(EXIT_FAILURE);
                    //     break;
                    // } 
                }
            };
            capture_thread_ptr = std::make_shared<std::thread>(capture_function);
        }

        void Realsense::transform_view_point(std::shared_ptr<Realsense_Frame> frame) {
            frame->transform_view_point();
        }

        void Realsense::process_frame(std::shared_ptr<Realsense_Frame> frame) {
            // Process the frame
            rs2::depth_frame depth_frame = frame->get_depth_frame();
            rs2::video_frame color_frame = frame->get_color_frame();
            
            auto points = frame->get_rs_pointcloud().calculate(depth_frame);
            auto texture_coordinates = points.get_texture_coordinates();
            auto vertices = points.get_vertices();

            const int texture_width  = color_frame.get_width();
            const int texture_height = color_frame.get_height();
            const int texture_x_step = color_frame.get_bytes_per_pixel();
            const int texture_y_step = color_frame.get_stride_in_bytes();

            const unsigned char *texture_data = (unsigned char*)color_frame.get_data();

            for (size_t i = 0; i < points.size(); i++)
            {
                if (vertices[i].z)
                {
                    float u = texture_coordinates[i].u;
                    float v = texture_coordinates[i].v;

                    int texture_x = int(0.5 + u*texture_width);
                    int texture_y = int(0.5 + v*texture_height);

                    // Unsure whether this ever happens: out-of-bounds u/v points are skipped
                    if (texture_x <= 0 || texture_x >= texture_width-1) {
                        continue;
                    }

                    if (texture_y <= 0 || texture_y >= texture_height-1) {
                        continue;
                    }

                    auto x_o = vertices[i].x;
                    auto y_o = vertices[i].y;
                    auto z_o = vertices[i].z;

                    float x = (device_info.transformation_matrix[0] * x_o + device_info.transformation_matrix[1] * y_o + device_info.transformation_matrix[2] * z_o + device_info.transformation_matrix[3]);
                    float y = (device_info.transformation_matrix[4] * x_o + device_info.transformation_matrix[5] * y_o + device_info.transformation_matrix[6] * z_o + device_info.transformation_matrix[7]);
                    float z = (device_info.transformation_matrix[8] * x_o + device_info.transformation_matrix[9] * y_o + device_info.transformation_matrix[10] * z_o + device_info.transformation_matrix[11]);

                    if (z < device_info.filter_config.min_z || z > device_info.filter_config.max_z) {
                        continue;
                    }

                    int idx = texture_x * texture_x_step + texture_y * texture_y_step;
                    uint8_t b =  texture_data[idx];
                    uint8_t g =  texture_data[idx + 1];
                    uint8_t r =  texture_data[idx + 2];
                    frame->get_frame_pointcloud()->add_point(x, y, z, r, g, b);
                }
            }
        }

        void Realsense::process_frame_voxel_subspace(std::shared_ptr<Realsense_Frame> frame) {
            // Process the frame
            rs2::depth_frame depth_frame = frame->get_depth_frame();
            rs2::video_frame color_frame = frame->get_color_frame();
            
            auto points = frame->get_rs_pointcloud().calculate(depth_frame);
            auto texture_coordinates = points.get_texture_coordinates();
            auto vertices = points.get_vertices();

            const int texture_width  = color_frame.get_width();
            const int texture_height = color_frame.get_height();
            const int texture_x_step = color_frame.get_bytes_per_pixel();
            const int texture_y_step = color_frame.get_stride_in_bytes();

            const unsigned char *texture_data = (unsigned char*)color_frame.get_data();

            for (size_t i = 0; i < points.size(); i++)
            {
                if (vertices[i].z)
                {
                    float u = texture_coordinates[i].u;
                    float v = texture_coordinates[i].v;

                    int texture_x = int(0.5 + u*texture_width);
                    int texture_y = int(0.5 + v*texture_height);

                    // Unsure whether this ever happens: out-of-bounds u/v points are skipped
                    if (texture_x <= 0 || texture_x >= texture_width-1) {
                        continue;
                    }

                    if (texture_y <= 0 || texture_y >= texture_height-1) {
                        continue;
                    }
                    
                    auto x_o = vertices[i].x;
                    auto y_o = vertices[i].y;
                    auto z_o = vertices[i].z;

                    float x = (device_info.transformation_matrix[0] * x_o + device_info.transformation_matrix[1] * y_o + device_info.transformation_matrix[2] * z_o + device_info.transformation_matrix[3]);
                    float y = (device_info.transformation_matrix[4] * x_o + device_info.transformation_matrix[5] * y_o + device_info.transformation_matrix[6] * z_o + device_info.transformation_matrix[7]);
                    float z = (device_info.transformation_matrix[8] * x_o + device_info.transformation_matrix[9] * y_o + device_info.transformation_matrix[10] * z_o + device_info.transformation_matrix[11]);

                    if (x > device_info.filter_config.min_z && x < device_info.filter_config.max_z &&
                        y > device_info.filter_config.min_xy && y < device_info.filter_config.max_xy &&
                        z > device_info.filter_config.min_z && z < device_info.filter_config.max_z)
                    {

                        glm::vec3 grid_point = grid_ptr->real_to_grid(x, y, z);

                        size_t idx = static_cast<size_t>(grid_point.y) / (frame->get_max_bound(1)/(frame->get_number_of_slices()));

                        if (idx < frame->get_subspace_fragments()->size())
                        {
                            int color_idx = texture_x * texture_x_step + texture_y * texture_y_step;
                            uint8_t r =  texture_data[color_idx];
                            uint8_t g =  texture_data[color_idx + 1];
                            uint8_t b =  texture_data[color_idx + 2];
                            frame->get_subspace_fragments()->at(idx)->add_point_subspace(grid_point.x, grid_point.y, grid_point.z, r, g, b, grid_ptr->get_grid_origin());   
                        }
                    }
                }
            }

            for (size_t i = 0; i < frame->get_subspace_fragments()->size(); i++)
            {
                frame->get_subspace_fragments()->at(i)->finallized();
            }

        }
        
        void Realsense::pack_fragment(std::shared_ptr<geometry::PclFragment> fragment_pcl, std::shared_ptr<geometry::MergeBufferPointCloud> _asisgned_merge_buffer) {
            if (fragment_pcl->prep_to_merge_buffer(_asisgned_merge_buffer)) {
                fragment_pcl->copy_to_merge_buffer();
            }
        }

        void Realsense::pointcloud_production_line() {
            if (sync_manager == nullptr)
            {
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Sync manager is not assigned\n");
                return;
            }

            int frame_count = 0;
            rs2::frameset frames;
            std::unique_lock<std::mutex> lock(sync_manager->sync_mx);

            warm_up();
            (*sync_manager->update_device_ready_fptr)(device_info.sync_index, false);

            while (is_started_flag)
            {
                sync_manager->Cap_permission_cv.wait(lock, [&]()
                {
                    return (((*sync_manager->get_num_cap_cam_fptr)() & (1 << device_info.sync_index)) != 0) || !is_started_flag;
                });

                if (!is_started_flag) {
                    break;
                }

                frames = m_pipeline.wait_for_frames(CAPTURE_TIMEOUT);
                if (frames && is_started_flag ) {
                    rs2::depth_frame depth_frame = frames.get_depth_frame();
                    rs2::video_frame color_frame = frames.get_color_frame();

                    std::shared_ptr<Realsense_Frame> frame = std::make_shared<Realsense_Frame>(frame_count, depth_frame, color_frame, device_info.pointcloud_config.min_bound, device_info.pointcloud_config.max_bound, device_info.pointcloud_config.number_of_slices);

                    std::shared_ptr<geometry::MergeBufferPointCloud> assigned_merge_buffer = sync_manager->m_merge_buffer;
                    auto transf_vp_job = std::make_shared<uvgvolucap::Job>("TransformViewPoint", 0, &Realsense::transform_view_point, this, frame);
                    auto process_frame_job = std::make_shared<uvgvolucap::Job>("ProcessFrame", 1, &Realsense::process_frame, this, frame);
                    auto pack_fragment_job = std::make_shared<uvgvolucap::Job>("PackFragment", 2, &Realsense::pack_fragment, this, frame->get_frame_pointcloud(), assigned_merge_buffer);
                    
                    process_frame_job->addDependency(transf_vp_job);
                    pack_fragment_job->addDependency(process_frame_job);
                    sync_manager->_job->addDependency(pack_fragment_job);

                    thread_queue->submitJob(transf_vp_job);
                    thread_queue->submitJob(process_frame_job);
                    thread_queue->submitJob(pack_fragment_job);

                    (*sync_manager->update_device_ready_fptr)(device_info.sync_index, false);
                    continue; 
                } else {
                    if (is_started_flag)
                    {
                        stop();
                        Logger::log(LogLevel::ERROR, device_info.serial_number, "Fail to get capture\n");
                        exit(EXIT_FAILURE);
                    } else {
                        break;
                    }
                }
            }
            Logger::log(LogLevel::INFO, device_info.serial_number, "Capture thread is stopped\n");
        }

        void Realsense::pointcloud_production_line_with_subsapce() {
            if (sync_manager == nullptr)
            {
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Sync manager is not assigned\n");
                return;
            }

            int frame_count = 0;
            rs2::frameset frames;
            std::unique_lock<std::mutex> lock(sync_manager->sync_mx);

            warm_up();
            (*sync_manager->update_device_ready_fptr)(device_info.sync_index, false);

            while (is_started_flag)
            {
                sync_manager->Cap_permission_cv.wait(lock, [&]()
                {
                    return (((*sync_manager->get_num_cap_cam_fptr)() & (1 << device_info.sync_index)) != 0) || !is_started_flag;
                });

                if (!is_started_flag) {
                    break;
                }

                frames = m_pipeline.wait_for_frames(CAPTURE_TIMEOUT);
                if (frames && is_started_flag ) {
                    (*sync_manager->update_device_capture_fptr)(device_info.sync_index, false);
                    rs2::depth_frame depth_frame = frames.get_depth_frame();
                    rs2::video_frame color_frame = frames.get_color_frame();

                    std::shared_ptr<Realsense_Frame> frame = std::make_shared<Realsense_Frame>( frame_count, 
                                                                                                depth_frame, 
                                                                                                color_frame, 
                                                                                                device_info.pointcloud_config.min_bound, 
                                                                                                device_info.pointcloud_config.max_bound, 
                                                                                                device_info.pointcloud_config.number_of_slices
                                                                                                );

                    std::shared_ptr<geometry::MergeBufferPointCloud> assigned_merge_buffer = sync_manager->m_merge_buffer;
                    auto transf_vp_job = std::make_shared<uvgvolucap::Job>("TransformViewPoint", 0, &Realsense::transform_view_point, this, frame);
                    auto process_frame_job = std::make_shared<uvgvolucap::Job>("ProcessFrame", 1, &Realsense::process_frame_voxel_subspace, this, frame);
                    process_frame_job->addDependency(transf_vp_job);

                    for (size_t i = 0; i < frame->get_number_of_slices(); i++) {
                        sync_manager->m_merge_buffer->slice_components->at(device_info.sync_index)->at(i) = frame->get_subspace_fragments()->at(i);
                    }

                    sync_manager->_job->addDependency(process_frame_job);
                    thread_queue->submitJob(transf_vp_job);
                    thread_queue->submitJob(process_frame_job);

                    (*sync_manager->update_device_ready_fptr)(device_info.sync_index, false);
                    continue;
                } else {
                    if (is_started_flag)
                    {
                        stop();
                        Logger::log(LogLevel::ERROR, device_info.serial_number, "Fail to get capture\n");
                        exit(EXIT_FAILURE);
                    } else {
                        break;
                    }
                }

            } 
        }
    }	// namespace camera
}   // namespace uvgvolucap