#include "kinect_camera.hpp"
#include "kinect_utilities.hpp"
#include "opencv2/opencv.hpp"

namespace uvgvolucap {
    namespace camera{
        Kinect::Kinect(uint32_t _index, std::string _serial, nlohmann::json _config) : BasedCamera()
        {
            init(_index, _serial, _config);
            setup_device_config();
        }

        void Kinect::init(uint32_t _index, std::string _serial, nlohmann::json _config) const
        {
            device_info.index = _index;
            device_info.serial_number = _serial;

            device_info.system_config.color_resolution = _config["setting"]["color_resolution"].get<int>();
            device_info.system_config.depth_resolution = _config["setting"]["depth_resolution"].get<int>();
            device_info.system_config.fps = _config["setting"]["fps"].get<int>();
            device_info.system_config.depth_to_color = _config["setting"]["depth_to_color"].get<bool>();
            device_info.system_config.max_size = _config["setting"]["max_size"].get<size_t>();

            device_info.filter_config.max_xy = _config["filter"]["max_xy"].get<float>();
            device_info.filter_config.min_xy = _config["filter"]["min_xy"].get<float>();
            device_info.filter_config.max_z = _config["filter"]["max_z"].get<float>();
            device_info.filter_config.min_z = _config["filter"]["min_z"].get<float>();

            auto device_attribute = _config["devices_config"].find(device_info.serial_number);
            device_info.roi.start_x = device_attribute->at("ROI").at("start_x").get<size_t>();
            device_info.roi.start_y = device_attribute->at("ROI").at("start_y").get<size_t>();
            device_info.roi.width = device_attribute->at("ROI").at("width").get<size_t>();
            device_info.roi.height = device_attribute->at("ROI").at("height").get<size_t>();

            for (int i = 0; i < device_info.transformation_matrix.size(); i++)
            {
                device_info.transformation_matrix[i] = device_attribute->at("coord_transform").at(std::to_string(i)).get<float>();
            }
        }

        void Kinect::setup_device_config()
        {
            m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
            m_config.color_resolution = get_color_resolution(device_info.system_config.color_resolution);
            m_config.depth_mode = get_depth_mode(device_info.system_config.depth_resolution);
            m_config.camera_fps = get_fps(device_info.system_config.fps);
            m_config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture
            m_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
        }

        void Kinect::open()
        {
            if (!is_opened_flag){
                k4a_device_open(device_info.index, &m_device);
                is_opened_flag = true;
            }
        }

        void Kinect::close()
        {
            if (is_opened_flag){
                k4a_device_close(m_device);
                is_opened_flag = false;
            }
        }

        void Kinect::start()
        {
            if (is_opened_flag && !is_started_flag){
                k4a_device_start_cameras(m_device, &m_config);
                is_started_flag = true;
            }
            else{
                throw std::runtime_error("Device is not opened yet");	
            }
        }

        void Kinect::stop()
        {
            if (is_started_flag){
                close();
                k4a_device_stop_cameras(m_device);
                is_started_flag = false;
            }
        }

        void Kinect::warm_up()
        {
            open();

            // Set color control
            k4a_device_set_color_control(m_device, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
            k4a_device_set_color_control(m_device, K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
            //the others control only have manual mode, if you need to add more control, you can add it here, always open after device opens

            if (k4a_device_get_calibration(m_device, m_config.depth_mode, m_config.color_resolution, &m_calibration) != K4A_RESULT_SUCCEEDED)
            {
                close();
                Logger::log(LogLevel::INFO, device_info.serial_number, "Fail to get calibration info\n");
            }

            transformation_handle = k4a_transformation_create(&m_calibration);
            if (!transformation_handle)
            {
                close();
                Logger::log(LogLevel::INFO, device_info.serial_number, "Fail to create transformation handle\n");
            }

            start();
            k4a_device_stop_imu(m_device); // We dont need IMU in our usecase.
            is_started_flag = true;

            // Startup phase - Check that devices can run normally.
            int try_count = 0;
            k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
            while (true) {
                k4a_capture_t capture;
                result = k4a_device_get_capture(m_device, &capture, 500);
                if (result == K4A_WAIT_RESULT_SUCCEEDED) {
                    k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
                    k4a_image_t color_image = k4a_capture_get_color_image(capture);

                    if (depth_image == NULL || color_image == NULL)
                    {
                        result = K4A_WAIT_RESULT_FAILED;
                    }

                    k4a_capture_release(capture);
                    k4a_image_release(depth_image);
                    k4a_image_release(color_image);
                    break;
                }
                else {
                    try_count++;
                    if (try_count > 5) {
                        break;
                    }
                }

            }
            if (result != K4A_WAIT_RESULT_SUCCEEDED)
            {
                stop();
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Fail to get first capture\n");
            }

            createXYTable(&m_calibration);
            Logger::log(LogLevel::INFO, device_info.serial_number, "Device is ready\n");
        }

        std::string Kinect::get_serial_number()
        {
            return device_info.serial_number;
        }

        void Kinect::createXYTable(const k4a_calibration_t *calibration)
        {
            if (device_info.system_config.depth_to_color)
            {
                target_viewpoint_width = calibration->color_camera_calibration.resolution_width;
                target_viewpoint_height = calibration->color_camera_calibration.resolution_height;
            }
            else
            {
                target_viewpoint_width = calibration->depth_camera_calibration.resolution_width;
                target_viewpoint_height = calibration->depth_camera_calibration.resolution_height;
            }

            k4a_result_t status;
            status = k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                      target_viewpoint_width,
                                      target_viewpoint_height,
                                      target_viewpoint_width * (int)sizeof(k4a_float2_t),
                                      &xy_table);
            if (status != K4A_RESULT_SUCCEEDED)
            {
                std::cerr << " Failed to create xy_table image: " << status << std::endl;
            }

            k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
            k4a_float2_t p;
            k4a_float3_t ray;
            int valid;

            for (int y = 0, idx = 0; y < target_viewpoint_height; y++)
            {
                p.xy.y = (float)y;
                for (int x = 0; x < target_viewpoint_width; x++, idx++)
                {
                    p.xy.x = (float)x;

                    if (device_info.system_config.depth_to_color)
                    {
                        k4a_calibration_2d_to_3d(
                            calibration, &p, 1.f, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ray, &valid);
                    }
                    else
                    {
                        k4a_calibration_2d_to_3d(
                            calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);
                    }

                    if (valid)
                    {
                        table_data[idx].xy.x = ray.xyz.x;
                        table_data[idx].xy.y = ray.xyz.y;
                    }
                    else
                    {
                        table_data[idx].xy.x = nanf("");
                        table_data[idx].xy.y = nanf("");
                    }
                }
            }
            xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
        }

        void Kinect::start_capture(std::shared_ptr<ThreadQueue> _thread_queue, std::shared_ptr<SyncManager> _sync_manager)
        {
            thread_queue = _thread_queue;
            sync_manager = _sync_manager;
            capture_function = [this]()
            {
                this->pointcloud_production_line();
            };
            capture_thread_ptr = std::make_shared<std::thread>(capture_function);
        }

        void Kinect::transform_view_point(std::shared_ptr<Frame> frame) {
            k4a_image_t transformed_image = NULL;
            if (device_info.system_config.depth_to_color)
            {
                k4a_result_t status = k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                       target_viewpoint_width,
                                                       target_viewpoint_height,
                                                       target_viewpoint_width * sizeof(uint16_t),
                                                       &transformed_image);

                if (status != K4A_RESULT_SUCCEEDED){
                    Logger::log(LogLevel::ERROR, device_info.serial_number, "Failed to create transformed image\n");                
                }

                status = k4a_transformation_depth_image_to_color_camera(transformation_handle,
                                                                        frame->depth_image,
                                                                        transformed_image);
                if (status != K4A_RESULT_SUCCEEDED){
                    Logger::log(LogLevel::ERROR, device_info.serial_number, "Failed to transform depth image to color camera\n");
                }


                k4a_image_release(frame->depth_image);
                frame->depth_image = transformed_image;
            }
            else
            {
                k4a_result_t status = k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                       target_viewpoint_width,
                                                       target_viewpoint_height,
                                                       target_viewpoint_width * sizeof(uint32_t),
                                                       &transformed_image);

                if (status != K4A_RESULT_SUCCEEDED){
                    Logger::log(LogLevel::ERROR, device_info.serial_number, "Failed to create transformed image\n");
                }

                status = k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                        frame->depth_image,
                                                                        frame->color_image,
                                                                        transformed_image);
                if (status != K4A_RESULT_SUCCEEDED){
                    Logger::log(LogLevel::ERROR, device_info.serial_number, "Failed to transform color image to depth camera\n");
                }

                k4a_image_release(frame->color_image);
                frame->color_image = transformed_image;
            }
        }

        void Kinect::process_frame(std::shared_ptr<Frame> frame) {
            int width = k4a_image_get_width_pixels(frame->depth_image);
            int height = k4a_image_get_height_pixels(frame->depth_image);

            uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(frame->depth_image);
            geometry::_bgra_t *color_data = (geometry::_bgra_t *)(void *)k4a_image_get_buffer(frame->color_image);

            for (size_t row = device_info.roi.start_y ; row < device_info.roi.start_y + device_info.roi.height; ++row) {
                for (size_t col = device_info.roi.start_x; col < device_info.roi.start_x + device_info.roi.width; ++col) {
                    size_t i = row * width + col;
                    if (depth_data[i] != 0 && !std::isnan(xy_table_data[i].xy.x) && !std::isnan(xy_table_data[i].xy.y)) // && depth_data[i] < 1702)
                    {
                        uint8_t b = color_data[i].bgra.b;
                        uint8_t g = color_data[i].bgra.g;
                        uint8_t r = color_data[i].bgra.r;

                        if (r == 0 && g == 0 && b == 0)
                        {
                            continue;
                        }

                        float z_o = static_cast<float>(depth_data[i]) / 1000.0f;
                        float x_o = static_cast<float>((xy_table_data[i].xy.x * z_o));
                        float y_o = static_cast<float>((xy_table_data[i].xy.y * z_o));

                        float x = (device_info.transformation_matrix[0] * x_o + device_info.transformation_matrix[1] * y_o + device_info.transformation_matrix[2] * z_o + device_info.transformation_matrix[3]);
                        float y = (device_info.transformation_matrix[4] * x_o + device_info.transformation_matrix[5] * y_o + device_info.transformation_matrix[6] * z_o + device_info.transformation_matrix[7]);
                        float z = (device_info.transformation_matrix[8] * x_o + device_info.transformation_matrix[9] * y_o + device_info.transformation_matrix[10] * z_o + device_info.transformation_matrix[11]);

                        // Filter by calculated radius
                        if (x > device_info.filter_config.min_xy && x < device_info.filter_config.max_xy &&
                            y > device_info.filter_config.min_xy && y < device_info.filter_config.max_xy &&
                            z > device_info.filter_config.min_z && z < device_info.filter_config.max_z)
                        {
                            frame->fragment_pcl->add_point(x, y, z, r, g, b);
                        }
                    }
                }
            }

            frame->fragment_pcl->finallized();
            k4a_image_release(frame->depth_image);
            k4a_image_release(frame->color_image);

            //print 5 points

        }

        void Kinect::pack_fragment(std::shared_ptr<geometry::PclFragment> fragment_pcl, std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer) {
            fragment_pcl->prep_to_merge_buffer(m_merge_buffer);
            fragment_pcl->copy_to_merge_buffer();
        }

        void Kinect::pointcloud_production_line() {
            if (sync_manager == nullptr)
            {
                Logger::log(LogLevel::ERROR, device_info.serial_number, "Sync manager is not assigned\n");
                return;
            }

            int frame_count = 0;
            k4a_wait_result_t result = K4A_WAIT_RESULT_TIMEOUT;
            std::unique_lock<std::mutex> lock(sync_manager->sync_mx);

            warm_up();
            (*sync_manager->update_device_ready_fptr)(device_info.index, false);

            while (is_started_flag)
            {
                sync_manager->Cap_permission_cv.wait(lock, [&]()
                {
                    return ((*sync_manager->get_num_cap_cam_fptr)() & (1 << device_info.index)) != 0;
                });

                k4a_capture_t capture;
                result = k4a_device_get_capture(m_device, &capture, 500);
                if (result == K4A_WAIT_RESULT_SUCCEEDED)
                {
                    (*sync_manager->update_device_capture_fptr)(device_info.index, false);
                    // Capture and process the frames
                    k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
                    k4a_image_t color_image = k4a_capture_get_color_image(capture);

                    // Form the frame
                    std::shared_ptr<Frame> frame = std::make_shared<Frame>( frame_count, 
                                                                            depth_image, 
                                                                            color_image
                                                                        );

                    auto transf_vp_job = std::make_shared<uvgvolucap::Job>("TransformViewPoint", 0, &Kinect::transform_view_point, this, frame);
                    auto process_frame_job = std::make_shared<uvgvolucap::Job>("ProcessFrame", 1, &Kinect::process_frame, this, frame);
                    // auto pack_fragment_job = std::make_shared<uvgvolucap::Job>("PackFragment", 2, &Kinect::pack_fragment, this, frame->fragment_pcl, m_merge_buffer);

                    process_frame_job->addDependency(transf_vp_job);
                    // pack_fragment_job->addDependency(process_frame_job);

                    //Missing the export job

                    thread_queue->submitJob(transf_vp_job);
                    thread_queue->submitJob(process_frame_job);
                    // thread_queue->submitJob(pack_fragment_job);

                }
                else if (result == K4A_WAIT_RESULT_FAILED)
                {
                    k4a_capture_release(capture);
                    stop();
                    Logger::log(LogLevel::ERROR, device_info.serial_number, "Fail to get capture\n");
                    std::throw_with_nested(std::runtime_error("Fail to get capture"));
                }

                k4a_capture_release(capture);
                (*sync_manager->update_device_ready_fptr)(device_info.index, false);
            }

            //thread_queue->waitForJob(prev_export_job);
            
          
            //while (true) {
                // Wait for all cameras to be ready

                //get capture
                //Sinal capture done

                //get depth and color image
                // Assign images to frame
                // Tranform job
                // process job
            //}
        }
    } // namespace camera
}   // namespace uvgvolucap