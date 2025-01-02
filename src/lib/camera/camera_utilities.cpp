#include "camera_utilities.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>

namespace uvgvolucap {
    namespace camera{
        nlohmann::json parse_config(std::string config_path) {
            // Check if the file exists
            if (!std::filesystem::exists(config_path))
            {
                Logger::log(LogLevel::ERROR, "INIT", "Config file does not exist\n");
                exit(EXIT_FAILURE);
            }
            std::ifstream file(config_path);
            nlohmann::json config = nlohmann::json::parse(file);

            //verify the config file
            std::vector<std::string> required_keys = {"system", "setting", "filter", "devices_config", "grid"};
            for (auto &key : required_keys)
            {
                if (config.find(key) == config.end())
                {
                    Logger::log(LogLevel::ERROR, "INIT", "Config file is missing key: " + key + "\n");
                    exit(EXIT_FAILURE);                
                }
            }

            if (config["system"]["version"] != "0.1.0")
            {
                Logger::log(LogLevel::ERROR, "INIT", "Config file version is not supported\n");
                exit(EXIT_FAILURE);            
            }
            Logger::log(LogLevel::INFO, "INIT", "Config file is verified - PASS\n");
            return config;
        }

        int get_voxelizer_mode(int mode) {
            switch (mode)
            {
            case 0:
                return VOXELIZER_SUBSPACE;
            default:
                Logger::log(LogLevel::ERROR, "INIT", "Invalid input voxelizer mode\n");
                return -1;
            }
        }

        namespace utils_k4a {
            k4a_fps_t get_fps(int fps_)
            {
                k4a_fps_t fps = K4A_FRAMES_PER_SECOND_30;
                switch (fps_)
                {
                case 15:
                    fps = K4A_FRAMES_PER_SECOND_15;
                    break;
                case 30:
                    fps = K4A_FRAMES_PER_SECOND_30;
                    break;
                case 5:
                    fps = K4A_FRAMES_PER_SECOND_5;
                    break;
                default:
                    Logger::log(LogLevel::ERROR, "INIT", "Invalid input fps\n");
                    exit(EXIT_FAILURE);
                    break;
                }
                return fps;
            }

            k4a_color_resolution_t get_k4a_color_resolution(int color_resolution) {
                k4a_color_resolution_t color_res = K4A_COLOR_RESOLUTION_OFF;
                switch (color_resolution)
                {
                case 2160:
                    color_res = K4A_COLOR_RESOLUTION_2160P;
                    break;
                case 3072:
                    color_res = K4A_COLOR_RESOLUTION_3072P;
                    break;
                case 1536:
                    color_res = K4A_COLOR_RESOLUTION_1536P;
                    break;
                case 1440:
                    color_res = K4A_COLOR_RESOLUTION_1440P;
                    break;
                case 1080:
                    color_res = K4A_COLOR_RESOLUTION_1080P;
                    break;
                case 720:
                    color_res = K4A_COLOR_RESOLUTION_720P;
                    break;
                default:
                    Logger::log(LogLevel::ERROR, "INIT", "Invalid input color resolution\n");
                    exit(EXIT_FAILURE);
                    break;
                }
                return color_res;
            }

            k4a_depth_mode_t get_k4a_depth_mode(int depth_resolution) {
                k4a_depth_mode_t depth_res = K4A_DEPTH_MODE_OFF;
                switch (depth_resolution)
                {
                    case 288:
                        depth_res = K4A_DEPTH_MODE_NFOV_2X2BINNED;
                        break;
                    case 576:
                        depth_res = K4A_DEPTH_MODE_NFOV_UNBINNED;
                        break;
                    case 512:
                        depth_res = K4A_DEPTH_MODE_WFOV_2X2BINNED;
                        break;
                    case 1024:
                        depth_res = K4A_DEPTH_MODE_WFOV_UNBINNED;
                        break;
                    default:
                        Logger::log(LogLevel::ERROR, "INIT", "Invalid input depth mode\n");
                        exit(EXIT_FAILURE);
                        break;
                }
                return depth_res;
            }

            uint32_t get_numb_connected_devices() {
                return k4a::device::get_installed_count();
            }

            std::string get_serial_by_index(uint32_t index)
            {
                k4a_device_t device;

                k4a_device_open(index, &device);
                char serial_buf[64];
                size_t serial_buf_size = sizeof(serial_buf) / sizeof(serial_buf[0]);
                k4a_buffer_result_t result = k4a_device_get_serialnum(device, serial_buf, &serial_buf_size);
                k4a_device_close(device);

                if (result != K4A_BUFFER_RESULT_SUCCEEDED)
                {
                Logger::log(LogLevel::ERROR, "INIT", "Fail to get serial number of device\n");
                exit(EXIT_FAILURE);
                }

                return std::string(serial_buf);
            }

            bool init_connected_k4a_device(_kinect_device_ptr_vector k4a_devices, std::string config_path, bool &is_voxelized) {
                nlohmann::json config_params = parse_config(config_path);

                uint32_t num_devices = get_numb_connected_devices();               
                k4a_devices->reserve(num_devices);

                Logger::log(LogLevel::INFO, "INIT", "Found " + std::to_string(num_devices) +  " device\n");
                uint32_t register_device = 0;
                for (uint32_t i = 0; i < num_devices; i++)
                {
                    std::string serial = get_serial_by_index(i);

                    if (config_params["devices_config"].find(serial) != config_params["devices_config"].end())
                    {
                        if (!config_params["devices_config"][serial].at("disabled").get<bool>()) {                
                            k4a_devices->push_back(std::make_shared<Kinect>(i, register_device, serial, config_params));
                            register_device++;
                        }
                        else {
                            Logger::log(LogLevel::INFO, "INIT", "Device with serial number: " + serial + " is disabled by configuration\n");
                        }
                    }
                    else
                    {
                        Logger::log(LogLevel::ERROR, "INIT", "No config found for device with serial number: " + serial + "\n");
                        return false;
                    }
                }

                k4a_devices->resize(static_cast<size_t>(register_device));

                std::string system_config = std::to_string(config_params["setting"]["kinect"]["fps"].get<int>()) + " fps, "
                                            + std::to_string(config_params["setting"]["kinect"]["color_resolution"].get<int>()) + " color, " 
                                            + std::to_string(config_params["setting"]["kinect"]["depth_resolution"].get<int>()) + " depth";

                if (k4a_devices->size() > 0 && k4a_devices->size() <= num_devices)
                {
                    Logger::log(LogLevel::INFO, "INIT", "Kinect config: "+ system_config +"\n");

                    if (!config_params["setting"]["voxelized"].get<bool>()) {
                        Logger::log(LogLevel::INFO, "INIT", "Voxelizer mode: disable\n");
                        is_voxelized = false;
                    }
                    else{
                        is_voxelized = true;
                        switch (get_voxelizer_mode(config_params["setting"]["voxelized_mode"].get<int>()))
                        {
                        case VOXELIZER_SUBSPACE:
                            Logger::log(LogLevel::INFO, "INIT", "Voxelizer mode: subspace\n");
                            break;
                        
                        default:
                            Logger::log(LogLevel::ERROR, "INIT", "Invalid voxelizer mode\n");
                            exit(EXIT_FAILURE);
                            break;
                        } 
                    }

                    if (config_params["setting"]["subsample_row"].get<int>() <= 0 || config_params["setting"]["subsample_col"].get<int>() <= 0){ 
                        Logger::log(LogLevel::ERROR, "INIT", "Subsample value must be greater than 0\n");
                    }
                    else {
                        Logger::log(LogLevel::INFO, "INIT", "Subsampling setup {row,col} : {" + std::to_string(config_params["setting"]["subsample_row"].get<int>()) 
                                                                                        + "," + std::to_string(config_params["setting"]["subsample_col"].get<int>())
                                                                                                + "}\n");
                        
                    }

                    if(config_params["setting"]["depth_to_color"].get<bool>()) {
                        Logger::log(LogLevel::INFO, "INIT", "Depth to color mode is enabled\n");
                    }
                    else {
                        Logger::log(LogLevel::INFO, "INIT", "Depth to color mode is disabled\n");
                    }
                }
                else
                {
                    Logger::log(LogLevel::ERROR, "INIT", "No device is initialized\n");
                    return false;
                }
                return true;
            }  

            void kinect_start_capture(_kinect_device_ptr_vector k4a_devices, std::shared_ptr<ThreadQueue> thread_queue, std::shared_ptr<SyncManager> _sync_manager) {
                Logger::log(LogLevel::INFO, "INIT", "Starting all Kinect devices\n");
                for (auto &device : *k4a_devices) {
                    device->start_capture(thread_queue, _sync_manager);
                }
            }

            void restart_connected_device() {
                k4a_device_t device;
                uint32_t num_devices = get_numb_connected_devices();
                for (uint32_t i = 0; i < num_devices; i++)
                {
                    k4a_device_open(i, &device);
                    k4a_device_close(device);
                }
            }
        } // namespace k4a

        namespace utils_rs2 {
            uint32_t get_numb_connected_devices() {
                rs2::context ctx;
                return ctx.query_devices().size();
            }

            std::string get_serial_by_index(uint32_t index)
            {
                rs2::context ctx;
                rs2::device_list devices = ctx.query_devices();
                if (index >= devices.size())
                {
                    Logger::log(LogLevel::ERROR, "INIT", "Index out of range\n");
                    exit(EXIT_FAILURE);
                }
                return devices[index].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            }

            void restart_connected_device() {
                rs2::context ctx;
                rs2::device_list devices = ctx.query_devices();
                for (auto &&device : devices)
                {
                    std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    rs2::device dev = device;
                    dev.hardware_reset();
                }

                // Check if all devices are reset and ready
                for (auto &&device : devices)
                {
                    std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    rs2::device dev = device;
                    if (!dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
                    {
                        Logger::log(LogLevel::ERROR, "INIT", "Device with serial number: " + serial + " is not ready\n");
                        exit(EXIT_FAILURE);
                    }
                }
            }

            rs2_depth_mode_t get_rs2_depth_mode(int depth_mode) {
                rs2_depth_mode_t depth_res;
                switch (depth_mode)
                {
                    case 360:
                        depth_res.width = 640;
                        depth_res.height = 360;
                        break;
                    case 480:
                        depth_res.width = 848;
                        depth_res.height = 480;
                        break;
                    case 720:
                        depth_res.width = 1280;
                        depth_res.height = 720;
                        break;
                    default:
                        Logger::log(LogLevel::ERROR, "INIT", "Invalid input depth mode\n");
                        exit(EXIT_FAILURE);
                        break;
                }
                return depth_res;
            }

            rs2_color_mode_t get_rs2_color_format(int color_format) {
                rs2_color_mode_t color_res;
                switch (color_format)
                {
                    case 480:
                        color_res.width = 640;
                        color_res.height = 480;
                        break;
                    case 540:
                        color_res.width = 960;
                        color_res.height = 540;
                        break;
                    case 720:
                        color_res.width = 1280;
                        color_res.height = 720;
                        break;
                    default:
                        Logger::log(LogLevel::ERROR, "INIT", "Invalid input color resolution\n");
                        exit(EXIT_FAILURE);
                        break;
                }
                return color_res;
            }

            rs2_fps_t get_rs2_fps(int fps) {
                rs2_fps_t fps_res;
                switch (fps)
                {
                    case 15:
                        fps_res = 15;
                        break;
                    case 30:
                        fps_res = 30;
                        break;
                    case 60:
                        fps_res = 60;
                        break;
                    default:
                        Logger::log(LogLevel::ERROR, "INIT", "Invalid input fps\n");
                        exit(EXIT_FAILURE);
                        break;
                }
                return fps_res;
            }

            bool init_connected_rs2_device(_realsense_device_ptr_vector rs2_devices, std::string config_path, bool &is_voxelized) {
                nlohmann::json config_params = parse_config(config_path);

                uint32_t num_devices = get_numb_connected_devices();
                rs2_devices->reserve(num_devices);

                Logger::log(LogLevel::INFO, "INIT", "Found " + std::to_string(num_devices) +  " device\n");
                uint32_t register_device = 0;
                for (uint32_t i = 0; i < num_devices; i++)
                {
                    std::string serial = get_serial_by_index(i);

                    if (config_params["devices_config"].find(serial) != config_params["devices_config"].end())
                    {
                        if (!config_params["devices_config"][serial].at("disabled").get<bool>()) {                
                            rs2_devices->push_back(std::make_shared<Realsense>(i, register_device, serial, config_params));
                            register_device++;
                        }
                        else {
                            Logger::log(LogLevel::INFO, "INIT", "Device with serial number: " + serial + " is disabled by configuration\n");
                        }
                    }
                    else
                    {
                        Logger::log(LogLevel::ERROR, "INIT", "No config found for device with serial number: " + serial + "\n");
                        return false;
                    }
                }

                rs2_devices->resize(static_cast<size_t>(register_device));

                std::string system_config = std::to_string(config_params["setting"]["realsense"]["fps"].get<int>()) + " fps, "
                                            + std::to_string(config_params["setting"]["realsense"]["color_resolution"].get<int>()) + " color, " 
                                            + std::to_string(config_params["setting"]["realsense"]["depth_resolution"].get<int>()) + " depth";
               
                if (rs2_devices->size() > 0 && rs2_devices->size() <= num_devices)
                {
                    Logger::log(LogLevel::INFO, "INIT", "Realsense2 config: "+ system_config +"\n");

                    if (!config_params["setting"]["voxelized"].get<bool>()) {
                        Logger::log(LogLevel::INFO, "INIT", "Voxelizer mode: disable\n");
                        is_voxelized = false;
                    }
                    else{
                        is_voxelized = true;
                        switch (get_voxelizer_mode(config_params["setting"]["voxelized_mode"].get<int>()))
                        {
                        case VOXELIZER_SUBSPACE:
                            Logger::log(LogLevel::INFO, "INIT", "Voxelizer mode: subspace\n");
                            break;
                        
                        default:
                            Logger::log(LogLevel::ERROR, "INIT", "Invalid voxelizer mode\n");
                            exit(EXIT_FAILURE);
                            break;
                        } 
                    }

                    if (config_params["setting"]["subsample_row"].get<int>() <= 0 || config_params["setting"]["subsample_col"].get<int>() <= 0){ 
                        Logger::log(LogLevel::ERROR, "INIT", "Subsample value must be greater than 0\n");
                    }
                    else {
                        Logger::log(LogLevel::INFO, "INIT", "Subsampling setup {row,col} : {" + std::to_string(config_params["setting"]["subsample_row"].get<int>()) 
                                                                                        + "," + std::to_string(config_params["setting"]["subsample_col"].get<int>())
                                                                                                + "}\n");
                        
                    }

                    if(config_params["setting"]["depth_to_color"].get<bool>()) {
                        Logger::log(LogLevel::INFO, "INIT", "Depth to color mode is enabled\n");
                    }
                    else {
                        Logger::log(LogLevel::INFO, "INIT", "Depth to color mode is disabled\n");
                    }
                }
                else
                {
                    Logger::log(LogLevel::ERROR, "INIT", "No device is initialized\n");
                    return false;
                }
                return true;

            }

            void realsense_start_capture(_realsense_device_ptr_vector rs2_devices, std::shared_ptr<ThreadQueue> thread_queue, std::shared_ptr<SyncManager> _sync_manager) {
                Logger::log(LogLevel::INFO, "INIT", "Starting all Realsense devices\n");
                for (auto &device : *rs2_devices) {
                    device->start_capture(thread_queue, _sync_manager);
                }
            }
        }
    
        bool init_connected_devices(_kinect_device_ptr_vector k4a_devices, _realsense_device_ptr_vector rs2_devices, std::string config_path, bool &is_voxelized) {
            nlohmann::json config_params = parse_config(config_path);

            // Check Kinect
            uint32_t num_k4a_devices = utils_k4a::get_numb_connected_devices();               
            k4a_devices->reserve(num_k4a_devices);
            Logger::log(LogLevel::INFO, "INIT", "Found " + std::to_string(num_k4a_devices) +  " Kinect device(s).\n");

            // Check RealSense
            uint32_t num_rs2_devices = utils_rs2::get_numb_connected_devices();
            rs2_devices->reserve(num_rs2_devices);
            Logger::log(LogLevel::INFO, "INIT", "Found " + std::to_string(num_rs2_devices) +  " RealSense device(s).\n");

            uint32_t register_device = 0;
            uint32_t total_connected_devices = num_k4a_devices + num_rs2_devices;

            for (uint32_t i = 0; i < total_connected_devices; i++)
            {
                std::string serial;
                if (i < num_k4a_devices) {
                    serial = utils_k4a::get_serial_by_index(i);
                    if (config_params["devices_config"].find(serial) != config_params["devices_config"].end())
                    {
                        if (!config_params["devices_config"][serial].at("disabled").get<bool>()) {                
                            k4a_devices->push_back(std::make_shared<Kinect>(i, register_device, serial, config_params));
                            std::cout << "Kinect device with serial number: " << serial << " is enabled by configuration with id " << register_device << "\n";
                            register_device++;
                        }
                        else {
                            Logger::log(LogLevel::INFO, "INIT", "Device with serial number: " + serial + " is disabled by configuration\n");
                        }
                    }
                    else
                    {
                        Logger::log(LogLevel::ERROR, "INIT", "No config found for device with serial number: " + serial + "\n");
                        return false;
                    }
                }
                else {
                    serial = utils_rs2::get_serial_by_index(i - num_k4a_devices);
                    if (config_params["devices_config"].find(serial) != config_params["devices_config"].end())
                    {
                        if (!config_params["devices_config"][serial].at("disabled").get<bool>()) {                
                            rs2_devices->push_back(std::make_shared<Realsense>(i, register_device, serial, config_params));
                            std::cout << "Realsense device with serial number: " << serial << " is enabled by configuration with id " << register_device << "\n";
                            register_device++;
                        }
                        else {
                            Logger::log(LogLevel::INFO, "INIT", "Device with serial number: " + serial + " is disabled by configuration\n");
                        }
                    }
                    else
                    {
                        Logger::log(LogLevel::ERROR, "INIT", "No config found for device with serial number: " + serial + "\n");
                        return false;
                    }
                }
            }

            std::string rs2_config_txt = std::to_string(config_params["setting"]["realsense"]["fps"].get<int>()) + " fps, "
                                            + std::to_string(config_params["setting"]["realsense"]["color_resolution"].get<int>()) + " color, " 
                                            + std::to_string(config_params["setting"]["realsense"]["depth_resolution"].get<int>()) + " depth";

            std::string k4a_config_txt = std::to_string(config_params["setting"]["kinect"]["fps"].get<int>()) + " fps, "
                                            + std::to_string(config_params["setting"]["kinect"]["color_resolution"].get<int>()) + " color, " 
                                            + std::to_string(config_params["setting"]["kinect"]["depth_resolution"].get<int>()) + " depth";
               
            if ((rs2_devices->size() > 0 && rs2_devices->size() <= num_rs2_devices) || (k4a_devices->size() > 0 && k4a_devices->size() <= num_k4a_devices))
            {
                Logger::log(LogLevel::INFO, "INIT", "Realsense2 config: "+ rs2_config_txt +"\n");
                Logger::log(LogLevel::INFO, "INIT", "Kinect config: "+ k4a_config_txt +"\n");

                if (!config_params["setting"]["voxelized"].get<bool>()) {
                    Logger::log(LogLevel::INFO, "INIT", "Voxelizer mode: disabled\n");
                    is_voxelized = false;
                }
                else{
                    is_voxelized = true;
                    switch (get_voxelizer_mode(config_params["setting"]["voxelized_mode"].get<int>()))
                    {
                    case VOXELIZER_SUBSPACE:
                        Logger::log(LogLevel::INFO, "INIT", "Voxelizer mode: subspace\n");
                        break;
                    
                    default:
                        Logger::log(LogLevel::ERROR, "INIT", "Invalid voxelizer mode\n");
                        exit(EXIT_FAILURE);
                        break;
                    } 
                }

                if (config_params["setting"]["subsample_row"].get<int>() <= 0 || config_params["setting"]["subsample_col"].get<int>() <= 0){ 
                    Logger::log(LogLevel::ERROR, "INIT", "Subsample value must be greater than 0\n");
                }
                else {
                    Logger::log(LogLevel::INFO, "INIT", "Subsampling setup {row,col} : {" + std::to_string(config_params["setting"]["subsample_row"].get<int>()) 
                                                                                    + "," + std::to_string(config_params["setting"]["subsample_col"].get<int>())
                                                                                            + "}\n");
                    
                }

                if(config_params["setting"]["depth_to_color"].get<bool>()) {
                    Logger::log(LogLevel::INFO, "INIT", "Depth to color mode is enabled\n");
                }
                else {
                    Logger::log(LogLevel::INFO, "INIT", "Depth to color mode is disabled\n");
                }
            }
            else
            {
                Logger::log(LogLevel::ERROR, "INIT", "No device is initialized\n");
                return false;
            }
            return true;
        }
    } // namespace camera
} // namespace uvgvolucap