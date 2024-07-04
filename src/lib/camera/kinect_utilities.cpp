#include "kinect_utilities.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>

namespace uvgvolucap {
    namespace camera{
        nlohmann::json parse_config(std::string config_path) {
            // Check if the file exists
            if (!std::filesystem::exists(config_path))
            {
                std::throw_with_nested(std::runtime_error("Config file does not exist"));
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
                    std::throw_with_nested(std::runtime_error("Config file is missing key: " + key));
                }
            }

            if (config["system"]["version"] != "0.1.0")
            {
                Logger::log(LogLevel::ERROR, "INIT", "Config file version is not supported\n");
                std::throw_with_nested(std::runtime_error("Config file version is not supported"));
            }

            return config;
        }

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
                break;
            }
            return fps;
        }

        k4a_color_resolution_t get_color_resolution(int color_resolution) {
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
                break;
            }
            return color_res;
        }

        k4a_depth_mode_t get_depth_mode(int depth_resolution) {
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
            }

            return std::string(serial_buf);
        }

        bool init_connected_device(std::shared_ptr<std::vector<_kinect_device_ptr>> devices, std::string config_path) {
            nlohmann::json config_params = parse_config(config_path);

            uint32_t num_devices = get_numb_connected_devices();               
            devices->reserve(num_devices);

            Logger::log(LogLevel::INFO, "INIT", "Found " + std::to_string(num_devices) +  " device\n");

            for (uint32_t i = 0; i < num_devices; i++)
            {
                std::string serial = get_serial_by_index(i);

                if (config_params["devices_config"].find(serial) != config_params["devices_config"].end())
                {
                    if (!config_params["devices_config"][serial].at("disabled").get<bool>()) {                
                        devices->push_back(std::make_shared<Kinect>(i, serial, config_params));
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

            std::string system_config = std::to_string(config_params["setting"]["fps"].get<int>()) + " fps, "
                                        + std::to_string(config_params["setting"]["color_resolution"].get<int>()) + " color, " 
                                        + std::to_string(config_params["setting"]["depth_resolution"].get<int>()) + " depth";

            if (devices->size() > 0 && devices->size() <= num_devices)
            {
                Logger::log(LogLevel::INFO, "INIT", "System config: "+ system_config +"\n");
                return true;
            }
            else
            {
                Logger::log(LogLevel::ERROR, "INIT", "No device is initialized\n");
                return false;
            }
        }  

        void start_capture(std::shared_ptr<std::vector<_kinect_device_ptr>> devices, std::shared_ptr<ThreadQueue> thread_queue, std::shared_ptr<SyncManager> _sync_manager) {
            for (auto &device : *devices) {
                device->start_capture(thread_queue, _sync_manager);
            }
        }
    } // namespace camera
} // namespace uvgvolucap