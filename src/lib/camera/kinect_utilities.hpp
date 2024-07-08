#pragma once
#ifndef UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP
#define UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP

#include "kinect_camera.hpp"

namespace uvgvolucap {
    namespace camera{
        nlohmann::json parse_config(std::string config_path);
        k4a_fps_t get_fps(int fps);
        k4a_color_resolution_t get_color_resolution(int color_resolution);
        k4a_depth_mode_t get_depth_mode(int depth_mode);
        uint32_t get_numb_connected_devices();
        std::string get_serial_number(int device_index);

        //Test
        bool init_connected_device(std::shared_ptr<std::vector<_kinect_device_ptr>> devices, std::string config_path);
        void start_capture(std::shared_ptr<std::vector<_kinect_device_ptr>> devices, std::shared_ptr<ThreadQueue> thread_queue, std::shared_ptr<SyncManager> _sync_manager);
    } // namespace camera
} // namespace uvgvolucap

#endif // UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP