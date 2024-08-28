#pragma once
#ifndef UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP
#define UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP

#include "kinect_camera.hpp"
#include "debug_macro.hpp"
#include "uvgvolucap/uvgvolucap.hpp"

namespace uvgvolucap {
    typedef std::shared_ptr<std::vector<uvgvolucap::camera::_kinect_device_ptr>> _kinect_device_ptr_vector;

    namespace camera{
        nlohmann::json parse_config(std::string config_path);
        k4a_fps_t get_fps(int fps);
        k4a_color_resolution_t get_color_resolution(int color_resolution);
        k4a_depth_mode_t get_depth_mode(int depth_mode);
        uint32_t get_numb_connected_devices();
        std::string get_serial_number(int device_index);

        enum voxelizer_mode {
            VOXELIZER_SUBSPACE
        };

        int get_voxelizer_mode(int mode);

        //Test
        void restart_connected_device();
        bool init_connected_device(_kinect_device_ptr_vector devices, std::string config_path, bool &is_voxelized);
        void start_capture(_kinect_device_ptr_vector devices, std::shared_ptr<ThreadQueue> thread_queue, std::shared_ptr<SyncManager> _sync_manager);
    } // namespace camera
} // namespace uvgvolucap

#endif // UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP