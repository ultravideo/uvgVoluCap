#pragma once
#ifndef UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP
#define UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP

#include "kinect_camera.hpp"
#include "realsense_camera.hpp"
#include "debug_macro.hpp"
#include "uvgvolucap/uvgvolucap.hpp"

namespace uvgvolucap {
    typedef std::shared_ptr<std::vector<uvgvolucap::camera::_kinect_device_ptr>> _kinect_device_ptr_vector;
    typedef std::shared_ptr<std::vector<uvgvolucap::camera::_realsense_device_ptr>> _realsense_device_ptr_vector;

    namespace camera {
        /**
         * @brief Parse configuration.
         * @details This function is used to parse the configuration file from the input path.
         * 
         * @param config_path The path of the configuration file.
         * @return nlohmann::json The parsed configuration file.
         */
        nlohmann::json parse_config(std::string config_path);

        /**
         * @brief Get voxelizer mode.
         * @details This function is used to get the voxelizer mode.
         * 
         * @param mode The mode of the voxelizer.
         * @return int The voxelizer mode.
         */
        int get_voxelizer_mode(int mode);

        namespace utils_k4a {
            /**
             * @brief Get fps.
             * @details This function is used to get the fps based on the configuration file.
             * 
             * @param fps The fps.
             * @return k4a_fps_t The fps.
             */
            k4a_fps_t get_fps(int fps);

            /**
             * @brief Get color resolution.
             * @details This function is used to get the color resolution based on the configuration file.
             * 
             * @param color_resolution The color resolution.
             * @return k4a_color_resolution_t The color resolution.
             */
            k4a_color_resolution_t get_k4a_color_resolution(int color_resolution);

            /**
             * @brief Get depth mode.
             * @details This function is used to get the depth mode based on the configuration file.
             * 
             * @param depth_mode The depth mode.
             * @return k4a_depth_mode_t The depth mode.
             */
            k4a_depth_mode_t get_k4a_depth_mode(int depth_mode);

            /**
             * @brief Get number of connected devices.
             * @details This function is used to get the number of connected devices.
             * 
             * @return uint32_t The number of connected devices.
             */
            uint32_t get_numb_connected_devices();

            /**
             * @brief Get serial number.
             * @details This function is used to get the serial number of the device.
             * 
             * @param device_index The index of the device.
             * @return std::string The serial number of the device.
             */
            std::string get_serial_number(int device_index);

            /**
             * @brief Restart connected device.
             * @details This function is used to restart the connected device.
             */
            void restart_connected_device();

            /**
             * @brief Initialize connected device.
             * @details This function is used to initialize the connected device based on the configuration file.
             * 
             * @param devices The vector of the connected device.
             * @param config_path The path of the configuration file.
             * @param is_voxelized The boolean value for voxelized.
             * @return bool The boolean value for the initialization of the connected device.
             */
            bool init_connected_k4a_device(_kinect_device_ptr_vector k4a_devices, std::string config_path, bool &is_voxelized);

            /**
             * @brief Start capture.
             * @details This function is used to start the capture of the connected device.
             * 
             * @param devices The vector of the connected device.
             * @param thread_queue The thread queue.
             * @param _sync_manager The sync manager.
             */
            void kinect_start_capture(_kinect_device_ptr_vector k4a_devices, std::shared_ptr<ThreadQueue> thread_queue, std::shared_ptr<SyncManager> _sync_manager);
        } // namespace utils_k4a

        namespace utils_rs2 {
            uint32_t get_numb_connected_devices();
            void restart_connected_device();
            rs2_depth_mode_t get_rs2_depth_mode(int depth_mode);
            rs2_color_mode_t get_rs2_color_format(int color_format);
            rs2_fps_t get_rs2_fps(int fps);
            bool init_connected_rs2_device(_realsense_device_ptr_vector rs2_devices, std::string config_path, bool &is_voxelized);
            void realsense_start_capture(_realsense_device_ptr_vector rs2_devices, std::shared_ptr<ThreadQueue> thread_queue, std::shared_ptr<SyncManager> _sync_manager);
        }// namespace utils_rs2

    } // namespace camera
} // namespace uvgvolucap

#endif // UVGVOLUCAP_CAMERA_KINECT_UTILITIES_HPP