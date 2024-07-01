#include "uvgvolucap/camera/kinect_camera.hpp"
#include "uvgvolucap/camera/kinect_utilities.hpp"

namespace uvgvolucap {
    namespace camera{
        Kinect::Kinect(uint32_t _index, std::string _serial, nlohmann::json _config) : BasedCamera()
        {
            init(_index, _serial, _config);
            setup_device();
        }

        void Kinect::init(uint32_t _index, std::string _serial, nlohmann::json _config) const
        {
            device_info.index = _index;
            device_info.serial_number = _serial;

            device_info.system_config.color_resolution = _config["setting"]["color_resolution"].get<int>();
            device_info.system_config.depth_resolution = _config["setting"]["depth_resolution"].get<int>();
            device_info.system_config.fps = _config["setting"]["fps"].get<int>();
            device_info.system_config.depth_to_color = _config["setting"]["depth_to_color"].get<bool>();

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

        void Kinect::setup_device()
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
            if (!is_opened){
                k4a_device_open(device_info.index, &m_device);
                is_opened = true;
            }
        }

        void Kinect::close()
        {
            if (is_opened){
                k4a_device_close(m_device);
                is_opened = false;
            }
        }

        void Kinect::warm_up()
        {
        }

        void Kinect::start()
        {
            if (is_opened && !is_started){
                k4a_device_start_cameras(m_device, NULL);
                is_started = true;
            }
            else{
                throw std::runtime_error("Device is not opened yet");	
            }
        }

        void Kinect::stop()
        {
            if (is_started){
                k4a_device_stop_cameras(m_device);
                is_opened = false;
            }
            else{
                throw std::runtime_error("Device is already closed");	
            }
        }

        std::string Kinect::get_serial_number()
        {
            return device_info.serial_number;
        }

        void Kinect::createXYTable(const k4a_calibration_t *calibration)
        {
        }


    } // namespace camera
}   // namespace uvgvolucap