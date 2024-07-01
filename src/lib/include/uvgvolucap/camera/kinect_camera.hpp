#ifndef UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP
#define UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP

#include "based_camera.hpp"	
#include "uvgvolucap/log.hpp"
#include "uvgvolucap/threadqueue.hpp"
#include <nlohmann/json.hpp>
#include <k4a/k4a.hpp>
#include <memory>

namespace uvgvolucap {
    namespace camera{
        struct MainSetting {
            mutable int color_resolution = 1536;
            mutable int depth_resolution = 576;
            mutable int fps = 30;
            mutable bool depth_to_color = true;
        };

        struct FilterConfig {
            mutable float max_xy = 0;
            mutable float min_xy = 0;
            mutable float max_z = 0;
            mutable float min_z = 0;
        };

        struct ROIConfig {
            mutable size_t start_x = 0;
            mutable size_t start_y = 0;
            mutable size_t width = 0;
            mutable size_t height = 0;
        };
        

        struct KinectCameraInfo {
            mutable std::string serial_number = "";
            mutable int index = 0;

            MainSetting system_config;
            FilterConfig filter_config;
            ROIConfig roi;

            mutable std::array<float, 16> transformation_matrix;
        };
        
        class Kinect : public BasedCamera<uint32_t, std::string, nlohmann::json> {
        private:
            k4a_device_t m_device;
            k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
            KinectCameraInfo device_info;


        public:
            Kinect(uint32_t _index, std::string _serial, nlohmann::json _config);
            ~Kinect() = default;

            void open() override;
            void close() override;
            void warm_up() override;
            void start() override;
            void stop() override;
            std::string get_serial_number();
        
        private:
            void setup_device();
            void createXYTable(const k4a_calibration_t *calibration);

        protected:
            void init(uint32_t _index, std::string _serial, nlohmann::json _config) const override;
        };

        
        typedef std::shared_ptr<Kinect> kinect_device_ptr;
    } // namespace camera
} // namespace uvgvolucap

#endif // UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP