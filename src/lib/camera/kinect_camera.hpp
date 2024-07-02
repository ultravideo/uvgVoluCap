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

        struct Frame {
            std::shared_ptr<k4a_image_t> depth_image = NULL; /**< Depth image */
            std::shared_ptr<k4a_image_t> color_image = NULL; /**< Color image */

            /**
             * @brief Constructor for Data_package.
             * @param depth_ The depth image.
             * @param color_ The color image.
             */
            Frame(std::shared_ptr<k4a_image_t> depth_, std::shared_ptr<k4a_image_t> color_)
                : depth_image(depth_), color_image(color_) {}
        };
        
        class Kinect : public BasedCamera<uint32_t, std::string, nlohmann::json> {
        private:
            k4a_device_t m_device;
            k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
            k4a_calibration_t m_calibration;
            k4a_transformation_t transformation_handle = nullptr;
            k4a_image_t xy_table = NULL;
            k4a_float2_t *xy_table_data = NULL;

            KinectCameraInfo device_info;

            // Get depth image size for Color2Depth
            int target_viewpoint_width = 0;
            int target_viewpoint_height = 0;

            std::shared_ptr<std::thread> capture_thread_ptr;
            std::function<void()> capture_function;

        public:
            Kinect(uint32_t _index, std::string _serial, nlohmann::json _config);
            ~Kinect() = default;

            std::string get_serial_number();
            void warm_up() override;
            void stop() override;
            void start_capture(std::shared_ptr<uvgvolucap::ThreadQueue> _thread_queue);
        
        private:
            void setup_device_config();
            void createXYTable(const k4a_calibration_t *calibration);

            void pointcloud_production_line();

        protected:
            void init(uint32_t _index, std::string _serial, nlohmann::json _config) const override;
            void open() override;
            void close() override;
            void start() override;
        };

        typedef std::shared_ptr<Kinect> kinect_device_ptr;
    } // namespace camera
} // namespace uvgvolucap

#endif // UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP