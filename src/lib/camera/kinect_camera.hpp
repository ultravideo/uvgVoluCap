#ifndef UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP
#define UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP

#include "based_camera.hpp"	
#include "uvgvolucap/log.hpp"
#include "uvgvolucap/threadqueue.hpp"
#include "geometry/point_cloud.hpp"
#include <nlohmann/json.hpp>
#include <k4a/k4a.hpp>

namespace uvgvolucap {
    namespace camera{
        struct MainSetting {
            mutable int color_resolution = 1536;
            mutable int depth_resolution = 576;
            mutable int fps = 30;
            mutable bool depth_to_color = true;
            mutable size_t max_size = 200000;
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

        struct SyncManager {
            std::shared_ptr<std::function<void(int, bool)>> update_device_ready_fptr = nullptr;
            std::shared_ptr<std::function<void(int, bool)>> update_device_capture_fptr = nullptr;
            std::shared_ptr<std::function<int()>> get_num_ready_cam_fptr = nullptr;
            std::shared_ptr<std::function<int()>> get_num_cap_cam_fptr = nullptr;

            std::mutex sync_mx; /**< Mutex for synchronization. -public usage */
            std::condition_variable Cap_permission_cv; /**< Condition variable for synchronization. - internal usage */
        
            size_t count_pcl = 0; //For control based on user input
            std::shared_ptr<geometry::MergeBufferPointCloud> m_merge_buffer = nullptr; /**< Point cloud buffer */
            std::shared_ptr<uvgvolucap::Job> send_job = nullptr; /**< Job for sending point cloud */
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
            int id = 0; /**< Frame ID */
            k4a_image_t depth_image = NULL; /**< Depth image */
            k4a_image_t color_image = NULL; /**< Color image */
            
            std::shared_ptr<geometry::PclFragment> fragment_pcl = std::make_shared<geometry::PclFragment>(); /**< Point cloud */

            /**
             * @brief Constructor for Data_package.
             * @param depth_ The depth image.
             * @param color_ The color image.
             */
            Frame(int _id, k4a_image_t depth_, k4a_image_t color_)
                : id(_id), depth_image(depth_), color_image(color_) {}
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
            std::shared_ptr<camera::SyncManager> sync_manager = nullptr;

        public:
            Kinect(uint32_t _index, std::string _serial, nlohmann::json _config);
            ~Kinect() = default;

            std::string get_serial_number();
            void warm_up() override;
            void stop() override;
            void start_capture(std::shared_ptr<ThreadQueue> _thread_queue, std::shared_ptr<SyncManager> _sync_manager);
        
        private:
            void setup_device_config();
            void createXYTable(const k4a_calibration_t *calibration);

            void transform_view_point(std::shared_ptr<Frame> frame);
            void process_frame(std::shared_ptr<Frame> frame);
            void pack_fragment(std::shared_ptr<geometry::PclFragment> fragment_pcl, std::shared_ptr<geometry::MergeBufferPointCloud> _asisgned_merge_buffer);
            void pointcloud_production_line();

        protected:
            void init(uint32_t _index, std::string _serial, nlohmann::json _config) const override;
            void open() override;
            void close() override;
            void start() override;
        };

        typedef std::shared_ptr<Kinect> _kinect_device_ptr;
    } // namespace camera
} // namespace uvgvolucap

#endif // UVG_VOLUCAP_CAMERA_KINECT_CAMERA_HPP