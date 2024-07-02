#include "uvgvolucap/uvgvolucap.hpp"
#include "camera/kinect_utilities.hpp"

namespace uvgvolucap {
    namespace core {
        PointCloudFactory::PointCloudFactory() {

        }

        PointCloudFactory::~PointCloudFactory() {
            
        }

        void PointCloudFactory::syncExecute() {
            std::unique_lock<std::mutex> lock(*sync_mx);

            while (!stop_flag)
            {
                main_cv.wait(lock, [&]
                        { return ready_Cam == limit; });

                update_device_ready(RESET, true);
                update_device_capture(RESET, true);

                // packageProvider->createPackageImage();

                Cap_permission_cv->notify_all();

                main_cv.wait(lock, [&]
                        { return cap_Cam == 0; });
            }
        }

        void PointCloudFactory::set_sync_limit(size_t total_cams) {
            limit = static_cast<int>((1 << total_cams) - 1);
        }

        void PointCloudFactory::update_device_ready(int camera_index, bool reset) {
            std::lock_guard<std::mutex> lock(readyCam_mx); // Lock the mutex for the duration of the function
            if (reset)
            {
                ready_Cam = 0;
            }
            else
            {
                ready_Cam += (1 << camera_index);
                main_cv.notify_one();
            }
        }

        void PointCloudFactory::update_device_capture(int camera_index, bool reset) {
            std::lock_guard<std::mutex> lock(capcam_mx); // Lock the mutex for the duration of the function
            if (reset)
            {
                cap_Cam = limit;
            }
            else
            {
                cap_Cam -= (1 << camera_index);
                main_cv.notify_one();
            }
        }

        int PointCloudFactory::get_num_ready_cam() {
            std::lock_guard<std::mutex> lock(readyCam_mx);
            return ready_Cam;
        }

        int PointCloudFactory::get_num_cap_cam() {
            std::lock_guard<std::mutex> lock(capcam_mx);
            return cap_Cam;
        }

        std::shared_ptr<std::condition_variable> PointCloudFactory::get_Cap_permission_cv() {
            return Cap_permission_cv;
        }

        std::shared_ptr<std::mutex> PointCloudFactory::get_sync_mx() {
            return sync_mx;
        }

        template <typename Func, typename... Args>
        void PointCloudFactory::start_producing(Func&& func, Args&&... args) {
            std::function<void()> f = std::bind(std::forward<Func>(func), std::forward<Args>(args)..., thread_queue);
            f();
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::cout << "Goodbye, World!" << std::endl;
        }
    }


    namespace API {
        void test() {
            std::cout << "Hello, World!" << std::endl;
            std::shared_ptr<std::vector<camera::kinect_device_ptr>> devices = std::make_shared<std::vector<camera::kinect_device_ptr>>();
            camera::init_connected_device(devices, "C:/Users/Guillaume/workspace/uvgvolucap/asset/cameraconfig.json");
            
            core::PointCloudFactory factory;
            factory.set_sync_limit(devices->size());
            factory.start_producing(camera::start_capture, devices);

        }
    } // namespace API
} // namespace uvgvolucap
