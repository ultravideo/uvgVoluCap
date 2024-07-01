#include <iostream>
#include "uvgvolucap/camera/kinect_utilities.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;
    std::shared_ptr<std::vector<uvgvolucap::camera::kinect_device_ptr>> devices = std::make_shared<std::vector<uvgvolucap::camera::kinect_device_ptr>>();
    uvgvolucap::camera::init_connected_device(devices, "C:/Users/Guillaume/workspace/uvgvolucap/asset/cameraconfig.json");
    return 0;
}