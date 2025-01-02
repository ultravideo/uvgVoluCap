#include <iostream>
#include "uvgvolucap/uvgvolucap.hpp"

void print_usage() {
    std::cout << "Usage: ./uvgVoluCap.exe -i <config_path> -c <color_address> -p <position_address> -t <running_time> -m <camera_mode>" << std::endl;
    std::cout << "Example: ./uvgVoluCap.exe -i path/to/cameraconfig.json -c tcp://*:5555 -p tcp://*:5556 -t 30 -m all" << std::endl;
    std::cout << "Running mode: all: 0, kinect: 1, realsense: 2" << std::endl;
    std::cout << "Note: If running time and running mode are not provided, the default value will be 30 seconds and all devices will be run (Kinect and RealSense)." << std::endl;
}

const enum CameraMode {
    ALL,
    KINECT,
    REALSENSE
};

int main(int argc, char* argv[]) {
    // -i: config_path, -c: color_address, -p: position_address
    uvgvolucap::API::input_config config;
    CameraMode mode = CameraMode::ALL;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "-i" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.config_path = argv[i + 1];
        } else if (std::string(argv[i]) == "-c" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.color_address = argv[i + 1];
        } else if (std::string(argv[i]) == "-p" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.position_address = argv[i + 1];
        } else if (std::string(argv[i]) == "-t" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.running_time = std::stoi(argv[i + 1]);
        } else if (std::string(argv[i]) == "-m" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            mode = static_cast<CameraMode>(std::stoi(argv[i + 1]));
        }
    }

    if (config.config_path.empty() || config.color_address.empty() || config.position_address.empty()) {
        std::cout << "Please provide the configuration path, color address, and position address" << std::endl;
        print_usage();
        return 1;
    }
    
    uvgvolucap::API::setup_config setup_config;

    switch (mode)
    {
    case CameraMode::KINECT:
        // Run only Kinect
        uvgvolucap::API::setup_k4a_devices(config, setup_config);
        uvgvolucap::API::k4a_run(config, setup_config);
        break;
    case CameraMode::REALSENSE:
        // Run only RealSense
        uvgvolucap::API::setup_rs2_devices(config, setup_config);
        uvgvolucap::API::rs2_run(config, setup_config);
        break;
    case CameraMode::ALL:
        // Run all devices
        uvgvolucap::API::setup_all_types_devices(config, setup_config);
        uvgvolucap::API::all_types_run(config, setup_config);
        break;
    default:
        std::cout << "Invalid running mode" << std::endl;
        break;
    }

    return 0;
}