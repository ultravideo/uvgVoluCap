#include <iostream>
#include "uvgvolucap/uvgvolucap.hpp"
#include <cstdlib>  // For system()
#include <filesystem>
#include <thread>
#include <chrono> 

void print_usage() {
    std::cout << "Usage: ./uvgVoluCap.exe --config <config_path> --addr_color <color_address> --addr_position <position_address> --running_time <running_time> --cam_type <cam_type> --running_mode <running_mode>" << std::endl;
    std::cout << "Example: ./uvgVoluCap.exe --config path/to/cameraconfig.json --addr_color tcp://*:5555 --addr_position tcp://*:5556 --running_time 30 --cam_type all --running_mode ply" << std::endl;
    std::cout << "Camera type: all: 0, kinect: 1, realsense: 2" << std::endl;
    std::cout << "Running mode: ply: 0, stream: 1" << std::endl;
    std::cout << "Note: If running time and running mode are not provided, the default value will be 30 seconds and all devices will be run (Kinect and RealSense)." << std::endl;
}

const enum CameraType {
    ALL,
    KINECT,
    REALSENSE
};

const enum RunningMode {
    PLY,
    STREAM
};

int main(int argc, char* argv[]) {
    // -i: config_path, -c: color_address, -p: position_address
    uvgvolucap::API::input_config config;
    CameraType cam_type = CameraType::ALL;
    RunningMode running_mode = RunningMode::PLY;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--config" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.config_path = argv[i + 1];
        } else if (std::string(argv[i]) == "--addr_color" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.color_address = argv[i + 1];
        } else if (std::string(argv[i]) == "--addr_position" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.position_address = argv[i + 1];
        } else if (std::string(argv[i]) == "--running_time" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.running_time = std::stoi(argv[i + 1]);
        } else if (std::string(argv[i]) == "--cam_type" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            try {
                cam_type = static_cast<CameraType>(std::stoi(argv[i + 1]));
            } catch (const std::exception& e) {
                std::cout << "Invalid camera type: " << e.what() << std::endl;
                print_usage();
                return 1;
            }
        } else if (std::string(argv[i]) == "--cam_type" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            try {
                running_mode = static_cast<RunningMode>(std::stoi(argv[i + 1]));
            } catch (const std::exception& e) {
                std::cout << "Invalid running mode: " << e.what() << std::endl;
                print_usage();
                return 1;
            }
        }
    }

    if (config.config_path.empty() || config.color_address.empty() || config.position_address.empty()) {
        std::cout << "Please provide the configuration path, color address, and position address" << std::endl;
        print_usage();
        return 1;
    }

    std::string exe_path = std::filesystem::path(argv[0]).parent_path().string();
    std::string exe_path_str;
    std::thread _thread;
    
    switch (running_mode)
    {
    case RunningMode::PLY:
        exe_path_str = exe_path + "/plyXporter.exe" 
        + " --addr_color " + config.color_address 
        + " --addr_position " + config.position_address
        + " --save_dir " + exe_path + "/PLY";
        std::cout << "Running PLY exporter: " << exe_path_str << std::endl;
        break;
    case RunningMode::STREAM:
        std::cout << "Running UVG Visualizer" << std::endl;
        exe_path_str = exe_path + "/uvgVisualizer.exe";
    default:
        break;
    }
    
    uvgvolucap::API::setup_config setup_config;
    int external_prog = 0;
    switch (cam_type)
    {
    case CameraType::KINECT:
        // Run only Kinect
        uvgvolucap::API::setup_k4a_devices(config, setup_config);
        _thread = std::thread([exe_path_str, &external_prog]() {
            // external_prog = system(exe_path_str.c_str());
        });
        // Sleep 1s
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (external_prog != 0) {  // If system() fails
            std::cerr << "Error: Execution failed with exit code " << external_prog << "\n";
            exit(EXIT_FAILURE);  // Force terminate if execution fails
        }
        uvgvolucap::API::k4a_run(config, setup_config);
        break;
    case CameraType::REALSENSE:
        // Run only RealSense
        uvgvolucap::API::setup_rs2_devices(config, setup_config);
        _thread = std::thread([exe_path_str, &external_prog]() {
            // external_prog = system(exe_path_str.c_str());
        });
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (external_prog != 0) {  // If system() fails
            std::cerr << "Error: Execution failed with exit code " << external_prog << "\n";
            exit(EXIT_FAILURE);  // Force terminate if execution fails
        }
        uvgvolucap::API::rs2_run(config, setup_config);
        break;
    case CameraType::ALL:
        // Run all devices
        uvgvolucap::API::setup_all_types_devices(config, setup_config);
        _thread = std::thread([exe_path_str, &external_prog]() {
            // external_prog = system(exe_path_str.c_str());
        });
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (external_prog != 0) {  // If system() fails
            std::cerr << "Error: Execution failed with exit code " << external_prog << "\n";
            exit(EXIT_FAILURE);  // Force terminate if execution fails
        }
        uvgvolucap::API::all_types_run(config, setup_config);
        break;
    default:
        std::cout << "Invalid running mode" << std::endl;
        break;
    }

    exit(EXIT_SUCCESS); 
}