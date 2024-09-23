#include <iostream>
#include "uvgvolucap/uvgvolucap.hpp"

void print_usage() {
    std::cout << "Usage: ./uvgVoluCap.exe -i <config_path> -c <color_address> -p <position_address>" << std::endl;
}

int main(int argc, char* argv[]) {
    
    // -i: config_path, -c: color_address, -p: position_address
    uvgvolucap::API::input_config config;
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "-i" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.config_path = argv[i + 1];
        }
        else if (std::string(argv[i]) == "-c" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.color_address = argv[i + 1];
        }
        else if (std::string(argv[i]) == "-p" && std::string(argv[i + 1]).substr(0, 1) != "-") {
            config.position_address = argv[i + 1];
        } 
    }

    if (config.config_path.empty() || config.color_address.empty() || config.position_address.empty()) {
        print_usage();
        return 1;
    }
    
    uvgvolucap::API::setup_config setup_config;
    uvgvolucap::API::setup_k4a_devices(config, setup_config);
    uvgvolucap::API::k4a_run(config, setup_config);

    return 0;
}