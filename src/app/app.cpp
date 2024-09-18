#include <iostream>
#include "uvgvolucap/uvgvolucap.hpp"

struct input_config {
    std::string config_path;
    std::string color_address;
    std::string position_address;
};

int main(int argc, char* argv[]) {
    
    // -i: config_path, -c: color_address, -p: position_address
    input_config config;
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

        std::cout << i << " " << argv[i] << std::endl;
    }

    uvgvolucap::API::run(config.config_path, config.color_address, config.position_address);
    return 0;
}