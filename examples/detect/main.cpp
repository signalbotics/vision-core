#include "AppManager.hpp"

int main(int argc, char** argv) {

    std::string configFile = "configs/config.yaml";
    if (argc != 2) {
        std::cerr << "Usage: ./app <config_file_path> usinf default config file " << configFile << std::endl;
    } else {
        configFile = argv[1];
    }

    vision_core::AppManager app(configFile);
    app.run();
    return 0;
}
