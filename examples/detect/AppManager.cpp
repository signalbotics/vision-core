#include "AppManager.hpp"

namespace vision_core{

AppManager::AppManager(const std::string& configFile)
{ 

    config =  parseConfigFile(configFile);
    videoProcessor = std::make_unique<VideoProcessor>(config);

}

void AppManager::run() {
    videoProcessor->processVideo();
}

} // namespace vision_core