#include <string>
#include "vision_core/detect/VideoProcessor.hpp"

namespace vision_core{

class AppManager {
public:
    AppManager(const std::string& configFile); 
    void run();

private:
    Configs config;
    std::unique_ptr<VideoProcessor> videoProcessor;  
};

} // namespace vision_core