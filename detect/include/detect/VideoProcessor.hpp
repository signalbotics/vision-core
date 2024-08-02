#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "../../gui/GUIRenderer.hpp"
#include "detect/OpenCVDetector.hpp"  
#ifdef WITH_OPENVINO
#include "detect/OpenVINODetector.hpp"
#else
#include "detect/TensorRTDetector.hpp"
#endif
#include "../../../common/util.hpp"


#include "../../gui/GUIRendererGL.hpp"
#include "../../gui/GUIRendererCV.hpp"

namespace vision_core{

class VideoProcessor {
public: 
    VideoProcessor(Configs& config);

    void processVideo();


private:
    std::unique_ptr<BaseDetector> detector;
    OpenCVDetector yoloDetector;
    std::string modelConfig, modelWeights;
    std::string classesFile, colorsFile;
    Configs& config;
    std::unique_ptr<GUIRenderer> guiRenderer;

};

} // namespace vision_core