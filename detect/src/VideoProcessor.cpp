#include "detect/VideoProcessor.hpp"

namespace vision_core{

VideoProcessor::VideoProcessor(Configs& config)  
  : config(config), yoloDetector(config)
{ 
    if (config.useOpenVINO) {
        #ifdef WITH_OPENVINO
            detector = std::make_unique<OpenVINODetector>(config);
        #else
            std::cerr << "OpenVINO support not enabled. Recompile with OpenVINO support" << std::endl;
            exit(1);
        #endif
    } else if (config.useTensorRT) {
        #ifdef WITH_TENSORRT
            detector = std::make_unique<TensorRTDetector>(config);
        #else
            std::cerr << "TensorRT support not enabled. Recompile with TensorRT support" << std::endl;
            exit(1);
        #endif
    } else
    detector = std::make_unique<OpenCVDetector>(config);

    if (config.renderer == "OPENGL") {
        #ifdef WITH_OPENGL
            std::cout << "Using OpenGL renderer" << std::endl;
            guiRenderer = std::make_unique<GUIRendererGL>();
        #else
            std::cerr << "OpenGL support not enabled. Recompile with OpenGL support" << std::endl;
            exit(1);
        #endif
    } else {
        std::cout << "Using OpenCV renderer" << std::endl;
        guiRenderer = std::make_unique<GUIRendererCV>();
    }
    guiRenderer->start(config);
}


void VideoProcessor::processVideo() {
    cv::VideoCapture capture(config.videoPath);
    if (!capture.isOpened()) {
    std::cerr << "Failed to open video file: " << config.videoPath << std::endl;
    return;
    }

    cv::Mat frame, processedFrame;

    while (!guiRenderer->isStopRequested()) {

    if (!capture.read(frame)) {
        capture.set(cv::CAP_PROP_POS_FRAMES, 0); // Reset to start
        continue;
        
    }
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;
    processedFrame = frame.clone();
    if (config.useInferenceThread) {
        
        std::thread inferenceThread([&]() {

            detector->detect(processedFrame, boxes, confidences, classIds); 
        });
        inferenceThread.join(); 

    } else {
        detector->detect(processedFrame, boxes, confidences, classIds); 
    }

    detector->drawDetections(processedFrame, boxes, confidences, classIds); 


    guiRenderer->updateFrames(frame, processedFrame);

    }
}


} // namespace vision_core