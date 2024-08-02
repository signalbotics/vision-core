#include "GUIRendererCV.hpp"
#include <iostream>

namespace vision_core{

GUIRendererCV::GUIRendererCV() : GUIRenderer() {}


void GUIRendererCV::start(Configs config) {
    this->config = config;
    if (config.useRenderingThread) {
        renderThread = std::thread(&GUIRendererCV::renderLoop, this);
    }
}




void GUIRendererCV::renderLoop() {
    while (!stopRequested) {
        std::unique_lock<std::mutex> lock(frameMutex);
        frameCondVar.wait(lock, [this] { return newFrameAvailable || stopRequested; });
        if (stopRequested) break;

        display();
        newFrameAvailable = false;
    }
}

void GUIRendererCV::display() {
    
    if (!originalFrame.empty() && !processedFrame.empty()) {
        cv::Mat combined;
        cv::hconcat(originalFrame, processedFrame, combined);
        cv::imshow("YOLO Detector", combined);
        int key = cv::waitKey(1);
        if (key == 27) { // ESC key
            cv::destroyAllWindows();
            stopRequested = true;
        }
    }
}

bool GUIRendererCV::isStopRequested() const {
    return stopRequested;
}

} // namespace vision_core