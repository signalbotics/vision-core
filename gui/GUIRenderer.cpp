#include "GUIRenderer.hpp"
#include <iostream>

namespace vision_core{

GUIRenderer::GUIRenderer() {}
GUIRenderer::~GUIRenderer() {
    stop();
}



void GUIRenderer::stop() {
    stopRequested = true;
    frameCondVar.notify_one();
    if (renderThread.joinable()) {
        renderThread.join();
    }
}

void GUIRenderer::updateFrames(const cv::Mat& original, const cv::Mat& processed) {
    std::lock_guard<std::mutex> lock(frameMutex);
    originalFrame = original.clone();
    processedFrame = processed.clone();
    newFrameAvailable = true;
    if (!useThread) {
        display();
    } else {
        frameCondVar.notify_one();
    }
}


bool GUIRenderer::isStopRequested() const {
    return stopRequested;
}

void GUIRenderer::display() {
    // This is a pure virtual function
    // Derived classes must implement this function
}

void GUIRenderer::renderLoop() {
    // This is a pure virtual function
    // Derived classes must implement this function
}

} // namespace vision_core