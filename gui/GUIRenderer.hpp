#ifndef GUIRENDERER_HPP
#define GUIRENDERER_HPP

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "../common/util.hpp"


namespace vision_core{

class GUIRenderer {
protected:
    cv::Mat originalFrame;
    cv::Mat processedFrame;
    std::mutex frameMutex;
    std::condition_variable frameCondVar;
    std::thread renderThread;
    bool newFrameAvailable = false;
    bool stopRequested = false;
    bool useThread = false;
    Configs config;


public:
    GUIRenderer();
    virtual ~GUIRenderer();
    virtual void start(Configs config) = 0;
    void stop();
    void updateFrames(const cv::Mat& original, const cv::Mat& processed);
    virtual void display();
    virtual void renderLoop(); 
    bool isStopRequested() const; 
};
} // namespace vision_core

#endif // GUIRENDERER_HPP
