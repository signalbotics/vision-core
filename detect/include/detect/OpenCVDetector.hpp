#ifndef OPENCV_DETECTOR_HPP
#define OPENCV_DETECTOR_HPP

#pragma once
#include "BaseDetector.hpp"
namespace vision_core{

class OpenCVDetector : public BaseDetector {
public:
    OpenCVDetector(const Configs& config);
    void detect(const cv::Mat& frame, std::vector<cv::Rect>& boxes,
                std::vector<float>& confidences, std::vector<int>& classIds) override;
    void postProcess(const cv::Mat& frame, const std::vector<cv::Mat>& outs,
                     std::vector<cv::Rect>& boxes, std::vector<float>& confidences,
                     std::vector<int>& classIds);
};


#endif // OPENCV_DETECTOR_HPP

} // namespace vision_core