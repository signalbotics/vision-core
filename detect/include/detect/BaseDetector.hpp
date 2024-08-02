#ifndef BASE_DETECTOR_HPP
#define BASE_DETECTOR_HPP

#pragma once
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>
#include "../../../common/util.hpp"
namespace vision_core{
class BaseDetector {
public:
    BaseDetector(const Configs& config);
    virtual void detect(const cv::Mat& frame, std::vector<cv::Rect>& boxes,
                        std::vector<float>& confidences, std::vector<int>& classIds) = 0;
    virtual void drawDetections(cv::Mat& frame, const std::vector<cv::Rect>& boxes,
                                const std::vector<float>& confidences, const std::vector<int>& classIds);

    virtual ~BaseDetector() {}
protected:
    cv::dnn::Net net;
    std::vector<std::string> classNames;
    std::vector<cv::Scalar> colors;
    float confidenceThreshold, nmsThreshold;
    Configs config;
};

#endif // BASE_DETECTOR_HPP

} // namespace vision_core