#include "detect/BaseDetector.hpp"

namespace vision_core{

BaseDetector::BaseDetector(const Configs& config) {

    classNames = loadClassNames(config.classesFile);
    colors = loadColors(config.colorsFile);
    confidenceThreshold = config.confidenceThreshold;
    nmsThreshold = config.nmsThreshold;
}

void BaseDetector::drawDetections(cv::Mat& frame, const std::vector<cv::Rect>& boxes,
                                  const std::vector<float>& confidences, const std::vector<int>& classIds) {
    for (size_t i = 0; i < boxes.size(); i++) {
        const cv::Scalar& color = colors[classIds[i]];
        cv::rectangle(frame, boxes[i], color, 2);

        std::string label = cv::format("%.2f", confidences[i]);
        if (!classNames.empty() && classIds[i] < classNames.size()) {
            label = classNames[classIds[i]] + ": " + label;
        }

        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        int top = std::max(boxes[i].y - labelSize.height, 0);

        cv::rectangle(frame, cv::Point(boxes[i].x, top), cv::Point(boxes[i].x + labelSize.width, top + labelSize.height + baseLine), color, cv::FILLED);
        cv::putText(frame, label, cv::Point(boxes[i].x, top + labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}

} // namespace vision_core