#include "detect/OpenCVDetector.hpp"


namespace vision_core{


std::vector<std::string> getOutputsNames(const cv::dnn::Net& net) {
  std::vector<int> outLayers = net.getUnconnectedOutLayers();
  std::vector<std::string> layerNames = net.getLayerNames();

  std::vector<std::string> outLayerNames;
  for (auto& layerIndex : outLayers) {
    outLayerNames.push_back(layerNames[layerIndex - 1]);  // Subtract 1 to get the correct index
  }

  return outLayerNames;
}
OpenCVDetector::OpenCVDetector(const Configs& config) : BaseDetector(config) {

    net = cv::dnn::readNetFromDarknet(config.modelConfig, config.modelWeights);
    if (config.device == "CUDA") {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } else {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

void OpenCVDetector::detect(const cv::Mat& frame, std::vector<cv::Rect>& boxes,
                            std::vector<float>& confidences, std::vector<int>& classIds) {
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outs;
    net.forward(outs, getOutputsNames(net));

    postProcess(frame, outs, boxes, confidences, classIds);
    
}



void OpenCVDetector::postProcess(const cv::Mat& frame, const std::vector<cv::Mat>& outs,
                                 std::vector<cv::Rect>& boxes, std::vector<float>& confidences,
                                 std::vector<int>& classIds) {
    for (const auto& output : outs) {
        for (int i = 0; i < output.rows; ++i) {
            cv::Mat scores = output.row(i).colRange(5, output.cols);
            cv::Point classIdPoint;
            double confidence;
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confidenceThreshold) {
                float* data = (float*)output.data + i * output.cols;
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                boxes.emplace_back(left, top, width, height);
                confidences.push_back((float)confidence);
                classIds.push_back(classIdPoint.x);
            }
        }
    }

    // Non-maximum suppression to remove overlapping boxes
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidenceThreshold, nmsThreshold, indices);
    std::vector<cv::Rect> filteredBoxes;
    std::vector<float> filteredConfidences;
    std::vector<int> filteredClassIds;
    for (int idx : indices) {
        filteredBoxes.push_back(boxes[idx]);
        filteredConfidences.push_back(confidences[idx]);
        filteredClassIds.push_back(classIds[idx]);
    }

    boxes = std::move(filteredBoxes);
    confidences = std::move(filteredConfidences);
    classIds = std::move(filteredClassIds);
}

} // namespace vision_core