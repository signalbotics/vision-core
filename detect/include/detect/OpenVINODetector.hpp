#pragma once
#include "BaseDetector.hpp"


#ifdef WITH_OPENVINO
#include <openvino/openvino.hpp>

namespace vision_core{

class OpenVINODetector : public BaseDetector {
public:
    OpenVINODetector(const Configs& config);
    void detect(const cv::Mat& frame, std::vector<cv::Rect>& boxes,
                std::vector<float>& confidences, std::vector<int>& classIds) override;
private:
    ov::Core core;
    ov::InferRequest infer_request;
    ov::CompiledModel compiled_model;

    void decodeYOLOOutput(const ov::Tensor& output_tensor, int grid_size, const cv::Mat& frame,
                                        std::vector<cv::Rect>& boxes, std::vector<float>& confidences, 
                                        std::vector<int>& classIds);
                                        
};
} // namespace vision_core

#endif // WITH_OPENVINO
