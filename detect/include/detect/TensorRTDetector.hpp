#ifdef WITH_TENSORRT
#pragma once
#include <NvOnnxParser.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <opencv2/cudaimgproc.hpp>

#include "BaseDetector.hpp"

namespace vision_core{

class TensorRTDetector : public BaseDetector {
public:
    TensorRTDetector(const Configs& config);
    void detect(const cv::Mat& frame, std::vector<cv::Rect>& boxes,
                std::vector<float>& confidences, std::vector<int>& classIds) override;
    void postProcess(const cv::Mat& frame, std::vector<float> output,
                     std::vector<cv::Rect>& boxes, std::vector<float>& confidences,
                     std::vector<int>& classIds);
private:
    nvinfer1::IRuntime* runtime = nullptr;
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    nvinfer1::INetworkDefinition* network;
    
    void buildEngine(const Configs& config);
    void saveEngine(const std::string& enginePath);
};

} // namespace vision_core

#endif // WITH_TENSORRT