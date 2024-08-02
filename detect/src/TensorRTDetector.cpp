#ifdef WITH_TENSORRT
#include "detect/TensorRTDetector.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <fstream>
#include <iostream>
#include <vector>

#define isFP16 true
#ifndef CUDA_CHECK
#define CUDA_CHECK(callstr)\
{\
    cudaError_t error_code = callstr;\
    if (error_code != cudaSuccess) {\
        std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__;\
        assert(0);\
    }\
}
#endif  // CUDA_CHECK
class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        // Only output logs with severity greater than warning
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} logger;


namespace vision_core{

TensorRTDetector::TensorRTDetector(const Configs& config) : BaseDetector(config) {
    if (config.onnxModelPath.find(".onnx") == std::string::npos)
    {   
        std::cout << "Reading the engine file " << config.onnxModelPath << std::endl;
        // read the engine file
        std::ifstream engineFile(config.onnxModelPath, std::ios::binary);
        if (engineFile)
        {
            engineFile.seekg(0, engineFile.end);
            int length = engineFile.tellg();
            engineFile.seekg(0, engineFile.beg);
            std::vector<char> engineData(length);
            engineFile.read(engineData.data(), length);
            engineFile.close();
            
            runtime = nvinfer1::createInferRuntime(logger);
            engine = runtime->deserializeCudaEngine(engineData.data(), length, nullptr);
            context = engine->createExecutionContext();
        }
        
    } else
    {
        std::cout << "Build the engine first from the ONNX model" << config.onnxModelPath << std::endl;
        buildEngine(config);
    }

    // // Print binding names
    // for (int i = 0; i < engine->getNbBindings(); ++i) {
    //     std::cout << "Binding " << i << ": " << engine->getBindingName(i) << std::endl;
    // }
}

void TensorRTDetector::detect(const cv::Mat& frame, std::vector<cv::Rect>& boxes,
                              std::vector<float>& confidences, std::vector<int>& classIds) {
    // Prepare frame data for TensorRT, may include specific preprocessing different from OpenCV
    cv::Mat gpuImg, rgbMat;
    cv::Mat resizedMat;

    const int inputIndex = engine->getBindingIndex("images");
    const int outputIndex = engine->getBindingIndex("output0");

    if (inputIndex == -1 || outputIndex == -1) {
        std::cerr << "Invalid binding index detected";
        return; 
    }
    nvinfer1::Dims inputDims = engine->getBindingDimensions(inputIndex);
    nvinfer1::Dims outputDims = engine->getBindingDimensions(outputIndex);

    int inputHeight = inputDims.d[2];
    int inputWidth = inputDims.d[3];
    cv::resize(frame, resizedMat, cv::Size(inputWidth, inputHeight));
    
    cv::cvtColor(resizedMat, rgbMat, cv::COLOR_BGR2RGB);
    if (rgbMat.type() != CV_32FC3) {
        rgbMat.convertTo(rgbMat, CV_32FC3);
    }
    // Calculate buffer sizes
    const size_t inputSize = inputWidth * inputHeight * 3 * sizeof(float); // Assuming RGB
    std::cout << "putput dims: " << outputDims.d[0] << " " << outputDims.d[1] << " " << outputDims.d[2] << " " << outputDims.d[3] << std::endl;
    const size_t outputSize = outputDims.d[0] * outputDims.d[1] * outputDims.d[2] * sizeof(float);

    std::cout << "Input size: " << inputSize << ", Output size: " << outputSize << std::endl;

    void* buffers[2] = {nullptr, nullptr};
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    CUDA_CHECK(cudaMalloc((void **)&buffers[inputIndex], inputSize));
    CUDA_CHECK(cudaMalloc((void **)&buffers[outputIndex], outputSize));

    if (buffers[inputIndex] == nullptr || buffers[outputIndex] == nullptr) {
        std::cerr << "Buffer allocation error: One of the buffers is null.\n";
        return;
    }

    cudaError_t status;
    std::cout << "Input buffer allocated at: " << buffers[inputIndex] << std::endl;
    std::cout << "Output buffer allocated at: " << buffers[outputIndex] << std::endl;

    status = cudaMemcpy(buffers[inputIndex], rgbMat.ptr<float>(), inputSize, cudaMemcpyHostToDevice);
    if (status != cudaSuccess) {
        std::cerr << "Failed to copy input to GPU: " << cudaGetErrorString(status) << std::endl;
        cudaFree(buffers[inputIndex]);
        cudaFree(buffers[outputIndex]);
        return;
    }

    cudaStreamSynchronize(0);  // Ensure all GPU operations are complete before proceeding

    if (!context->setBindingDimensions(inputIndex, nvinfer1::Dims4(1, 3, inputHeight, inputWidth))) {
        std::cerr << "Failed to set binding dimensions.\n";
        cudaFree(buffers[inputIndex]);
        cudaFree(buffers[outputIndex]);
        return;
    }

    // Verify buffer pointers before calling enqueueV2
    if (!buffers[inputIndex] || !buffers[outputIndex]) {
        std::cerr << "Buffer allocation error: One of the buffers is null.\n";
        cudaFree(buffers[inputIndex]);
        cudaFree(buffers[outputIndex]);
        return;
    }

    try {
        // Run inference
        if (!context->enqueueV2(buffers, 0, nullptr)) {
            std::cerr << "Inference failed." << std::endl;
            cudaFree(buffers[inputIndex]);
            cudaFree(buffers[outputIndex]);
            return;
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception during inference: " << e.what() << '\n';
        cudaFree(buffers[inputIndex]);
        cudaFree(buffers[outputIndex]);
        return;
    }
    std::vector<float> output(outputSize / sizeof(float)); // Declare and initialize output vector

    // Copy output data back to host
    status = cudaMemcpy(output.data(), buffers[outputIndex], outputSize, cudaMemcpyDeviceToHost);
    if (status != cudaSuccess) {
        std::cerr << "Failed to copy output data from GPU: " << cudaGetErrorString(status) << std::endl;
        cudaFree(buffers[inputIndex]);
        cudaFree(buffers[outputIndex]);
        return;
    }

    // Free GPU memory
    cudaFree(buffers[inputIndex]);
    cudaFree(buffers[outputIndex]);


    postProcess(frame, output, boxes, confidences, classIds);
    
}


void TensorRTDetector::postProcess(const cv::Mat& frame, std::vector<float> output,
                                   std::vector<cv::Rect>& boxes, std::vector<float>& confidences,
                                   std::vector<int>& classIds) {
    int numClasses = classNames.size();
    for (int i = 0; i < output.size() / (5 + numClasses); ++i) {
        float* data = &output[i * (5 + numClasses)];
        float x = data[0]; // Example assuming these are normalized to width and height
        float y = data[1];
        float w = data[2];
        float h = data[3];
        float confidence = data[4];

        if (confidence > confidenceThreshold) {
            float* classScores = data + 5;
            int maxClassId = std::max_element(classScores, classScores + numClasses) - classScores;
            float maxClassScore = classScores[maxClassId];

            if (maxClassScore > confidenceThreshold) {
                int centerX = static_cast<int>(x * frame.cols);
                int centerY = static_cast<int>(y * frame.rows);
                int width = static_cast<int>(w * frame.cols);
                int height = static_cast<int>(h * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                boxes.push_back(cv::Rect(left, top, width, height));
                confidences.push_back(confidence * maxClassScore);
                classIds.push_back(maxClassId);
            }
        }
    }


    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidenceThreshold, nmsThreshold, indices);

    std::vector<cv::Rect> nmsBoxes;
    std::vector<float> nmsConfidences;
    std::vector<int> nmsClassIds;
    for (int idx : indices) {
        nmsBoxes.push_back(boxes[idx]);
        nmsConfidences.push_back(confidences[idx]);
        nmsClassIds.push_back(classIds[idx]);
    }

    boxes = nmsBoxes;
    confidences = nmsConfidences;
    classIds = nmsClassIds;
}
void TensorRTDetector::buildEngine(const Configs& config) {
    std::cout << "Building the engine" << std::endl;
    auto builder = nvinfer1::createInferBuilder(logger);
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::IBuilderConfig* nvconfig = builder->createBuilderConfig();

    if (isFP16)
    {
        nvconfig->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);
    bool parsed = parser->parseFromFile(config.onnxModelPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO));
    

    if (!parsed)
    {
        std::cout << "Error parsing ONNX file" << std::endl;
        return;
    }
    nvinfer1::IHostMemory* plan{ builder->buildSerializedNetwork(*network, *nvconfig) };

    runtime = nvinfer1::createInferRuntime(logger);
    engine = runtime->deserializeCudaEngine(plan->data(), plan->size(), nullptr);
    context = engine->createExecutionContext();
    std::string enginePath = config.onnxModelPath.substr(0, config.onnxModelPath.find_last_of('.')) + ".engine";
    saveEngine(config.onnxModelPath);
    
}

void TensorRTDetector::saveEngine(const std::string& enginePath) {
    // Serialize the engine to a file
    if (engine)
    {
        std::ofstream engineFile(enginePath, std::ios::binary);
        if (engineFile)
        {
            nvinfer1::IHostMemory* serializedEngine = engine->serialize();
            engineFile.write((char*)serializedEngine->data(), serializedEngine->size());
            serializedEngine->destroy();
            engineFile.close();
        }
    }
}


} // namespace vision_core

#endif // WITH_TENSORRT