#include "mono_depth.hpp"
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include "NvInferPlugin.h"
#include "cuda_runtime_api.h"
#include <filesystem>

#define isFP16 true

using namespace nvinfer1;

/**
 * @brief Setting up Tensorrt logger
*/
class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char* msg) noexcept override
    {
        // Only output logs with severity greater than warning
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
} logger;

class MonoDepth::Impl {
public:
	int input_w = 518;
	int input_h = 518;
	float mean[3] = { 123.675, 116.28, 103.53 };
	float std[3] = { 58.395, 57.12, 57.375 };


	std::vector<float> preprocess(cv::Mat& image);
	std::vector<DepthEstimation> postprocess(std::vector<int> mask, int img_w, int img_h);

    nvinfer1::IRuntime* runtime;
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    void* buffer[2];
    float* depth_data;
    cudaStream_t stream;
    
    void build(std::string onnxPath, nvinfer1::ILogger& logger);
    bool saveEngine(const std::string& filename);
    ~Impl() {
        cudaFree(stream);
        cudaFree(buffer[0]);
        cudaFree(buffer[1]);
        delete[] depth_data;
    }
};

/**
 * @brief MonoDepth`s constructor
*/
MonoDepth::MonoDepth() : pImpl(new Impl)
{   
    std::string model_path = "/usr/local/vision_core/models/light_mono.engine";
    // Add error checking
    if (!std::filesystem::exists(model_path)) {
        throw std::runtime_error("Engine file not found: " + model_path);
    }
    // Deserialize an engine
    if (model_path.find(".onnx") == std::string::npos)
    {   
        // read the engine file
        std::ifstream engineStream(model_path, std::ios::binary);
        if (!engineStream) {
            throw std::runtime_error("Failed to open engine file: " + model_path);
        }
        engineStream.seekg(0, std::ios::end);
        const size_t modelSize = engineStream.tellg();
        engineStream.seekg(0, std::ios::beg);
        std::unique_ptr<char[]> engineData(new char[modelSize]);
        engineStream.read(engineData.get(), modelSize);
        engineStream.close();
        // create tensorrt model
        pImpl->runtime = nvinfer1::createInferRuntime(logger);
        if (!pImpl->runtime) {
            throw std::runtime_error("Failed to create TensorRT runtime");
        }
        pImpl->engine = pImpl->runtime->deserializeCudaEngine(engineData.get(), modelSize);
        if (!pImpl->engine) {
            throw std::runtime_error("Failed to deserialize CUDA engine");
        }
        pImpl->context = pImpl->engine->createExecutionContext();
        if (!pImpl->context) {
            throw std::runtime_error("Failed to create execution context");
        }
        std::cout << "vision_core::MonoDepth Intialized" << std::endl;
    } else if (model_path.find(".onnx") != std::string::npos)     // Build an engine from an onnx model
    {
        pImpl->build(model_path, logger);
        pImpl->saveEngine(model_path);
    }

    // Define input dimensions
    auto input_dims = pImpl->engine->getBindingDimensions(0);
    pImpl->input_h = input_dims.d[2];
    pImpl->input_w = input_dims.d[3];

    // create CUDA stream
    cudaStreamCreate(&pImpl->stream);

    cudaMalloc(&pImpl->buffer[0], 3 * pImpl->input_h * pImpl->input_w * sizeof(float));
    cudaMalloc(&pImpl->buffer[1], pImpl->input_h * pImpl->input_w * sizeof(float));

    pImpl->depth_data = new float[pImpl->input_h * pImpl->input_w];
}

/**
 * @brief RTMSeg`s destructor
*/
MonoDepth::~MonoDepth() = default;  // Needed for unique_ptr with incomplete type

/**
 * @brief Network preprocessing function
 * @param image Input image
 * @return Processed Tensor
*/
std::vector<float> MonoDepth::Impl::preprocess(cv::Mat& image)
{
    std::tuple<cv::Mat, int, int> resized = resize_to_fixed_size(image);
    cv::Mat resized_image = std::get<0>(resized);
    std::vector<float> input_tensor;
    for (int k = 0; k < 3; k++)
    {
        for (int i = 0; i < resized_image.rows; i++)
        {
            for (int j = 0; j < resized_image.cols; j++)
            {
                input_tensor.emplace_back(((float)resized_image.at<cv::Vec3b>(i, j)[k] - mean[k]) / std[k]);
            }
        }
    }
    return input_tensor;
}

cv::Mat MonoDepth::predict(cv::Mat& image)
{
    cv::Mat clone_image;
    image.copyTo(clone_image);

    int img_w = image.cols;
    int img_h = image.rows;

    // Preprocessing
    std::vector<float> input = pImpl->preprocess(clone_image);
    cudaMemcpyAsync(pImpl->buffer[0], input.data(), 3 * pImpl->input_h * pImpl->input_w * sizeof(float), cudaMemcpyHostToDevice, pImpl->stream);
    // Inference depth model
    pImpl->context->enqueueV2(pImpl->buffer, pImpl->stream, nullptr);
    cudaStreamSynchronize(pImpl->stream);
    // Postprocessing
    cudaMemcpyAsync(pImpl->depth_data, pImpl->buffer[1], pImpl->input_h * pImpl->input_w * sizeof(float), cudaMemcpyDeviceToHost);
    // Convert the entire depth_data vector to a CV_32FC1 Mat
    cv::Mat depth_mat(pImpl->input_h, pImpl->input_w, CV_32FC1, pImpl->depth_data);
    cv::Mat depth_mat1 = 1/depth_mat;

    cv::resize(depth_mat1, depth_mat1, cv::Size(img_w, img_h));
    return depth_mat1;
}

void MonoDepth::Impl::build(std::string onnxPath, nvinfer1::ILogger& logger)
{
    auto builder = createInferBuilder(logger);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    INetworkDefinition* network = builder->createNetworkV2(explicitBatch);
    IBuilderConfig* config = builder->createBuilderConfig();
    if (isFP16) {
        config->setFlag(BuilderFlag::kFP16);
    }
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);
    parser->parseFromFile(onnxPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO));
    IHostMemory* plan{ builder->buildSerializedNetwork(*network, *config) };

    runtime = createInferRuntime(logger);
    engine = runtime->deserializeCudaEngine(plan->data(), plan->size(), nullptr);
    context = engine->createExecutionContext();

    delete network;
    delete config;
    delete parser;
    delete plan;
}

bool MonoDepth::Impl::saveEngine(const std::string& onnxpath)
{
    std::string engine_path;
    size_t dotIndex = onnxpath.find_last_of(".");
    if (dotIndex != std::string::npos) {
        engine_path = onnxpath.substr(0, dotIndex) + ".engine";
    } else {
        return false;
    }

    if (engine) {
        nvinfer1::IHostMemory* data = engine->serialize();
        std::ofstream file;
        file.open(engine_path, std::ios::binary | std::ios::out);
        if (!file.is_open()) {
            std::cout << "Create engine file" << engine_path << " failed" << std::endl;
            return 0;
        }
        file.write((const char*)data->data(), data->size());
        file.close();
        delete data;
    }
    return true;
}
