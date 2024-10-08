#include "mono_depth.hpp"
#include <NvOnnxParser.h>

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
/**
 * @brief MonoDepth`s constructor
*/
MonoDepth::MonoDepth()
{   
    std::string model_path = "/usr/local/vision_core/models/light_mono.engine";
    // Deserialize an engine
    if (model_path.find(".onnx") == std::string::npos)
    {
        // read the engine file
        std::ifstream engineStream(model_path, std::ios::binary);
        engineStream.seekg(0, std::ios::end);
        const size_t modelSize = engineStream.tellg();
        engineStream.seekg(0, std::ios::beg);
        std::unique_ptr<char[]> engineData(new char[modelSize]);
        engineStream.read(engineData.get(), modelSize);
        engineStream.close();

        // create tensorrt model
        runtime = nvinfer1::createInferRuntime(logger);
        engine = runtime->deserializeCudaEngine(engineData.get(), modelSize);
        context = engine->createExecutionContext();
    }
    // Build an engine from an onnx model
    else
    {
        build(model_path, logger);
        saveEngine(model_path);
    }

    // Define input dimensions
    auto input_dims = engine->getBindingDimensions(0);
    input_h = input_dims.d[2];
    input_w = input_dims.d[3];

    // create CUDA stream
    cudaStreamCreate(&stream);

    cudaMalloc(&buffer[0], 3 * input_h * input_w * sizeof(float));
    cudaMalloc(&buffer[1], input_h * input_w * sizeof(float));

    depth_data = new float[input_h * input_w];
}

/**
 * @brief RTMSeg`s destructor
*/
MonoDepth::~MonoDepth()
{
    cudaFree(stream);
    cudaFree(buffer[0]);
    cudaFree(buffer[1]);

    delete[] depth_data;
}

/**
 * @brief Network preprocessing function
 * @param image Input image
 * @return Processed Tensor
*/
std::vector<float> MonoDepth::preprocess(cv::Mat& image)
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

std::pair<cv::Mat, cv::Mat>  MonoDepth::predict(cv::Mat& image)
{
    cv::Mat clone_image;
    image.copyTo(clone_image);

    int img_w = image.cols;
    int img_h = image.rows;

    // Preprocessing
    std::vector<float> input = preprocess(clone_image);
    cudaMemcpyAsync(buffer[0], input.data(), 3 * input_h * input_w * sizeof(float), cudaMemcpyHostToDevice, stream);

    // Inference depth model
    context->enqueueV2(buffer, stream, nullptr);
    cudaStreamSynchronize(stream);

    // Postprocessing
    cudaMemcpyAsync(depth_data, buffer[1], input_h * input_w * sizeof(float), cudaMemcpyDeviceToHost);

    // Convert the entire depth_data vector to a CV_32FC1 Mat
    cv::Mat depth_mat(input_h, input_w, CV_32FC1, depth_data);
    cv::Mat depth_mat1 = depth_mat;
    cv::normalize(depth_mat, depth_mat, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Create a colormap from the depth data
    cv::Mat colormap;
    cv::applyColorMap(depth_mat, colormap, cv::COLORMAP_JET);
    // cv::applyColorMap(depth_mat, colormap, cv::COLORMAP_INFERNO);

    // Rescale the colormap
    int limX, limY;
    if (img_w > img_h)
    {
        limX = input_w;
        limY = input_w * img_h / img_w;
    }
    else
    {
        limX = input_w * img_w / img_h;
        limY = input_w;
    }
    cv::Mat rgb8Image;

    cv::resize(colormap, colormap, cv::Size(img_w, img_h));
    cv::cvtColor(colormap, rgb8Image, cv::COLOR_BGR2RGB);
    // cv::resize(depth_mat1, depth_mat1, cv::Size(img_w, img_h));
    return std::make_pair(rgb8Image, depth_mat1);
}


void MonoDepth::build(std::string onnxPath, nvinfer1::ILogger& logger)
{
    auto builder = createInferBuilder(logger);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    INetworkDefinition* network = builder->createNetworkV2(explicitBatch);
    IBuilderConfig* config = builder->createBuilderConfig();
    if (isFP16)
    {
        config->setFlag(BuilderFlag::kFP16);
    }
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, logger);
    bool parsed = parser->parseFromFile(onnxPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO));
    IHostMemory* plan{ builder->buildSerializedNetwork(*network, *config) };

    runtime = createInferRuntime(logger);
    engine = runtime->deserializeCudaEngine(plan->data(), plan->size(), nullptr);
    context = engine->createExecutionContext();

    delete network;
    delete config;
    delete parser;
    delete plan;
}

bool MonoDepth::saveEngine(const std::string& onnxpath)
{
    // Create an engine path from onnx path
    std::string engine_path;
    size_t dotIndex = onnxpath.find_last_of(".");
    if (dotIndex != std::string::npos) {
        engine_path = onnxpath.substr(0, dotIndex) + ".engine";
    }
    else
    {
        return false;
    }

    // Save the engine to the path
    if (engine)
    {
        nvinfer1::IHostMemory* data = engine->serialize();
        std::ofstream file;
        file.open(engine_path, std::ios::binary | std::ios::out);
        if (!file.is_open())
        {
            std::cout << "Create engine file" << engine_path << " failed" << std::endl;
            return 0;
        }
        file.write((const char*)data->data(), data->size());
        file.close();

        delete data;
    }
    return true;
}
