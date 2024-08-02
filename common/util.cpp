

#include "util.hpp"




Configs parseConfigFile(const std::string& configFile) {
    std::cout << "Parsing configuration file: " << configFile << std::endl;
    YAML::Node config = YAML::LoadFile(configFile);
    Configs configs;
    configs.videoPath = config["general"]["video_path"].as<std::string>();
    configs.device = config["detect"]["device"].as<std::string>();
    configs.useOpenVINO = config["detect"]["use_openvino"].as<bool>();
    configs.useTensorRT = config["detect"]["use_tensorrt"].as<bool>();
    configs.renderer = config["detect"]["renderer"].as<std::string>();
    configs.useRenderingThread = config["detect"]["use_rendering_thread"].as<bool>();
    configs.useInferenceThread = config["detect"]["use_inference_thread"].as<bool>();
    configs.modelConfig = config["detect"]["model"]["config"].as<std::string>();
    configs.modelWeights = config["detect"]["model"]["weights"].as<std::string>();
    configs.classesFile = config["detect"]["model"]["classes"].as<std::string>();
    configs.colorsFile = config["detect"]["model"]["colors"].as<std::string>();
    configs.confidenceThreshold = config["detect"]["model"]["confidence_threshold"].as<float>();
    configs.nmsThreshold = config["detect"]["model"]["nms_threshold"].as<float>();
    configs.onnxModelPath = config["detect"]["model"]["onnx_model_path"].as<std::string>();
    configs.openvinoDevice = config["detect"]["model"]["openvino_device"].as<std::string>();
    configs.openvinoModelXML = config["detect"]["model"]["openvino_model_xml"].as<std::string>();
    configs.openvinoModelBIN = config["detect"]["model"]["openvino_model_bin"].as<std::string>();
    return configs;
}
    

std::vector<std::string> loadClassNames(const std::string& filename) {
    std::cout << "Loading class names from file: " << filename << std::endl;
    std::vector<std::string> classNames;
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
    classNames.push_back(line);
    }
    return classNames;
}



std::vector<cv::Scalar> loadColors(const std::string& filename) {
    std::cout << "Loading colors from file: " << filename << std::endl;
    std::vector<cv::Scalar> colors;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string item;
    std::vector<int> rgbValues;

    while (std::getline(ss, item, ',')) {
        try {
            rgbValues.push_back(std::stoi(item));
        } catch (const std::invalid_argument&) {
        std::cerr << "Invalid number in color file: " << item << std::endl;
        continue;
        }
    }

    if (rgbValues.size() == 3) {
        colors.emplace_back(cv::Scalar(rgbValues[0], rgbValues[1], rgbValues[2]));
    } else {
        std::cerr << "Error parsing color line: " << line << std::endl;
    }
    }

    std::cout << "Loaded " << colors.size() << " colors." << std::endl;
    return colors;
}