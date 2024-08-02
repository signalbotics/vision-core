#ifndef UTIL_HPP
#define UTIL_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

struct Configs
{   
    std::string videoPath;
    std::string modelConfig;
    std::string modelWeights;
    std::string onnxModelPath;
    std::string classesFile;
    std::string colorsFile;
    bool useOpenVINO;
    bool useTensorRT;
    bool useRenderingThread;
    bool useInferenceThread;
    std::string renderer;
    std::string openvinoDevice;
    std::string openvinoModelXML;
    std::string openvinoModelBIN;
    std::string device;
    float confidenceThreshold;
    float nmsThreshold;
};

std::vector<std::string> loadClassNames(const std::string& filename);
std::vector<cv::Scalar> loadColors(const std::string& filename);
Configs parseConfigFile(const std::string& configFile);



#endif // UTIL_HPP