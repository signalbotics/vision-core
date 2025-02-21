#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "utils.hpp"

class MonoDepth
{
public:
	MonoDepth();
	cv::Mat  predict(cv::Mat& image);
	~MonoDepth();
	
private:
	class Impl;
	std::unique_ptr<Impl> pImpl;
};
