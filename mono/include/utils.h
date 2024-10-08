#pragma once
#include <iostream>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

/**
 * @brief Depth_estimation structure
*/
struct DepthEstimation
{
	int x;
	int y;
	int label;

	DepthEstimation()
	{
		x = 0;
		y = 0;
		label = -1;
	}
};


std::tuple<cv::Mat, int, int> resize_depth(cv::Mat& img, int w, int h);

std::tuple<cv::Mat, int, int> resize_to_fixed_size(cv::Mat& img);