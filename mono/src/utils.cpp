#include "utils.h"

std::tuple<cv::Mat, int, int> resize_depth(cv::Mat& img, int w, int h)
{
	cv::Mat result;
	int nw, nh;
	int ih = img.rows;
	int iw = img.cols;
	float aspectRatio = (float)img.cols / (float)img.rows;

	if (aspectRatio >= 1)
	{
		nw = w;
		nh = int(h / aspectRatio);
	}
	else
	{
		nw = int(w * aspectRatio);
		nh = h;
	}
	cv::resize(img, img, cv::Size(nw, nh));
	result = cv::Mat::ones(cv::Size(w, h), CV_8UC3) * 128; // Directly create a 3-channel image
	cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

	cv::Mat out(h, w, CV_8UC3, cv::Scalar(128, 128, 128)); // Initialize with gray color
	img.copyTo(out(cv::Rect((w - nw) / 2, (h - nh) / 2, nw, nh)));

	std::tuple<cv::Mat, int, int> res_tuple = std::make_tuple(out, (w - nw) / 2, (h - nh) / 2);

	return res_tuple;
}


std::tuple<cv::Mat, int, int> resize_to_fixed_size(cv::Mat& img)
{
	cv::Mat result;
	int nw = 518;
	int nh = 518;
	cv::resize(img, img, cv::Size(nw, nh));
	result = cv::Mat::ones(cv::Size(nw, nh), CV_8UC3) * 128; // Directly create a 3-channel image
	cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

	cv::Mat out(nh, nw, CV_8UC3, cv::Scalar(128, 128, 128)); // Initialize with gray color
	img.copyTo(out);

	std::tuple<cv::Mat, int, int> res_tuple = std::make_tuple(out, 0, 0);

	return res_tuple;
}