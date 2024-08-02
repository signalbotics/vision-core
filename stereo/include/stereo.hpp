
#pragma once
#ifndef _STEREO_ALGORITHM_
#define _STEREO_ALGORITHM_
#ifdef _WINDOWS 
#define STEREO_ALGORITHM_EXPORTS
#ifdef STEREO_ALGORITHM_EXPORTS
#define STEREO __declspec(dllexport)
#else
#define STEREO __declspec(dllimport)
#endif
#else
#define STEREO
#endif
#endif // !_V8_3DAlgorithm_

#include<stdio.h>
#include<opencv2/opencv.hpp>

namespace vision_core
{
//Description   ModuleConfig           
//Params1       m_3DAlgorithmCallBack1                  
//Params2       config					
//Return		int						
//				other					
extern "C" STEREO void* Initialize(char*calibration_path);

//Description   
//Params		img			
//Return        int          
extern "C" STEREO int runStereo(void* p,cv::Mat&left_image,cv::Mat&right_image,float*pointcloud,cv::Mat&disparity);

//Description   
//Params		img			
//Return        int          
extern "C" STEREO int getDisparity(void* p,cv::Mat&left_image,cv::Mat&right_image,float*pointcloud,cv::Mat&disparity);

//Description   
//Params		
//Return
extern "C" STEREO const char* Version(void* p);

//Description   
//Params		
//Return		int			
extern "C" STEREO int Release(void* p);

} // namespace vision_core