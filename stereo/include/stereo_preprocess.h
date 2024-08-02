#ifndef __STEREO_PREPROCESS_H
#define __STEREO_PREPROCESS_H

#include <cuda_runtime.h>
#include <cstdint>


struct AffineMatrix 
{
  float value[6];
};



void stereo_preprocess(uint8_t* src, float* dst, int src_width, int src_height, cudaStream_t stream);

void stereo_reprojectImageTo3D(uint8_t* left_img,float*disparity,float*pointcloud,float*Q_device,int disparity_rows,int disparity_cols);

#endif  // __STEREO_PREPROCESS_H
