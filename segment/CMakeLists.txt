cmake_minimum_required(VERSION 3.12)
project(segment)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
set(TENSORRT_LIBS nvinfer nvinfer_plugin nvparsers nvonnxparser cudnn cublas cudart)
include_directories(${TENSORRT_DIR}/include)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("embed_platform on")
  include_directories(/usr/local/cuda/targets/aarch64-linux/include)
  link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
  message("embed_platform off")
  # cuda
  include_directories(/usr/local/cuda/include)
  link_directories(/usr/local/cuda/lib64)
endif()

add_library(segment SHARED
  src/segment.cpp
  src/trt_module.cpp
)
target_link_libraries(segment 
  ${OpenCV_LIBS}
  ${TENSORRT_LIBS}
  ${CUDA_CUDART_LIBRARY}  # Link against CUDA Runtime library
)
