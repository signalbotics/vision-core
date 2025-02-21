cmake_minimum_required (VERSION 3.8)

project(stereo_depth)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O0 -g -rdynamic -g2 -ggdb")


SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

set(TENSORRT_ROOT "/usr/src/tensorrt")
# Find TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS ${TENSORRT_ROOT}/include)
find_library(TENSORRT_LIB nvinfer HINTS ${TENSORRT_ROOT}/lib)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS ${TENSORRT_ROOT}/lib)
find_library(NVPARSERS_LIB nvparsers HINTS ${TENSORRT_ROOT}/lib)
find_library(NVONNXPARSER_LIB nvonnxparser HINTS ${TENSORRT_ROOT}/lib)
# Use pkg-config to find the hidapi package
include_directories(${TENSORRT_INCLUDE_DIR})
message(STATUS "CMAke CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/local/cuda/include)

file(GLOB lib_src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB common_src "${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp")


include_directories(${TensorRT_INCLUDE})
link_directories(${TensorRT_LIB}/lib)

cuda_add_library(${PROJECT_NAME} SHARED ${lib_src} ${common_src} src/stereo_preprocess.cu)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(${PROJECT_NAME} "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so")
else()
    target_link_libraries(${PROJECT_NAME} "/usr/local/cuda/lib64/libcudart.so")
endif()
target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS} 
    ${TENSORRT_LIB} 
    ${NVINFER_PLUGIN_LIB} 
    # ${NVPARSERS_LIB} 
    ${NVONNXPARSER_LIB}
    ${CUDA_LIBRARIES}
)
