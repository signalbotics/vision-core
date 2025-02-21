cmake_minimum_required(VERSION 3.9)
project(monodepth)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

# Set TensorRT root directory (adjust this path to your TensorRT installation)
# set(TENSORRT_ROOT "/usr/local/cuda-12.2/targets/x86_64-linux/")
set(TENSORRT_ROOT "/usr/local/TensorRT")

# Find TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS ${TENSORRT_ROOT}/include)
find_library(TENSORRT_LIB nvinfer HINTS ${TENSORRT_ROOT})
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS ${TENSORRT_ROOT})
# find_library(NVPARSERS_LIB nvparsers HINTS ${TENSORRT_ROOT})
find_library(NVONNXPARSER_LIB nvonnxparser HINTS ${TENSORRT_ROOT})
find_library(CUDA_CUDART_LIBRARY libcudart.so HINTS ${CUDA_ROOT}/lib64)

# Print TensorRT paths
message(STATUS "TensorRT include directory: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TensorRT library: ${TENSORRT_LIB}")
message(STATUS "NVINFER_PLUGIN_LIB: ${NVINFER_PLUGIN_LIB}")
# message(STATUS "NVPARSERS_LIB: ${NVPARSERS_LIB}")
message(STATUS "NVONNXPARSER_LIB: ${NVONNXPARSER_LIB}")
# Verify that all TensorRT components are found
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIB OR NOT NVINFER_PLUGIN_LIB OR NOT NVONNXPARSER_LIB)
    message(FATAL_ERROR "Could not find all TensorRT components. Please check your TENSORRT_ROOT path.")
endif()


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${TENSORRT_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(include)
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
link_directories(${TENSORRT_ROOT}/lib)
link_directories(${CUDA_LIBRARY_DIRS})

file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE HDR_FILES include/*.hpp)

add_library(${PROJECT_NAME} SHARED ${HDR_FILES} ${SRC_FILES})

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# Print CUDA variables
message(STATUS "CUDA_LIBRARIES: ${CUDA_LIBRARIES}")
message(STATUS "CUDA_CUDART_LIBRARY: ${CUDA_CUDART_LIBRARY}")

target_include_directories(${PROJECT_NAME} PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBS}
    ${TENSORRT_LIB}
    ${NVINFER_PLUGIN_LIB}
    # ${NVPARSERS_LIB}
    ${NVONNXPARSER_LIB}
    ${CUDA_CUDART_LIBRARY}
)
