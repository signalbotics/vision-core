CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(monodepth)

SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")

find_package(CUDA  REQUIRED)
find_package(OpenCV REQUIRED)

# Find TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS ${TENSORRT_ROOT}/include)
find_library(TENSORRT_LIB nvinfer HINTS ${TENSORRT_ROOT}/lib)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS ${TENSORRT_ROOT}/lib)
find_library(NVPARSERS_LIB nvparsers HINTS ${TENSORRT_ROOT}/lib)
find_library(NVONNXPARSER_LIB nvonnxparser HINTS ${TENSORRT_ROOT}/lib)


include_directories(${OpenCV_INCLUDE_DIRS})
# Find and include Open3D
# find_package(Open3D REQUIRED)
# Set TensorRT path if not set in environment variables

# Include TensorRT
include_directories(${TENSORRT_DIR}/include)
link_directories(${TENSORRT_DIR}/lib)
set(TENSORRT_LIBS nvinfer nvinfer_plugin nvparsers nvonnxparser)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)
FILE(GLOB_RECURSE HDR_FILES include/*.h)

add_library(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_libraries(${PROJECT_NAME} "/usr/local/cuda/targets/aarch64-linux/lib/libcudart.so")
else()
    target_link_libraries(${PROJECT_NAME} "/usr/local/cuda/lib64/libcudart.so")
endif()
target_link_libraries(${PROJECT_NAME} 
    ${TENSORRT_LIBS}
    ${OpenCV_LIBS}
    ${NVINFER_PLUGIN_LIB} 
    ${NVPARSERS_LIB} 
    ${NVONNXPARSER_LIB}
    ${CUDA_LIBRARIES}
)