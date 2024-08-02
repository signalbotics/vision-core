cmake_minimum_required (VERSION 3.8)

project(nreal_light)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O0 -g -rdynamic -g2 -ggdb")

set(CMAKE_CXX_STANDARD 17)

SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# Use pkg-config to find the hidapi package
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-libusb)

include_directories(${TENSORRT_INCLUDE_DIR})
message(STATUS "CMAke CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/local/cuda/include)
include_directories(${HIDAPI_INCLUDE_DIRS})

file(GLOB lib_src "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")


add_library(${PROJECT_NAME} SHARED ${lib_src} src/nreal_light.cpp)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS} 
    ${HIDAPI_LIBRARIES}
    hidapi-libusb 
)
