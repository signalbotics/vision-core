cmake_minimum_required(VERSION 3.8)
project(vision_core)
set(CMAKE_CXX_STANDARD 17)

# Build options for each module
option(BUILD_DETECT "Build detection module" OFF)
option(BUILD_SEGMENT "Build segmentation module" OFF)
option(BUILD_DEPTH "Build depth estimation modules (mono and stereo)" OFF)
option(BUILD_ALL "Build all modules" OFF)

# If BUILD_ALL is ON, enable all modules
if(BUILD_ALL)
    set(BUILD_DETECT ON)
    set(BUILD_SEGMENT ON)
    set(BUILD_DEPTH ON)
endif()

# Add modules based on build options
if(BUILD_DETECT)
    add_subdirectory("detect")
endif()

if(BUILD_SEGMENT)
    add_subdirectory("segment")
endif()

if(BUILD_DEPTH)
    add_subdirectory("mono")
    add_subdirectory("stereo")
endif()

# Install common files
install(DIRECTORY common/
        DESTINATION usr/local/vision_core/common
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.cpp")

install(DIRECTORY cmake/
        DESTINATION usr/local/vision_core/cmake
        FILES_MATCHING PATTERN "*.cmake")
