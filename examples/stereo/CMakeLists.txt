cmake_minimum_required (VERSION 3.8)
project(stereo_demo)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -Wall -g")
set(CMAKE_CXX_STANDARD 17)

file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

file(GLOB HDR_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

if(vision_core_FOUND)
  message(STATUS "vision_core_INCLUDE_DIRS: ${vision_core_INCLUDE_DIRS}")
  message(STATUS "vision_core_LIBRARIES: ${vision_core_LIBRARIES}")
else()
  message(STATUS "vision_core not found")
endif()


add_executable(${PROJECT_NAME} ${SRC_FILES} ${HDR_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs cv_bridge tf2_ros tf2_msgs)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${vision_core_LIBRARIES})
