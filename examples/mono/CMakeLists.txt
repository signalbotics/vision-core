CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(monodepth)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_core REQUIRED)

if(vision_core_FOUND)
  message(STATUS "vision_core_INCLUDE_DIRS: ${vision_core_INCLUDE_DIRS}")
  message(STATUS "vision_core_LIBRARIES: ${vision_core_LIBRARIES}")
else()
  message(STATUS "vision_core not found")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${vision_core_INCLUDE_DIRS})

FILE(GLOB_RECURSE SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
FILE(GLOB_RECURSE HDR_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

add_executable(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp 
    sensor_msgs 
    message_filters 
    cv_bridge
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS}
    ${vision_core_LIBRARIES}
)