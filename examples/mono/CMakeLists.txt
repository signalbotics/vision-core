CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(mono_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(OpenCV REQUIRED)
find_package(vision_core REQUIRED)
find_package(PCL REQUIRED)
if(vision_core_FOUND)
  message(STATUS "vision_core_INCLUDE_DIRS: ${vision_core_INCLUDE_DIRS}")
  message(STATUS "vision_core_LIBRARIES: ${vision_core_LIBRARIES}")
else()
  message(STATUS "vision_core not found")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${vision_core_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

FILE(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/mono_cv.cpp")

add_executable(${PROJECT_NAME}_cv ${SRC_FILES})

ament_target_dependencies(${PROJECT_NAME}_cv
    rclcpp 
    sensor_msgs 
    message_filters 
    cv_bridge
)

target_link_libraries(${PROJECT_NAME}_cv
    ${OpenCV_LIBS}
    ${vision_core_LIBRARIES}
    ${PCL_LIBRARIES}
)
#check if ros 2 is installed
if (NOT DEFINED ENV{AMENT_PREFIX_PATH})
    message(WARNING "ROS 2 is not installed. Skipping ROS 2 specific code.")
    return()
else()
  message(STATUS "ROS 2 is installed. Adding ROS 2 specific code.")
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(cv_bridge REQUIRED)

  FILE(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/mono_ros.cpp")

  add_executable(${PROJECT_NAME}_ros ${SRC_FILES})

  ament_target_dependencies(${PROJECT_NAME}_ros
      rclcpp 
      sensor_msgs 
      message_filters 
      cv_bridge
  )

  target_link_libraries(${PROJECT_NAME}_ros
      ${OpenCV_LIBS}
      ${vision_core_LIBRARIES}
      ${PCL_LIBRARIES}
  )
endif()