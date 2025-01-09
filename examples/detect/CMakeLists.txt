cmake_minimum_required(VERSION 3.10)
project(detect_app)


set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(vision_core REQUIRED)
if(vision_core_FOUND)
  message(STATUS "vision_core_INCLUDE_DIRS: ${vision_core_INCLUDE_DIRS}")
  message(STATUS "vision_core_LIBRARIES: ${vision_core_LIBRARIES}")
else()
  message(STATUS "vision_core not found")
  exit()
endif()
include_directories(
  include
  ${vision_core_INCLUDE_DIRS}
)
# find src files
file(GLOB_RECURSE SRC_FILES *.cpp)
file(GLOB_RECURSE HDR_FILES *.hpp)
add_executable(${PROJECT_NAME}
  ${SRC_FILES}
)
target_link_libraries(${PROJECT_NAME} 
  ${vision_core_LIBRARIES}
)
