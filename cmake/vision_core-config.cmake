# vision-core-config.cmake
include(CMakeFindDependencyMacro)

# Set the include directory
set(vision_core_INCLUDE_DIRS "/usr/local/vision_core/include")
# Set the library directory
set(vision_core_LIBRARIES 

    "/usr/local/vision_core/lib/libstereo_depth.so" 
    "/usr/local/vision_core/lib/libnreal_light.so" 
    "/usr/local/vision_core/lib/libsegment.so" 
    "/usr/local/vision_core/lib/libmonodepth.so"
    "/usr/local/vision_core/lib/libdetect.so"
)
# Create an imported target for the library
add_library(vision_core SHARED IMPORTED)
set_target_properties(vision_core PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${vision_core_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${vision_core_LIBRARIES}"
)

# Provide include directories and libraries for find_package calls
set(vision_core_INCLUDE_DIR ${vision_core_INCLUDE_DIRS})
set(vision_core_LIBRARIES ${vision_core_LIBRARIES})
