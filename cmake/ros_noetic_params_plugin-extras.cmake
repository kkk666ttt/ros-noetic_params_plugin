# catkin consumers also need the standalone Core and yaml-cpp link dependencies.
find_package(vision_kernel CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
list(APPEND ros_noetic_params_plugin_LIBRARIES vision::kernel yaml-cpp)

if(NOT TARGET vision::ros_params)
  add_library(vision::ros_params INTERFACE IMPORTED)
  set_target_properties(vision::ros_params PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${ros_noetic_params_plugin_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${ros_noetic_params_plugin_LIBRARIES}"
    INTERFACE_COMPILE_FEATURES cxx_std_17)
endif()
