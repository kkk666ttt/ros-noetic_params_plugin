cmake_minimum_required(VERSION 3.16)
project(ros_noetic_params_plugin VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure)
# Core is an external standalone CMake prerequisite, installed before this catkin package.
find_package(vision_kernel CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_noetic_params_plugin
  CATKIN_DEPENDS roscpp dynamic_reconfigure
  CFG_EXTRAS ros_noetic_params_plugin-extras.cmake)

add_library(ros_noetic_params_plugin
  src/ros_params_manager.cpp src/yaml_adapter.cpp src/error.cpp)
add_library(vision::ros_params ALIAS ros_noetic_params_plugin)
target_compile_features(ros_noetic_params_plugin PUBLIC cxx_std_17)
target_include_directories(ros_noetic_params_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${catkin_INCLUDE_DIRS})
target_link_libraries(ros_noetic_params_plugin PUBLIC vision::kernel yaml-cpp ${catkin_LIBRARIES})
target_compile_options(ros_noetic_params_plugin PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ros_noetic_params_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ros_params DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES README.md PROJECT_STATE.md LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include(CTest)
if(BUILD_TESTING AND CATKIN_ENABLE_TESTING)
  # Generate real Config fixtures in a test-only directory; do not install/export them.
  set(test_generated ${CMAKE_CURRENT_BINARY_DIR}/test_generated)
  foreach(config Primary Secondary)
    set(config_header ${test_generated}/include/${PROJECT_NAME}/${config}Config.h)
    add_custom_command(OUTPUT ${config_header}
      COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/cfg/${config}.cfg
        ${dynamic_reconfigure_BASE_DIR} ${test_generated}/share
        ${test_generated}/include/${PROJECT_NAME} ${test_generated}/python
      DEPENDS tests/cfg/${config}.cfg
        ${dynamic_reconfigure_BASE_DIR}/templates/ConfigType.h.template
        ${dynamic_reconfigure_BASE_DIR}/templates/ConfigType.py.template)
    list(APPEND test_config_headers ${config_header})
  endforeach()
  add_custom_target(ros_params_test_configs DEPENDS ${test_config_headers})
  add_executable(ros_params_contract_tests tests/contract_tests.cpp)
  add_dependencies(ros_params_contract_tests ros_params_test_configs)
  target_include_directories(ros_params_contract_tests PRIVATE ${test_generated}/include)
  target_link_libraries(ros_params_contract_tests PRIVATE vision::ros_params Threads::Threads)
  target_compile_options(ros_params_contract_tests PRIVATE -Wall -Wextra -Wpedantic)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  foreach(suite yaml_binding yaml_transaction dynamic_binding dynamic_initialization dynamic_rollback manager_lifecycle)
    add_test(NAME ${suite} COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_with_roscore.py $<TARGET_FILE:ros_params_contract_tests> ${suite})
    set_tests_properties(${suite} PROPERTIES TIMEOUT 60)
  endforeach()
endif()
