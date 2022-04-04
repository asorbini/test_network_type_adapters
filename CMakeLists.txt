cmake_minimum_required(VERSION 3.8)
project(test_network_type_adapters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rmw_connextdds REQUIRED)
find_package(connext_node_helpers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

function(generate_c_typesupport idl_file)
  cmake_parse_arguments(_idl
    "" # boolean arguments
    "PACKAGE" # single value arguments
    "" # multi-value arguments
    ${ARGN} # current function arguments
  )

  set(idl_out_dir "${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen")
  get_filename_component(idl_filename "${idl_file}" NAME)
  string(REGEX REPLACE "\.idl$" "" idl_base "${idl_filename}")
  if(NOT "${_idl_PACKAGE}" STREQUAL "")
    set(idl_ns "${_idl_PACKAGE}/")
  endif()
  set(idl_generated
    "${idl_out_dir}/${idl_ns}${idl_base}.c"
    "${idl_out_dir}/${idl_ns}${idl_base}.h"
    "${idl_out_dir}/${idl_ns}${idl_base}Support.c"
    "${idl_out_dir}/${idl_ns}${idl_base}Support.h"
    "${idl_out_dir}/${idl_ns}${idl_base}Plugin.c"
    "${idl_out_dir}/${idl_ns}${idl_base}Plugin.h"
  )
  find_program(rtiddsgen_bin
    NAMES rtiddsgen
    PATHS "${CONNEXTDDS_DIR}/bin"
  )
  add_custom_command(
    OUTPUT ${idl_generated}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${idl_out_dir}/${idl_ns}
    COMMAND ${rtiddsgen_bin} -language C
                             -replace
                             -unboundedSupport
                             -d ${idl_out_dir}/${idl_ns}
                             -I "${CMAKE_CURRENT_SOURCE_DIR}/idl"
                             ${idl_file}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS ${idl_file}
  )

  string(REGEX REPLACE "/" "_" out_var "${idl_ns}${idl_base}_FILES")
  set(${out_var} ${idl_generated} PARENT_SCOPE)
endfunction()

generate_c_typesupport(idl/ros2/builtin_interfaces/msg/Time.idl
  PACKAGE ros2/builtin_interfaces/msg
)
generate_c_typesupport(idl/ros2/std_msgs/msg/Header.idl
  PACKAGE ros2/std_msgs/msg
)
generate_c_typesupport(idl/ros2/sensor_msgs/msg/Image.idl
  PACKAGE ros2/sensor_msgs/msg
)
generate_c_typesupport(idl/ShapeType.idl)

add_library(${PROJECT_NAME} SHARED
  src/image_transform.cpp
  src/image_transform_c.cpp
  src/string_to_shape_transform.cpp
  include/${PROJECT_NAME}/image_transform.hpp
  include/${PROJECT_NAME}/visibility_control.h
  src/shape_talker.cpp
  ${ros2_builtin_interfaces_msg_Time_FILES}
  ${ros2_std_msgs_msg_Header_FILES}
  ${ros2_sensor_msgs_msg_Image_FILES}
  ${ShapeType_FILES}
)
target_link_libraries(${PROJECT_NAME}
  RTIConnextDDS::metp
  rmw_connextdds_common::rmw_connextdds_common_pro
)
ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  std_msgs
  rclcpp
  rclcpp_components
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_include_directories(${PROJECT_NAME}
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/rtiddsgen"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "test_network_type_adapters::ShapeTalker"
  EXECUTABLE shape_talker)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY resource/connext DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

ament_export_dependencies(
  sensor_msgs
  std_msgs
  rclcpp
  rclcpp_components
  connext_node_helpers
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
