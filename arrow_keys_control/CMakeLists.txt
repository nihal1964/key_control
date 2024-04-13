cmake_minimum_required(VERSION 3.8)
project(arrow_keys_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the ROS Distribution (humble)
set(ROS_DISTRO "humble")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cartesian_controller_base REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        cartesian_controller_base
        Eigen3
        hardware_interface
        rcl_lifecycle
        realtime_tools
        class_loader
        pluginlib
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
configure_file(/home/nihal/hc_ur10e/src/cartesian_controller_base/include/cartesian_controller_base/ROS2VersionConfig.h.in ROS2VersionConfig.h)

# Add executable
add_executable(arrow_keys_control src/arrow_keys_control.cpp)
target_include_directories(arrow_keys_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${hardware_interface_INCLUDE_DIRS}
    ${rcl_lifecycle_INCLUDE_DIRS}
    ${realtime_tools_INCLUDE_DIRS}
    ${class_loader_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${cartesian_controller_base_INCLUDE_DIRS}
)
ament_target_dependencies(arrow_keys_control
        ${arrow_keys_control_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(arrow_keys_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include
)

# Install
install(TARGETS arrow_keys_control
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS arrow_keys_control
  #EXPORT my_targets_from_this_package
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

#install(FILES
#  /home/nihal/hc_ur10e/src/cartesian_controller_base/include/cartesian_controller_base/ROS2VersionConfig.h
#  DESTINATION share/${PROJECT_NAME}
#)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
