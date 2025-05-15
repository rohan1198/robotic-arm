cmake_minimum_required(VERSION 3.16)
project(robotic_arm LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(
  robotic_arm
  SHARED
  hardware/robotic_arm_hardware.cpp
  controller/robotic_arm_controller.cpp
)

target_compile_features(robotic_arm PUBLIC cxx_std_17)
target_include_directories(robotic_arm PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/robotic_arm>
)
ament_target_dependencies(
  robotic_arm PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robotic_arm.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface robotic_arm.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/robotic_arm
)
install(
  DIRECTORY controller/include/
  DESTINATION include/robotic_arm
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/robotic_arm
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/robotic_arm
)
install(
  DIRECTORY robotic_arm_description/
  DESTINATION share/robotic_arm/robotic_arm_description
)
install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/robotic_arm
)
install(
  PROGRAMS
  teleop/scripts/joystick_control.py
  DESTINATION lib/robotic_arm
)
install(
  DIRECTORY description/gazebo
  DESTINATION share/robotic_arm
)
install(
  DIRECTORY bringup/config
  DESTINATION share/robotic_arm
)

install(TARGETS robotic_arm
  EXPORT export_robotic_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_robotic_arm HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
