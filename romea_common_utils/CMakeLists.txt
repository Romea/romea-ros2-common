cmake_minimum_required(VERSION 3.5)
project(romea_common_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_common_msgs REQUIRED)
#find_package(rviz_visual_tools REQUIRED)


add_library(${PROJECT_NAME} SHARED
    src/conversions/diagnostic_conversions.cpp
    src/conversions/time_conversions.cpp
    src/conversions/geometry_conversions.cpp
    src/conversions/transform_conversions.cpp
    src/conversions/pose2d_conversions.cpp
    src/conversions/pose3d_conversions.cpp
    src/conversions/pose_and_twist2d_conversions.cpp
    src/conversions/pose_and_twist3d_conversions.cpp
    src/conversions/position2d_conversions.cpp
    src/conversions/twist2d_conversions.cpp
    src/conversions/twist3d_conversions.cpp
    src/joint_states.cpp
    src/qos.cpp)
#    src/rviz.cpp

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    romea_core_common
    romea_common_msgs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_ros
    tf2_eigen)
#    rviz_visual_tools


target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PUBLIC ROS_DISTRO=${ROS_DISTRO})
#-DROS_DISTRO=${ROS_DISTRO}
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_common_msgs)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_eigen)
#ament_export_dependencies(rviz_visual_tools)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(export_${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)

endif()

ament_package()
