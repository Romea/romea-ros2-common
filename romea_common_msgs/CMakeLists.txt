cmake_minimum_required(VERSION 3.5)
project(romea_common_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Position2D.msg
    msg/Position3D.msg
    msg/Pose2D.msg
    msg/Twist2D.msg
    msg/PoseAndTwist2D.msg
    msg/Position2DStamped.msg
    msg/Position3DStamped.msg
    msg/Pose2DStamped.msg
    msg/Twist2DStamped.msg
    msg/PoseAndTwist2DStamped.msg
    msg/Vector2.msg
    DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

