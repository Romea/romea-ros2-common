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
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Position2D.msg
    msg/Pose2D.msg
    msg/Twist2D.msg
    msg/PoseAndTwist2D.msg
    msg/Position2DStamped.msg
    msg/Pose2DStamped.msg
    msg/Twist2DStamped.msg
    msg/PoseAndTwist2DStamped.msg
    msg/Vector2.msg
    DEPENDENCIES std_msgs geometry_msgs
    )

ament_export_dependencies(rosidl_default_runtime)
ament_package()



#cmake_minimum_required(VERSION 3.0.2)
#project(romea_common_msgs)

#set(${PROJECT_NAME}_CATKIN_DEPS std_msgs geometry_msgs message_generation)
#find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

#add_message_files(
#  DIRECTORY msg
#  FILES
#  )

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  )

#catkin_package(
#  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
#  )
