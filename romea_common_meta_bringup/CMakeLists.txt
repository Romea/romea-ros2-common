cmake_minimum_required(VERSION 3.5)
project(romea_common_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# ament_python_install_module(python/romea_common_meta_bringup.py)
ament_python_install_package(romea_common_meta_bringup)

add_executable(urdf_broadcaster_node src/urdf_broadcaster.cpp)

ament_target_dependencies(urdf_broadcaster_node rclcpp std_msgs)

install(TARGETS urdf_broadcaster_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.flake8")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_common_meta_bringup.yaml test_common_meta_bringup.yaml)
  # configure_file(test/test_launch_file.yaml test_launch_file.yaml)
  # configure_file(test/test_node_profile.yaml test_node_profile.yaml)
  ament_add_pytest_test(test_launch_file test/test_launch_file.py)
  # ament_add_pytest_test(test_node_launch_file_configuration test/test_node_profile.py)
  ament_add_pytest_test(test_common_meta_bringup test/test_common_meta_bringup.py)
endif()



ament_package()
