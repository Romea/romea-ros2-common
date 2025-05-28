cmake_minimum_required(VERSION 3.5)
project(romea_common_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

ament_python_install_module(python/romea_common_meta_bringup.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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
