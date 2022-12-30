cmake_minimum_required(VERSION 3.5)
project(romea_common_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# install(
  # DIRECTORY launch
  # DESTINATION share/${PROJECT_NAME}
# )

ament_python_install_module(python/romea_common_bringup.py)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_common_bringup.yaml test_common_bringup.yaml)
  ament_add_pytest_test(test_common_bringup test/test_common_bringup.py)
endif()



ament_package()
