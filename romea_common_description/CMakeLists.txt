cmake_minimum_required(VERSION 3.5)
project(romea_common_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)


ament_python_install_module(python/romea_common_description.py)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # configure_file(test/test_common_description.yaml test_common_description.yaml)
  ament_add_pytest_test(test_common_description test/test_common_description.py)
endif()



ament_package()
