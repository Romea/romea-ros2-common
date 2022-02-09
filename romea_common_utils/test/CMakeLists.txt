find_package(ament_cmake_gtest REQUIRED)

get_filename_component(TEST_DIR_SRC ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
configure_file(test_helper.h.in test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_add_gtest(${PROJECT_NAME}_test_ros_conversions test_ros_conversions.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_ros_params test_ros_params.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_params ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_ros_publisher test_ros_publisher.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_publisher ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_ros_transform test_ros_transform.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_transform ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_pose2d_conversions test_pose2d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_pose2d_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_pose3d_conversions test_pose3d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_pose3d_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_twist2d_conversions test_twist2d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_twist2d_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_twist3d_conversions test_twist3d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_twist3d_conversions ${PROJECT_NAME})
