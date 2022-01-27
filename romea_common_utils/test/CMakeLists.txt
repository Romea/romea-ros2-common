find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_test_ros_conversions test_ros_conversions.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_ros_param_test test_ros_param_test.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_param_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_ros_publisher test_ros_publisher_test.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_publisher ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_ros_transform test_ros_transform_test.cpp)
target_link_libraries(${PROJECT_NAME}_test_ros_transform ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_pose2d_conversions test_pose2d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_pose2d_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_pose3d_conversions test_pose3d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_pose3d_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_twist2d_conversions test_twist2d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_twist2d_conversions ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_test_twist3d_conversions test_twist3d_conversions.cpp )
target_link_libraries(${PROJECT_NAME}_test_twist3d_conversions ${PROJECT_NAME})
