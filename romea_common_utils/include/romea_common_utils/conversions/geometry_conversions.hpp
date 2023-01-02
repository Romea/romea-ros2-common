// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__GEOMETRY_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__GEOMETRY_CONVERSIONS_HPP_

// ros
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// eigen
#include <Eigen/Geometry>


namespace romea
{

void to_romea(
  const geometry_msgs::msg::Vector3 & position_msg,
  Eigen::Vector3d & eigen_position);

void to_ros_msg(
  const Eigen::Vector3d & eigen_position,
  geometry_msgs::msg::Vector3 & position_msg);

void to_romea(
  const geometry_msgs::msg::Quaternion & quaternion_msg,
  Eigen::Quaterniond & eigen_quaternion);

void to_ros_msg(
  const Eigen::Quaterniond & eigen_quaternion,
  geometry_msgs::msg::Quaternion & quaternion_msg);

void to_romea(
  const geometry_msgs::msg::Quaternion & quaternion_msg,
  Eigen::Matrix3d & eigen_rotation_matrix);

void to_ros_msg(
  const Eigen::Matrix3d & eigen_rotation_matrix,
  geometry_msgs::msg::Quaternion & quaternion_msg);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__GEOMETRY_CONVERSIONS_HPP_
