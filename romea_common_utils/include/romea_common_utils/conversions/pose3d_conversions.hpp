// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__POSE3D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__POSE3D_CONVERSIONS_HPP_

// romea core
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/Pose3D.hpp>

// ros
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// romea ros
#include <romea_common_utils/conversions/time_conversions.hpp>

// std
#include <string>

namespace romea
{

void to_ros_transform_msg(
  const Pose3D & romea_pose_3d,
  geometry_msgs::msg::Transform & ros_transform_msg);


void to_ros_transform_msg(
  const rclcpp::Time & stamp,
  const Pose3D & romea_pose_3d,
  const std::string & frame_id,
  const std::string & child_frame_id,
  geometry_msgs::msg::TransformStamped & tf_msg);

void to_ros_msg(
  const Pose3D & romea_pose_3d,
  geometry_msgs::msg::PoseWithCovariance & ros_pose_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Pose3D & romea_pose_3d,
  geometry_msgs::msg::PoseWithCovarianceStamped & ros_pose_msg);


void to_romea(
  const geometry_msgs::msg::PoseWithCovariance & ros_pose_msg,
  Pose3D & romea_pose_3d);

Pose3D to_romea(const geometry_msgs::msg::PoseWithCovariance & ros_pose_msg);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__POSE3D_CONVERSIONS_HPP_
