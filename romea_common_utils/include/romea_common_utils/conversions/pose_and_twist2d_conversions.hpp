// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST2D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST2D_CONVERSIONS_HPP_


// romea ros
#include <romea_common_msgs/msg/pose_and_twist2_d.hpp>
#include <romea_common_msgs/msg/pose_and_twist2_d_stamped.hpp>

// romea core
#include <romea_core_common/geometry/PoseAndTwist2D.hpp>

// std
#include <string>

// local
#include "romea_common_utils/conversions/pose2d_conversions.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

namespace romea
{

void to_ros_msg(
  const PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped);

PoseAndTwist2D to_romea(const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg);

void to_romea(
  const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg,
  PoseAndTwist2D & romea_pose_and_twist2d);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST2D_CONVERSIONS_HPP_
