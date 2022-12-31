#ifndef ROMEA_COMMON_UTILS_CONVERSIONS_POSE_AND_TWIST2D_CONVERSIONS_HPP
#define ROMEA_COMMON_UTILS_CONVERSIONS_POSE_AND_TWIST2D_CONVERSIONS_HPP

// std
#include <string>

// romea
#include <romea_core_common/geometry/PoseAndTwist2D.hpp>
#include "romea_common_utils/conversions/pose2d_conversions.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

// romea_ros_msg
#include <romea_common_msgs/msg/pose_and_twist2_d.hpp>
#include <romea_common_msgs/msg/pose_and_twist2_d_stamped.hpp>

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

#endif  // ROMEA_COMMON_UTILS_CONVERSIONS_POSE_AND_TWIST2D_CONVERSIONS_HPP
