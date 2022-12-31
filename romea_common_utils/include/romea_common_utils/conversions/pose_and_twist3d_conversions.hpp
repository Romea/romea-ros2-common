#ifndef ROMEA_COMMON_UTILS_CONVERSIONS_POSE_AND_TWIST3D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS_CONVERSIONS_POSE_AND_TWIST3D_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include <romea_core_common/time/Time.hpp>
#include <romea_core_common/geometry/PoseAndTwist3D.hpp>
#include "romea_common_utils/conversions/pose3d_conversions.hpp"
#include "romea_common_utils/conversions/twist3d_Conversions.hpp"

// romea_ros_msg
#include <nav_msgs/msg/odometry.hpp>

namespace romea
{

void  to_ros_odom_msg(
  const rclcpp::Time & stamp,
  const PoseAndTwist3D & poseAndBodyTwist3D,
  const std::string & frame_id,
  const std::string & child_frame_id,
  nav_msgs::msg::Odometry & odom_msg);


}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_CONVERSIONS_POSE_AND_TWIST3D_CONVERSIONS_HPP_
