// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST3D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST3D_CONVERSIONS_HPP_


// std
#include <string>

// ros
#include "nav_msgs/msg/odometry.hpp"

// romea core
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/geometry/PoseAndTwist3D.hpp"

// local
#include "romea_common_utils/conversions/pose3d_conversions.hpp"
#include "romea_common_utils/conversions/twist3d_conversions.hpp"

namespace romea
{

void  to_ros_odom_msg(
  const rclcpp::Time & stamp,
  const PoseAndTwist3D & poseAndBodyTwist3D,
  const std::string & frame_id,
  const std::string & child_frame_id,
  nav_msgs::msg::Odometry & odom_msg);


}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST3D_CONVERSIONS_HPP_
