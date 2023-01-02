// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__TWIST3D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__TWIST3D_CONVERSIONS_HPP_

// romea core
#include <romea_core_common/geometry/Twist3D.hpp>

// ros
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

// romea ros
#include <romea_common_utils/conversions/time_conversions.hpp>

// std
#include <string>

namespace romea
{

void to_ros_msg(
  const Twist3D & romea_twist_3d,
  geometry_msgs::msg::TwistWithCovariance & ros_twist_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Twist3D & romea_twist_3d,
  geometry_msgs::msg::TwistWithCovarianceStamped & ros_twist3d_msg);

void to_romea(
  const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg,
  Twist3D & romea_twist3d);

Twist3D to_romea(const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg);


}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__TWIST3D_CONVERSIONS_HPP_
