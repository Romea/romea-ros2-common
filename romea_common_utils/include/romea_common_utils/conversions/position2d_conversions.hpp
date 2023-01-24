// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__POSITION2D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__POSITION2D_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/geometry/Position2D.hpp"

// romea ros
#include "romea_common_msgs/msg/position2_d.hpp"
#include "romea_common_msgs/msg/position2_d_stamped.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"


namespace romea
{

void to_ros_msg(
  const Position2D & romea_position2d,
  romea_common_msgs::msg::Position2D & ros_position2d_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Position2D & position2D,
  romea_common_msgs::msg::Position2DStamped & ros_position2d_stamped);


void to_romea(
  const romea_common_msgs::msg::Position2D & msg,
  Position2D & position2d);

Position2D to_romea(const romea_common_msgs::msg::Position2D & msg);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__POSITION2D_CONVERSIONS_HPP_
