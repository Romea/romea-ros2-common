#ifndef ROMEA_COMMON_UTILS_CONVERSIONS_TWIST2D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS_CONVERSIONS_TWIST2D_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include <romea_core_common/geometry/Twist2D.hpp>

// romea_ros_msg
#include <romea_common_msgs/msg/twist2_d_stamped.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>

namespace romea
{

void to_ros_msg(
  const Twist2D & romea_twist2d,
  romea_common_msgs::msg::Twist2D & ros_twist2d_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Twist2D & romea_twist2d,
  romea_common_msgs::msg::Twist2DStamped & ros_twist2d_msg);

void to_romea(
  const romea_common_msgs::msg::Twist2D & ros_twist2d_msg,
  Twist2D & romea_twist2d);

Twist2D to_romea(const romea_common_msgs::msg::Twist2D & ros_twist2d_msg);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_CONVERSIONS_TWIST2D_CONVERSIONS_HPP_
