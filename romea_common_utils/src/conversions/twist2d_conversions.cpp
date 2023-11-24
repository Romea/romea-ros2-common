// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <string>

// local
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

namespace romea
{
namespace ros2
{


//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::Twist2D & romea_twist2d,
  romea_common_msgs::msg::Twist2D & ros_twist2d_msg)
{
  ros_twist2d_msg.linear_speeds.x = romea_twist2d.linearSpeeds.x();
  ros_twist2d_msg.linear_speeds.y = romea_twist2d.linearSpeeds.y();
  ros_twist2d_msg.angular_speed = romea_twist2d.angularSpeed;

  for (size_t n = 0; n < 9; ++n) {
    ros_twist2d_msg.covariance[n] = romea_twist2d.covariance(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Twist2D & romea_twist_2d,
  romea_common_msgs::msg::Twist2DStamped & ros_twist2d_msg)
{
  ros_twist2d_msg.header.stamp = stamp;
  ros_twist2d_msg.header.frame_id = frame_id;
  to_ros_msg(romea_twist_2d, ros_twist2d_msg.twist);
}


//-----------------------------------------------------------------------------
void to_romea(
  const romea_common_msgs::msg::Twist2D & ros_twist2d_msg,
  core::Twist2D & romea_twist2d)
{
  romea_twist2d.linearSpeeds.x() = ros_twist2d_msg.linear_speeds.x;
  romea_twist2d.linearSpeeds.y() = ros_twist2d_msg.linear_speeds.y;
  romea_twist2d.angularSpeed = ros_twist2d_msg.angular_speed;
  romea_twist2d.covariance = Eigen::Matrix3d(ros_twist2d_msg.covariance.data());
}

//-----------------------------------------------------------------------------
core::Twist2D to_romea(const romea_common_msgs::msg::Twist2D & ros_twist2d_msg)
{
  core::Twist2D romea_twist2d;
  to_romea(ros_twist2d_msg, romea_twist2d);
  return romea_twist2d;
}

}  // namespace ros2
}  // namespace romea
