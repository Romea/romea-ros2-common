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
#include "romea_common_utils/conversions/twist3d_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::Twist3D & romea_twist3d,
  geometry_msgs::msg::TwistWithCovariance & ros_twist_msg)
{
  const auto & linearSpeeds = romea_twist3d.linearSpeeds;
  const auto & angularSpeeds = romea_twist3d.angularSpeeds;
  const auto & covariance = romea_twist3d.covariance;

  ros_twist_msg.twist.linear.x = linearSpeeds.x();
  ros_twist_msg.twist.linear.y = linearSpeeds.y();
  ros_twist_msg.twist.linear.z = linearSpeeds.z();

  ros_twist_msg.twist.angular.x = angularSpeeds.x();
  ros_twist_msg.twist.angular.y = angularSpeeds.y();
  ros_twist_msg.twist.angular.z = angularSpeeds.z();

  for (size_t n = 0; n < 36; ++n) {
    ros_twist_msg.covariance[n] = covariance(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Twist3D & romea_twist_3d,
  geometry_msgs::msg::TwistWithCovarianceStamped & ros_twist3d_msg)
{
  ros_twist3d_msg.header.stamp = stamp;
  ros_twist3d_msg.header.frame_id = frame_id;
  to_ros_msg(romea_twist_3d, ros_twist3d_msg.twist);
}

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg,
  core::Twist3D & romea_twist3d)
{
  romea_twist3d.linearSpeeds.x() = ros_twist_msg.twist.linear.x;
  romea_twist3d.linearSpeeds.y() = ros_twist_msg.twist.linear.y;
  romea_twist3d.linearSpeeds.z() = ros_twist_msg.twist.linear.z;
  romea_twist3d.angularSpeeds.x() = ros_twist_msg.twist.angular.x;
  romea_twist3d.angularSpeeds.y() = ros_twist_msg.twist.angular.y;
  romea_twist3d.angularSpeeds.z() = ros_twist_msg.twist.angular.z;
  romea_twist3d.covariance = Eigen::Matrix6d(ros_twist_msg.covariance.data());
}

//-----------------------------------------------------------------------------
core::Twist3D to_romea(const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg)
{
  core::Twist3D twist3d;
  to_romea(ros_twist_msg, twist3d);
  return twist3d;
}

}  // namespace ros2
}  // namespace romea
