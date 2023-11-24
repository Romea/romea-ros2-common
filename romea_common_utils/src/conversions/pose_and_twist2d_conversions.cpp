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
#include "romea_common_utils/conversions/pose_and_twist2d_conversions.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg)
{
  to_ros_msg(romea_pose_and_twist2d.pose, ros_pose_and_twist2d_msg.pose);
  to_ros_msg(romea_pose_and_twist2d.twist, ros_pose_and_twist2d_msg.twist);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped)
{
  ros_pose_and_twist2d_msg_stamped.header.stamp = stamp;
  ros_pose_and_twist2d_msg_stamped.header.frame_id = frame_id;
  to_ros_msg(romea_pose_and_twist2d.pose, ros_pose_and_twist2d_msg_stamped.pose);
  to_ros_msg(romea_pose_and_twist2d.twist, ros_pose_and_twist2d_msg_stamped.twist);
}

//-----------------------------------------------------------------------------
core::PoseAndTwist2D to_romea(
  const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg)
{
  core::PoseAndTwist2D romea_pose_and_twist2d;
  to_romea(ros_pose_and_twist2d_msg, romea_pose_and_twist2d);
  return romea_pose_and_twist2d;
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg,
  core::PoseAndTwist2D & romea_pose_and_twist2d)
{
  to_romea(ros_pose_and_twist2d_msg.pose, romea_pose_and_twist2d.pose);
  to_romea(ros_pose_and_twist2d_msg.twist, romea_pose_and_twist2d.twist);
}

}  // namespace ros2
}  // namespace romea
