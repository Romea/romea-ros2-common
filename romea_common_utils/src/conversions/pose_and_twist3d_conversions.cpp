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
#include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_odom_msg(
  const rclcpp::Time & stamp,
  const PoseAndTwist3D & poseAndBodyTwist3D,
  const std::string & frame_id,
  const std::string & child_frame_id,
  nav_msgs::msg::Odometry & odom_msg)
{
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = frame_id;
  odom_msg.child_frame_id = child_frame_id;
  to_ros_msg(poseAndBodyTwist3D.pose, odom_msg.pose);
  to_ros_msg(poseAndBodyTwist3D.twist, odom_msg.twist);
}

}  // namespace romea
