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


#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST2D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST2D_CONVERSIONS_HPP_

// std
#include <string>

// romea core
#include "romea_core_common/geometry/PoseAndTwist2D.hpp"

// romea ros
#include "romea_common_msgs/msg/pose_and_twist2_d.hpp"
#include "romea_common_msgs/msg/pose_and_twist2_d_stamped.hpp"

// local
#include "romea_common_utils/conversions/pose2d_conversions.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

namespace romea
{
namespace ros2
{

void to_ros_msg(
  const core::PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::PoseAndTwist2D & romea_pose_and_twist2d,
  romea_common_msgs::msg::PoseAndTwist2DStamped & ros_pose_and_twist2d_msg_stamped);

core::PoseAndTwist2D to_romea(
  const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg);

void to_romea(
  const romea_common_msgs::msg::PoseAndTwist2D & ros_pose_and_twist2d_msg,
  core::PoseAndTwist2D & romea_pose_and_twist2d);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__POSE_AND_TWIST2D_CONVERSIONS_HPP_
