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


#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__POSE2D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__POSE2D_CONVERSIONS_HPP_


// std
#include <string>

// ros
#include "geometry_msgs/msg/transform_stamped.hpp"

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/geometry/Pose2D.hpp"

// romea ros
#include "romea_common_msgs/msg/pose2_d_stamped.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"


namespace romea
{
namespace ros2
{

void to_ros_msg(
  const core::Pose2D & romea_pose2d,
  romea_common_msgs::msg::Pose2D & ros_pose2d_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Pose2D & romea_pose2d,
  romea_common_msgs::msg::Pose2DStamped & ros_pose2d_msg);

void to_ros_transform_msg(
  const core::Pose2D & romea_pose2d,
  geometry_msgs::msg::Transform & ros_transform_msg);

void to_ros_transform_msg(
  const rclcpp::Time & stamp,
  const core::Pose2D & romea_pose_2d,
  const std::string & frame_id,
  const std::string & child_frame_id,
  geometry_msgs::msg::TransformStamped & ros_transform_msg);

void to_romea(
  const romea_common_msgs::msg::Pose2D & msg,
  core::Pose2D & romea_pose_2d);

core::Pose2D to_romea(const romea_common_msgs::msg::Pose2D & msg);

// void to_romea(const romea_localisation_msgs::Pose2DStamped &msg,
//             Pose2D::Stamped & romea_pose_2d_stamped,
//             std::string & frame_id);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__POSE2D_CONVERSIONS_HPP_
