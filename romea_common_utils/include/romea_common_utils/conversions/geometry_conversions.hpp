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


#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__GEOMETRY_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__GEOMETRY_CONVERSIONS_HPP_

// eigen
#include <Eigen/Geometry>

// ros
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


namespace romea
{

void to_romea(
  const geometry_msgs::msg::Vector3 & position_msg,
  Eigen::Vector3d & eigen_position);

void to_ros_msg(
  const Eigen::Vector3d & eigen_position,
  geometry_msgs::msg::Vector3 & position_msg);

void to_romea(
  const geometry_msgs::msg::Quaternion & quaternion_msg,
  Eigen::Quaterniond & eigen_quaternion);

void to_ros_msg(
  const Eigen::Quaterniond & eigen_quaternion,
  geometry_msgs::msg::Quaternion & quaternion_msg);

void to_romea(
  const geometry_msgs::msg::Quaternion & quaternion_msg,
  Eigen::Matrix3d & eigen_rotation_matrix);

void to_ros_msg(
  const Eigen::Matrix3d & eigen_rotation_matrix,
  geometry_msgs::msg::Quaternion & quaternion_msg);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__GEOMETRY_CONVERSIONS_HPP_
