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


#include "romea_common_utils/conversions/geometry_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::Vector3 & position_msg,
  Eigen::Vector3d & eigen_position)
{
  eigen_position.x() = position_msg.x;
  eigen_position.y() = position_msg.y;
  eigen_position.z() = position_msg.z;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const Eigen::Vector3d & eigen_position,
  geometry_msgs::msg::Vector3 & position_msg)
{
  position_msg.x = eigen_position.x();
  position_msg.y = eigen_position.y();
  position_msg.z = eigen_position.z();
}

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::Quaternion & quaternion_msg,
  Eigen::Quaterniond & eigen_quaternion)
{
  eigen_quaternion.x() = quaternion_msg.x;
  eigen_quaternion.y() = quaternion_msg.y;
  eigen_quaternion.z() = quaternion_msg.z;
  eigen_quaternion.w() = quaternion_msg.w;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const Eigen::Quaterniond & eigen_quaternion,
  geometry_msgs::msg::Quaternion & quaternion_msg)
{
  quaternion_msg.x = eigen_quaternion.x();
  quaternion_msg.y = eigen_quaternion.y();
  quaternion_msg.z = eigen_quaternion.z();
  quaternion_msg.w = eigen_quaternion.w();
}

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::Quaternion & quaternion_msg,
  Eigen::Matrix3d & eigen_rotation_matrix)
{
  Eigen::Quaterniond quaternion;
  to_romea(quaternion_msg, quaternion);
  eigen_rotation_matrix = quaternion.toRotationMatrix();
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const Eigen::Matrix3d & eigen_rotation_matrix,
  geometry_msgs::msg::Quaternion & quaternion_msg)
{
  Eigen::Quaterniond quaternion(eigen_rotation_matrix);
  to_ros_msg(quaternion, quaternion_msg);
}

}  // namespace romea
