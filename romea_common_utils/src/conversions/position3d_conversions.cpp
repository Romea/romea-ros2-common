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
#include "romea_common_utils/conversions/position3d_conversions.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::Position3D & romea_position3d,
  romea_common_msgs::msg::Position3D & ros_position3d_msg)
{
  ros_position3d_msg.x = romea_position3d.position.x();
  ros_position3d_msg.y = romea_position3d.position.y();
  ros_position3d_msg.z = romea_position3d.position.z();

  for (size_t n = 0; n < 9; ++n) {
    ros_position3d_msg.covariance[n] = romea_position3d.covariance(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Position3D & romea_position3d,
  romea_common_msgs::msg::Position3DStamped & ros_position3d_stamped)
{
  ros_position3d_stamped.header.frame_id = frame_id;
  ros_position3d_stamped.header.stamp = stamp;
  to_ros_msg(romea_position3d, ros_position3d_stamped.position);
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_common_msgs::msg::Position3D & msg,
  core::Position3D & position3d)
{
  position3d.position.x() = msg.x;
  position3d.position.y() = msg.y;
  position3d.position.z() = msg.z;
  position3d.covariance = Eigen::Matrix3d(msg.covariance.data());
}

//-----------------------------------------------------------------------------
core::Position3D to_romea(const romea_common_msgs::msg::Position3D & msg)
{
  core::Position3D position3d;
  to_romea(msg, position3d);
  return position3d;
}

}  // namespace ros2
}  // namespace romea
