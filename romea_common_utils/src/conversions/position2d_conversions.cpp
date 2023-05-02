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
#include "romea_common_utils/conversions/position2d_conversions.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const Position2D & romea_position2d,
  romea_common_msgs::msg::Position2D & ros_position2d_msg)
{
  ros_position2d_msg.x = romea_position2d.position.x();
  ros_position2d_msg.y = romea_position2d.position.y();

  for (size_t n = 0; n < 4; ++n) {
    ros_position2d_msg.covariance[n] = romea_position2d.covariance(n);
  }
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Position2D & romea_position2d,
  romea_common_msgs::msg::Position2DStamped & ros_position2d_stamped)
{
  ros_position2d_stamped.header.frame_id = frame_id;
  ros_position2d_stamped.header.stamp = stamp;
  to_ros_msg(romea_position2d, ros_position2d_stamped.position);
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_common_msgs::msg::Position2D & msg,
  Position2D & position2d)
{
  position2d.position.x() = msg.x;
  position2d.position.y() = msg.y;
  position2d.covariance = Eigen::Matrix2d(msg.covariance.data());
}

//-----------------------------------------------------------------------------
Position2D to_romea(const romea_common_msgs::msg::Position2D & msg)
{
  Position2D position2d;
  to_romea(msg, position2d);
  return position2d;
}

}  // namespace romea
