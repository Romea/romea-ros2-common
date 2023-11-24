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


#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__POSITION2D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__POSITION2D_CONVERSIONS_HPP_

// std
#include <string>

// romea
#include "romea_core_common/time/Time.hpp"
#include "romea_core_common/geometry/Position2D.hpp"

// romea ros
#include "romea_common_msgs/msg/position2_d.hpp"
#include "romea_common_msgs/msg/position2_d_stamped.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"


namespace romea
{
namespace ros2
{

void to_ros_msg(
  const core::Position2D & romea_position2d,
  romea_common_msgs::msg::Position2D & ros_position2d_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const core::Position2D & position2D,
  romea_common_msgs::msg::Position2DStamped & ros_position2d_stamped);


void to_romea(
  const romea_common_msgs::msg::Position2D & msg,
  core::Position2D & position2d);

core::Position2D to_romea(const romea_common_msgs::msg::Position2D & msg);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__POSITION2D_CONVERSIONS_HPP_
