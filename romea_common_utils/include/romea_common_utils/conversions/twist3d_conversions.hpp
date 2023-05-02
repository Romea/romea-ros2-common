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


#ifndef ROMEA_COMMON_UTILS__CONVERSIONS__TWIST3D_CONVERSIONS_HPP_
#define ROMEA_COMMON_UTILS__CONVERSIONS__TWIST3D_CONVERSIONS_HPP_

// std
#include <string>

// ros
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

// romea core
#include "romea_core_common/geometry/Twist3D.hpp"

// romea ros
#include "romea_common_utils/conversions/time_conversions.hpp"


namespace romea
{

void to_ros_msg(
  const Twist3D & romea_twist_3d,
  geometry_msgs::msg::TwistWithCovariance & ros_twist_msg);

void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const Twist3D & romea_twist_3d,
  geometry_msgs::msg::TwistWithCovarianceStamped & ros_twist3d_msg);

void to_romea(
  const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg,
  Twist3D & romea_twist3d);

Twist3D to_romea(const geometry_msgs::msg::TwistWithCovariance & ros_twist_msg);


}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__CONVERSIONS__TWIST3D_CONVERSIONS_HPP_
