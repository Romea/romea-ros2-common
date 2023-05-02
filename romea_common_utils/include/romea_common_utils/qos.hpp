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



#ifndef ROMEA_COMMON_UTILS__QOS_HPP_
#define ROMEA_COMMON_UTILS__QOS_HPP_

#include "rclcpp/qos.hpp"

namespace  romea
{

rclcpp::QoS sensor_data_qos();

rclcpp::QoS best_effort(const size_t & history_size);

rclcpp::QoS reliable(const size_t & history_size);

rclcpp::QoS best_effort(const size_t & history_size, const rclcpp::Duration & timeout);

rclcpp::QoS reliable(const size_t & history_size, const rclcpp::Duration & timeout);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__QOS_HPP_
