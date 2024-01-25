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


#ifndef ROMEA_COMMON_UTILS__JOINT_STATES_HPP_
#define ROMEA_COMMON_UTILS__JOINT_STATES_HPP_


// std
#include <optional>
#include <string>
#include <vector>
#include <memory>

// ros
#include "sensor_msgs/msg/joint_state.hpp"

namespace romea
{
namespace ros2
{

sensor_msgs::msg::JointState make_joint_state_msg(
  const size_t & number_of_joints);

sensor_msgs::msg::JointState make_joint_state_msg(
  const std::vector<std::string> joint_names);

std::shared_ptr<sensor_msgs::msg::JointState> make_shared_joint_state_msg(
  const size_t & number_of_joints);

std::shared_ptr<sensor_msgs::msg::JointState> make_shared_joint_state_msg(
  const std::vector<std::string> joint_names);

size_t get_joint_id(
  const sensor_msgs::msg::JointState & joint_states,
  const std::string & joint_name);

std::optional<size_t> find_joint_id(
  const sensor_msgs::msg::JointState & joint_states,
  const std::string & joint_name);

double get_position(
  const sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id);

double get_velocity(
  const sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id);

double get_effort(
  const sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id);

void set_position(
  sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id,
  const double & position);

void set_velocity(
  sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id,
  const double & velocity);

void set_effort(
  sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id,
  const double & effort);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__JOINT_STATES_HPP_
