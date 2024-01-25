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
#include <cassert>
#include <memory>
#include <string>
#include <vector>
#include <limits>

// local
#include "romea_common_utils/joint_states.hpp"

namespace
{

void fill(
  sensor_msgs::msg::JointState & joint_states,
  const size_t & number_of_joints)
{
  joint_states.name.resize(number_of_joints);
  joint_states.position.resize(number_of_joints, std::numeric_limits<double>::quiet_NaN());
  joint_states.velocity.resize(number_of_joints, std::numeric_limits<double>::quiet_NaN());
  joint_states.effort.resize(number_of_joints, std::numeric_limits<double>::quiet_NaN());
}

void fill(
  sensor_msgs::msg::JointState & joint_states,
  const std::vector<std::string> & joint_names)
{
  joint_states.name = joint_names;
  joint_states.position.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  joint_states.velocity.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  joint_states.effort.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
}

}  // namespace

namespace romea
{
namespace ros2
{


//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState make_joint_state_msg(
  const size_t & number_of_joints)
{
  auto msg = sensor_msgs::msg::JointState();
  fill(msg, number_of_joints);
  return msg;
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState make_joint_state_msg(
  const std::vector<std::string> joint_names)
{
  auto msg = sensor_msgs::msg::JointState();
  fill(msg, joint_names);
  return msg;
}
//-----------------------------------------------------------------------------
std::shared_ptr<sensor_msgs::msg::JointState> make_shared_joint_state_msg(
  const size_t & number_of_joints)
{
  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  fill(*msg, number_of_joints);
  return msg;
}

//-----------------------------------------------------------------------------
std::shared_ptr<sensor_msgs::msg::JointState> make_shared_joint_state_msg(
  const std::vector<std::string> joint_names)
{
  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  fill(*msg, joint_names);
  return msg;
}

//-----------------------------------------------------------------------------
size_t get_joint_id(
  const sensor_msgs::msg::JointState & joint_states,
  const std::string & joint_name)
{
  auto it = std::find(
    joint_states.name.cbegin(),
    joint_states.name.cend(),
    joint_name);

  assert(it != joint_states.name.end());
  return std::distance(joint_states.name.cbegin(), it);
}

//-----------------------------------------------------------------------------
std::optional<size_t> find_joint_id(
  const sensor_msgs::msg::JointState & joint_states,
  const std::string & joint_name)
{
  auto it = std::find(
    joint_states.name.cbegin(),
    joint_states.name.cend(),
    joint_name);

  if (it != joint_states.name.end()) {
    return std::distance(joint_states.name.cbegin(), it);
  } else {
    return {};
  }
}


//-----------------------------------------------------------------------------
double get_position(
  const sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id)
{
  return joint_states.position[joint_id];
}

//-----------------------------------------------------------------------------
double get_velocity(
  const sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id)
{
  return joint_states.velocity[joint_id];
}

//-----------------------------------------------------------------------------
double get_effort(
  const sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id)
{
  return joint_states.effort[joint_id];
}

//-----------------------------------------------------------------------------
void set_position(
  sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id,
  const double & position)
{
  joint_states.position[joint_id] = position;
}

//-----------------------------------------------------------------------------
void set_velocity(
  sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id,
  const double & velocity)
{
  joint_states.velocity[joint_id] = velocity;
}

//-----------------------------------------------------------------------------
void set_effort(
  sensor_msgs::msg::JointState & joint_states,
  const std::size_t joint_id,
  const double & effort)
{
  joint_states.effort[joint_id] = effort;
}

}  // namespace ros2
}  // namespace romea
