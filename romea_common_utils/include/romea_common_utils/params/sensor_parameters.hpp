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


#ifndef ROMEA_COMMON_UTILS__PARAMS__SENSOR_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS__PARAMS__SENSOR_PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// romea
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_device(std::shared_ptr<NodeType> node)
{
  declare_parameter<std::string>(node, "device");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_baudrate(std::shared_ptr<NodeType> node)
{
  declare_parameter<int>(node, "baudrate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_ip(std::shared_ptr<NodeType> node)
{
  declare_parameter<std::string>(node, "ip");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_port(std::shared_ptr<NodeType> node)
{
  declare_parameter<int>(node, "port");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_rate(std::shared_ptr<NodeType> node)
{
  declare_parameter<int>(node, "rate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_frame_id(std::shared_ptr<NodeType> node)
{
  declare_parameter<std::string>(node, "frame_id");
}
//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_device(std::shared_ptr<NodeType> node)
{
  return get_parameter<std::string>(node, "device");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_baudrate(std::shared_ptr<NodeType> node)
{
  return get_parameter<int>(node, "baudrate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_ip(std::shared_ptr<NodeType> node)
{
  return get_parameter<std::string>(node, "ip");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_port(std::shared_ptr<NodeType> node)
{
  return get_parameter<int>(node, "port");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_rate(std::shared_ptr<NodeType> node)
{
  return get_parameter<int>(node, "rate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_frame_id(std::shared_ptr<NodeType> node)
{
  return get_parameter<std::string>(node, "frame_id");
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PARAMS__SENSOR_PARAMETERS_HPP_
