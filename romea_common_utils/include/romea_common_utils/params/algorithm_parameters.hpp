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


#ifndef ROMEA_COMMON_UTILS__PARAMS__ALGORITHM_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS__PARAMS__ALGORITHM_PARAMETERS_HPP_

// std
#include <string>
#include <memory>

// local
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_debug(std::shared_ptr<NodeType> node)
{
  declare_parameter_with_default<bool>(node, "debug", false);
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline bool get_debug(std::shared_ptr<NodeType> node)
{
  return get_parameter<bool>(node, "debug");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_log_directory(std::shared_ptr<NodeType> node)
{
  declare_parameter_with_default<std::string>(
    node, "log_directory", rclcpp::get_logging_directory().string());
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_log_directory(std::shared_ptr<NodeType> node)
{
  return get_parameter<std::string>(node, "log_directory");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_log_filename(
  std::shared_ptr<NodeType> node,
  const std::string & log_name = "")
{
  std::string filename;
  if (get_debug(node)) {
    std::string ns = std::string(node->get_namespace());
    std::string node_name = std::string(node->get_name());

    filename = ns == "/" ? ns + node_name : ns + "/" + node_name;

    if (!log_name.empty()) {
      filename += "/" + log_name;
    }

    filename += "/debug.csv";

    std::replace_copy(
      std::begin(filename) + 1,
      std::end(filename),
      std::begin(filename) + 1, '/', '_');

    filename = get_log_directory(node) + filename;
  }
  return filename;
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_base_footprint_frame_id(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  declare_parameter_with_default<std::string>(
    node, ns, "base_footprint_frame_id", "base_footprint");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_base_footprint_frame_id(std::shared_ptr<NodeType> node)
{
  declare_base_footprint_frame_id(node, "");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_base_footprint_frame_id(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  return get_parameter<std::string>(node, ns, "base_footprint_frame_id");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_base_footprint_frame_id(std::shared_ptr<NodeType> node)
{
  return get_base_footprint_frame_id(node, "");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_odom_frame_id(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  declare_parameter_with_default<std::string>(
    node, ns, "odom_frame_id", "odom");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_odom_frame_id(std::shared_ptr<NodeType> node)
{
  declare_odom_frame_id(node, "");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_odom_frame_id(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  return get_parameter<std::string>(node, ns, "odom_frame_id");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_odom_frame_id(std::shared_ptr<NodeType> node)
{
  return get_odom_frame_id(node, "");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_map_frame_id(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  declare_parameter_with_default<std::string>(
    node, ns, "map_frame_id", "map");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_map_frame_id(std::shared_ptr<NodeType> node)
{
  declare_map_frame_id(node, "");
}


//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_map_frame_id(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  return get_parameter<std::string>(node, ns, "map_frame_id");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_map_frame_id(std::shared_ptr<NodeType> node)
{
  return get_map_frame_id(node, "");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_publish_rate(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  declare_parameter<int>(node, ns, "publish_rate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_publish_rate(
  std::shared_ptr<NodeType> node,
  const std::string & ns,
  const int & default_value)
{
  declare_parameter_with_default<int>(node, ns, "publish_rate", default_value);
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_publish_rate(std::shared_ptr<NodeType> node)
{
  declare_publish_rate(node, "");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_publish_rate(std::shared_ptr<NodeType> node, const int & default_value)
{
  declare_publish_rate(node, "", default_value);
}


//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_publish_rate(
  std::shared_ptr<NodeType> node,
  const std::string & ns)
{
  return get_parameter<int>(node, ns, "publish_rate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_publish_rate(std::shared_ptr<NodeType> node)
{
  return get_publish_rate(node, "");
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PARAMS__ALGORITHM_PARAMETERS_HPP_
