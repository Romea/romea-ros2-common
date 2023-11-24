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


#ifndef ROMEA_COMMON_UTILS__PARAMS__GEODESY_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS__PARAMS__GEODESY_PARAMETERS_HPP_


// std
#include <exception>
#include <string>
#include <vector>
#include <memory>

// romea core
#include "romea_core_common/geodesy/GeodeticCoordinates.hpp"

// local
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename Node>
void declare_wgs84_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  declare_parameter<double>(node, param_name, "latitude");
  declare_parameter<double>(node, param_name, "longitude");
}

//-----------------------------------------------------------------------------
template<typename Node>
void declare_wgs84_coordinates_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const core::WGS84Coordinates & default_coordinates)
{
  declare_parameter_with_default<double>(
    node, param_name, "latitude",
    default_coordinates.latitude * 180 / M_PI);
  declare_parameter_with_default<double>(
    node, param_name, "longitude",
    default_coordinates.longitude * 180 / M_PI);
}


//-----------------------------------------------------------------------------
template<typename Node>
void declare_geodetic_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  declare_wgs84_coordinates_parameter(node, param_name),
  declare_parameter<double>(node, param_name, "altitude");
}

//-----------------------------------------------------------------------------
template<typename Node>
void declare_geodetic_coordinates_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const core::GeodeticCoordinates & default_coordinates)
{
  declare_wgs84_coordinates_parameter_with_default(
    node, param_name, default_coordinates);

  declare_parameter_with_default<double>(
    node, param_name, "altitude",
    default_coordinates.altitude);
}

//-----------------------------------------------------------------------------
template<typename Node>
core::WGS84Coordinates get_wgs84_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  return core::makeWGS84Coordinates(
    get_parameter<double>(node, param_name, "latitude") / 180 * M_PI,
    get_parameter<double>(node, param_name, "longitude") / 180 * M_PI);
}


//-----------------------------------------------------------------------------
template<typename Node>
core::GeodeticCoordinates get_geodetic_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  return core::makeGeodeticCoordinates(
    get_wgs84_coordinates_parameter(node, param_name),
    get_parameter<double>(node, param_name, "altitude"));
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PARAMS__GEODESY_PARAMETERS_HPP_
