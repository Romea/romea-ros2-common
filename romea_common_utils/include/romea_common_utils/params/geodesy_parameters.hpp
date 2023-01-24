// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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

//-----------------------------------------------------------------------------
template<typename Node>
void declare_geodetic_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  declare_vector_parameter<double>(node, param_name);
}

//-----------------------------------------------------------------------------
template<typename Node>
void declare_geodetic_coordinates_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const GeodeticCoordinates & default_coordinates)
{
  std::vector<double> default_vector = {default_coordinates.latitude,
    default_coordinates.longitude,
    default_coordinates.altitude};

  declare_vector_parameter_with_default<double>(node, param_name, default_vector);
}

//-----------------------------------------------------------------------------
template<typename Node>
GeodeticCoordinates get_geodetic_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  std::vector<double> vector = get_vector_parameter<double>(node, param_name);
  return makeGeodeticCoordinates(vector[0] / 180. * M_PI, vector[1] / 180. * M_PI, vector[2]);
}

//-----------------------------------------------------------------------------
template<typename Node>
void declare_wgs84_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  declare_vector_parameter<double>(node, param_name);
}

//-----------------------------------------------------------------------------
template<typename Node>
void declare_wgs84_coordinates_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const WGS84Coordinates & default_coordinates)
{
  std::vector<double> default_vector = {default_coordinates.latitude,
    default_coordinates.longitude};

  declare_vector_parameter_with_default<double>(node, param_name, default_vector);
}

//-----------------------------------------------------------------------------
template<typename Node>
WGS84Coordinates get_wgs84_coordinates_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  std::vector<double> vector = get_vector_parameter<double>(node, param_name);
  return makeWGS84Coordinates(vector[0] / 180. * M_PI, vector[1] / 180. * M_PI);
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PARAMS__GEODESY_PARAMETERS_HPP_
