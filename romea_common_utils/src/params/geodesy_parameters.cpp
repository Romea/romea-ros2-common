#include "romea_common_utils/params/geodesy_parameters.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
void declare_geodetic_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                            const std::string &param_name)
{
  declare_vector_parameter<double>(node,param_name);
}

//-----------------------------------------------------------------------------
GeodeticCoordinates get_geodetic_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string &param_name)
{
  std::vector<double> vector = get_vector_parameter<double>(node,param_name);
  return makeGeodeticCoordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI,vector[2]);
}

//-----------------------------------------------------------------------------
void declare_wgs84_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                         const std::string &param_name)
{
  declare_vector_parameter<double>(node,param_name);
}

//-----------------------------------------------------------------------------
WGS84Coordinates get_wgs84_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string &param_name)
{
  std::vector<double> vector = get_vector_parameter<double>(node,param_name);
  return makeWGS84Coordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI);
}


}
