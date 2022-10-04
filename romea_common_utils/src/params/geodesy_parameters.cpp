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
void declare_geodetic_coordinates_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                         const std::string &param_name,
                                                         const GeodeticCoordinates & default_coordinates)
{
  std::vector<double> default_vector ={default_coordinates.latitude,
                                       default_coordinates.longitude,
                                       default_coordinates.altitude};

  declare_vector_parameter_with_default<double>(node,param_name,default_vector);
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
void declare_wgs84_coordinates_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                      const std::string &param_name,
                                                      const WGS84Coordinates & default_coordinates)
{
  std::vector<double> default_vector ={default_coordinates.latitude,
                                       default_coordinates.longitude};

  declare_vector_parameter_with_default<double>(node,param_name,default_vector);
}

//-----------------------------------------------------------------------------
WGS84Coordinates get_wgs84_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string &param_name)
{
  std::vector<double> vector = get_vector_parameter<double>(node,param_name);
  return makeWGS84Coordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI);
}


}
