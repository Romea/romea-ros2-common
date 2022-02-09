#ifndef _romea_NodeParams_hpp_
#define _romea_NodeParams_hpp_


//romea
#include <romea_core_common/geodesy/GeodeticCoordinates.hpp>
#include "node_parameters.hpp"

//std
#include <exception>

namespace romea {


void declare_geodetic_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                            const std::string &param_name);

GeodeticCoordinates get_geodetic_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string &param_name);

void declare_wgs84_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                         const std::string &param_name);

WGS84Coordinates get_wgs84_coordinates_parameter(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string &param_name);

////-----------------------------------------------------------------------------
//inline GeodeticCoordinates loadGeodeticCoordinates(NodeParameters & node_parameters,
//                                                   const std::string &param_name)
//{
//  std::vector<double> vector = node_parameters.loadVector<double>(param_name);
//  return makeGeodeticCoordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI,vector[2]);
//}

////-----------------------------------------------------------------------------
//inline WGS84Coordinates loadWGS84Coordinates(NodeParameters & node_parameters,
//                                             const std::string &param_name)
//{
//  std::vector<double> vector =  node_parameters.loadVector<double>(param_name);
//  return makeWGS84Coordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI);
//}


}

#endif

