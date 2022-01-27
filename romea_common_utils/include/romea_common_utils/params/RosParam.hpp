//#ifndef _romea_RosParam_hpp_
//#define _romea_RosParam_hpp_

////ros
//#include <rclcpp/rclcpp.hpp>

////romea
//#include <romea_common/geodesy/GeodeticCoordinates.hpp>

////std
//#include <exception>

//namespace romea {


//////-----------------------------------------------------------------------------
////inline std::string searchParam(const ros::NodeHandle &nodeHandle,const std::string &paramName)
////{
////  std::string result;
////  if(!nodeHandle.searchParam(paramName,result))
////  {
////    throw(std::runtime_error("Failed to find "+ paramName +" from param server"));
////  }
////  return result;
////}


////-----------------------------------------------------------------------------
//inline bool hasParam(std::shared_ptr<rclcpp::Node> node,const std::string &paramName)
//{
//    return node->has_parameter(paramName);
//}

////-----------------------------------------------------------------------------
//inline std::string resolveParamName(std::shared_ptr<rclcpp::Node> node, const std::string &paramName)
//{
//   std::string ns = node->get_effective_namespace();
//   if(ns.empty() || (ns.back() !='/'))
//   {
//       ns+="/";
//   }
//   ns+=paramName;
//   return ns;
//}

////-----------------------------------------------------------------------------
//template <typename T>
//inline bool loadParam(std::shared_ptr<rclcpp::Node> node,const std::string &paramName, T & value)
//{
//  if(!node->get_node_options().allow_undeclared_parameters())
//  {
//    std::cout << " declare "<< paramName << std::endl;
//      node->declare_parameter(paramName);
//  }
//  return node->get_parameter(paramName,value);
//}

////-----------------------------------------------------------------------------
//template <typename T>
//inline T loadParam(std::shared_ptr<rclcpp::Node> node,const std::string &paramName)
//{
//    T value;
//    if(!node->get_node_options().allow_undeclared_parameters())
//    {
//        node->declare_parameter(paramName);
//    }

//    if(!node->get_parameter(paramName,value))
//    {
//        std::stringstream ss;
//        ss << "Failed to read ";
//        ss << resolveParamName(node,paramName);
//        ss << " from param server";
//        throw(std::runtime_error(ss.str()));
//    }
//    return value;
//}

////-----------------------------------------------------------------------------
//template <typename T>
//inline std::vector<T> loadVector(std::shared_ptr<rclcpp::Node> node,
//                                 const std::string &paramName)
//{
//    return loadParam<std::vector<T>>(node,paramName);
//}

////-----------------------------------------------------------------------------
//inline GeodeticCoordinates loadGeodeticCoordinates(std::shared_ptr<rclcpp::Node> node,
//                                                   const std::string &paramName)
//{
//    std::vector<double> vector = loadVector<double>(node,paramName);
//    return GeodeticCoordinates(vector[0]/180.*M_PI,vector[1]/180.*M_PI,vector[2]);
//}

////-----------------------------------------------------------------------------
//template <typename T>
//inline std::map<std::string,T> loadMap(std::shared_ptr<rclcpp::Node> node,
//                                       const std::string &paramName)
//{

//    std::map<std::string,T> map;
//    if(!node->get_node_options().allow_undeclared_parameters())
//    {
//        std::stringstream ss;
//        ss << "Failed to read map ";
//        ss << resolveParamName(node,paramName);
//        ss << " from param server, because undeclare parameters are not allowed";
//        throw(std::runtime_error(ss.str()));
//    }

//    std::cout << " paramName" << paramName << std::endl;
//    if(!node->get_parameters(paramName,map))
//    {
//        std::stringstream ss;
//        ss << "Failed to read parameters map from namespace ";
//        ss << resolveParamName(node,paramName);
//        throw(std::runtime_error(ss.str()));
//    }
//    return map;
//}


//}

//#endif

