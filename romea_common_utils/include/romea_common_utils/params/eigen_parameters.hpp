#ifndef _romea_RosEigenParam_hpp_
#define _romea_RosEigenParam_hpp_

//ros
#include "RosParam.hpp"
#include "node_parameters.hpp"

//romea
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"

namespace romea {


////-----------------------------------------------------------------------------
//template <typename EigenVector>
//inline EigenVector loadEigenVector(std::shared_ptr<rclcpp::Node> node,
//                                   const std::string & paramName)
//{
//  return EigenVector(loadVector<typename EigenVector::Scalar>(node,paramName).data());
//}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector loadEigenVector(NodeParameters & node_parameters,
                                   const std::string & paramName)
{
  return EigenVector(node_parameters.loadVector<typename EigenVector::Scalar>(paramName).data());
}


////-----------------------------------------------------------------------------
//template <typename V>
//inline VectorOfEigenVector<V> loadVectorOfEigenVector(ros::NodeHandle &nodeHandle,
//                                                      const std::string &paramName)
//{

//  VectorOfEigenVector<V> vector;
//  std::string errorMessage;
//  if(!loadVectorOfEigenVector(nodeHandle,paramName,vector,errorMessage))
//  {
//    ROS_ERROR("%s",errorMessage.c_str());
//  }
//  return vector;
//}


}

#endif

