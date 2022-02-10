#ifndef _romea_RosEigenParam_hpp_
#define _romea_RosEigenParam_hpp_

//eigen
#include <Eigen/Core>

//ros
#include "node_parameters.hpp"

//romea
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"

namespace romea {

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & param_name)
{
  declare_vector_parameter<typename EigenVector::Scalar>(node,param_name);
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_vector_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                        const std::string & param_name,
                                                        const EigenVector & default_values)
{
  using StdVector = std::vector<typename EigenVector::Scalar>;
  StdVector default_vector(default_values.data(),default_values.data()+default_values.size());
  declare_vector_parameter_with_default<typename EigenVector::Scalar>(node,param_name,default_vector);
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & param_namespace,
                                           const std::string & param_name)
{
  declare_eigen_vector_parameter<EigenVector>(
        node,full_param_name(param_namespace,param_name));
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_vector_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                        const std::string & param_namespace,
                                                        const std::string & param_name,
                                                        const EigenVector & default_values)
{
  declare_eigen_vector_parameter_with_default<EigenVector>(
        node,full_param_name(param_namespace,param_name),default_values);
}


//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector get_eigen_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & param_name)
{
  return EigenVector(get_vector_parameter<typename EigenVector::Scalar>(node,param_name).data());
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector get_eigen_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & param_namespace,
                                              const std::string & param_name)
{
  return get_eigen_vector_parameter<EigenVector>(
        node,full_param_name(param_namespace,param_name));
}


////-----------------------------------------------------------------------------
//template <typename EigenVector>
//inline EigenVector loadEigenVector(NodeParameters & node_parameters,
//                                   const std::string & paramName)
//{

//  return EigenVector(node_parameters.loadVector<typename EigenVector::Scalar>(paramName).data());
//}


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
