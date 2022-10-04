#ifndef _romea_RosEigenParam_hpp_
#define _romea_RosEigenParam_hpp_

//eigen
#include <Eigen/Core>

//ros
#include "node_parameters.hpp"

//romea
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_common/math/Transformation.hpp"

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


//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_xyz_vector_parameter(std::shared_ptr<rclcpp::Node> node)
{
  declare_eigen_vector_parameter<EigenVector>(node,"xyz");
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_xyz_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                               const std::string & param_namespace)
{
  declare_eigen_vector_parameter<EigenVector>(
        node,full_param_name(param_namespace,"xyz"));
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_xyz_vector_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                            const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(node,"xyz",default_values);
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_xyz_vector_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                            const std::string & param_namespace,
                                                            const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(
        node,full_param_name(param_namespace,"xyz"),default_values);
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector get_eigen_xyz_vector_parameter(std::shared_ptr<rclcpp::Node> node)
{
  return get_eigen_vector_parameter<EigenVector>(node,"xyz");
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector get_eigen_xyz_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                                  const std::string & param_namespace)
{
  return get_eigen_vector_parameter<EigenVector>(
        node,full_param_name(param_namespace,"xyz"));
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_rpy_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                               const std::string & param_namespace)
{
  declare_eigen_vector_parameter<EigenVector>(
        node,full_param_name(param_namespace,"rpy"));
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_rpy_vector_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                            const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(node,"rpy",default_values);
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline void declare_eigen_rpy_vector_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                            const std::string & param_namespace,
                                                            const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(
        node,full_param_name(param_namespace,"rpy"),default_values);
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector get_eigen_rpy_vector_parameter(std::shared_ptr<rclcpp::Node> node)
{
  return get_eigen_vector_parameter<EigenVector>(node,"rpy");
}

//-----------------------------------------------------------------------------
template <typename EigenVector>
inline EigenVector get_eigen_rpy_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                                  const std::string & param_namespace)
{
  return get_eigen_vector_parameter<EigenVector>(
        node,full_param_name(param_namespace,"rpy"));
}


//-----------------------------------------------------------------------------
template <typename EigenAffine>
void declare_eigen_rigid_transformation_parameter(std::shared_ptr<rclcpp::Node> node)
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar,3,1>;
  
  declare_eigen_rpy_vector_parameter<EigenVector>(node);
  declare_eigen_xyz_vector_parameter<EigenVector>(node);
}

//-----------------------------------------------------------------------------
template <typename EigenAffine>
void declare_eigen_rigid_transformation_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                               const EigenAffine & default_value = EigenAffine::Identity())
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar,3,1>;

  auto translation = default_value.translation();
  auto angles = rotation3DToEulerAngles(default_value.rotation());
  declare_eigen_rpy_vector_parameter_with_default<EigenVector>(node,angles);
  declare_eigen_xyz_vector_parameter_with_default<EigenVector>(node,translation);
}

//-----------------------------------------------------------------------------
template <typename EigenAffine>
void declare_eigen_rigid_transformation_parameter(std::shared_ptr<rclcpp::Node> node,
                                                  const std::string & param_namespace)
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar,3,1>;

  declare_eigen_rpy_vector_parameter<EigenVector>(node,param_namespace);
  declare_eigen_xyz_vector_parameter<EigenVector>(node,param_namespace);
}

//-----------------------------------------------------------------------------
template <typename EigenAffine>
void declare_eigen_rigid_transformation_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                                               const std::string & param_namespace,
                                                               const EigenAffine & default_value = EigenAffine::Identity())
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar,3,1>;

  auto translation = default_value.translation();
  auto angles = rotation3DToEulerAngles(default_value.rotation());
  declare_eigen_rpy_vector_parameter_with_default<EigenVector>(node,param_namespace,angles);
  declare_eigen_xyz_vector_parameter_with_default<EigenVector>(node,param_namespace,translation);
}

//-----------------------------------------------------------------------------
template <typename EigenAffine>
inline EigenAffine get_eigen_rigid_transformation_parameter(std::shared_ptr<rclcpp::Node> node,
                                                            const std::string & param_namespace)
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar,3,1>;

  auto rotation= get_eigen_rpy_vector_parameter<EigenVector>(node,param_namespace);
  auto translation= get_eigen_xyz_vector_parameter<EigenVector>(node,param_namespace);
  return rigid_transformation3<Scalar>(translation,rotation);
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
