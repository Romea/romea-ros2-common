#ifndef _romea_NodeParameters_hpp_
#define _romea_NodeParameters_hpp_

//ros
#include <rclcpp/rclcpp.hpp>

//std
#include <exception>

namespace romea {

//-----------------------------------------------------------------------------
inline std::string full_param_name(const std::string & ns,
                                   const std::string & param_name)
{
  return ns.empty() ? param_name : ns+"."+param_name;
}

//-----------------------------------------------------------------------------
template <typename T>
inline void declare_parameter(std::shared_ptr<rclcpp::Node> node,
                              const std::string & param_name)
{
  try
  {
    node->declare_parameter<T>(param_name);
  }
  catch (std::runtime_error & e)
  {
    throw std::runtime_error("Declare parameter " + param_name +" : "+e.what());
  }
}

//-----------------------------------------------------------------------------
template <typename T>
inline void declare_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & param_name,
                                           const T & default_value)
{
  node->declare_parameter<T>(param_name,default_value);
}

//-----------------------------------------------------------------------------
template <typename T>
inline void declare_parameter(std::shared_ptr<rclcpp::Node> node,
                              const std::string & param_namespace,
                              const std::string & param_name)
{
  declare_parameter<T>(node,full_param_name(param_namespace,param_name));
}

//-----------------------------------------------------------------------------
template <typename T>
inline void declare_parameter_with_default(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & param_namespace,
                                           const std::string & param_name,
                                           const T & default_value)
{
  declare_parameter_with_default<T>(node,full_param_name(param_namespace,param_name),
                                    default_value);
}

//-----------------------------------------------------------------------------
template <typename T>
inline T get_parameter(std::shared_ptr<rclcpp::Node> node,
                       const std::string & param_name)
{
  T value;
  if(!node->get_parameter(param_name,value))
  {
    std::stringstream ss;
    ss << "Failed to read";
    ss << param_name;
    ss << " from param server";
    throw(std::runtime_error(ss.str()));
  }
  return value;
}

//-----------------------------------------------------------------------------
template <typename T>
inline T get_parameter(std::shared_ptr<rclcpp::Node> node,
                       const std::string & param_namespace,
                       const std::string & param_name)
{
  return get_parameter<T>(node,full_param_name(param_namespace,param_name));
}


//-----------------------------------------------------------------------------
template <typename T>
inline T get_parameter_or(std::shared_ptr<rclcpp::Node> node,
                          const std::string &param_name,
                          const T & default_value)
{
  T value;
  node->get_parameter_or(param_name,value,default_value);
  return value;
}

//-----------------------------------------------------------------------------
template <typename T>
inline T get_parameter_or(std::shared_ptr<rclcpp::Node> node,
                          const std::string &param_namespace,
                          const std::string &param_name,
                          const T & default_value)
{
  T value;
  node->get_parameter_or(full_param_name(param_namespace,param_name),
                         value,default_value);
  return value;
}



//-----------------------------------------------------------------------------
template <typename T>
inline void declare_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & param_name)
{
  node->declare_parameter<std::vector<T>>(param_name);
}

//-----------------------------------------------------------------------------
template <typename T>
inline void declare_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & param_namespace,
                                     const std::string & param_name)
{
  declare_vector_parameter<T>(node,full_param_name(param_namespace,param_name));
}

//-----------------------------------------------------------------------------
template <typename T>
inline std::vector<T> get_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                           const std::string &param_name)
{
  return get_parameter<std::vector<T>>(node,param_name);
}

//-----------------------------------------------------------------------------
template <typename T>
inline std::vector<T> get_vector_parameter(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & param_namespace,
                                           const std::string & param_name)
{
  return get_vector_parameter<T>(node,full_param_name(param_namespace,param_name));
}

//-----------------------------------------------------------------------------
template <typename T>
inline std::map<std::string,T> get_parameters(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & params_namespace)
{
  std::map<std::string,T> map ;
  if(!node->get_parameters(params_namespace,map))
  {
    std::stringstream ss;
    ss << "Failed to read parameters from namespace ";
    ss << params_namespace;
    ss << " from param server";
    throw(std::runtime_error(ss.str()));
  }
  return map;
}

}

#endif

