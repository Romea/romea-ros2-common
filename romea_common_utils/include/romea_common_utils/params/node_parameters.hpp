#ifndef ROMEA_COMMON_UTILS_PARAMS_NODE_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS_PARAMS_NODE_PARAMETERS_HPP_

// std
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ros
#include <rclcpp/rclcpp.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
inline std::string full_param_name(
  const std::string & ns,
  const std::string & param_name)
{
  return ns.empty() ? param_name : ns + "." + param_name;
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline void declare_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  try {
    node->template declare_parameter<T>(param_name);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Declare parameter " + param_name + " : " + e.what());
  }
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline void declare_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const T & default_value)
{
  node->template declare_parameter<T>(param_name, default_value);
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline void declare_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name)
{
  declare_parameter<T>(node, full_param_name(param_namespace, param_name));
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline void declare_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name,
  const T & default_value)
{
  declare_parameter_with_default<T>(
    node, full_param_name(param_namespace, param_name),
    default_value);
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline T get_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  T value;
  if (!node->template get_parameter(param_name, value)) {
    std::stringstream ss;
    ss << "Failed to read ";
    ss << param_name;
    ss << " from param server";
    throw(std::runtime_error(ss.str()));
  }
  return value;
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline T get_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name)
{
  return get_parameter<T>(node, full_param_name(param_namespace, param_name));
}


//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline T get_parameter_or(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const T & default_value)
{
  T value;
  node->template get_parameter_or(param_name, value, default_value);
  return value;
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline T get_parameter_or(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name,
  const T & default_value)
{
  T value;
  node->template get_parameter_or(
    full_param_name(param_namespace, param_name), value, default_value);
  return value;
}


//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline void declare_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  node->template declare_parameter<std::vector<T>>(param_name);
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline void declare_vector_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const std::vector<T> & default_values)
{
  node->template declare_parameter<std::vector<T>>(param_name, default_values);
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline void declare_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name)
{
  declare_vector_parameter<T>(node, full_param_name(param_namespace, param_name));
}

//-----------------------------------------------------------------------------
template<typename T>
inline void declare_vector_parameter_with_default(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & param_namespace,
  const std::string & param_name,
  const std::vector<T> & default_values)
{
  declare_vector_parameter_with_default<T>(
    node, full_param_name(param_namespace, param_name), default_values);
}


//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline std::vector<T> get_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  return get_parameter<std::vector<T>>(node, param_name);
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline std::vector<T> get_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name)
{
  return get_vector_parameter<T>(node, full_param_name(param_namespace, param_name));
}

//-----------------------------------------------------------------------------
template<typename T, typename Node>
inline std::map<std::string, T> get_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_namespace)
{
  std::map<std::string, T> map;
  if (!node->template get_parameters(params_namespace, map)) {
    std::stringstream ss;
    ss << "Failed to read parameters from namespace ";
    ss << params_namespace;
    ss << " from param server";
    throw(std::runtime_error(ss.str()));
  }
  return map;
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_PARAMS_NODE_PARAMETERS_HPP_
