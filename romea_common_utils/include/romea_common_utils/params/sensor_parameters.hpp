#ifndef ROMEA_COMMON_UTILS_PARAMS_SENSOR_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS_PARAMS_SENSOR_PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// romea
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_device(std::shared_ptr<NodeType> node)
{
  declare_parameter<std::string>(node, "device");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_baudrate(std::shared_ptr<NodeType> node)
{
  declare_parameter<int>(node, "baudrate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_ip(std::shared_ptr<NodeType> node)
{
  declare_parameter<std::string>(node, "ip");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_port(std::shared_ptr<NodeType> node)
{
  declare_parameter<int>(node, "port");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_rate(std::shared_ptr<NodeType> node)
{
  declare_parameter<int>(node, "rate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline void declare_frame_id(std::shared_ptr<NodeType> node)
{
  declare_parameter<std::string>(node, "frame_id");
}
//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_device(std::shared_ptr<NodeType> node)
{
  return get_parameter<std::string>(node, "device");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_baudrate(std::shared_ptr<NodeType> node)
{
  return get_parameter<int>(node, "baudrate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_ip(std::shared_ptr<NodeType> node)
{
  return get_parameter<std::string>(node, "ip");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_port(std::shared_ptr<NodeType> node)
{
  return get_parameter<int>(node, "port");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline int get_rate(std::shared_ptr<NodeType> node)
{
  return get_parameter<int>(node, "rate");
}

//-----------------------------------------------------------------------------
template<typename NodeType>
inline std::string get_frame_id(std::shared_ptr<NodeType> node)
{
  return get_parameter<std::string>(node, "frame_id");
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_PARAMS_SENSOR_PARAMETERS_HPP_
