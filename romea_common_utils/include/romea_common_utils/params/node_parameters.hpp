#ifndef _romea_NodeParameters_hpp_
#define _romea_NodeParameters_hpp_

//ros
#include <rclcpp/rclcpp.hpp>

//std
#include <exception>

namespace romea {

class NodeParameters
{

public :

  //-----------------------------------------------------------------------------
  NodeParameters(std::shared_ptr<rclcpp::Node> node, const std::string & ns=""):
    ns_(ns),
    node_(node)
  {
  }

  ////-----------------------------------------------------------------------------
  //inline std::string searchParam(const ros::NodeHandle &nodeHandle,const std::string &paramName)
  //{
  //  std::string result;
  //  if(!nodeHandle.searchParam(paramName,result))
  //  {
  //    throw(std::runtime_error("Failed to find "+ paramName +" from param server"));
  //  }
  //  return result;
  //}


  //-----------------------------------------------------------------------------
  inline bool hasParam(const std::string &paramName)
  {
    return node_->has_parameter(paramName);
  }

  //-----------------------------------------------------------------------------
  inline std::string resolveParamName(const std::string &paramName)
  {
    std::string ns = node_->get_effective_namespace()+ns_;
    if(ns.empty() || (ns.back() !='/'))
    {
      ns+="/";
    }
    ns+=paramName;
    return ns;
  }

  //-----------------------------------------------------------------------------
  template <typename T>
  inline bool loadParam(const std::string &param_name, T & value)
  {
    std::string full_param_name=fullParamName_(param_name);
//    if (!node_->has_parameter(name))
//    {
//      return node_->declare_parameter<ParameterT>(name, default_value);
//    }
//    else
//    {
//      return node_->get_parameter(name).get_value<ParameterT>();
//    }
    if(!node_->get_node_options().allow_undeclared_parameters())
    {
      node_->declare_parameter<T>(full_param_name);
    }
    return node_->get_parameter(full_param_name,value);
  }

  //-----------------------------------------------------------------------------
  template <typename T>
  inline T loadParam(const std::string &param_name)
  {
    T value;
    std::string full_param_name=fullParamName_(param_name);
    if(!node_->get_node_options().allow_undeclared_parameters())
    {
      node_->declare_parameter<T>(full_param_name);
    }

    if(!node_->get_parameter(full_param_name,value))
    {
      std::stringstream ss;
      ss << "Failed to read ";
      ss << resolveParamName(full_param_name);
      ss << " from param server";
      throw(std::runtime_error(ss.str()));
    }
    return value;
  }

  //-----------------------------------------------------------------------------
  template <typename T>
  inline T loadParamOr(const std::string &param_name, const T & default_value)
  {
    T value;
    std::string full_param_name=fullParamName_(param_name);
    if(!node_->get_node_options().allow_undeclared_parameters())
    {
      node_->declare_parameter<T>(full_param_name);
    }

    node_->get_parameter_or(full_param_name,value,default_value);
    return value;
  }


  //-----------------------------------------------------------------------------
  template <typename T>
  inline std::vector<T> loadVector(const std::string &param_name)
  {
    return loadParam<std::vector<T>>(param_name);
  }


  //-----------------------------------------------------------------------------
  template <typename T>
  inline std::map<std::string,T> loadMap(const std::string & ns)
  {

    std::map<std::string,T> map;
    std::string full_ns=fullNamespace_(ns);

    if(!node_->get_node_options().allow_undeclared_parameters())
    {
      std::stringstream ss;
      ss << "Failed to read map ";
      ss << resolveParamName(full_ns);
      ss << " from param server, because undeclare parameters are not allowed";
      throw(std::runtime_error(ss.str()));
    }

    if(!node_->get_parameters(full_ns,map))
    {
      std::stringstream ss;
      ss << "Failed to read parameters map from namespace ";
      ss << resolveParamName(full_ns);
      throw(std::runtime_error(ss.str()));
    }
    return map;
  }


private :

  std::string fullParamName_(const std::string &param_name)
  {
    if(ns_.empty())
    {
      return param_name;
    }
    else
    {
      return ns_+"."+param_name;
    }
  }

  std::string fullNamespace_(const std::string & sub_ns ) {
    if(ns_.empty() && sub_ns.empty())
    {
      return "";
    }
    else if(ns_.empty())
    {
      return sub_ns;
    }
    else if(sub_ns.empty())
    {
      return ns_;
    }
    else
    {
      return ns_+"."+sub_ns;
    }
  }

private :


  std::string ns_;
  std::shared_ptr<rclcpp::Node> node_;
};

}

#endif

