// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__SERVICES__SERVICE_CLIENT_ASYNC_HPP_
#define ROMEA_COMMON_UTILS__SERVICES__SERVICE_CLIENT_ASYNC_HPP_

// std
#include <memory>
#include <string>
#include <chrono>
#include <sstream>

// ros
#include "rclcpp/rclcpp.hpp"

namespace romea
{

template<typename Service, typename Node = rclcpp::Node>
class ServiceClientAsync
{
public:
  using Request = typename Service::Request::SharedPtr;
  using Response = typename Service::Response::SharedPtr;

public:
  template<typename RepT = int64_t, typename RatioT = std::milli>
  ServiceClientAsync(
    std::shared_ptr<Node> node,
    const std::string & service_name,
    const std::chrono::duration<RepT, RatioT> & timeout,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group = nullptr)
  : name_(service_name), node_(node), client_(nullptr)
  {
    client_ = node->template create_client<Service>(
      service_name, rmw_qos_profile_services_default, callback_group);

    if (!client_->wait_for_service(timeout)) {
      std::stringstream msg;
      msg << "Failed to call ";
      msg << client_->get_service_name();
      msg << " service";
      throw(std::runtime_error(msg.str()));
    }
  }

  Response send_request(Request request)
  {
    if (!service_server_is_always_ready()) {
      return nullptr;
    }

    auto result = client_->async_send_request(request);
    result.wait();
    return result.get();
  }

  template<typename RepT = int64_t, typename RatioT = std::milli>
  Response send_request(
    Request request,
    const std::chrono::duration<RepT, RatioT> & timeout)
  {
    if (!service_server_is_always_ready()) {
      return nullptr;
    }

    auto result = client_->async_send_request(request);
    auto status = result.wait_for(timeout);
    assert(status != std::future_status::deferred);

    if (status == std::future_status::ready) {
      return result.get();
    } else {
      std::stringstream msg;
      msg << "Failed to get response from ";
      msg << name_;
      msg << " service ";
      RCLCPP_ERROR_STREAM(node_->get_logger(), msg.str());
      return nullptr;
    }
  }

private:
  bool service_server_is_always_ready()
  {
    if (!client_->service_is_ready()) {
      std::stringstream msg;
      msg << "Failed to send request to ";
      msg << name_;
      msg << " service because is not ready anymore";
      RCLCPP_ERROR_STREAM(node_->get_logger(), msg.str());
      return false;
    } else {
      return true;
    }
  }

private:
  std::string name_;
  std::shared_ptr<Node> node_;
  std::shared_ptr<rclcpp::Client<Service>> client_;
};

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__SERVICES__SERVICE_CLIENT_ASYNC_HPP_
