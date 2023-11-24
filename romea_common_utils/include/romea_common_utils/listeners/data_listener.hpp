// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_COMMON_UTILS__LISTENERS__DATA_LISTENER_HPP_
#define ROMEA_COMMON_UTILS__LISTENERS__DATA_LISTENER_HPP_

// std
#include <memory>
#include <mutex>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

// romea ros
#include "romea_core_common/concurrency/SharedVariable.hpp"


namespace romea
{
namespace ros2
{

template<typename DataType>
class DataListenerBase
{
public:
  DataListenerBase() {}

  virtual ~DataListenerBase() = default;

  DataType get_data()const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  virtual std::string get_topic_name()const = 0;

protected:
  mutable std::mutex mutex_;
  DataType data_;
};


template<typename DataType, typename MsgType, typename NodeType>
class DataListener : public DataListenerBase<DataType>
{
public:
  DataListener(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos)
  {
    auto callback = std::bind(&DataListener::callback_, this, std::placeholders::_1);
    data_sub_ = node->template create_subscription<MsgType>(topic_name, qos, callback);
  }

  virtual std::string get_topic_name()const
  {
    return data_sub_->get_topic_name();
  }

  virtual ~DataListener() = default;

private:
  void callback_(typename MsgType::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    to_romea(*msg, this->data_);
  }

  std::shared_ptr<rclcpp::Subscription<MsgType>> data_sub_;
};

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
std::shared_ptr<DataListener<DataType, MsgType, NodeType>>
make_data_listener(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const rclcpp::QoS & qos)
{
  using Listener = DataListener<DataType, MsgType, NodeType>;
  return std::make_shared<Listener>(node, topic_name, qos);
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__LISTENERS__DATA_LISTENER_HPP_
