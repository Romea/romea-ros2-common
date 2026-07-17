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

#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_PUBLISHER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// romea core
#include "romea_core_common/time/Time.hpp"

// local
#include "romea_common_utils/publishers/ros_publisher.hpp"

namespace romea
{
namespace ros2
{

template<typename DataType>
class StampedPublisherBase
{
public:
  StampedPublisherBase() {}

  virtual ~StampedPublisherBase() = default;

  virtual std::string get_topic_name() const = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual bool is_activated() = 0;

  virtual void publish(const core::Duration & stamp, const DataType & data) = 0;

  virtual void publish(const rclcpp::Time & stampe, const DataType & data) = 0;
};

template<typename DataType, typename MsgType, typename NodeType>
class StampedPublisher : public StampedPublisherBase<DataType>
{
public:
  using Options = typename ROSPublisher<MsgType, NodeType>::Options;

public:
  StampedPublisher(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const Options & options,
    const bool & activated)
  : ros_publisher_(node, topic_name, qos, options, activated)
  {
  }

  virtual ~StampedPublisher() = default;

  std::string get_topic_name() const override { return ros_publisher_.get_topic_name(); }

  void activate() override { ros_publisher_.activate(); }

  void deactivate() override { ros_publisher_.deactivate(); }

  bool is_activated() override { return ros_publisher_.is_activated(); }

protected:
  void publish_message_(std::unique_ptr<MsgType> message)
  {
    ros_publisher_.publish(std::move(message));
  }

  void publish_message_(const MsgType & message) { ros_publisher_.publish(message); }

protected:
  ROSPublisher<MsgType, NodeType> ros_publisher_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__STAMPED_PUBLISHER_HPP_
