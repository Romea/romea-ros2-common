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

#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__ROS_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__ROS_PUBLISHER_HPP_

// std
#include <atomic>
#include <memory>
#include <string>
#include <utility>

// ros
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace romea
{
namespace ros2
{

template<typename MsgType>
class ROSPublisherBase
{
public:
  ROSPublisherBase() {}

  virtual ~ROSPublisherBase() = default;

  virtual std::string get_topic_name() const = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual bool is_activated() = 0;

  virtual void publish(std::unique_ptr<MsgType> message) = 0;

  virtual void publish(const MsgType & message) = 0;
};

template<typename MsgType, typename NodeType>
class ROSPublisher
{
};

template<typename MsgType>
class ROSPublisher<MsgType, rclcpp::Node> : public ROSPublisherBase<MsgType>
{
public:
  using Options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

public:
  ROSPublisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const Options & options,
    bool activated)
  : is_activated_(activated),
    pub_(node->create_publisher<MsgType>(topic_name, qos, options)),
    should_log_(true),
    logger_(rclcpp::get_logger("ROSPublisher"))
  {
  }

  virtual ~ROSPublisher() = default;

  std::string get_topic_name() const override { return pub_->get_topic_name(); }

  void activate() override { is_activated_.store(true); }

  void deactivate() override { is_activated_.store(false); }

  bool is_activated() override { return is_activated_.load(); }

  void publish(std::unique_ptr<MsgType> message) override
  {
    if (!this->is_activated()) {
      log_publisher_not_enabled();
      return;
    }
    pub_->publish(std::move(message));
  }

  void publish(const MsgType & message) override
  {
    if (!this->is_activated()) {
      log_publisher_not_enabled();
      return;
    }

    pub_->publish(message);
  }

private:
  void log_publisher_not_enabled()
  {
    if (!should_log_) {
      return;
    }

    RCLCPP_WARN(
      logger_,
      "Trying to publish message on the topic '%s', but the publisher is not activated",
      this->get_topic_name().c_str());

    should_log_ = false;
  }

private:
  std::atomic<bool> is_activated_;
  std::shared_ptr<rclcpp::Publisher<MsgType>> pub_;

  bool should_log_;
  rclcpp::Logger logger_;
};

template<typename MsgType>
class ROSPublisher<MsgType, rclcpp_lifecycle::LifecycleNode> : public ROSPublisherBase<MsgType>
{
public:
  using Options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

public:
  ROSPublisher(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const Options & options,
    bool activated)
  : pub_(node->create_publisher<MsgType>(topic_name, qos, options))
  {
    if (activated) {
      pub_->on_activate();
    }
  }

  virtual ~ROSPublisher() = default;

  std::string get_topic_name() const override { return pub_->get_topic_name(); }

  void activate() override { pub_->on_activate(); }

  void deactivate() override { pub_->on_deactivate(); }

  bool is_activated() override { return pub_->is_activated(); }

  void publish(std::unique_ptr<MsgType> message) override { pub_->publish(std::move(message)); }

  void publish(const MsgType & message) override { pub_->publish(message); }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgType>> pub_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__ROS_PUBLISHER_HPP_
