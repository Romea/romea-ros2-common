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

#ifndef ROMEA_COMMON_UTILS__LOG__TOPICLOGGER_HPP_
#define ROMEA_COMMON_UTILS__LOG__TOPICLOGGER_HPP_

#include <memory>
#include <rclcpp/logging.hpp>
#include <romea_common_msgs/msg/data_logger.hpp>
#include <romea_common_utils/publishers/ros_publisher.hpp>
#include <romea_core_common/log/Logger.hpp>

namespace romea::ros2
{

template<typename Node>
class TopicLogger : public core::Logger
{
public:
  using DataLoggerMsg = romea_common_msgs::msg::DataLogger;
  using Options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

public:
  TopicLogger(
    std::shared_ptr<Node> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const Options & options,
    bool activated)
  : node_(std::move(node)),
    pub_(node_, topic_name, qos, options, activated)
  {
  }

  void addEntry(const std::string & name, double value) override
  {
    msg_.names.emplace_back(name);
    msg_.values.emplace_back(value);
  }

  void addEntry(const std::string & name, int value) override
  {
    addEntry(name, static_cast<double>(value));
  }

  void addEntry(const std::string & name, const std::string & /*value*/) override
  {
    auto logger = rclcpp::get_logger("TopicLogger");
    RCLCPP_WARN_STREAM(logger, "ignoring log entry '" << name << "': type string is not handled");
  }

  void writeRow() override
  {
    msg_.header.stamp = node_->get_clock()->now();
    pub_.publish(msg_);
    msg_.names.clear();
    msg_.values.clear();
  }

  void clearRow() override
  {
    msg_.names.clear();
    msg_.values.clear();
  }

  void activate() { pub_.activate(); }

  void deactivate() { pub_.deactivate(); }

private:
  std::shared_ptr<Node> node_;
  ROSPublisher<DataLoggerMsg, Node> pub_;
  DataLoggerMsg msg_;
};

}  // namespace romea::ros2

#endif
