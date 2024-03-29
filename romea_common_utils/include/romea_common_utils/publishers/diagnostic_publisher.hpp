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


#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__DIAGNOSTIC_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__DIAGNOSTIC_PUBLISHER_HPP_


// std
#include <memory>
#include <string>
#include <utility>

// ros
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

// local
#include "romea_common_utils/publishers/stamped_publisher.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_common_utils/conversions/diagnostic_conversions.hpp"

namespace romea
{
namespace ros2
{

template<typename DataType, typename NodeType>
class DiagnosticPublisher : public StampedPublisher<DataType, diagnostic_msgs::msg::DiagnosticArray,
    NodeType>
{
private:
  using Base = StampedPublisher<DataType, diagnostic_msgs::msg::DiagnosticArray, NodeType>;
  using Options = typename Base::Options;

public:
  DiagnosticPublisher(
    std::shared_ptr<NodeType> node,
    const std::string & diagnostic_name,
    const double & diagnostic_period,
    const std::string & hardware_id,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const bool & activated);

  void publish(
    const core::Duration & duration,
    const DataType & data);

  void publish(
    const rclcpp::Time & stamp,
    const DataType & data);

private:
  void publish_(
    const rclcpp::Time & stamp,
    const DataType & data);

private:
  std::string diagnostic_name_;
  std::string hardware_id_;
  rclcpp::Time next_time_;
  rclcpp::Duration diagnostic_period_;
};


//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
DiagnosticPublisher<DataType, NodeType>::DiagnosticPublisher(
  std::shared_ptr<NodeType> node,
  const std::string & diagnostic_name,
  const double & diagnostic_period,
  const std::string & hardware_id,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const bool & activated)
: Base(node, topic_name, qos, Options(), activated),
  diagnostic_name_(diagnostic_name),
  hardware_id_(hardware_id),
  next_time_(node->get_clock()->now()),
  diagnostic_period_(rclcpp::Duration(romea::core::durationFromSecond(diagnostic_period)))
{
}

//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
void DiagnosticPublisher<DataType, NodeType>::publish(
  const core::Duration & duration,
  const DataType & data)
{
  publish(to_ros_time(duration), data);
}

//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
void DiagnosticPublisher<DataType, NodeType>::publish(
  const rclcpp::Time & stamp,
  const DataType & data)
{
  if (stamp > next_time_) {
    publish_(stamp, data);
    next_time_ = stamp + diagnostic_period_;
  }
}

//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
void DiagnosticPublisher<DataType, NodeType>::publish_(
  const rclcpp::Time & stamp,
  const DataType & data)
{
  auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
  msg->header.stamp = stamp;
  msg->status.push_back(diagnostic_msgs::msg::DiagnosticStatus());
  to_ros_diagnostic_msg(diagnostic_name_, hardware_id_, data, msg->status.back());
  this->publish_message_(std::move(msg));
}

//-----------------------------------------------------------------------------
template<typename DataType, typename NodeType>
std::shared_ptr<DiagnosticPublisher<DataType, NodeType>>
make_diagnostic_publisher(
  std::shared_ptr<NodeType> node,
  const std::string & diagnostic_name,
  const double & diagnostic_period,
  const std::string & hardware_id = "",
  const std::string & topic_name = "/diagnostics",
  const rclcpp::QoS & qos = rclcpp::SystemDefaultsQoS(),
  const bool & activated = true)
{
  using Publisher = DiagnosticPublisher<DataType, NodeType>;
  return std::make_shared<Publisher>(
    node,
    diagnostic_name,
    diagnostic_period,
    hardware_id,
    topic_name,
    qos,
    activated);
}

}  // namespace ros2
}  // namespace romea


#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__DIAGNOSTIC_PUBLISHER_HPP_
