// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_COMMON_UTILS__PUBLISHERS__DATA_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS__PUBLISHERS__DATA_PUBLISHER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// local
#include "romea_common_utils/publishers/publisher.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea
{

template<typename DataType, typename MsgType, typename NodeType>
class DataPublisher : public Publisher<DataType, MsgType, NodeType>
{
private:
  using Base = Publisher<DataType, MsgType, NodeType>;
  using Options = typename Base::Options;

public:
  DataPublisher(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const bool & activated);

  virtual ~DataPublisher() = default;

  void publish(const DataType & data);
};


//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
DataPublisher<DataType, MsgType, NodeType>::DataPublisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const bool & activated)
: Base(node, topic_name, qos, Options(), activated)
{
}

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
void DataPublisher<DataType, MsgType, NodeType>::publish(const DataType & data)
{
  auto msg = std::make_unique<MsgType>();
  to_ros_msg(data, *msg.get());
  this->publish_message_(std::move(msg));
}

//-----------------------------------------------------------------------------
template<typename DataType, typename MsgType, typename NodeType>
std::shared_ptr<DataPublisher<DataType, MsgType, NodeType>>
make_data_publisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const bool & activated)
{
  using Publisher = DataPublisher<DataType, MsgType, NodeType>;
  return std::make_shared<Publisher>(node, topic_name, qos, activated);
}

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PUBLISHERS__DATA_PUBLISHER_HPP_
