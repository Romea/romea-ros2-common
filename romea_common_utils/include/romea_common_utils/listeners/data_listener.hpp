#ifndef ROMEA_COMMON_UTILS_LISTENERS_DATA_LISTENER_HPP_ 
#define ROMEA_COMMON_UTILS_LISTENERS_DATA_LISTENER_HPP_ 

// std
#include <string>
#include <mutex>
#include <memory>

// ros
#include <rclcpp/rclcpp.hpp>

// romea
#include <romea_core_common/concurrency/SharedVariable.hpp>

namespace romea
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

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_LISTENERS_DATA_LISTENER_HPP_ 
