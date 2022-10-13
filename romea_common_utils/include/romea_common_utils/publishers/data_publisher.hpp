#ifndef _romea_DataPublisher_hpp_
#define _romea_DataPublisher_hpp_

//ros
#include "publisher.hpp"
#include "../conversions/time_conversions.hpp"

namespace romea {

template <typename DataType, typename MsgType, typename NodeType>
class DataPublisher : public Publisher<DataType,MsgType,NodeType>
{

private:

  using Base = Publisher<DataType,MsgType,NodeType>;
  using Options = typename Base::Options;

public :

  DataPublisher(std::shared_ptr<NodeType> node,
                const std::string & topic_name,
                const rclcpp::QoS & qos,
                const bool & activated);

  virtual ~DataPublisher()=default;

  void publish(const DataType & data);

};


//-----------------------------------------------------------------------------
template <typename DataType, typename MsgType, typename NodeType>
DataPublisher<DataType,MsgType,NodeType>::
DataPublisher(std::shared_ptr<NodeType> node,
              const std::string & topic_name,
              const rclcpp::QoS &qos,
              const bool &activated):
  Base(node,topic_name,qos,Options(), activated)
{
}

//-----------------------------------------------------------------------------
template <typename DataType, typename MsgType, typename NodeType>
void DataPublisher<DataType,MsgType,NodeType>::publish(const DataType &data)
{
  auto msg= std::make_unique<MsgType>();
  to_ros_msg(data,*msg.get());
  this->publish_message_(std::move(msg));
}

//-----------------------------------------------------------------------------
template <typename DataType, typename MsgType, typename NodeType>
std::shared_ptr<DataPublisher<DataType,MsgType,NodeType>>
make_data_publisher(std::shared_ptr<NodeType> node,
                    const std::string & topic_name,
                    const rclcpp::QoS & qos,
                    const bool & activated)
{
  using Publisher = DataPublisher<DataType,MsgType,NodeType>;
  return std::make_shared<Publisher>(node,topic_name,qos,activated);
}

}

#endif
