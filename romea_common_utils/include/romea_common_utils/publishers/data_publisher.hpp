#ifndef _romea_MessagePublisher_hpp_
#define _romea_MessagePublisher_hpp_

//ros
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include "../conversions/time_conversions.hpp"

namespace romea {

template<class DataType>
class DataPublisherBase
{
public :

  DataPublisherBase(){}

  virtual void publish(const DataType & data)=0;

  virtual std::string get_topic_name()const =0;

  virtual ~DataPublisherBase()=default;
};


template <class DataType, class MessageType>
class DataPublisher : public DataPublisherBase<DataType>
{

public :

  DataPublisher(std::shared_ptr<rclcpp::Node> node,
                const std::string & topic_name,
                const rclcpp::QoS & qos);

  virtual void publish(const DataType & data)override;

  virtual std::string get_topic_name()const override;

  virtual ~DataPublisher()=default;

protected :

  std::shared_ptr<rclcpp::Publisher<MessageType>> pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
DataPublisher<DataType,MessageType>::DataPublisher(std::shared_ptr<rclcpp::Node> node,
                                                   const std::string & topic_name,
                                                   const rclcpp::QoS &qos):
  pub_(node->create_publisher<MessageType>(topic_name,qos))
{
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void DataPublisher<DataType,MessageType>::publish(const DataType &data)
{
  auto msg= std::make_unique<MessageType>();
  to_ros_msg(data,*msg.get());
  pub_->publish(std::move(msg));
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
std::string DataPublisher<DataType,MessageType>::get_topic_name() const
{
  return pub_->get_topic_name();
}


}

#endif
