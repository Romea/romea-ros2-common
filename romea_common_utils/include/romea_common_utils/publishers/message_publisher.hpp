#ifndef _romea_MessagePublisher_hpp_
#define _romea_MessagePublisher_hpp_

//ros
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include "../conversions/time_conversions.hpp"

namespace romea {

template<class DataType>
class MessagePublisherBase
{
public :

  MessagePublisherBase(){}

  virtual void publish(const DataType & data)=0;

  virtual ~MessagePublisherBase()=default;
};


template <class DataType, class MessageType>
class MessagePublisher : public MessagePublisherBase<DataType>
{

public :

    MessagePublisher();

    MessagePublisher(std::shared_ptr<rclcpp::Node> node,
                     const std::string & topic_name,
                     const size_t &queue_size);

    virtual ~MessagePublisher()=default;

public :

    void init(std::shared_ptr<rclcpp::Node> node,
              const std::string & topic_name,
              const size_t &queue_size);

    virtual void publish(const DataType & data)override;

protected :

    std::shared_ptr<rclcpp::Publisher<MessageType>> pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
MessagePublisher<DataType,MessageType>::MessagePublisher():
    pub_()
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
MessagePublisher<DataType,MessageType>::MessagePublisher(std::shared_ptr<rclcpp::Node> node,
                                                         const std::string & topic_name,
                                                         const size_t &queue_size):
    pub_()
{
    init(node,topic_name,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void MessagePublisher<DataType,MessageType>::init(std::shared_ptr<rclcpp::Node> node,
                                                  const std::string & topic_name,
                                                  const size_t &queue_size)
{
    pub_=node->create_publisher<MessageType>(topic_name,queue_size);
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void MessagePublisher<DataType,MessageType>::publish(const DataType &data)
{
    auto msg= std::make_unique<MessageType>();
    to_ros_msg(data,*msg.get());
    pub_->publish(std::move(msg));
}

}

#endif
